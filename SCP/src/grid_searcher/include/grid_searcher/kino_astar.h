#ifndef KINO_ASTAR_H    //如果不加这个，该头文件里的函数会重复定义
#define KINO_ASTAR_H

#include<eigen3/Eigen/Eigen>
#include<eigen3/unsupported/Eigen/CXX11/Tensor>
#include<unordered_map>
#include<ros/ros.h>
#include<queue>
#include<functional>
#include<vector>


#define IN_CLOSED_SET 'a'
#define IN_OPEN_SET 'b'
#define NOT_EXPAND 'c'   
namespace kino_astar{

    using namespace std;
    using namespace Eigen;

    /*节点数据结构 */
    struct PathNode
    {
        //无人机
        Eigen::Vector3i index;
        Eigen::Matrix<double, 6, 1> state;  
                                         
        //无人车
        double x;
        double y;        
        double phi;
        double v;
        double a;
        double steer;

        double f_score = INFINITY;
        double g_score = INFINITY;
        Eigen::Vector3d input;
        double duration;
        shared_ptr<PathNode> parent = NULL;
        char node_state = NOT_EXPAND;

        PathNode(int x,int y,int z):index(Eigen::Vector3i(x,y,z)){};
        PathNode(Vector3i &idx):index(idx){};
        PathNode(){};

        bool operator<(const PathNode &node2)   //重载一个tie_breaker版本。但一般需要与值比，所以代码中对h稍微放大，采用另一种tie_breaker了
        {
            return f_score == node2.f_score ? g_score < node2.g_score : f_score < node2.f_score;
        }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    };
    typedef std::shared_ptr<PathNode> PathNodePtr;
    

    inline size_t HashFun(const Vector3i &matrix)
    {
        size_t seed=0;
        for (int i = 0; i < matrix.size(); ++i)
        {
            auto elem=*(matrix.data()+i);
            seed ^=hash<Vector3i::Scalar>()(elem)+0x9e3779b9 + (seed << 6) +(seed >> 2);  //标准库哈希+魔数+左右移位
        }
        return seed;
    }

    struct NodeHashTable
    {
        public:
            void insert(const Vector3i &idx,const PathNodePtr &node_ptr)
            {
                data_3d.insert(make_pair(idx,node_ptr));
            }

            PathNodePtr find(const Vector3i &idx)
            {
                auto iter=data_3d.find(idx);
                return iter ==data_3d.end() ? NULL : iter->second;     
            }

            void clear()
            {
                data_3d.clear();
            } 

            int size()
            {
                return data_3d.size();
            }
            
            NodeHashTable()     
            {                  
                data_3d = unordered_map<Vector3i,PathNodePtr,decltype(&HashFun)>(1,HashFun);
            }                                           
        private:
            unordered_map<Vector3i,PathNodePtr,decltype(&HashFun)> data_3d;  
                                                             //function<size_t( const Vector3i &matrix )>                           
    };

    struct OpenCom
    {
        bool operator()(const PathNodePtr &node1, const PathNodePtr &node2)
        {
            return node1->f_score == node2->f_score ? node1->g_score > node2->g_score : node1->f_score > node2->f_score;
        }
    };

    /* kino_astar 类 */
    class kino_astar_finder  //静态 ob 版
    {

        public:      
           
            void init(ros::NodeHandle &nh);
            
            inline void SetObs(const double &coord_x,const double &coord_y,const double &coord_z);

            bool search(Vector3d start_pt, Vector3d start_vel, Vector3d start_acc, Vector3d end_pt, Vector3d end_vel, bool init_search = true);

            vector<Vector3d> GetKinoTraj(double delta_t);

            vector<Matrix<double, 10, 1>> GetSamples(double delta_t);

            ~kino_astar_finder() = default;
              
        protected:

            
            NodeHashTable expanded_nodes;   
            Tensor<bool, 3> ob_nodes;       // 
            priority_queue<PathNodePtr,vector<PathNodePtr>,OpenCom> open_set;

            /* shared data */
            PathNodePtr start_node,end_node;   //
            Matrix<double, 6, 6> ph_i;

            /* parameter */            
            //离散输入
            double max_tau, init_max_tau;
            double max_vel, max_acc;

            double lambda_heu;               //
            double w_time;        //
            double tie_breaker;   //
            bool dim_flag;          //
            int check_num;        //          

            //map
            double resolution, inv_resolution;
            Vector3d map_size;
            double gl_xl, gl_yl, gl_zl, gl_xu, gl_yu, gl_zu;

            /* helper fun */            
            inline void stateTransit(const Eigen::Matrix<double, 6, 1> &state0, Eigen::Matrix<double, 6, 1> &state1,const Eigen::Vector3d &um, double tau);
            void GetInputs(vector<Vector3d> &inputs, vector<double> &durations, Vector3d start_acc_,  bool init_search = true);
            bool check_col(const Eigen::Matrix<double, 6, 1> &state0, const Eigen::Vector3d &um, double tot_tau);
            inline void reset();

            /* potryagin shot trajectory  我没有隔一段时间就one_shot一次*/
            vector<double> cubic(double a, double b, double c, double d);
            vector<double> quartic(double a, double b, double c, double d, double e);
            double estimateHeuristic(const Matrix<double, 6, 1> x1, const Matrix<double, 6, 1> x2, double& optimal_time);
    
            /* index transition */
            inline Eigen::Vector3i posToIndex(const Eigen::Vector3d &pt);
    };

    inline void kino_astar_finder::SetObs(const double & coord_x, const double & coord_y, const double & coord_z)
    {
        Vector3i ob_idx;
        ob_idx = posToIndex(Vector3d(coord_x, coord_y, coord_z));  
        if (ob_idx(0) == -1) return;
        ob_nodes(ob_idx(0),ob_idx(1),ob_idx(2)) = true;
        // ROS_INFO("idx_x: %d, idx_y: %d, idx_z: %d",idx_x,idx_y,idx_z);        
    }

    inline Eigen::Vector3i kino_astar_finder::posToIndex(const Eigen::Vector3d &pt)
    {        
        if( pt(0) < gl_xl  || pt(1) < gl_yl  || pt(2) <  gl_zl || 
                pt(0) > gl_xu || pt(1) > gl_yu || pt(2) > gl_zu )
                {
                    // ROS_INFO("%f, %f, %f :", gl_xl, gl_yl, gl_zl);
                    // ROS_INFO("%f, %f, %f :", gl_xu, gl_yu, gl_zu);
                    // ROS_INFO("%f, %f, %f :", pt(0), pt(1), pt(2));
                    return Vector3i(-1, -1, -1);
                }
                
        int idx_x = static_cast<int>( (pt(0) - gl_xl) * inv_resolution);  //向下取整数，会有一定误差，此时分辨率越大越贴切实际ob
        int idx_y = static_cast<int>( (pt(1) - gl_yl) * inv_resolution);
        int idx_z = static_cast<int>( (pt(2) - gl_zl) * inv_resolution); 
        // ROS_INFO("idx_x: %d, idx_y: %d, idx_z: %d",idx_x,idx_y,idx_z);
        Vector3i idx (idx_x, idx_y, idx_z);
        return idx;
    }

    /* 此处状态转换等价于双积分器 */
    inline void kino_astar_finder::stateTransit(const Eigen::Matrix<double, 6, 1> &state0, Eigen::Matrix<double, 6, 1> &state1,const Eigen::Vector3d &um, double tau)
    {
        for (int i = 0; i < 3; ++i)   
                ph_i(i, i + 3) = tau;    //凑线性系统的准确状态转移表达式

            Eigen::Matrix<double, 6, 1> integral;
            integral.head(3) = 0.5 * pow(tau, 2) * um;
            integral.tail(3) = tau * um;

            state1 = ph_i * state0 + integral;
    }

    inline void kino_astar_finder::reset()
    {
        expanded_nodes.clear();

        priority_queue<PathNodePtr,vector<PathNodePtr>,OpenCom> empty_queue;
        open_set.swap(empty_queue);  
        start_node = make_shared<PathNode>();
        end_node = make_shared<PathNode>();      

    }

}

#endif