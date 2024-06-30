#ifndef SEARCHER_MANAGE_H
#define SEARCHER_MANAGE_H

#include"grid_searcher/kino_astar.h"
#include"grid_searcher/hybrid_a_star.h"

#include<sensor_msgs/PointCloud2.h>
#include<ros/ros.h>
#include<cmath>
#include<geometry_msgs/PoseStamped.h>
#include<eigen3/Eigen/Eigen>
#include<visualization_msgs/Marker.h>
#include "global_map/Obstacles.h"

using namespace std;
using namespace Eigen;

                 
class searcher_manage
{

    public:

        searcher_manage(ros::NodeHandle &nh);
        searcher_manage():start_pt(0.0, 0.0, 0.0), start_vel(0.0, 0.0, 0.0), start_acc(0.0, 0.0, 0.0), end_pt(0.0, 0.0, 0.0), end_vel(0.0, 0.0, 0.0){}
        vector<Matrix<double, 10, 1>> GetTrajPoints()
        {
            vector<Matrix<double, 10, 1>> state_list;
            if(has_path)
            {                
                state_list = kino_astar_finder_->GetSamples(sam_kino_interval);
                has_path = false;
                return std::move(state_list);
            }
            else
            {
                state_list.clear();
                return state_list;
            }
        }
        std::shared_ptr<HybridAStartResult> get_car_Traj()
        {
            if(has_car_path)
            {
                has_car_path = false;
                return result;                
            }
            return nullptr;
        }
    private:
        std::unique_ptr<kino_astar::kino_astar_finder> kino_astar_finder_;
        
        int searcher_flag;  //1:kino_astar  

        double resolution,inv_resoultion,x_size,y_size,z_size,gl_xl,gl_yl,gl_zl;
        int GLX_SIZE,GLY_SIZE,GLZ_SIZE;

        Vector3d start_pt, start_vel, start_acc, end_pt, end_vel;   //没有姿态信息
        ros::Subscriber GlobalMapSub;
        ros::Subscriber GlobalMapObSub;
        ros::Subscriber static_obs_sub;
        void ObstaclesCallback(const global_map::ObstaclesConstPtr &msg);

        bool has_map, dim_flag;
        void global_mapCallback(const sensor_msgs::PointCloud2 &GlobalMap);        

        double sam_kino_interval;

        /* map_visulaize */ 
        sensor_msgs::PointCloud2 DisplayMap;  //
        ros::Publisher DisMapPub;       
        inline Eigen::Vector3d coordRounding(const Eigen::Vector3d & coord);
        inline Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d & pt);
        inline Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i & index);

        /* SubGoal */
        ros::Subscriber goal_sub;
        void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

        /* kino_vis */
        ros::Publisher kino_vis;
        void PubKinoTraj(const vector<Vector3d> &nodes);
        double vis_kino_interval;

        /* kino_complete */
        bool has_path = false;

        /* apollo info */
        bool has_map_ob;
        bool use_car_search;
        bool has_car_path = false;;
        void global_mapObCallback(const sensor_msgs::PointCloud2 &GlobalMap);
        std::vector<std::vector<common::math::Vec2d>> obstacles_vertices_vec;
        std::vector<double> XYbounds;
        std::unique_ptr<HybridAStar> kino_astar_car;
        std::shared_ptr<HybridAStartResult> result;
        void clear_result();

        ros::Publisher kino_car_vis;
        void PubCarKinoTraj();

        void test();
};

inline Eigen::Vector3d searcher_manage::coordRounding(const Eigen::Vector3d & coord)
{
    return gridIndex2coord(coord2gridIndex(coord));  //这个是为了把8.5变成0.2精度下，也就是8.4。8.4可直接在。0.2栅格下表示
    
}

inline Eigen::Vector3i searcher_manage::coord2gridIndex(const Eigen::Vector3d & pt) 
{
    Eigen::Vector3i idx;
    idx <<  min( max( static_cast<int>( (pt(0) - gl_xl) * inv_resoultion), 0), GLX_SIZE - 1),
            min( max( static_cast<int>( (pt(1) - gl_yl) * inv_resoultion), 0), GLY_SIZE - 1),
            min( max( static_cast<int>( (pt(2) - gl_zl) * inv_resoultion), 0), GLZ_SIZE - 1);                  
  
    return idx;
}

inline Eigen::Vector3d searcher_manage::gridIndex2coord(const Eigen::Vector3i & index) 
{
    Eigen::Vector3d pt;

    pt(0) = (static_cast<double>(index(0)) + 0.5) * resolution + gl_xl;
    pt(1) = (static_cast<double>(index(1)) + 0.5) * resolution + gl_yl;
    pt(2) = (static_cast<double>(index(2))+ 0.5) * resolution + gl_zl;

    return pt;
}

#endif

