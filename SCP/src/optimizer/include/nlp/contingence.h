# pragma once

#include <eigen3/Eigen/Eigen>
#define HAVE_CSTDDEF
#include <coin/IpTNLP.hpp>
#include <coin/IpTypes.hpp>
#undef HAVE_CSTDDEF
#include <ros/ros.h>
#include <vector>
#include <geometry_msgs/PoseArray.h>
#include <unordered_map>
#include <adolc/adolc.h>
#include <adolc/adolc_sparse.h>
#include <adolc/adouble.h>


#include "nlp/ipopt_interface.h"
#include "corridor/vehicle_param.h"

namespace nlp 
{

using namespace Ipopt;
using namespace std;
using namespace Eigen;

using Ipopt::Index;  //因为 Eigen 中也有Index
using Ipopt::Number;

#define tag_f 1    //object 
#define tag_g 2
#define tag_L 3

class ipopt_contingence : public ipopt_interface_entrance
{
    public:    
        virtual ~ipopt_contingence() = default;

        /* API */
        ipopt_contingence(ros::NodeHandle &nh);

        void init_path_contigence(const vector<Eigen::Matrix<double, 7, 1>> &state_list, const vector<std::array<double, 8>> &boxes) override;

        void init_path_info(const std::vector<Eigen::Matrix<double, 6, 1>> &state_list,const std::vector<Eigen::Matrix<double, 4, 1>> &bounds, const std::vector<Eigen::Matrix<double, 4, 1>> &currents) override;

        void get_opt_result(MatrixXd &state_result, MatrixXd &control_result, double &time_result) override;

        //***************    start TNLP中的虚函数 ***********************************

        //NLP问题的基本信息，返回值为true时正常执行，若调用者发现优化过程中的异常可以返回 false 中止优化
        bool get_nlp_info(Index & n, Index & m, Index & nnz_jac_g, Index & nnz_h_lag, IndexStyleEnum & index_style) override;

        /* 自变量边界约束是线性，应该放到这里。 不知为何 apollo 加到约束复杂化问题
            对于状态约束，在SCP中已经线性化，当成边界约束 */
        bool get_bounds_info(Index n, Number * x_l, Number * x_u, Index m, Number * g_l, Number * g_u) override;

        /* 获得 warm start，由于是继承的 TNLP 类，所以函数是可以访问该类内部数据的。 里面默认只有 init_x 是 true*/
        bool get_starting_point(Index n, bool init_x, Number * x, bool init_z, Number * z_L, Number * z_U, Index m, bool init_lambda, Number * lambda) override;

        /**********************通用******************************** */
        /* bool new_x 忽略即可，这里涉及当 x 重复时，可以高效获得相应其他值的东西，IPOPT内部有自己的优化。  以下同理*/
        bool eval_f(Index n, const Number * x, bool new_x, Number & obj_value) override;

        bool eval_grad_f(Index n, const Number * x, bool new_x, Number * grad_f) override;

        bool eval_g(Index n, const Number * x, bool new_x, Index m, Number * g) override;

        bool eval_jac_g(Index n, const Number * x, bool new_x, Index m, Index nele_jac, Index * iRow, Index * jCol, Number * values) override;

        bool eval_h(Index n, const Number * x, bool new_x, Number obj_factor, Index m, const Number * lambda, bool new_lambda, Index nele_hess, Index * iRow, Index * jCol, Number * values) override;
        /**********************通用******************************** */

        /* 获得优化的结果 */
        void finalize_solution(SolverReturn status, Index n, const Number * x, const Number * z_L, const Number * z_U, Index m, const Number * g, const Number * lambda, Number obj_value, const IpoptData * ip_data, IpoptCalculatedQuantities * ip_cq) override;
        

        template <typename T> void eval_obj(Index n, const T *x, T& obj_value);

        template <typename T> void eval_constraints(Index n, const T *x, Index m, T *g);

        /* ********************通用************************** 第一次生成 tap***************/
        void generate_tapes(Index n, Index m, Index& nnz_jac_g, Index& nnz_h_lag);
        /**********************通用******************************** */

    private:

        /* traj param */
        MatrixXd xWS;
        MatrixXd uWS;
        Eigen::Matrix<double, 4, 1> start;
        Eigen::Matrix<double, 4, 1> end;

        /* ipopt param */
        Index num_of_variables;
        Index num_of_constraints;
        Index horizon;
        std::vector<std::array<double, 8>> xy_bounds;
        int control_start_index;
        int time_index;                              //时间用缩放比例优化，且只有比例一个变量
        double ts;                                   //采样时间
        VehicleParam_corridor vehicle;
        double f2x_, r2x_;
        

        double w_steer, w_a, w_current, w_steer_rate, w_a_rate, w_t;

        double max_speed_forward, max_speed_reverse, max_acc_forward, max_acc_reverse, min_time_scal, max_time_scal, max_steer_rate;
        double max_steer_angle, steer_ratio;

        double wheel_base;

        /* result */
        MatrixXd state_result, control_result;
        double time_result;

        //jacabian
        unsigned int *rind_g; 
        unsigned int *cind_g;
        double *jacval;       
        int nnz_jac;          
        int options_g[4];     

        //hessian
        unsigned int *rind_L;  
        unsigned int *cind_L;  
        double *hessval;       
        double *obj_lam;                    
        int nnz_L;               
        int options_L[4];     
        
};
}

