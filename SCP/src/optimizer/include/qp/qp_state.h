
#pragma once

#include <tuple>
#include <utility>
#include <vector>
#include <eigen3/Eigen/Eigen>
#include <osqp/osqp.h>
#include <ros/ros.h>
#include <iostream>

namespace qp{

    struct HybridAStartResult 
    {
        std::vector<double> x;             // x坐标
        std::vector<double> y;             // y坐标
        std::vector<double> phi;           //横摆角
        std::vector<double> v;             //速度
        std::vector<double> a;             //加速度
        std::vector<double> steer;         //方向盘转角
        std::vector<double> accumulated_s; //累计里程
    };

    using namespace std;
    using namespace Eigen;

    class FindStateProblem
    {
        public:
            FindStateProblem(ros::NodeHandle &nh);

            bool optimize(const vector<Matrix<double, 6, 1>> &init_lsit_car);

            vector<Matrix<double, 7, 1>> get_result();

        private:

            /* param */  
            std::array<double, 3> scale_factor_ = {{1.0, 1.0, 1.0}};   
            double acc_weight = 1.0;             
            double jerk_weight = 2.0;           
            double kappa_penalty_weight = 3.0;    
            double weight_x_ref_ = 4.0;       
            double ref_v_weight = 5.0;           
            
            /* variable */
            size_t num_of_knots_;  //node 数量
            double path_length_ = 0.0;

            vector<pair<double, double>> x_bounds_;  // s            
            vector<pair<double, double>> dx_bounds_;  // v
            vector<pair<double, double>> ddx_bounds_;  //a
            pair<double, double>  dddx_bound_;    //jerk

            double weight_x_ = 0.0;      
            double weight_dx_ = 0.0;   
            double weight_ddx_ = 0.0;
            double weight_dddx_ = 0.0;

            bool has_x_ref_ = true;  
            bool has_dx_ref_ = true;  
            bool has_end_state_ref_ = true;   
             
 

            /* object */   
            //与终点距离  
            std::vector<double> x_ref_;
            std::vector<double> weight_x_ref_vec_;   
            //与参考 v 距离
            std::vector<double> dx_ref_;          //参考v
            std::vector<double> weight_dx_ref_;
            //终点状态  --没用
            std::array<double, 3> weight_end_state_ = {0.0, 0.0, 0.0}; 
            std::array<double, 3> end_state_ref_;
            // 惩罚速度，没用
            std::vector<double> penalty_dx_;
            

            /*helper */
            void init_info(const vector<Matrix<double, 6, 1>> &init_lsit_car);
            double max_forward_v_,max_forward_acc_,max_jerk_;
            double delta_s_;  
            std::array<double, 3> x_init_;
            double weel_base_;
            void printSparseMatrix(int n, int* col_ptrs, int* row_indices, float* values);
            void lowerToUpperCSC(int n, int* col_ptrs, int* row_indices, float* values,
                        int* new_col_ptrs, int* new_row_indices, float* new_values);

            /* fun */
            void CalculateKernel_eigen(Eigen::MatrixXd& p, c_int* &P_indptr, c_int* &P_indices, c_float* &P_data);

            bool FormulateProblem(OSQPData* data);
            void CalculateKernel(std::vector<c_float>* P_data, std::vector<c_int>* P_indices, std::vector<c_int>* P_indptr);
            void CalculateAffineConstraint(std::vector<c_float>* A_data,
                                        std::vector<c_int>* A_indices,
                                        std::vector<c_int>* A_indptr,
                                        std::vector<c_float>* lower_bounds,
                                        std::vector<c_float>* upper_bounds);
            void CalculateOffset(std::vector<c_float>* q);

            void FreeData(OSQPData* data);

            OSQPSettings* SolverDefaultSettings();

            /* 设计很巧秒，利用模板，将vector中的内容，存储到一块新开辟的堆内存，并用指针指向该内存 */
            template <typename T>
            T* CopyData(const std::vector<T>& vec) 
            {
                T* data = new T[vec.size()];
                memcpy(data, vec.data(), sizeof(T) * vec.size());
                return data;
            }

            /* result */
            // osqp result
            std::vector<double> x_;    //应该都是升序排列的
            std::vector<double> dx_;
            std::vector<double> ddx_;

            vector<Matrix<double, 7, 1>> result;
    };
}