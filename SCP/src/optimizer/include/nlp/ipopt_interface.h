#pragma once
#define HAVE_CSTDDEF
#include <coin/IpTNLP.hpp>
#include <coin/IpTypes.hpp>
#undef HAVE_CSTDDEF
#include <vector>
#include <ros/ros.h>
#include <array>

namespace nlp 
{

using namespace Ipopt;
using namespace std;
using namespace Eigen;

using Ipopt::Index;  //因为 Eigen 中也有Index
using Ipopt::Number;

class ipopt_interface_entrance : public TNLP
{
    public:    
        virtual ~ipopt_interface_entrance() = default;

        /* API */  
        virtual void init_path_contigence(const vector<Eigen::Matrix<double, 7, 1>> &state_list, const std::vector<std::array<double, 8>> &boxes) = 0;

        virtual void init_path_info(const std::vector<Eigen::Matrix<double, 6, 1>> &state_list,const std::vector<Eigen::Matrix<double, 4, 1>> &bounds, const std::vector<Eigen::Matrix<double, 4, 1>> &currents) = 0;

        virtual void get_opt_result(Eigen::MatrixXd &state_result, Eigen::MatrixXd &control_result, double &time_result) = 0;

        
        virtual bool get_nlp_info(Index & n, Index & m, Index & nnz_jac_g, Index & nnz_h_lag, IndexStyleEnum & index_style) = 0;

            virtual bool get_bounds_info(Index n, Number * x_l, Number * x_u, Index m, Number * g_l, Number * g_u) = 0;

         virtual bool get_starting_point(Index n, bool init_x, Number * x, bool init_z, Number * z_L, Number * z_U, Index m, bool init_lambda, Number * lambda) = 0;

          virtual bool eval_f(Index n, const Number * x, bool new_x, Number & obj_value)= 0;

        virtual bool eval_grad_f(Index n, const Number * x, bool new_x, Number * grad_f) = 0;

        virtual bool eval_g(Index n, const Number * x, bool new_x, Index m, Number * g) = 0;

        virtual bool eval_jac_g(Index n, const Number * x, bool new_x, Index m, Index nele_jac, Index * iRow, Index * jCol, Number * values) = 0;

        virtual bool eval_h(Index n, const Number * x, bool new_x, Number obj_factor, Index m, const Number * lambda, bool new_lambda, Index nele_hess, Index * iRow, Index * jCol, Number * values) = 0;
        
          virtual void finalize_solution(SolverReturn status, Index n, const Number * x, const Number * z_L, const Number * z_U, Index m, const Number * g, const Number * lambda, Number obj_value, const IpoptData * ip_data, IpoptCalculatedQuantities * ip_cq) = 0;

};
}