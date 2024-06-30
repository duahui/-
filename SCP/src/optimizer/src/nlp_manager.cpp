#include "manager/nlp_manager.h"
#include <iostream>

namespace nlp_manager
{

    nlp_solver::nlp_solver(ros::NodeHandle & nh)
    {
        // Create an instance of  nlp  涉及虚继承 以及 继承的多态性  
        ptop = new nlp::ipopt_contingence(nh);

        // Create an instance of the IpoptApplication
        app = IpoptApplicationFactory();

        // 设置MUMPS库使用的内存百分比
        // app->Options()->SetIntegerValue("mumps_mem_percent", 2);

        // 设置MUMPS库的主元选取阈值
        // app->Options()->SetNumericValue("mumps_pivtol", 3);

        //设置迭代的最大次数
        app->Options()->SetIntegerValue("max_iter", 2000);

        //设置收敛容许误差
        app->Options()->SetNumericValue("tol", 2);

        //设置可接受的约束违反容许误差
        app->Options()->SetNumericValue("acceptable_constr_viol_tol",0.1);

        //设置最小海森矩阵扰动值
        // app->Options()->SetNumericValue("min_hessian_perturbation",7);

        //设置雅可比矩阵正则化值
        // app->Options()->SetNumericValue("jacobian_regularization_value",8);

        //设置Y向量的alpha值
        // app->Options()->SetStringValue("alpha_for_y", "10");

        //设置是否重新计算Y向量
        // app->Options()->SetStringValue("recalc_y", "11");

        //设置初始mu值
        // app->Options()->SetNumericValue("mu_init",12);

        Ipopt::ApplicationReturnStatus status = app->Initialize();
        if (status != Ipopt::Solve_Succeeded) 
        {
            std::cout << "*** Distance Approach problem error during initialization!"<<endl;
        }
    }    
    bool nlp_solver::Optimize(std::string flag, const vector<Eigen::Matrix<double, 7, 1>> &state_list, const std::vector<std::array<double, 8>> &boxes)
    {
        if (flag == "contingence")
        {
            bool flag_ = solve(state_list, boxes);
            return flag_;            
        }
        
    }
    bool nlp_solver::solve(const vector<Matrix<double, 7, 1>> &state_list, const std::vector<std::array<double, 8>> &boxes)  
    {
        ptop->init_path_contigence(state_list, boxes);

        Ipopt::SmartPtr<Ipopt::TNLP> nlp_solver = ptop;                   //双层多态应该在初始化完毕后，再传给 TNLP 吧

            // Initialize the IpoptApplication and process the options
        Ipopt::ApplicationReturnStatus status = app->OptimizeTNLP(nlp_solver);
        std::cout<<status<<std::endl;
        if (status == Ipopt::Solve_Succeeded || status == Ipopt::Solved_To_Acceptable_Level || true) 
        {
            // Retrieve some statistics about the solve
            Ipopt::Index iter_count = app->Statistics()->IterationCount();
            printf("\n\n*** The problem solved in %d iterations!\n", iter_count);

            Ipopt::Number final_obj = app->Statistics()->FinalObjective();
            printf("\n\n*** The final value of the objective function is %e.\n", final_obj);

            //convert_result
            MatrixXd state_result_ = MatrixXd::Zero(4, state_list.size());        
            MatrixXd control_result_ = MatrixXd::Zero(2, state_list.size() - 1);; 
            double time_result_;
            opt_state.resize(state_list.size());

            ptop->get_opt_result(state_result_, control_result_, time_result_);
            for (int i = 0; i < state_list.size(); ++i)
            {
                //x,y,phi,v,steer,a,t
                opt_state[i](0) = state_result_(0, i);
                opt_state[i](1) = state_result_(1, i);
                opt_state[i](2) = state_result_(2, i);
                opt_state[i](3) = state_result_(3, i);

                opt_state[i](6) = time_result_ * i;

                if(i == state_list.size() - 1)
                {
                    opt_state[i](4) = control_result_(0, i - 1);
                    opt_state[i](5) = control_result_(1, i - 1);
                    continue;
                }
                opt_state[i](4) = control_result_(0, i);
                opt_state[i](5) = control_result_(1, i);
            }
            

            return true;
        }
    }
}
