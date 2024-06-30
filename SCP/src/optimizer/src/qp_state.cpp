

#include "qp/qp_state.h"
#include "qp/speed_data.h"
#include "qp/discretized_path.h"
#include <iostream>

namespace qp{

    FindStateProblem::FindStateProblem(ros::NodeHandle & nh)
    {
        nh.param("opt/max_speed_forward", max_forward_v_, 2.0);
        nh.param("opt/max_acc_forward", max_forward_acc_, 2.0);
        nh.param("opt/delta_t", delta_s_, 0.2);
        nh.param("opt/max_jerk", max_jerk_, 4.0);
        nh.param("opt/weel_base", weel_base_, 2.8448);

    }

    
    vector<Matrix<double, 7, 1>> FindStateProblem::get_result()
    {
        /* 
        cout<< "x:"<<endl;
        for (int i = 0; i < result.size(); i++)
        {
            cout<<  result[i](0)<<",";
        }
        cout<<endl;

        cout<< "y:"<<endl;
        for (int i = 0; i < result.size(); i++)
        {
            cout<< result[i](1)<<",";
        }
        cout<<endl;

        cout<< "phi:"<<endl;
        for (int i = 0; i < result.size(); i++)
        {
            cout<< result[i](2)<<",";
        }
        cout<<endl;
        */

        //x,y,phi,v,steer,a
        //start
        result[0](3) = x_init_[1];   //v
        result[0](5) = x_init_[2];   //a

        result.back()(3) = dx_.back();   //v
        result.back()(5) = ddx_.back();   //a
        result.back()(4) = 0;   //steer

        double length_temp = 0.0; 
        vector<double> lengths;
        lengths.push_back(length_temp);
        for (size_t i = 1; i < result.size(); ++i) 
        {
            double x_diff = result[i](0) - result[i-1](0);
            double y_diff = result[i](1) - result[i-1](1);
            length_temp += std::sqrt(x_diff * x_diff + y_diff * y_diff);
            lengths.push_back(length_temp);
        }

        bool gear = true;    //true 是往前开
        // extract output
        const std::vector<double> &s = x_;
        const std::vector<double> &ds = dx_;
        const std::vector<double> &dds = ddx_;

        size_t num_of_knots = num_of_knots_;
        double delta_t = delta_s_;
        // assign speed point by gear
        SpeedData speed_data;
        speed_data.AppendSpeedPoint(s[0], 0.0, ds[0], dds[0], 0.0);
        const double kEpislon = 1.0e-6;
        const double sEpislon = 1.0e-6;
        for (size_t i = 1; i < num_of_knots; ++i) {
            if (s[i - 1] - s[i] > kEpislon) {
            // std::cout << "unexpected decreasing s in speed smoothing at time "
            //             << static_cast<double>(i) * delta_t << "with total time "
            //             << total_t;
            break;
            }
            speed_data.AppendSpeedPoint(s[i], delta_t * static_cast<double>(i), ds[i],
                                        dds[i], (dds[i] - dds[i - 1]) / delta_t);
            // cut the speed data when it is about to meet end condition
            if (length_temp - s[i] < sEpislon) {
            break;
            }
        }

        // combine speed and path profile


        DiscretizedPath path_data;
        for (size_t i = 0; i < result.size(); ++i) {
            common::PathPoint path_point;
            path_point.set_x(result[i](0));
            path_point.set_y(result[i](1));
            path_point.set_theta(result[i](2));
            path_point.set_s(lengths[i]);
            path_data.push_back(std::move(path_point));
        }

        HybridAStartResult combined_result;

        // TODO(Jinyun): move to confs
        const double kDenseTimeResoltuion = 0.5;
        const double time_horizon =
            speed_data.TotalTime() + kDenseTimeResoltuion * 1.0e-6;
        if (path_data.empty()) {
            std::cout << "path data is empty";
        }
        vector<double> time;
        
        for (double cur_rel_time = 0.0; cur_rel_time < time_horizon;
            cur_rel_time += kDenseTimeResoltuion) {
            common::SpeedPoint speed_point;
            if (!speed_data.EvaluateByTime(cur_rel_time, &speed_point)) {
            std::cout << "Fail to get speed point with relative time "
                        << cur_rel_time;
            }

            if (speed_point.s() > path_data.Length()) {
            break;
            }

            common::PathPoint path_point = path_data.Evaluate(speed_point.s());

            combined_result.x.push_back(path_point.x());
            combined_result.y.push_back(path_point.y());
            combined_result.phi.push_back(path_point.theta());
            combined_result.accumulated_s.push_back(path_point.s());
            if (!gear) {
            combined_result.v.push_back(-speed_point.v());
            combined_result.a.push_back(-speed_point.a());
            } else {
            combined_result.v.push_back(speed_point.v());
            combined_result.a.push_back(speed_point.a());
            }
            time.emplace_back(cur_rel_time);
        }

        combined_result.a.pop_back();

        // recalc step size
        size_t path_points_size = combined_result.x.size();

        // load steering from phi
        for (size_t i = 0; i + 1 < path_points_size; ++i) {
            double discrete_steer =
                (combined_result.phi[i + 1] - combined_result.phi[i]) *
                weel_base_ /
                (combined_result.accumulated_s[i + 1] -
                combined_result.accumulated_s[i]);
            discrete_steer =
                gear ? std::atan(discrete_steer) : std::atan(-discrete_steer);
            combined_result.steer.push_back(discrete_steer);
        }

        result.resize(combined_result.x.size());

        result.back()(3) = dx_.back();   //v
        result.back()(5) = ddx_.back();   //a
        result.back()(4) = 0;   //steer

        for(int i = 0; i < result.size(); ++i)
        {
            result[i](0) = combined_result.x[i];
            result[i](1) = combined_result.y[i];
            result[i](2) = combined_result.phi[i];
            result[i](3) = combined_result.v[i];
            result[i](6) = time[i];
            if (i == result.size() -1 )
                continue;
            result[i](4) = combined_result.steer[i];
            result[i](5) = combined_result.a[i];            
        }
        // cout<< combined_result.a.size() << "   " << combined_result.steer.size() << "   " << combined_result.v.size() << "   " << combined_result.x.size() << endl;
        cout<< "x:"<<endl;
        for (int i = 0; i < result.size(); i++)
        {
            cout<<  result[i](0)<<",";
        }
        cout<<endl;

        cout<< "y:"<<endl;
        for (int i = 0; i < result.size(); i++)
        {
            cout<< result[i](1)<<",";
        }
        cout<<endl;

        cout<< "phi:"<<endl;
        for (int i = 0; i < result.size(); i++)
        {
            cout<< result[i](2)<<",";
        }
        cout<<endl;

        cout<< "t:"<<endl;
        for (int i = 0; i < result.size(); i++)
        {
            cout<< result[i](6)<<",";
        }
        cout<<endl;

        cout<< "v:"<<endl;
        for (int i = 0; i < result.size(); i++)
        {
            cout<<  result[i](3)<<",";
        }
        cout<<endl;

        cout<< "a:"<<endl;
        for (int i = 0; i < result.size(); i++)
        {
            cout<< result[i](5)<<",";
        }
        cout<<endl;

        cout<< "steer:"<<endl;
        for (int i = 0; i < result.size(); i++)
        {
            cout<< result[i](4)<<",";
        }
        cout<<endl;
        
        return result;
    }


    bool FindStateProblem::optimize(const vector<Matrix<double, 6, 1>> &init_lsit_car)
    {
        init_info(init_lsit_car);

        /*reinterpret_cast 类型转换更接近底层，不需要类型检查。c_malloc 返回 void* 指针，在C语言中可直接转换为 OSQPData*，但C++不允许*/
        OSQPData* data = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));
        if(!FormulateProblem(data))
        {
            cout<< "problem_false"<< endl;
        }

        OSQPSettings* settings = SolverDefaultSettings();
        settings->max_iter = 100000;
        OSQPWorkspace* osqp_work = nullptr;
        // osqp_work = osqp_setup(data, settings);
        osqp_setup(&osqp_work, data, settings);
        osqp_solve(osqp_work);
        auto status = osqp_work->info->status_val;
          
        // fail
        if (status < 0 || (status != 1 && status != 2)) 
        {
            cout << "failed optimization status:\t" << osqp_work->info->status << endl;
            osqp_cleanup(osqp_work);
            FreeData(data);
            c_free(settings);
            return false;
        } 
        else if (osqp_work->solution == nullptr) 
        {
            cout << "The solution from OSQP is nullptr" <<endl;
            osqp_cleanup(osqp_work);
            FreeData(data);
            c_free(settings);
            return false;
        }

        // extract primal results
        x_.resize(num_of_knots_);
        dx_.resize(num_of_knots_);
        ddx_.resize(num_of_knots_);
        for (size_t i = 0; i < num_of_knots_; ++i) 
        {
            x_.at(i) = osqp_work->solution->x[i] / scale_factor_[0];
            dx_.at(i) = osqp_work->solution->x[i + num_of_knots_] / scale_factor_[1];
            ddx_.at(i) = osqp_work->solution->x[i + 2 * num_of_knots_] / scale_factor_[2];
        }

        // Cleanup
        osqp_cleanup(osqp_work);
        FreeData(data);
        c_free(settings);
        return true;
    }

    void FindStateProblem::init_info(const vector<Matrix<double, 6, 1>> &init_lsit_car)
    {
        result.resize(init_lsit_car.size());
        for (int i = 0; i < init_lsit_car.size(); i++)
        {
            result[i].block(0, 0, 6, 1) = init_lsit_car[i];
            
        }
        
        //以直线的形式近似得到 path_length_
        path_length_ = 0.0; 
        for (size_t i = 1; i < init_lsit_car.size(); ++i) 
        {
            double x_diff = init_lsit_car[i](0) - init_lsit_car[i-1](0);
            double y_diff = init_lsit_car[i](1) - init_lsit_car[i-1](1);
            path_length_ += std::sqrt(x_diff * x_diff + y_diff * y_diff);
        }
        double total_t = 1.5 * (max_forward_v_ * max_forward_v_ +
                           path_length_ * max_forward_acc_) /
                          (max_forward_acc_ * max_forward_v_);

        num_of_knots_ = static_cast<size_t>(total_t / delta_s_) + 1;

        //s
        x_bounds_.clear();
        x_bounds_.resize(num_of_knots_, {0.0, path_length_});
        x_bounds_[0] = std::make_pair(0.0, 0.0);  //start
        x_init_ = {0.0, 0.0, 0.1};     //s v a   a不能为0的，不然车怎么移动
        x_bounds_[num_of_knots_ - 1] = std::make_pair(path_length_, path_length_);   //end

        //v
        dx_bounds_.clear();
        dx_bounds_.resize(num_of_knots_, {0.0, max_forward_v_});
        dx_bounds_[0] = std::make_pair(0.0, 0.0);  //start
        dx_bounds_[num_of_knots_ - 1] = std::make_pair(0.0, 0.0);   //end

        // a
        ddx_bounds_.clear();
        ddx_bounds_.resize(num_of_knots_, {-max_forward_acc_, max_forward_acc_});
        ddx_bounds_[num_of_knots_ - 1] = std::make_pair(0.0, 0.0);   //end

        // jerk
        dddx_bound_.first = -max_jerk_;
        dddx_bound_.second = max_jerk_;

        //与终点距离
        x_ref_.clear();
        weight_x_ref_vec_.clear();
        weight_x_ref_vec_.resize(num_of_knots_, weight_x_ref_);
        x_ref_.resize(num_of_knots_,path_length_);

        //与参考 v 距离
        weight_dx_ref_.resize(num_of_knots_, ref_v_weight);
        dx_ref_.resize(num_of_knots_, max_forward_v_ * 0.8);

        //weight a
        weight_ddx_ = acc_weight;

        //weight jerk
        weight_dddx_ = jerk_weight;

        // 惩罚速度，没用
        penalty_dx_.resize(num_of_knots_, 0.0);
    }

    bool FindStateProblem::FormulateProblem(OSQPData* data)
    {
        // 不适用于 新版本，新版本必须上三角
        // calculate kernel  object 中的二次项矩阵，即半正定矩阵，存放系数
        // std::vector<c_float> P_data;
        // std::vector<c_int> P_indices;
        // std::vector<c_int> P_indptr;
        // CalculateKernel(&P_data, &P_indices, &P_indptr);

        // 将Eigen矩阵转换为OSQP格式
        c_int* P_indptr;
        c_int* P_indices;
        c_float * P_data;
        const int n = static_cast<int>(num_of_knots_);
        const int kernel_dim = 3 * n;
        Eigen::MatrixXd p = MatrixXd::Zero(kernel_dim, kernel_dim);
        CalculateKernel_eigen(p, P_indptr, P_indices, P_data);

        // cout<< "P_data:" << endl; 
        // for(int i = 0; i< P_data.size(); ++i)
        // {
        //     cout<< P_data[i] << ",";
        //     // cout<< data_temp[i]<<endl;
        // }
        // cout<< P_data.size()<< endl;

        // cout<< "P_indices:" << endl; 
        // for(int i = 0; i< P_indices.size(); ++i)
        // {
        //     cout<< P_indices[i] << ",";
        //     // cout<< data_temp[i]<<endl;
        // }
        // cout<<P_indices.size()<<endl;

        // cout<< "P_indptr:" <<endl; 
        // for(int i = 0; i< P_indptr.size(); ++i)
        // {
        //     cout<< P_indptr[i] << ",";
        //     // cout<< data_temp[i]<<endl;
        // }
        // cout<<P_indptr.size()<<endl;

        // calculate affine constraints   约束中的 A和 b矩阵，等式约束直接上下界为0
        std::vector<c_float> A_data;
        std::vector<c_int> A_indices;
        std::vector<c_int> A_indptr;
        std::vector<c_float> lower_bounds;
        std::vector<c_float> upper_bounds;
        CalculateAffineConstraint(&A_data, &A_indices, &A_indptr, &lower_bounds, &upper_bounds);

        // calculate offset   object中的 q线性项矩阵
        std::vector<c_float> q;
        CalculateOffset(&q);
        
        size_t num_affine_constraint = lower_bounds.size();

        /* osqp 只接受 c_float c_int 等的指针，且此指针还要指向新开辟的一块连续内存，所以需要把 vector 中的内容换成指针指向的堆内存*/
        data->n = kernel_dim;
        data->m = num_affine_constraint;

        // data->P = csc_to_triu(csc_matrix(kernel_dim, kernel_dim, P_data.size(), CopyData(P_data),
        //                     CopyData(P_indices), CopyData(P_indptr)));  //apollo 弄错了，我自己用这个函数可以转换一下

        // data->P = csc_matrix(kernel_dim, kernel_dim, P_data.size(), CopyData(P_data), CopyData(P_indices), CopyData(P_indptr));

        data->P = csc_to_triu(csc_matrix(kernel_dim, kernel_dim, p.nonZeros(), P_data, P_indices, P_indptr));


        data->q = CopyData(q);
        data->A = csc_matrix(num_affine_constraint, kernel_dim, A_data.size(),
                        CopyData(A_data), CopyData(A_indices), CopyData(A_indptr));
        data->l = CopyData(lower_bounds);
        data->u = CopyData(upper_bounds);

        if(lower_bounds.size() != upper_bounds.size())
        {
            cout<< "bounds_fault"<<endl;
            return false;
        }

        return true;
    }

    // calculate kernel  object 中的二次项矩阵，即半正定矩阵，存放系数
    /*存放二次项时，对于差值项，是需要展开的。平方项给到P矩阵的斜对角; 自变量之间的乘积由对角线之外位置表示；对于单个一次方自变量给到线性项。 */
    void FindStateProblem::CalculateKernel(std::vector<c_float>* P_data, std::vector<c_int>* P_indices, std::vector<c_int>* P_indptr)
    {
        const int n = static_cast<int>(num_of_knots_);
        const int kNumParam = 3 * n;
        const int kNumValue = 4 * n - 1;

        /* vector 嵌套 vector 来实现csc矩阵。 csc矩阵是按列存储的 indices 是 第一列所有非0行，第二列所有非0行.....
            columns 是P矩阵的每一列，每一列里first是行索引，second是值。这样indices 直接就是一次first，data 直接就是second.
            所以 indices 和 data 只需要 将每个列的 pair 依次存储即可，对于 indptr 只需要加一个计数器即可
            所以对于平方项，是columns[i][i]，依次类推 */
        /* 该嵌套 vector 与后续存放 csc 数据的方式，只能存放下三角的数据，所以变量相乘项需要额外乘 2， 相当于 不是对称矩阵。适用于 0.50版本
           对应新版本 0.6 以上 必须为右上角对称，所以该方法不适用
           我发现可以嵌套一层 csc_to_triu 函数来实现上三角，但我不知道这样对不对 ，同时其他人一般直接用 eigen 然后函数转 csc，当然 apollo这种又快又省内存*/

        std::vector<std::vector<std::pair<c_int, c_float>>> columns;
        columns.resize(kNumParam);
        int value_index = 0;

        // x(i)^2 * w_x_ref    --终点不用差值
        for (int i = 0; i < n - 1; ++i) {
            columns[i].emplace_back(
                i, weight_x_ref_ / (scale_factor_[0] * scale_factor_[0]));
            ++value_index;
        }
        // x(n-1)^2 * (w_x_ref + w_end_x)   --终点惩罚，此处没用
        columns[n - 1].emplace_back(n - 1, (weight_x_ref_ + weight_end_state_[0]) /
                                                (scale_factor_[0] * scale_factor_[0]));
        ++value_index;

        // x(i)'^2 * (w_dx_ref + penalty_dx)
        for (int i = 0; i < n - 1; ++i) {
            columns[n + i].emplace_back(n + i,
                                        (weight_dx_ref_[i] + penalty_dx_[i]) /
                                            (scale_factor_[1] * scale_factor_[1]));
            ++value_index;
        }
        // x(n-1)'^2 * (w_dx_ref + penalty_dx + w_end_dx)
        columns[2 * n - 1].emplace_back(
            2 * n - 1,
            (weight_dx_ref_[n - 1] + penalty_dx_[n - 1] + weight_end_state_[1]) /
                (scale_factor_[1] * scale_factor_[1]));
        ++value_index;

        auto delta_s_square = delta_s_ * delta_s_;
        // x(i)''^2 * (w_ddx + 2 * w_dddx / delta_s^2)
        columns[2 * n].emplace_back(2 * n,
                                    (weight_ddx_ + weight_dddx_ / delta_s_square) /
                                        (scale_factor_[2] * scale_factor_[2]));
        ++value_index;

        for (int i = 1; i < n - 1; ++i) {
            columns[2 * n + i].emplace_back(
                2 * n + i, (weight_ddx_ + 2.0 * weight_dddx_ / delta_s_square) /     //有2是因为，xi+1 - xi， xi-xi-1，平方项后有两个xi的平方
                            (scale_factor_[2] * scale_factor_[2]));
            ++value_index;
        }

        columns[3 * n - 1].emplace_back(
            3 * n - 1,
            (weight_ddx_ + weight_dddx_ / delta_s_square + weight_end_state_[2]) /
                (scale_factor_[2] * scale_factor_[2]));
        ++value_index;

        // -2 * w_dddx / delta_s^2 * x(i)'' * x(i + 1)''    //不在对角线的位置  
        for (int i = 0; i < n - 1; ++i) {
            columns[2 * n + i].emplace_back(2 * n + i + 1,
                                            -2.0 * weight_dddx_ / delta_s_square /
                                                (scale_factor_[2] * scale_factor_[2]));   //此处是下三角啊，我服了 apollo 了，apollo弄错了

            // columns[2 * n + i + 1].emplace_back(2 * n + i - 1,
            //                                 -2.0 * weight_dddx_ / delta_s_square /
            //                                     (scale_factor_[2] * scale_factor_[2]));   //此处是上三角
                    
            ++value_index;
        }

        if(value_index != kNumValue)
        {
            cout<< "kernal index fault"<<endl;
        }

        int ind_p = 0;
        for (int i = 0; i < kNumParam; ++i) {
            P_indptr->push_back(ind_p);
            for (const auto& row_data_pair : columns[i]) {
            P_data->push_back(row_data_pair.second * 2.0);    //为了数值稳定
            P_indices->push_back(row_data_pair.first);
            ++ind_p;
            }
        }
        P_indptr->push_back(ind_p);
    }

    /* eigen 转 csc，虽然方便，但不像apollo节约内存，但apollo不适用于 0.6 版本 */
    void FindStateProblem::CalculateKernel_eigen(Eigen::MatrixXd& p, c_int* &P_indptr, c_int* &P_indices, c_float * &P_data)
    {
        const int n = static_cast<int>(num_of_knots_);
        const int kNumParam = 3 * n;

        // x(i)^2 * w_x_ref    --终点不用差值
        for (int i = 0; i < n - 1; ++i) {
            p(i, i) = weight_x_ref_ / (scale_factor_[0] * scale_factor_[0]);
        }
        // x(n-1)^2 * (w_x_ref + w_end_x)   --终点惩罚，此处没用
        p(n - 1, n - 1) = (weight_x_ref_ + weight_end_state_[0]) / (scale_factor_[0] * scale_factor_[0]);

        // x(i)'^2 * (w_dx_ref + penalty_dx)
        for (int i = 0; i < n - 1; ++i) {
            p(n + i, n + i) = (weight_dx_ref_[i] + penalty_dx_[i]) / (scale_factor_[1] * scale_factor_[1]);
        }

        // x(n-1)'^2 * (w_dx_ref + penalty_dx + w_end_dx)
        p(2 * n - 1, 2 * n - 1) = (weight_dx_ref_[n - 1] + penalty_dx_[n - 1] + weight_end_state_[1]) / (scale_factor_[1] * scale_factor_[1]);

        auto delta_s_square = delta_s_ * delta_s_;
        // x(i)''^2 * (w_ddx + 2 * w_dddx / delta_s^2)
        p(2 * n, 2 * n) = (weight_ddx_ + weight_dddx_ / delta_s_square) / (scale_factor_[2] * scale_factor_[2]);

        for (int i = 1; i < n - 1; ++i) {
             p(2 * n + i, 2 * n + i) = (weight_ddx_ + 2.0 * weight_dddx_ / delta_s_square) /     //有2是因为，xi+1 - xi， xi-xi-1，平方项后有两个xi的平方
                            (scale_factor_[2] * scale_factor_[2]);

        }

        p(3 * n - 1, 3 * n - 1) = (weight_ddx_ + weight_dddx_ / delta_s_square + weight_end_state_[2]) / (scale_factor_[2] * scale_factor_[2]);


        // // -2 * w_dddx / delta_s^2 * x(i)'' * x(i + 1)''    //不在对角线的位置  
        for (int i = 0; i < n - 1; ++i) {
            p(2 * n + i + 1, 2 * n + i) = -1.0 * weight_dddx_ / delta_s_square / (scale_factor_[2] * scale_factor_[2]);
            p(2 * n + i, 2 * n + i + 1) = -1.0 * weight_dddx_ / delta_s_square / (scale_factor_[2] * scale_factor_[2]);
        }

        // cout<< p <<endl;
        SparseMatrix<double> sparseMatrix = p.sparseView();
        sparseMatrix.makeCompressed();

        double *values;
        int *rows;
        int *cols;
        // 3.保存矩阵的非0值以及行列位置
        values  = sparseMatrix.valuePtr();
        rows    = sparseMatrix.innerIndexPtr();
        cols  = sparseMatrix.outerIndexPtr();
        // 4.计算矩阵列数
        c_int matrix_nnz, matrix_col;
        matrix_col = sparseMatrix.outerSize();
        // 5.初始化csc_matrix的相关变量
        matrix_nnz = sparseMatrix.nonZeros();
        P_data = new c_float[matrix_nnz];
        P_indices = new c_int[matrix_nnz];
        P_indptr = new c_int[matrix_col + 1];

        // std::cout << matrix_col << std::endl;
        // 6.将稀疏矩阵的values赋值给csc_matrix相关变量
        for (int i = 0; i < matrix_nnz; i++) {
            P_data[i] = *(values + i);
            P_indices[i] = *(rows + i);
        // std::cout << matrix_x[i] << std::endl;
        // std::cout << matrix_i[i] << std::endl;
        }
        for (int i = 0; i <= matrix_col; i++) {
            P_indptr[i] = *(cols + i);
            // std::cout << matrix_p[i] << std::endl;
        }
    }

    void FindStateProblem::CalculateAffineConstraint(std::vector<c_float>* A_data,
                                         std::vector<c_int>* A_indices,
                                         std::vector<c_int>* A_indptr,
                                         std::vector<c_float>* lower_bounds,
                                         std::vector<c_float>* upper_bounds)
    {
        // 3N params bounds on x, x', x''
        // 3(N-1) constraints on x, x', x''
        // 3 constraints on x_init_
        const int n = static_cast<int>(num_of_knots_);
        const int num_of_variables = 3 * n;
        const int num_of_constraints = num_of_variables + 3 * (n - 1) + 3;   //变量边界约束 + picewise 等式约束 + 三个init约束  ?? 这里有问题啊，第一个init 和最后不会冲突吗
        lower_bounds->resize(num_of_constraints);
        upper_bounds->resize(num_of_constraints);

        /*和上述同理，以嵌套 vector 实现 A 的csc矩阵 */
        std::vector<std::vector<std::pair<c_int, c_float>>> variables(
            num_of_variables);

        int constraint_index = 0;
        // set x, x', x'' bounds
        for (int i = 0; i < num_of_variables; ++i) {
            if (i < n) {
            variables[i].emplace_back(constraint_index, 1.0);
            lower_bounds->at(constraint_index) =
                x_bounds_[i].first * scale_factor_[0];
            upper_bounds->at(constraint_index) =
                x_bounds_[i].second * scale_factor_[0];
            } else if (i < 2 * n) {
            variables[i].emplace_back(constraint_index, 1.0);

            lower_bounds->at(constraint_index) =
                dx_bounds_[i - n].first * scale_factor_[1];
            upper_bounds->at(constraint_index) =
                dx_bounds_[i - n].second * scale_factor_[1];
            } else {
            variables[i].emplace_back(constraint_index, 1.0);
            lower_bounds->at(constraint_index) =
                ddx_bounds_[i - 2 * n].first * scale_factor_[2];
            upper_bounds->at(constraint_index) =
                ddx_bounds_[i - 2 * n].second * scale_factor_[2];
            }
            ++constraint_index;
        }

        if(constraint_index != num_of_variables)
        {
            cout<< "variable_num fault"<<endl;
        }

        // x(i->i+1)''' = (x(i+1)'' - x(i)'') / delta_s
        for (int i = 0; i + 1 < n; ++i) {
            variables[2 * n + i].emplace_back(constraint_index, -1.0);
            variables[2 * n + i + 1].emplace_back(constraint_index, 1.0);
            lower_bounds->at(constraint_index) =
                dddx_bound_.first * delta_s_ * scale_factor_[2];
            upper_bounds->at(constraint_index) =
                dddx_bound_.second * delta_s_ * scale_factor_[2];
            ++constraint_index;
        }

        // x(i+1)' - x(i)' - 0.5 * delta_s * x(i)'' - 0.5 * delta_s * x(i+1)'' = 0
        for (int i = 0; i + 1 < n; ++i) {
            variables[n + i].emplace_back(constraint_index, -1.0 * scale_factor_[2]);
            variables[n + i + 1].emplace_back(constraint_index, 1.0 * scale_factor_[2]);
            variables[2 * n + i].emplace_back(constraint_index,
                                            -0.5 * delta_s_ * scale_factor_[1]);
            variables[2 * n + i + 1].emplace_back(constraint_index,
                                                -0.5 * delta_s_ * scale_factor_[1]);
            lower_bounds->at(constraint_index) = 0.0;
            upper_bounds->at(constraint_index) = 0.0;
            ++constraint_index;
        }

        // x(i+1) - x(i) - delta_s * x(i)'
        // - 1/3 * delta_s^2 * x(i)'' - 1/6 * delta_s^2 * x(i+1)''
        auto delta_s_sq_ = delta_s_ * delta_s_;
        for (int i = 0; i + 1 < n; ++i) {
            variables[i].emplace_back(constraint_index,
                                    -1.0 * scale_factor_[1] * scale_factor_[2]);
            variables[i + 1].emplace_back(constraint_index,
                                        1.0 * scale_factor_[1] * scale_factor_[2]);
            variables[n + i].emplace_back(
                constraint_index, -delta_s_ * scale_factor_[0] * scale_factor_[2]);
            variables[2 * n + i].emplace_back(
                constraint_index,
                -delta_s_sq_ / 3.0 * scale_factor_[0] * scale_factor_[1]);
            variables[2 * n + i + 1].emplace_back(
                constraint_index,
                -delta_s_sq_ / 6.0 * scale_factor_[0] * scale_factor_[1]);

            lower_bounds->at(constraint_index) = 0.0;
            upper_bounds->at(constraint_index) = 0.0;
            ++constraint_index;
        }

        // constrain on x_init
        variables[0].emplace_back(constraint_index, 1.0);
        lower_bounds->at(constraint_index) = x_init_[0] * scale_factor_[0];
        upper_bounds->at(constraint_index) = x_init_[0] * scale_factor_[0];
        ++constraint_index;

        variables[n].emplace_back(constraint_index, 1.0);
        lower_bounds->at(constraint_index) = x_init_[1] * scale_factor_[1];
        upper_bounds->at(constraint_index) = x_init_[1] * scale_factor_[1];
        ++constraint_index;

        variables[2 * n].emplace_back(constraint_index, 1.0);
        lower_bounds->at(constraint_index) = x_init_[2] * scale_factor_[2];
        upper_bounds->at(constraint_index) = x_init_[2] * scale_factor_[2];
        ++constraint_index;

        if(constraint_index != num_of_constraints)
        {
            cout<< " total constraint_num fault"<<endl;
        }

        int ind_p = 0;
        for (int i = 0; i < num_of_variables; ++i) {
            A_indptr->push_back(ind_p);
            for (const auto& variable_nz : variables[i]) {
            // coefficient
            A_data->push_back(variable_nz.second);

            // constraint index
            A_indices->push_back(variable_nz.first);
            ++ind_p;
            }
        }
        // We indeed need this line because of
        // https://github.com/oxfordcontrol/osqp/blob/master/src/cs.c#L255
        A_indptr->push_back(ind_p);
    }

    void FindStateProblem::CalculateOffset(std::vector<c_float>* q) 
    {
        const int n = static_cast<int>(num_of_knots_);
        const int kNumParam = 3 * n;
        q->resize(kNumParam);
        for (int i = 0; i < n; ++i) {
            if (has_x_ref_) {
            q->at(i) += -2.0 * weight_x_ref_ * x_ref_[i] / scale_factor_[0];
            }
            if (has_dx_ref_) {
            q->at(n + i) += -2.0 * weight_dx_ref_[i] * dx_ref_[i] / scale_factor_[1];
            }
        }

        if (has_end_state_ref_) {
            q->at(n - 1) +=
                -2.0 * weight_end_state_[0] * end_state_ref_[0] / scale_factor_[0];
            q->at(2 * n - 1) +=
                -2.0 * weight_end_state_[1] * end_state_ref_[1] / scale_factor_[1];
            q->at(3 * n - 1) +=
                -2.0 * weight_end_state_[2] * end_state_ref_[2] / scale_factor_[2];
        }
    }

    OSQPSettings* FindStateProblem::SolverDefaultSettings() 
    {
        // Define Solver default settings
        /*reinterpret_cast 类型转换更接近底层，不需要类型检查。c_malloc 返回 void* 指针，在C语言中可直接转换为 OSQPData*，但C++不允许*/
        OSQPSettings* settings = reinterpret_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));
        osqp_set_default_settings(settings);
        settings->eps_abs = 1e-4;
        settings->eps_rel = 1e-4;
        settings->eps_prim_inf = 1e-5;
        settings->eps_dual_inf = 1e-5;
        settings->polish = true;
        settings->verbose = 1;    //1 是看输出
        settings->scaled_termination = true;

        return settings;
    }

    void FindStateProblem::FreeData(OSQPData* data) 
    {
        delete[] data->q;
        delete[] data->l;
        delete[] data->u;

        delete[] data->P->i;
        delete[] data->P->p;
        delete[] data->P->x;

        delete[] data->A->i;
        delete[] data->A->p;
        delete[] data->A->x;
    }
}