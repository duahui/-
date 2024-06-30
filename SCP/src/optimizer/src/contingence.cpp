
#include "nlp/contingence.h"
#include <cmath>
#include <iostream>
#include <cassert>

namespace nlp{
void ipopt_contingence::init_path_info(const std::vector<Eigen::Matrix<double, 6, 1>> &state_list,const std::vector<Eigen::Matrix<double, 4, 1>> &bounds, const std::vector<Eigen::Matrix<double, 4, 1>> &currents) 
{
    int a = 0;
}

/* API */
ipopt_contingence::ipopt_contingence(ros::NodeHandle &nh)
{
    nh.param("opt/max_speed_forward", max_speed_forward, 2.0);
    nh.param("opt/max_speed_reverse", max_speed_reverse, -1.0);
    nh.param("opt/max_acc_forward", max_acc_forward, 2.0);
    nh.param("opt/max_acc_reverse", max_acc_reverse, -0.5);
    nh.param("opt/min_time_scal", min_time_scal, 1.0);
    nh.param("opt/max_time_scal", max_time_scal, 10.0);    

    nh.param("opt/steer_ratio", steer_ratio, 16.0);
    nh.param("opt/max_steer_angle", max_steer_angle, 8.20304748437);
    max_steer_angle = max_steer_angle / steer_ratio;           //方向盘转角与车轮转角之比 前轮转角8.2/16=29.36度
       
    nh.param("opt/max_steer_rate", max_steer_rate, 8.55211);
    max_steer_rate = max_steer_rate / steer_ratio;

    nh.param("opt/w_steer", w_steer, 0.3);
    nh.param("opt/w_a", w_a, 1.0);    
    nh.param("opt/w_steer_rate", w_steer_rate, 2.0);
    nh.param("opt/w_a_rate", w_a_rate, 2.5);

    nh.param("opt/w_current", w_current, 2.5);
    nh.param("opt/w_t", w_t, 3.0);

    nh.param("opt/weel_base", wheel_base, 2.8448);

    f2x_ = vehicle.f2x;
    r2x_ = vehicle.r2x;
}

void ipopt_contingence::init_path_contigence(const vector<Matrix<double, 7, 1>> &state_list, const std::vector<std::array<double, 8>> &boxes)
{  
    xy_bounds.clear();
    xy_bounds = std::move(boxes);
    ts = state_list[1](6);
    horizon = int(state_list.size() -1);
    control_start_index = 4 * (horizon + 1);
    time_index = control_start_index + 2 * horizon;
    std::cout << "control_start_index : "<< control_start_index << std::endl;
    std::cout << "time_start_index : "<< time_index << std::endl;

    num_of_variables = 4 * (horizon + 1) + 2 * horizon + 1;    //状态量 + 控制量 + 时间
    num_of_constraints = 4 * horizon + horizon + 4 * (horizon + 1);                         //动力学 + 方向盘变化率 + ob_boxes 约束
    std::cout << "num_of_variables : "<< num_of_variables << std::endl;
    std::cout << "num_of_constraints : "<< num_of_constraints << std::endl;

    xWS = MatrixXd::Zero(4, horizon + 1);
    uWS = MatrixXd::Zero(2, horizon);

    for(int i = 0; i <= horizon; ++i)
    {
        //无人车
        xWS(0, i) = state_list[i](0); //x
        xWS(1, i) = state_list[i](1); //y
        xWS(2, i) = state_list[i](2); //phi
        xWS(3, i) = state_list[i](3); //v  其实是不准确的，这个 v 的方向不是和 phi 对应的
        
        cout<< "x: " << xWS(0, i) << "y: " << xWS(1, i) << " phi: " << xWS(2, i)<<"v: " << xWS(3, i)<<endl;
        if(i == horizon)
            continue;
        uWS(0, i) = state_list[i](4); //steer  无人机里没有steer的概念,只能瞎赋值了
        uWS(1, i) = state_list[i](5); //a
        cout<< "steer: " << uWS(0, i) << "a: " << uWS(1, i)  <<endl;

    }  
    
    start(0) = xWS(0, 0);
    start(1) = xWS(1, 0);
    start(2) = xWS(2, 0);
    start(3) = xWS(3, 0);

    end(0) = xWS(0, horizon);
    end(1) = xWS(1, horizon);
    end(2) = xWS(2, horizon);
    end(3) = xWS(3, horizon);

}

void ipopt_contingence::get_opt_result(MatrixXd &state_result_, MatrixXd &control_result_, double &time_result_)
{
    state_result_ = state_result;
    control_result_ = control_result;
    time_result_ = time_result;
}

bool ipopt_contingence::get_nlp_info(Index & n, Index & m, Index & nnz_jac_g, Index & nnz_h_lag, IndexStyleEnum & index_style)
{
    n = num_of_variables;
    m = num_of_constraints;

    /* ADOLC 会顺带自行计算各矩阵的非 0 个数 */
    generate_tapes(n, m, nnz_jac_g, nnz_h_lag);

    index_style = TNLP::C_STYLE;
    return true;
}

bool ipopt_contingence::get_bounds_info(Index n, Number * x_l, Number * x_u, Index m, Number * g_l, Number * g_u)
{
    int variable_index = 0;

    /* start point x,y,phi,v*/
    for(int i = 0; i < 4; ++i)
    {
        x_l[variable_index] = start(i);
        x_u[variable_index] = start(i);
        ++variable_index;
    }

    /* mid point x,y,phi,v*/
    for(int i = 0; i < horizon - 1; ++i)
    {
        // x
        x_l[variable_index] = -2e19;;
        x_u[variable_index] = 2e19;

        // y
        x_l[variable_index + 1] = -2e19;
        x_u[variable_index + 1] = 2e19;

        // phi
        x_l[variable_index + 2] = -2e19;
        x_u[variable_index + 2] = 2e19;

        // v
        x_l[variable_index + 3] = max_speed_reverse;
        x_u[variable_index + 3] = max_speed_forward;

        variable_index += 4;
    }

    /* end point x,y,phi,v 放宽一下，后面用成松弛变量*/ 
    //x  
    x_l[variable_index] = end(0) - 0.1;
    x_u[variable_index] = end(0) + 0.1;
    //y
    x_l[variable_index + 1] = end(1) - 0.1;
    x_u[variable_index + 1] = end(1) + 0.1;
    //phi
    x_l[variable_index + 2] = end(2) - 0.1;
    x_u[variable_index + 2] = end(2) + 0.1;
    //v
    x_l[variable_index + 3] = end(3) - 0.1;
    x_u[variable_index + 3] = end(3) + 0.1;
    variable_index += 4;
    // std::cout << "variable_index after adding state variables : "<< variable_index << std::endl;

    /* control variables */
    for(int i = 0; i < horizon; ++i)
    {
        //steer
        x_l[variable_index] = -max_steer_angle;
        x_u[variable_index] = max_steer_angle;
        //a
        x_l[variable_index + 1] = max_acc_reverse;
        x_u[variable_index + 1] = max_acc_forward;
        variable_index += 2;
    }
    /* 约束边界 */

    /* 时间比例 */
    x_l[variable_index] = min_time_scal;
    x_u[variable_index] = max_time_scal;
    assert(variable_index == time_index);
    
    /*动力学约束 */
    int constraint_index = 0;
    for (int i = 0; i < 4 * horizon; ++i)
    {
        g_l[i] = 0;
        g_u[i] = 0;
    }
    constraint_index += 4 * horizon;

    /* steer angle rate -- only apply steering rate as of now */  // 第一个应该和初始状态相比  
    for (int i = 0; i < horizon; ++i)
    {
        g_l[constraint_index] = -max_steer_rate;
        g_u[constraint_index] = max_steer_rate;
        ++constraint_index;
    }   

    for (int i = 0; i < (horizon + 1); ++i)  //xf, yf, xr, yr;
    {
        //xf
        g_l[constraint_index] = xy_bounds[i][0];
        g_u[constraint_index] = xy_bounds[i][1];
        ++constraint_index;
        //yf
        g_l[constraint_index] = xy_bounds[i][2];
        g_u[constraint_index] = xy_bounds[i][3];
        ++constraint_index;
        //xr
        g_l[constraint_index] = xy_bounds[i][4];
        g_u[constraint_index] = xy_bounds[i][5];
        ++constraint_index;
        //yr
        g_l[constraint_index] = xy_bounds[i][6];
        g_u[constraint_index] = xy_bounds[i][7];
        ++constraint_index;
    }

    std::cout << "constraints for all : "<< constraint_index << std::endl;
    return true;
}

bool ipopt_contingence::get_starting_point(Index n, bool init_x, Number * x, bool init_z, Number * z_L, Number * z_U, Index m, bool init_lambda, Number * lambda)
{
    
    assert(init_x == true);
    assert(init_z == false);
    assert(init_lambda == false);
    assert(n == num_of_variables);
   
    for (int i = 0; i < horizon + 1; ++i)
    {
        int index = i * 4;
        for (int j = 0; j < 4; ++j)
        {
            x[index + j] = xWS(j, i);
        }        
    }

    /*control variable */
    for (int i = 0; i < horizon; ++i)
    {
        int index = i * 2;
        x[control_start_index + index] = uWS(0, i);
        x[control_start_index + index + 1] = uWS(1, i);
    }
    
    /* 时间比例初始值*/
    x[time_index] = 1;
    /*  //输出 x
    int j = 0;
    for (int i = 0; i < n; ++i)
    {    
        if(j == 0)
        {
            cout<<"node"<<endl;
        }     
        cout<<x[i]<<endl;

       j++;
              if (j==4)
       {
           j = 0;
       }
    }
    */
    return true;
}

bool ipopt_contingence::eval_f(Index n, const Number * x, bool new_x, Number & obj_value)
{
    
    eval_obj(n, x, obj_value);

    return true;
}

bool ipopt_contingence::eval_grad_f(Index n, const Number * x, bool new_x, Number * grad_f)
{
   
    gradient(tag_f,n,x,grad_f);

    return true;
}

bool ipopt_contingence::eval_g(Index n, const Number * x, bool new_x, Index m, Number * g)
{
   
    eval_constraints(n,x,m,g);

    return true;

}

bool ipopt_contingence::eval_jac_g(Index n, const Number * x, bool new_x, Index m, Index nele_jac, Index * iRow, Index * jCol, Number * values)
{
    //ADOLC
    if (values == NULL) 
    {
        // return the structure of the jacobian
        for(Index idx=0; idx<nnz_jac; idx++)
        {
            iRow[idx] = rind_g[idx];
            jCol[idx] = cind_g[idx];
        }
    }
    else 
    {
        // return the values of the jacobian of the constraints

        sparse_jac(tag_g, m, n, 1, x, &nnz_jac, &rind_g, &cind_g, &jacval, options_g); 

        for(Index idx=0; idx<nnz_jac; idx++)
        {
            values[idx] = jacval[idx];
        }
    }

    return true;
}

bool ipopt_contingence::eval_h(Index n, const Number * x, bool new_x, Number obj_factor, Index m, const Number * lambda, bool new_lambda, Index nele_hess, Index * iRow, Index * jCol, Number * values)
{
    //ADOLC
    if (values == NULL) 
    {
        // return the structure. This is a symmetric matrix, fill the lower left
        // triangle only.

        for(Index idx=0; idx<nnz_L; idx++)
        {
            iRow[idx] = rind_L[idx];
            jCol[idx] = cind_L[idx];
        }
    }
    else 
    {
        // return the values. This is a symmetric matrix, fill the lower left
        // triangle only

        obj_lam[0] = obj_factor;
        for(Index idx = 0; idx<m ; idx++)
        obj_lam[1+idx] = lambda[idx];

        set_param_vec(tag_L, m+1, obj_lam);
        sparse_hess(tag_L, n, 1, const_cast<double*>(x), &nnz_L, &rind_L, &cind_L, &hessval, options_L);
        
        for(Index idx = 0; idx < nnz_L ; idx++)
        {
            values[idx] = hessval[idx];
        }
    }

    return true;
}

/* 获得优化的结果 */
void ipopt_contingence::finalize_solution(SolverReturn status, Index n, const Number * x, const Number * z_L, const Number * z_U, Index m, const Number * g, const Number * lambda, Number obj_value, const IpoptData * ip_data, IpoptCalculatedQuantities * ip_cq)
{
    int state_index = 0;
    int control_index = control_start_index;

    state_result = MatrixXd::Zero(4, horizon + 1);
    control_result = MatrixXd::Zero(2, horizon);
    time_result = 0.0;

    for (int i = 0; i < horizon + 1; ++i) 
    {
        state_result(0, i) = x[state_index];
        state_result(1, i) = x[state_index + 1];
        state_result(2, i) = x[state_index + 2];
        state_result(3, i) = x[state_index + 3];

        cout<< "x: " << x[state_index] << "y: " << x[state_index + 1] << " phi: " << x[state_index + 2] <<"v: " << x[state_index + 3]<<endl;
        if (i == horizon)
            continue;
        control_result(0, i) = x[control_index];
        control_result(1, i) = x[control_index + 1];
        cout<< "steer: " << x[control_index] << "a: " << x[control_index + 1]  <<endl;

        state_index += 4;
        control_index += 2;
    }
    time_result = x[time_index] * ts;
    cout<< "time: " << time_result << "time_scal: " <<x[time_index]<<endl;
    
    // cout<< "max_steer_rate  " << max_steer_rate <<endl;
    // for (int i = 0; i < m; i++)
    // {
    //     if(i == m - horizon)
    //     {
    //         cout<<"steer_rate"<<endl;
    //     }
    //     cout<< g[i]<<endl;
    // }
    

    /**********************通用******************************** */
    // memory deallocation of ADOL-C variables
    delete[] obj_lam;
    free(rind_g);
    free(cind_g);
    free(rind_L);
    free(cind_L);
    free(jacval);
    free(hessval);
    /**********************通用******************************** */
}

template <typename T> 
void  ipopt_contingence::eval_obj(Index n, const T *x, T& obj_value)
{
    int control_index = control_start_index;
    int state_index = 0;

    //最小化输入的平方
    for (int i = 0; i < horizon; ++i)
    {
        obj_value += w_steer * x[control_index] * x[control_index] + w_a * x[control_index + 1] * x[control_index + 1];
        control_index += 2;
    }
    
    //最小化控制变化率
    control_index = control_start_index;   
    for (int i = 0; i < horizon; ++i)
    {
        if(i == 0)
        {
            //初始状态
            T steering_rate = (x[control_index] - uWS(0, 0)) / x[time_index] / ts;
            T a_rate = (x[control_index + 1] - uWS(1, 0)) / x[time_index] / ts;
            obj_value += w_steer_rate * steering_rate * steering_rate +  w_a_rate * a_rate * a_rate;
            continue;
        }
        T steering_rate = (x[control_index + 2] - x[control_index]) / x[time_index] / ts;
        T a_rate = (x[control_index + 3] - x[control_index + 1]) / x[time_index] / ts;
        obj_value += w_steer_rate * steering_rate * steering_rate +  w_a_rate * a_rate * a_rate;
        control_index += 2;
    }

    //最小化时间
    T time_pen = w_t * horizon * x[time_index] * ts;
    obj_value += time_pen;

}

template <class T>    
void ipopt_contingence::eval_constraints(Index n, const T *x, Index m, T *g)
{
    int control_index = control_start_index;
    int state_index = 0;
    int constraint_index = 0;

    
    for (int i = 0; i < horizon; ++i)
    {
        //x
        g[constraint_index] =
        x[state_index + 4] -
        (x[state_index] +
         ts * x[time_index] *
             (x[state_index + 3] +
              ts * x[time_index] * 0.5 * x[control_index + 1]) *
             cos(x[state_index + 2] + ts * x[time_index] * 0.5 *
                                          x[state_index + 3] *
                                          tan(x[control_index]) / wheel_base));

        // y
        g[constraint_index + 1] =
                x[state_index + 5] -
                (x[state_index + 1] +
                ts * x[time_index] *
                    (x[state_index + 3] +
                    ts * x[time_index] * 0.5 * x[control_index + 1]) *
                    sin(x[state_index + 2] + ts * x[time_index] * 0.5 *
                                                x[state_index + 3] *
                                                tan(x[control_index]) / wheel_base));
        // phi
        g[constraint_index + 2] =
        x[state_index + 6] -
        (x[state_index + 2] +
         ts * x[time_index] *
             (x[state_index + 3] +
              ts * x[time_index] * 0.5 * x[control_index + 1]) *
             tan(x[control_index]) / wheel_base);
             
        // v
        g[constraint_index + 3] =
        x[state_index + 7] -
        (x[state_index + 3] + ts * x[time_index] * x[control_index + 1]);

        control_index += 2;
        constraint_index += 4;
        state_index += 4;
    }

    //方向盘变化率约束
    control_index = control_start_index;    
    for (int i = 0; i < horizon; ++i) 
    { 
         if(i == 0)
        {
            //初始状态
            g[constraint_index] = (x[control_index] - uWS(0, 0)) /x[time_index] / ts;
            constraint_index++;
            control_index += 2;
            continue;
        }
        g[constraint_index] = (x[control_index] - x[control_index - 2]) /x[time_index] / ts;
        constraint_index++;
        control_index += 2;
    }

    //boxes 约束
    for (int i = 0; i < (horizon + 1); ++i)  //xf, yf, xr, yr;
    {
        // T xf, yf, xr, yr;
        // std::tie(xf, yf, xr, yr) = vehicle.GetDiscPositions(x[ i * 4 ], x[i * 4  + 1], x[i * 4  + 2]);
        // //xf
        // g[constraint_index] = xf;
        // ++constraint_index;
        // //yf
        // g[constraint_index] = yf;
        // ++constraint_index;
        // //xr
        // g[constraint_index] = xr;
        // ++constraint_index;
        // //yr
        // g[constraint_index] = yr;
        // ++constraint_index;               

        //xf
        g[constraint_index] = x[ i * 4 ] + f2x_ * cos(x[i * 4  + 2]);
        ++constraint_index;
        //yf
        g[constraint_index] = x[i * 4  + 1] + f2x_ * sin(x[i * 4  + 2]);
        ++constraint_index;
        //xr
        g[constraint_index] = x[ i * 4 ] + r2x_ * cos(x[i * 4  + 2]);
        ++constraint_index;
        //yr
        g[constraint_index] = x[i * 4  + 1] + r2x_ * sin(x[i * 4  + 2]);
        ++constraint_index;
    }
}


void ipopt_contingence::generate_tapes(Index n, Index m, Index& nnz_jac_g, Index& nnz_h_lag)
{
    
    Number *xp    = new double[n];
    Number *lamp  = new double[m];
    Number *zl    = new double[m];
    Number *zu    = new double[m];

    // ADOLC 变量 需要上述自变量转
    adouble *xa   = new adouble[n];
    adouble *g    = new adouble[m];
    adouble obj_value;

    //为了把 lamp 转成 double  obj_value 转成 double
    double dummy = 0.0;
    double *lam   = new double[m];   
    
    double sig;

    obj_lam   = new double[m+1];

    get_starting_point(n, 1, xp, 0, zl, zu, m, 0, lamp);

    //gradient object
    trace_on(tag_f);
    
    for(Index idx=0;idx<n;idx++)
      xa[idx] <<= xp[idx];

    eval_obj(n,xa,obj_value);

    obj_value >>= dummy;

    trace_off();

    //jacbian constraints
    trace_on(tag_g);
    
    for(Index idx=0;idx<n;idx++)
      xa[idx] <<= xp[idx];

    eval_constraints(n,xa,m,g);

    for(Index idx=0;idx<m;idx++)
      g[idx] >>= dummy;

    trace_off();

    // hessian object and constraints
    trace_on(tag_L);
    
    for(Index idx=0;idx<n;idx++)
      xa[idx] <<= xp[idx];
    for(Index idx=0;idx<m;idx++)
      lam[idx] = 1.0;
    sig = 1.0;

    eval_obj(n,xa,obj_value);

    obj_value *= mkparam(sig);
    eval_constraints(n,xa,m,g);
 
    for(Index idx=0;idx<m;idx++)
      obj_value += g[idx]*mkparam(lam[idx]);

    obj_value >>= dummy;

    trace_off();

    rind_g = NULL; 
    cind_g = NULL;
    rind_L = NULL;
    cind_L = NULL;

    options_g[0] = 0;          /* sparsity pattern by index domains (default) */ 
    options_g[1] = 0;          /*                         safe mode (default) */ 
    options_g[2] = 0;
    options_g[3] = 0;          /*                column compression (default) */ 
    
    jacval=NULL;
    hessval=NULL;
    sparse_jac(tag_g, m, n, 0, xp, &nnz_jac, &rind_g, &cind_g, &jacval, options_g); 

    nnz_jac_g = nnz_jac;

    options_L[0] = 0;         
    options_L[1] = 1;        

    sparse_hess(tag_L, n, 0, xp, &nnz_L, &rind_L, &cind_L, &hessval, options_L);
    nnz_h_lag = nnz_L;

    delete[] lam;
    delete[] g;
    delete[] xa;
    delete[] zu;
    delete[] zl;
    delete[] lamp;
    delete[] xp;
}

}