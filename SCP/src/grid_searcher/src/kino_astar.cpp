#include"grid_searcher/kino_astar.h"
#include<cmath>
#include <iostream>
namespace kino_astar{

    void kino_astar_finder::init(ros::NodeHandle &nh)
    {
        nh.param("resoultion",resolution,0.2);        
        nh.param("/map/x_size",map_size(0),20.0);   //Vector3d 类型没有x, y, z
        nh.param("/map/y_size",map_size(1),20.0);
        nh.param("/map/z_size",map_size(2),10.0);
        nh.param("/map/dim_flag",dim_flag,false);   //目前也只有二维的搜索

        /* 在这里加上search对了，将功能参数和管理类分开啦 */
        nh.param("search/max_tau", max_tau, 0.6);  
        nh.param("search/init_max_tau", init_max_tau, 0.8);
        nh.param("search/max_vel", max_vel, 3.0);
        nh.param("search/max_acc", max_acc, 2.0);
        nh.param("search/lambda_heu", lambda_heu, 5.0);
        nh.param("search/lambda_heu", w_time, 10.0);   
        nh.param("search/check_num", check_num, 5);     

        gl_xl = -map_size(0)/2.0;
        gl_yl = -map_size(1)/2.0;
        gl_zl = 0.0;

        gl_xu = map_size(0)/2.0;
        gl_yu = map_size(1)/2.0;
        gl_zu = map_size(2);

        inv_resolution = 1/resolution;
        
        tie_breaker = 1.0 + 1.0/10000;

        ph_i = MatrixXd::Identity(6, 6);

        ob_nodes.resize(static_cast<int>(map_size(0)*inv_resolution),static_cast<int>(map_size(1)*inv_resolution),static_cast<int>(map_size(2)*inv_resolution));
        ob_nodes.setConstant(false);

    }    

    bool kino_astar_finder::search(Vector3d start_pt_, Vector3d start_vel_, Vector3d start_acc_, Vector3d end_pt_, Vector3d end_vel_, bool init_search)
    {
        reset();
        start_node->index = posToIndex(start_pt_);
        start_node->state.head(3) = start_pt_;      //状态转换是用实际位置，不能用索引呀
        start_node->state.tail(3) = start_vel_;
        start_node->g_score = 0.0;

        end_node->index = posToIndex(end_pt_);
        end_node->state.head(3) = end_pt_;      //状态转换是用实际位置，不能用索引呀
        end_node->state.tail(3) = end_vel_;

        double optimal_time;
        start_node->f_score = lambda_heu * estimateHeuristic(start_node->state, end_node->state, optimal_time);
        start_node->node_state = IN_OPEN_SET;
        open_set.push(start_node);
        expanded_nodes.insert(start_node->index, start_node);

        vector<Vector3d> inputs;
        vector<double> durations;
        GetInputs(inputs, durations, start_acc_, init_search);

        PathNodePtr cur_node = NULL;
        int num = 0;      
        while(!open_set.empty())
        {          
            ++num;
            cur_node = open_set.top();
            open_set.pop();
            
            // if (cur_node->index == end_node->index && fabs(cur_node->state(3)-end_node->state(3))< 0.05 && fabs(cur_node->state(4)-end_node->state(4))< 0.05)
            if (cur_node->index == end_node->index)
            {
                // cout<< cur_node->index << "end:  " << end_node->index << endl;
                end_node = cur_node;
                ROS_INFO("iter_num: %d, expanded_nodes_num: %d", num, expanded_nodes.size());
                return true; 
            }

            cur_node->node_state = IN_CLOSED_SET;
            expanded_nodes.insert(cur_node->index, cur_node);

            vector<PathNodePtr> tmp_expanded_nodes;            

            for (int i = 0; i < inputs.size(); ++i)
                for (int j = 0; j < durations.size(); ++j)
                {
                    Matrix<double, 6, 1> pro_state;
                    Vector3d um = inputs[i];
                    double tau = durations[j];
                    stateTransit(cur_node->state, pro_state, um, tau);

                    Vector3d tmp_pt = pro_state.head(3);
                    Vector3i pro_idx = posToIndex(tmp_pt);

                    PathNodePtr pro_node = expanded_nodes.find(pro_idx);                   

                    //已扩展
                    if(pro_node != NULL && pro_node->node_state == IN_CLOSED_SET)
                        continue;

                    //超出速度限制
                    if(fabs(pro_state(3)) > max_vel || fabs(pro_state(4)) > max_vel || fabs(pro_state(5)) > max_vel)   
                        continue;

                    //超出地图
                    if(fabs(pro_state(0)) >= gl_xu || fabs(pro_state(1)) >= gl_yu || fabs(pro_state(2)) >= gl_zu || pro_state(2)<0)   
                        continue;

                    //在同一栅格内
                    if((cur_node->index - pro_idx).norm() == 0)
                        continue;    

                    // 由于扩展时可能会跨栅格，所以需要离散碰撞检测
                    bool is_col = check_col(cur_node->state, um, tau);
                    if(is_col)
                        continue;
                    
                    double time_to_goal;
                    double tmp_g_score = (um.squaredNorm() + w_time) * tau + cur_node->g_score;
                    double tmp_f_score = tmp_g_score + lambda_heu * estimateHeuristic(pro_state, end_node->state, time_to_goal);

                    
                    bool prune = false;     
                    for (int k = 0; k < tmp_expanded_nodes.size(); ++k)
                    {
                        PathNodePtr expanded_node = tmp_expanded_nodes[k];
                        if((pro_idx - expanded_node->index).norm() == 0)
                        {
                            // 需要prune
                            prune = true;
                            if(tmp_f_score < expanded_node->f_score)
                            {
                                expanded_node->f_score = tmp_f_score;
                                expanded_node->g_score = tmp_g_score;
                                expanded_node->state = pro_state;
                                expanded_node->input = um;
                                expanded_node->duration = tau;
                            }
                        }
                        break;   
                    }
                    
                    /*  */
                    if(!prune)   
                    {
                        if(pro_node == NULL)   
                        {      
                            pro_node = make_shared<PathNode>(pro_idx);                      
                            pro_node->f_score = tmp_f_score;
                            pro_node->g_score = tmp_g_score;
                            pro_node->state = pro_state;
                            pro_node->input = um;
                            pro_node->duration = tau;
                            pro_node->parent = cur_node;
                            pro_node->node_state = IN_OPEN_SET;

                            open_set.push(pro_node);
                            expanded_nodes.insert(pro_idx, pro_node);
                            tmp_expanded_nodes.push_back(pro_node);
                        }
                        else if (pro_node -> node_state == IN_OPEN_SET)   //需要更新节点以保证open_set只有一个最优节点
                        {
                            if(tmp_g_score < pro_node->g_score)
                            {
                                pro_node->f_score = tmp_f_score;
                                pro_node->g_score = tmp_g_score;
                                pro_node->state = pro_state;
                                pro_node->input = um;
                                pro_node->duration = tau;
                                pro_node->parent = cur_node;
                            }
                        }
                        else                    //错误
                        {
                            ROS_INFO("error");
                        }
                        
                    }                      

                }
            
            tmp_expanded_nodes.clear();
        }

        ROS_INFO("fail to search");
        ROS_INFO("iter_num: %d, expanded_nodes_num: %d", num, expanded_nodes.size());
        return false;
    }   

    /* 三维空间搜索真的慢，难以想象三维加时间该多慢 */
    void kino_astar_finder::GetInputs(vector<Vector3d> &inputs, vector<double> &durations, Vector3d start_acc_, bool init_search)
    {
        /* 通过离散输入 进行周围节点的遍历 通过输入幅度或时间幅值来确定节点扩展的幅度及个数 自然越精细越好*/
        /* 由于扩展时可能会跨栅格，所以需要离散碰撞检测,检测个数与栅格跨度有关，应确保所有跨越栅格均被检测到 */
        double res = 1/2.0, time_res = 1/1.0, time_res_init = 1/20.0;
        Matrix<double, 6, 1> pro_state;
        vector<PathNodePtr> tmp_expand_nodes;
        Vector3d um;

        /* init_search  true离散u  false是离散时间  */
        if(init_search)      //时间域相同            若初始加速度为0,则离散得到一系列加速度在相同时间域下进行扩展。
        {                    //t = 0.6  acc:-2 -1 0 1 2
            for (double ax = -max_acc; ax <= max_acc + 1e-3; ++ax)
                for (double ay = -max_acc; ay <= max_acc + 1e-3; ++ay)
                {
                    if (dim_flag)   //二维下 z输入为0
                    {
                        um << ax, ay, 0.0;
                        inputs.push_back(um); 
                        continue;                                   
                    }  
                    for (double az = -max_acc; az <= max_acc + 1e-3; ++az)
                    {
                        um << ax, ay, az;
                        inputs.push_back(um);
  
                    }
                }                    
            
            for (double tau = time_res * max_tau; tau <= max_tau; tau += time_res * max_tau)
                durations.push_back(tau);
            
            init_search = false;
        }
        else              //所有都是相同的加速度       若有初始加速度则，所有都按这个加速度在不同时间域下进行扩展。
        {                 //t = 0.04 +=0.04 -o.8 acc:start_acc
            inputs.push_back(start_acc_);
            for (double tau = time_res_init * init_max_tau; tau <= init_max_tau + 1e-3;
                tau += time_res_init * init_max_tau)
                durations.push_back(tau);                    
        }
    }

    bool kino_astar_finder::check_col(const Eigen::Matrix<double, 6, 1> &state0, const Eigen::Vector3d &um, double tot_tau)
    {
        Matrix<double, 6, 1> tmp_state;
        for (int i =1; i<= check_num; ++i)
        {
            double dt = tot_tau * static_cast<double>(i)/static_cast<double>(check_num);
            stateTransit(state0, tmp_state, um, dt);
            Vector3d tmp_pt = tmp_state.head(3);
            Vector3i tmp_idx = posToIndex(tmp_pt);
            if(ob_nodes(tmp_idx(0), tmp_idx(1), tmp_idx(2)) == true)
                return true;
        }
        return false;
    }

    double kino_astar_finder::estimateHeuristic(const Matrix<double, 6, 1> x1, const Matrix<double, 6, 1> x2, double& optimal_time)
    {
        const Vector3d dp = x2.head(3) - x1.head(3);
        const Vector3d v0 = x1.segment(3, 3);
        const Vector3d v1 = x2.segment(3, 3);

        double c1 = -36 * dp.dot(dp);
        double c2 = 24 * (v0 + v1).dot(dp);
        double c3 = -4 * (v0.dot(v0) + v0.dot(v1) + v1.dot(v1));
        double c4 = 0;
        double c5 = w_time;   //10   heu_cost也有时间因素

        std::vector<double> ts = quartic(c5, c4, c3, c2, c1);

        double v_max = max_vel * 0.5;
        double t_bar = (x1.head(3) - x2.head(3)).lpNorm<Infinity>() / v_max;
        ts.push_back(t_bar);

        double cost = 100000000;
        double t_d = t_bar;

        for (auto t : ts)
        {
            if (t < t_bar)
            continue;
            double c = -c1 / (3 * t * t * t) - c2 / (2 * t * t) - c3 / t + w_time * t;
            if (c < cost)
            {
            cost = c;
            t_d = t;
            }
        }

        optimal_time = t_d;

        return 1.0 * (1 + tie_breaker) * cost;
    }

    vector<double> kino_astar_finder::cubic(double a, double b, double c, double d)
    {
        vector<double> dts;

        double a2 = b / a;
        double a1 = c / a;
        double a0 = d / a;

        double Q = (3 * a1 - a2 * a2) / 9;
        double R = (9 * a1 * a2 - 27 * a0 - 2 * a2 * a2 * a2) / 54;
        double D = Q * Q * Q + R * R;
        if (D > 0)
        {
            double S = std::cbrt(R + sqrt(D));
            double T = std::cbrt(R - sqrt(D));
            dts.push_back(-a2 / 3 + (S + T));
            return dts;
        }
        else if (D == 0)
        {
            double S = std::cbrt(R);
            dts.push_back(-a2 / 3 + S + S);
            dts.push_back(-a2 / 3 - S);
            return dts;
        }
        else
        {
            double theta = acos(R / sqrt(-Q * Q * Q));
            dts.push_back(2 * sqrt(-Q) * cos(theta / 3) - a2 / 3);
            dts.push_back(2 * sqrt(-Q) * cos((theta + 2 * M_PI) / 3) - a2 / 3);
            dts.push_back(2 * sqrt(-Q) * cos((theta + 4 * M_PI) / 3) - a2 / 3);
            return dts;
        }
    }
 
    vector<double> kino_astar_finder::quartic(double a, double b, double c, double d, double e)
    {
        vector<double> dts;

        double a3 = b / a;
        double a2 = c / a;
        double a1 = d / a;
        double a0 = e / a;

        vector<double> ys = cubic(1, -a2, a1 * a3 - 4 * a0, 4 * a2 * a0 - a1 * a1 - a3 * a3 * a0);
        double y1 = ys.front();
        double r = a3 * a3 / 4 - a2 + y1;
        if (r < 0)
            return dts;

        double R = sqrt(r);
        double D, E;
        if (R != 0)
        {
            D = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 + 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
            E = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 - 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
        }
        else
        {
            D = sqrt(0.75 * a3 * a3 - 2 * a2 + 2 * sqrt(y1 * y1 - 4 * a0));
            E = sqrt(0.75 * a3 * a3 - 2 * a2 - 2 * sqrt(y1 * y1 - 4 * a0));
        }

        if (!std::isnan(D))
        {
            dts.push_back(-a3 / 4 + R / 2 + D / 2);
            dts.push_back(-a3 / 4 + R / 2 - D / 2);
        }
        if (!std::isnan(E))
        {
            dts.push_back(-a3 / 4 - R / 2 + E / 2);
            dts.push_back(-a3 / 4 - R / 2 - E / 2);
        }

        return dts;
    }

    vector<Vector3d> kino_astar_finder::GetKinoTraj(double delta_t)
    {
        vector<Vector3d> state_list;

        PathNodePtr node = end_node;

        while (node->parent != NULL)
        {
            Eigen::Matrix<double, 6, 1> xt, x0;
            Vector3d ut = node->input;
            double duration = node->duration;
            x0 = node->parent->state;

            for (double t = duration; t >= -1e-5; t -= delta_t)
            {
                stateTransit(x0, xt, ut, t);
                state_list.push_back(xt.head(3));
            }
            node = node->parent;
        }

        reverse(state_list.begin(), state_list.end());

        return std::move(state_list);
    }  

    vector<Matrix<double, 10, 1>> kino_astar_finder::GetSamples(double delta_t)
    {
        vector<Matrix<double, 10, 1>> state_list;

        PathNodePtr node = end_node;

        //vis initial
        // ROS_INFO("initial");
        // while (node->parent != NULL)
        // {            
        //     ROS_INFO("position x: %f, y: /%f, t: %f", node->state(0), node->state(0), node->duration);
        //     node = node->parent;
        // }

        // node = end_node;
        // ROS_INFO("sample");
        while (node->parent != NULL)
        {
            Eigen::Matrix<double, 6, 1> xt, x0;
            Vector3d ut = node->input;
            double duration = node->duration;
            x0 = node->parent->state;

            for (double t = duration; t > 0; t -= delta_t)
            {
                stateTransit(x0, xt, ut, t);

                Matrix<double, 10, 1> state;
                state.head(6) = xt;
                state.block(6, 0, 3, 1) = ut;
                state_list.push_back(state);
            }
            node = node->parent;
        }

        Matrix<double, 10, 1> state;
        state.head(6) = start_node->state;
        state.block(6, 0, 3, 1) = start_node->input;
        state_list.push_back(state);
        
        reverse(state_list.begin(), state_list.end());

        for (int i = 0; i < state_list.size(); ++i)
        {
            state_list[i](9) = i * delta_t;
        }

        return std::move(state_list);
    }

}