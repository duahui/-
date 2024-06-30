#include "manager/opt_manager.h"
#include "visualization/plot.h"

namespace optimize
{
    opt_manager::opt_manager(ros::NodeHandle & nh)
    {
        nh.param("/nav/use_snap_closed", use_snap_closed, true);
        nh.param("/nav/use_scp_nlp", use_scp_nlp, true);

        // min_snap.reset(new snap_optimizer::snap(nh));

        // scp_nlp.reset(new SCP::scp(nh));

        speed_plan.reset(new qp::FindStateProblem(nh));

        generate_corridor.reset(new corridor::construct_corridor(nh));

        contingence_opt.reset(new nlp_manager::nlp_solver(nh));

        cartesian_planner::visualization::Init(nh, "world", "/nav/vis");
    }

    void opt_manager::OptiPathUAV(const vector<Matrix<double, 10, 1>> &state_list)
    {        
        
        min_snap->SnapOptimize(state_list);
            
    }

    void opt_manager::OptiPathCAR(const vector<Matrix<double, 6, 1>> &state_list)
    {
        bool flag = speed_plan->optimize(state_list);
        if(flag)
        {
            // picewise jerk speed
            vector<Matrix<double, 7, 1>> new_state = speed_plan->get_result(); //x,y,phi,v,steer,a,t

            // corridor constraints  xf, yf, xr, yr;
            std::vector<std::array<double, 8>> boxes;
            bool flag_box = generate_corridor->FormulateCorridorConstraints(new_state, boxes);
            if(flag_box)
            {
                cout<<"corridor_generated"<<endl;
                new_state = generate_corridor->get_unknown_obs_states();
                vis_path(new_state, "unknown_obs_path", cartesian_planner::visualization::Color::Grey);

                bool flag_opt = contingence_opt->Optimize("contingence", new_state, boxes);
                if (flag_opt)
                {
                    vector<Matrix<double, 7, 1>> new_opt_state = contingence_opt->get_opt_result();
                    vis_path(new_opt_state, "optimize_path", cartesian_planner::visualization::Color::Red);
                    vis_traj(new_opt_state);
                }
                // vector<Matrix<double, 7, 1>> new_opt_state = generate_corridor->get_unknown_obs_states();
                // vis_traj(new_opt_state);
            }

            // contingence opt
        }        
        // scp_nlp->OptimizeScp(state_list);
    }

    void opt_manager::vis_traj(const vector<Matrix<double, 7, 1>> &new_state)
    {
        std::vector<corridor::DynamicObstacle_> dynamic_obs = generate_corridor->get_dynamic_obs();
        for (int i = 0; i < new_state.size(); i++) 
        {
            double time = new_state[i](6);

            auto dynamic_obstacles = find_dynamic_obs(time, dynamic_obs);
            for (auto &obstacle: dynamic_obstacles) 
            {                  
                cartesian_planner::visualization::PlotPolygon(obstacle.second, 0.2, cartesian_planner::visualization::Color::Magenta, obstacle.first,
                                    "Online Obstacle");
                
            }
            cartesian_planner::visualization::PlotVehicle(1, {new_state[i](0), new_state[i](1), new_state[i](2)}, new_state[i](4));

            ros::Duration(new_state[1](6)).sleep();
        }
        cartesian_planner::visualization::Trigger();
    }
    
    void opt_manager::vis_path(const vector<Matrix<double, 7, 1>> &new_state, string name, cartesian_planner::visualization::Color color)
    {
        std::vector<double> x, y;
        for (auto &point: new_state) 
        {
            x.push_back(point(0));
            y.push_back(point(1));
        }

        cartesian_planner::visualization::Plot(x, y, 0.1, color, 1, name);
        cartesian_planner::visualization::Trigger();
    }

    std::unordered_map<int, cartesian_planner::math::Polygon2d> opt_manager::find_dynamic_obs(double time, std::vector<corridor::DynamicObstacle_> &dynamic_obs)
    {
        std::unordered_map<int, cartesian_planner::math::Polygon2d> filtered;
        int idx = 0;
        for (auto &obstacle: dynamic_obs) 
        {
            idx++;
            if (obstacle.front().first > time || obstacle.back().first < time) 
            {
                continue;
            }
            auto result = std::upper_bound(obstacle.begin(), obstacle.end(), time,
                                        [](double val, const std::pair<double, cartesian_planner::math::Polygon2d> &ob) {
                                            return val < ob.first;
                                        });

            filtered.insert({idx, result->second});
        }

        return filtered;
    } 

}