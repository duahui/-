#include "corridor/corridor.h"
#include <iostream>

namespace corridor
{
    construct_corridor::construct_corridor(ros::NodeHandle &nh)
    {
        double x_size;
        double y_size;
        nh.param("/map/x_size", x_size, 20.0);   //Vector3d 类型没有x, y, z
        nh.param("/map/y_size", y_size, 20.0);

        xl = x_size/2.0;
        yl = y_size/2.0;

        xu = x_size/2.0;
        yu = y_size/2.0;

        dynamic_sub_ = nh.subscribe("/map_generator/dynamic_obs", 1,&construct_corridor::DynamicObstaclesCallback, this);
        static_sub_ = nh.subscribe("/map_generator/static_obs", 1,&construct_corridor::ObstaclesCallback, this);
        
    }
    // contraints 前四个是 front 圆， 后四个是rear圆
    bool construct_corridor::FormulateCorridorConstraints(const std::vector<Matrix<double, 7, 1>> &states, std::vector<std::array<double, 8>> &constraints)
    {
        //x,y,phi,v,steer,a,t
        //start                
        cartesian_planner::visualization::PlotVehicle(1, {states[0](0), states[0](1), states[0](2)}, states[0](2));
        unknown_obs_states = states;
        avoid_unknown_obs();

        constraints.resize(states.size());
       
        // for (int i = 0; i < states.size(); ++i)
        // {
        //     double xf, yf, xr, yr;
        //     std::tie(xf, yf, xr, yr) = vehicle_.GetDiscPositions(states[i](0), states[i](1), states[i](2));

        //     AABox2d box1;
        //     if (!GenerateBox(states[i](6), xf, yf, vehicle_.radius, box1)) 
        //     {
        //         return false;
        //     }
            

        //     AABox2d box2;
        //     if (!GenerateBox(states[i](6), xr, yr, vehicle_.radius, box2)) 
        //     {
        //         return false;
        //     }

        //     constraints[i] = {box1.min_x(), box1.max_x(), box1.min_y(), box1.max_y(), box2.min_x(), box2.max_x(), box2.min_y(), box2.max_y()};
        //     // std::cout << box1.min_x() << "," <<box1.max_x() <<"," << box1.min_y() <<","<<box1.max_y()<< std::endl;
        //     // std::cout << box2.min_x() << "," <<box2.max_x() <<"," << box2.min_y() <<","<<box2.max_y()<< std::endl;
        // }

        for (int i = 0; i < states.size(); ++i)
        {
            double xf, yf, xr, yr;
            std::tie(xf, yf, xr, yr) = vehicle_.GetDiscPositions(unknown_obs_states[i](0), unknown_obs_states[i](1), states[i](2));

            AABox2d box1;
            if (!GenerateBox(states[i](6), xf, yf, vehicle_.radius, box1)) 
            {
                return false;
            }
            

            AABox2d box2;
            if (!GenerateBox(states[i](6), xr, yr, vehicle_.radius, box2)) 
            {
                return false;
            }

            constraints[i] = {box1.min_x(), box1.max_x(), box1.min_y(), box1.max_y(), box2.min_x(), box2.max_x(), box2.min_y(), box2.max_y()};
            // std::cout << box1.min_x() << "," <<box1.max_x() <<"," << box1.min_y() <<","<<box1.max_y()<< std::endl;
            // std::cout << box2.min_x() << "," <<box2.max_x() <<"," << box2.min_y() <<","<<box2.max_y()<< std::endl;
        }

        if(vis_boxes)
        {
            // std::cout << "success" << std::endl;
            // vis_constraints(constraints);
        }
        return true;
    }

    void construct_corridor::avoid_unknown_obs()
    {
        for (int i = 0; i < unknown_obs_states.size(); i++)
        {

            double time = unknown_obs_states[i](6);
            if (CheckCollision_unknown(time, unknown_obs_states[i](0), unknown_obs_states[i](1), unknown_obs_states[i](2))) 
            {
                
                int inc = 4;
                double real_x, real_y;
                double x = unknown_obs_states[i](0);
                double y = unknown_obs_states[i](1);
                double phi = unknown_obs_states[i](2);
                
                do 
                {
                    int iter = inc / 4;
                    uint8_t edge = inc % 4;

                    real_x = x;
                    real_y = y;
                    if (edge == 0) {
                        real_x = x - iter * 0.05;
                    } else if (edge == 1) {
                        real_x = x + iter * 0.05;
                    } else if (edge == 2) {
                        real_y = y - iter * 0.05;
                    } else if (edge == 3) {
                        real_y = y + iter * 0.05;
                    }

                    inc++;
                } while (CheckCollision_unknown(time, real_x, real_y, phi));

                unknown_obs_states[i](0) = real_x;
                unknown_obs_states[i](1) = real_y;
            }
        }
    }

    bool construct_corridor::GenerateBox(double time, double x, double y, double radius, AABox2d &result)
    {

        // initial condition not satisfied, involute to find feasible box
        double ri = radius;
        AABox2d bound({-ri, -ri}, {ri, ri});        
        if (CheckCollision(time, x, y, bound)) 
        {
            
            int inc = 4;
            double real_x, real_y;
            
            do 
            {
                int iter = inc / 4;
                uint8_t edge = inc % 4;

                real_x = x;
                real_y = y;
                if (edge == 0) {
                    real_x = x - iter * 0.05;
                } else if (edge == 1) {
                    real_x = x + iter * 0.05;
                } else if (edge == 2) {
                    real_y = y - iter * 0.05;
                } else if (edge == 3) {
                    real_y = y + iter * 0.05;
                }

                inc++;
            } while (CheckCollision(time, real_x, real_y, bound));

            x = real_x;
            y = real_y;
        }

        // generate box
        int inc = 4;
        std::bitset<4> blocked;
        double incremental[4] = {0.0};
        double step = radius * 0.2;

        do 
        {
            int iter = inc / 4;
            uint8_t edge = inc % 4;
            inc++;

            if (blocked[edge]) continue;

            incremental[edge] = iter * step;

            AABox2d test({-ri - incremental[0], -ri - incremental[2]}, {ri + incremental[1], ri + incremental[3]});  //对ego膨胀

            if (CheckCollision(time, x, y, test)) 
            {
                incremental[edge] -= step;
                blocked[edge] = true;
                // std::cout << "collision" << std::endl;
                // break;
            }
        } while (!blocked.all());  

        result = {{x - incremental[0], y - incremental[2]}, {x + incremental[1], y + incremental[3]}};
        return true;
    }

    // apollo 的hybrid A* 只涉及静态障碍物，没有动态障碍物避障
    bool construct_corridor::CheckCollision(double time, double x, double y, const AABox2d &bound)
    {
        Box2d box(bound);
        box.Shift({x, y});

        //dynamic check
        for (auto &obstacle: dynamic_obs_) 
        {
            if (obstacle.front().first > time || obstacle.back().first < time) 
            {
                continue;
            }
            auto result = std::upper_bound(obstacle.begin(), obstacle.end(), time,
                                    [](double val, const std::pair<double, Polygon2d> &ob) {
                                        return val < ob.first;
                                    });

            if (result->second.HasOverlap(box)) 
            {
                return true;
            }
        }

        //static check
        for (auto &obstacle: obstacles_) 
        {
            if (obstacle.HasOverlap(box)) 
            {
                return true;
            }
        }

        return false;
    }
    
    bool construct_corridor::CheckCollision_unknown(double time, double x, double y, double phi)
    {
        Pose vehicle_pose(x, y, phi);

        Box2d box = vehicle_.GenerateBox(vehicle_pose);
        //dynamic check
        for (auto &obstacle: dynamic_obs_) 
        {
            if (obstacle.front().first > time || obstacle.back().first < time) 
            {
                continue;
            }
            auto result = std::upper_bound(obstacle.begin(), obstacle.end(), time,
                                    [](double val, const std::pair<double, Polygon2d> &ob) {
                                        return val < ob.first;
                                    });

            if (result->second.HasOverlap(box)) 
            {
                return true;
            }
        }

        //static check
        for (auto &obstacle: obstacles_) 
        {
            if (obstacle.HasOverlap(box)) 
            {
                return true;
            }
        }

        return false;
    }

    void construct_corridor::DynamicObstaclesCallback(const global_map::DynamicObstaclesConstPtr &msg) 
    {
        if(!use_dynamic_box)
            return;
            
        if(has_dynamic_ob)
            return;
        
        dynamic_obs_.clear();
        for (auto &obstacle: msg->obstacles) 
        {
            DynamicObstacle_ dynamic_temp;

            for (auto &tp: obstacle.trajectory) 
            {
                Pose coord(tp.x, tp.y, tp.theta);
                std::vector<Vec2d> points;
                for (auto &pt: obstacle.polygon.points) 
                {
                    points.push_back(coord.transform({pt.x, pt.y, 0.0}));
                }
                // math::Polygon2d polygon(points);
                dynamic_temp.emplace_back(tp.time, points);   //polygon 由 points 自动构造
            }

            dynamic_obs_.push_back(dynamic_temp);
        }
        has_dynamic_ob = true;
        
        
        // plot first frame of dynamic obstacles
        int idx = 0;
        for (auto &obstacle: dynamic_obs_) 
        {
            ++idx;
            cartesian_planner::visualization::PlotPolygon(obstacle[0].second, 0.1, cartesian_planner::visualization::Color::Magenta, idx, "Online Obstacle");
        }

        cartesian_planner::visualization::Trigger();

    }

    void construct_corridor::ObstaclesCallback(const global_map::ObstaclesConstPtr &msg) 
    {
        if(has_static_ob)
            return;

        obstacles_.clear();
        for (auto &obstacle: msg->obstacles) 
        {
            std::vector<Vec2d> points;
            for (auto &pt: obstacle.points) 
            {
                points.emplace_back(pt.x, pt.y);
            }
            obstacles_.emplace_back(points);
        }
        has_static_ob = true;

        pub_static_obs();
    }

    void construct_corridor::pub_static_obs()
    {
        
        int idx = 0;
        for (auto &ob:obstacles_) 
        {
            cartesian_planner::visualization::PlotPolygon(ob, 0.1, cartesian_planner::visualization::Color::Black, idx++, "Obstacles");
        }
    }

    void construct_corridor::vis_constraints(const std::vector<std::array<double, 8>> &constraints)
    {
        int idx = 0;
        for (auto &box:constraints) 
        {
            for (int i = 0; i < 2; i++)
            {
                std::vector<Vec2d> points;
 
                points.emplace_back(box[0 + i * 4],box[2 + i * 4]);
                points.emplace_back(box[1 + i * 4],box[2 + i * 4]);
                points.emplace_back(box[1 + i * 4],box[3 + i * 4]);
                points.emplace_back(box[0 + i * 4],box[3 + i * 4]);

                Polygon2d polygon(points);
                cartesian_planner::visualization::PlotPolygon(polygon, 0.1, cartesian_planner::visualization::Color::Blue, idx++, "boxes");
            } 
        }
        cartesian_planner::visualization::Trigger();
    }
}