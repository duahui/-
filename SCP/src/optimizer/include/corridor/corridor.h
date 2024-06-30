#pragma once

#include "math/aabox2d.h"
#include "math/box2d.h"
#include "math/line_segment2d.h"
#include "math/polygon2d.h"
#include "vehicle_param.h"
#include "global_map/DynamicObstacles.h"
#include "global_map/Obstacles.h"
#include "visualization/plot.h"

#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <bitset>
namespace corridor
{
    using namespace cartesian_planner::math;
    using namespace Eigen;
    
    using DynamicObstacle_ = std::vector<std::pair<double, Polygon2d>>;

    class construct_corridor
    {
        public:
            construct_corridor(ros::NodeHandle &nh);
            
            bool FormulateCorridorConstraints(const std::vector<Matrix<double, 7, 1>> &states,  std::vector<std::array<double, 8>> &constraints);

            std::vector<Matrix<double, 7, 1>> get_unknown_obs_states()
            {
                return unknown_obs_states;
            }

            std::vector<DynamicObstacle_> get_dynamic_obs()
            {
                return dynamic_obs_;
            }
        private:
            bool use_dynamic_box = true;
            bool vis_boxes = true;

            double xl,xu,yl,yu;
            VehicleParam_corridor vehicle_;
            bool GenerateBox(double time, double x, double y, double radius, AABox2d &result);

            
            std::vector<DynamicObstacle_> dynamic_obs_;
            ros::Subscriber dynamic_sub_;
            void DynamicObstaclesCallback(const global_map::DynamicObstaclesConstPtr &msg); 
            bool has_dynamic_ob = false;

            std::vector<Polygon2d> obstacles_;
            ros::Subscriber static_sub_;
            void ObstaclesCallback(const global_map::ObstaclesConstPtr &msg);
            bool has_static_ob = false;


            bool CheckCollision(double time, double x, double y, const AABox2d &bound);   

            void pub_static_obs(); 
            void vis_constraints(const std::vector<std::array<double, 8>> &constraints);    

            /* 没完成 */
            std::vector<Matrix<double, 7, 1>> unknown_obs_states;
            void avoid_unknown_obs();
            bool CheckCollision_unknown(double time, double x, double y, double phi);
    };
}