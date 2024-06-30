#pragma once

#include<geometry_msgs/PoseArray.h>
#include<ros/ros.h>
#include<vector>
#include<math.h>
#include<eigen3/Eigen/Eigen>
#include<tf2/LinearMath/Quaternion.h>


namespace map_
{
    using namespace std;
    using namespace Eigen;

    class current_map
    {
        public:
            void init(ros::NodeHandle &nh);

            inline void pub_current();
            

        private:
            /* current param */
            int vortexes_num;
            bool current_flag;   //true 是二维
            vector<Vector4d> vortexes;   //x, y, z, radius
            bool has_current = false;
            double resolution, x_size, y_size, z_size;

            /* pub */
            ros::Publisher current_pub;
            ros::Publisher current_pub_vis;            
            geometry_msgs::PoseArray current_info;
            geometry_msgs::PoseArray current_info_vis;

            /* generate current */

            void GenCurrentPoint();

            void lambVortexFun(double coordx, double coordy, double coordz = 0.0);

            inline double GetRad(double coordx, double coordy);

            
    };
    
    inline double current_map::GetRad(double coordx, double coordy)
    {
        Vector2d current_(coordx, coordy);
        Vector2d axis(1.0, 0.0);
        double V_dot = current_.dot(axis);
        double V_n = current_.norm();
        double cosTheta = V_dot/V_n;
        double rad = acos(cosTheta);
        if(coordy < 0)
        {
            rad=2*M_PI-rad;
        }
    }

    inline void current_map::pub_current()
    {
        if (has_current)
        {
            current_pub.publish(current_info);
            current_pub_vis.publish(current_info_vis);
            return;
        }
        else
        {
            GenCurrentPoint();
            current_info_vis.header.frame_id = "world";
            current_info_vis.header.stamp = ros::Time(0);            

            current_info.header.frame_id = "world";
            current_info.header.stamp = ros::Time(0); 

            
            has_current = true;
        }               
    }
}
