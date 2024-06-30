#ifndef RANDOM_MAP_H
#define RANDOM_MAP_H

#include<ros/ros.h>
#include<eigen3/Eigen/Eigen>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>
#include "global_map/Obstacles.h"
#include "global_map/DynamicObstacles.h"

namespace map_{

class random_map{
    public:

        void PubMap();
        double get_rate()
        {
            return rate;
        }
        void init(ros::NodeHandle &nh);

    private:

        /*  map_info_paramter  */
        double x_size,y_size,z_size,resolution;
        int ob_num;

        double x_l,x_h,y_l,y_h,w_l,w_h,h_l,h_h;  //ob大小
        double init_x,init_y;
        bool dim_flag; //true是二维
        bool has_map;
        double rate;//地图发布频率

        /* generate */
        pcl::PointCloud<pcl::PointXYZ> random_cloud_3;
        sensor_msgs::PointCloud2 global_map;  //发布

        pcl::PointCloud<pcl::PointXYZ> random_ob;
        sensor_msgs::PointCloud2 global_map_ob;  //发布

        

        global_map::Obstacles static_obs;
        global_map::DynamicObstacles dynamic_obs;

        void RandomMapGenerate();  //生成随机ob
        void ob_polygon();
        void generate_polygon(double x_l, double y_l, double x_h, double y_h, geometry_msgs::Polygon &polygon);
        void generate_dynamic_polygon(double start_x, double start_y, double length, double width, double start_t, double ts, double tot_t, double phi);
    

        /* PubMap */
        ros::Publisher map_pub;
        ros::Publisher map_ob_pub;
        ros::Publisher static_ob_pub;
        ros::Publisher dynamic_ob_pub;
};
}

#endif