#pragma once

#include "nlp/nlp_scp.h"
#include <unordered_map>
#include <geometry_msgs/PoseArray.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp> 
#include <eigen3/Eigen/Eigen>
#include <vector>
#include <visualization_msgs/Marker.h>
#define HAVE_CSTDDEF
#include <coin/IpIpoptApplication.hpp>
#include <coin/IpSolveStatistics.hpp>
#undef HAVE_CSTDDEF


namespace SCP
{

using namespace Eigen;
using namespace std;

struct current
{
    double x;
    double y;
    double c_x;
    double c_y;
};


template <typename T>
struct matrix_hash : std::unary_function<T, size_t> 
{
  std::size_t operator()(T const& matrix) const
   {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

class scp
{
    public:

        scp(ros::NodeHandle &nh);

        // void init(ros::NodeHandle &nh);

        void OptimizeScp(const vector<Matrix<double, 6, 1>> &state_list);

    private:

        /* currents */
        unordered_map<Matrix<double, 2 ,1>, current, matrix_hash<Matrix<double, 2 ,1>>> current_info;
        pcl::PointCloud<pcl::PointXYZ> cloudMap;
        pcl::search::KdTree<pcl::PointXYZ> kdtreeMap;
        bool has_current = false;
        ros::Subscriber current_sub;
        void currentCallback(const geometry_msgs::PoseArray::ConstPtr &msg);

        /* SCP param*/
        int max_Seg;        
        nlp::ipopt_interface_entrance *ptop;                
        Ipopt::SmartPtr<Ipopt::IpoptApplication> app;

        vector<Matrix<double, 6, 1>> state_all;
        vector<Matrix<double, 6, 1>> traj_list;

        double x_size;
        double y_size;

        /* SCP Helper */
        
        bool stop();

        void get_state();
        void get_current(vector<Matrix<double, 4, 1>> &currents);
        void get_bounds(vector<Matrix<double, 4, 1>> &bounds);

        /* pub_tra */
        double vis_traj_width;
        void pub_traj();
        ros::Publisher scp_nlp_vis;

};
}
