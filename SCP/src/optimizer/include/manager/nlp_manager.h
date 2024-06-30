

#pragma once

#include <geometry_msgs/PoseArray.h>
#include <eigen3/Eigen/Eigen>
#include <vector>
#include <visualization_msgs/Marker.h>
#define HAVE_CSTDDEF
#include <coin/IpIpoptApplication.hpp>
#include <coin/IpSolveStatistics.hpp>
#undef HAVE_CSTDDEF

#include <string>
#include "nlp/ipopt_interface.h"
#include <ros/ros.h>
#include "nlp/contingence.h"

namespace nlp_manager
{

using namespace Eigen;
using namespace std;


class nlp_solver
{
    public:

        nlp_solver(ros::NodeHandle &nh);

        bool Optimize(std::string flag, const vector<Eigen::Matrix<double, 7, 1>> &state_list, const std::vector<std::array<double, 8>> &boxes);

        vector<Matrix<double, 7, 1>> get_opt_result()
        {
            return std::move(opt_state);
        }

    private:
        nlp::ipopt_interface_entrance *ptop;                 //双层多态指针 
        Ipopt::SmartPtr<Ipopt::IpoptApplication> app;

        bool solve(const vector<Matrix<double, 7, 1>> &state_list, const std::vector<std::array<double, 8>> &boxes); 

        vector<Matrix<double, 7, 1>> opt_state; //x,y,phi,v,steer,a,t
};
}
