#pragma once

#include "grid_searcher/searcher_manage.h"
#include "manager/opt_manager.h"

#include <ros/ros.h>
#include <vector>
#include <eigen3/Eigen/Eigen>

namespace nav
{

using namespace std;
using namespace Eigen;

class planner
{
    public:

    planner(ros::NodeHandle &nh);

    void planning();

    private:

    unique_ptr<searcher_manage> grid_searcher;

    unique_ptr<optimize::opt_manager> opt_path;

    vector<Matrix<double, 6, 1>> get_points(const HybridAStartResult *result);

    bool use_car_search;
    int searcher_flag;
 
};


}

