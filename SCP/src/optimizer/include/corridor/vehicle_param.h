/***********************************************************************************
 *  C++ Source Codes for "Autonomous Driving on Curvy Roads without Reliance on
 *  Frenet Frame: A Cartesian-based Trajectory Planning Method".
 ***********************************************************************************
 *  Copyright (C) 2022 Bai Li
 *  Users are suggested to cite the following article when they use the source codes.
 *  Bai Li et al., "Autonomous Driving on Curvy Roads without Reliance on
 *  Frenet Frame: A Cartesian-based Trajectory Planning Method",
 *  IEEE Transactions on Intelligent Transportation Systems, 2022.
 ***********************************************************************************/

#pragma once

#include <tuple>

#include "math/pose.h"
#include "math/box2d.h"



class VehicleParam_corridor {
public:

    /* vehicle_config */
  double front_edge_to_center = 3.89; //前面->中心的距离
  double back_edge_to_center = 1.043;
  double left_edge_to_center = 1.055;
  double right_edge_to_center = 1.055;

  double length = 4.933; //车辆长度4m
  double width = 2.11;
  double height = 1.48;
  double wheel_base = 2.8448;
  double radius;
  double f2x, r2x;

  VehicleParam_corridor(){    
    radius = hypot(0.25 * length, 0.5 * width);
    r2x = 0.25 * length - back_edge_to_center;
    f2x = 0.75 * length - back_edge_to_center;
  }

  template<class T>    //这里用模板是因为要用 ADOLC 进行自动微分
  std::tuple<T, T, T, T> GetDiscPositions(const T &x, const T &y, const T &theta) const {
    auto xf = x + f2x * cos(theta);
    auto xr = x + r2x * cos(theta);
    auto yf = y + f2x * sin(theta);
    auto yr = y + r2x * sin(theta);
    return std::make_tuple(xf, yf, xr, yr);
  }

  cartesian_planner::math::Box2d GenerateBox(const cartesian_planner::math::Pose &pose) const {    
    double distance = length / 2 - back_edge_to_center;
    return {pose.extend(distance), pose.theta(), length, width};
  }

};
