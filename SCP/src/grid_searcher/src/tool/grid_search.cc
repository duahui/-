

#include "coarse_trajectory_generator/grid_search.h"

GridSearch::GridSearch(const PlannerOpenSpaceConfig &open_space_conf) {
  xy_grid_resolution_ =
      open_space_conf.warm_start_config.grid_a_star_xy_resolution;
  node_radius_ = open_space_conf.warm_start_config.node_radius; //节点半径
}

double GridSearch::EuclidDistance(const double x1, const double y1,
                                  const double x2, const double y2) {
  return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

bool GridSearch::CheckConstraints(std::shared_ptr<Node2d> node) {
  const double node_grid_x = node->GetGridX();
  const double node_grid_y = node->GetGridY();
  if (node_grid_x > max_grid_x_ || node_grid_x < 0 ||
      node_grid_y > max_grid_y_ || node_grid_y < 0) {
    return false;
  }
  if (obstacles_linesegments_vec_.empty()) {
    return true;
  }
  for (const auto &obstacle_linesegments : obstacles_linesegments_vec_) {
    for (const common::math::LineSegment2d &linesegment :
         obstacle_linesegments) {
      if (linesegment.DistanceTo({node->GetGridX(), node->GetGridY()}) <
          node_radius_) {
        return false;
      }
    }
  }
  return true;
}
//扩展节点并计算其cost
std::vector<std::shared_ptr<Node2d>>
GridSearch::GenerateNextNodes(std::shared_ptr<Node2d> current_node) {
  double current_node_x = current_node->GetGridX();
  double current_node_y = current_node->GetGridY();
  double current_node_path_cost = current_node->GetPathCost();
  double diagonal_distance = std::sqrt(2.0); //对角线距离根号2
  std::vector<std::shared_ptr<Node2d>> next_nodes;
  std::shared_ptr<Node2d> up =
      std::make_shared<Node2d>(current_node_x, current_node_y + 1.0, XYbounds_);
  up->SetPathCost(current_node_path_cost + 1.0);
  std::shared_ptr<Node2d> up_right = std::make_shared<Node2d>(
      current_node_x + 1.0, current_node_y + 1.0, XYbounds_);
  up_right->SetPathCost(current_node_path_cost + diagonal_distance);
  std::shared_ptr<Node2d> right =
      std::make_shared<Node2d>(current_node_x + 1.0, current_node_y, XYbounds_);
  right->SetPathCost(current_node_path_cost + 1.0);
  std::shared_ptr<Node2d> down_right = std::make_shared<Node2d>(
      current_node_x + 1.0, current_node_y - 1.0, XYbounds_);
  down_right->SetPathCost(current_node_path_cost + diagonal_distance);
  std::shared_ptr<Node2d> down =
      std::make_shared<Node2d>(current_node_x, current_node_y - 1.0, XYbounds_);
  down->SetPathCost(current_node_path_cost + 1.0);
  std::shared_ptr<Node2d> down_left = std::make_shared<Node2d>(
      current_node_x - 1.0, current_node_y - 1.0, XYbounds_);
  down_left->SetPathCost(current_node_path_cost + diagonal_distance);
  std::shared_ptr<Node2d> left =
      std::make_shared<Node2d>(current_node_x - 1.0, current_node_y, XYbounds_);
  left->SetPathCost(current_node_path_cost + 1.0);
  std::shared_ptr<Node2d> up_left = std::make_shared<Node2d>(
      current_node_x - 1.0, current_node_y + 1.0, XYbounds_);
  up_left->SetPathCost(current_node_path_cost + diagonal_distance);

  next_nodes.emplace_back(up);
  next_nodes.emplace_back(up_right);
  next_nodes.emplace_back(right);
  next_nodes.emplace_back(down_right);
  next_nodes.emplace_back(down);
  next_nodes.emplace_back(down_left);
  next_nodes.emplace_back(left);
  next_nodes.emplace_back(up_left);
  return next_nodes;
}

bool GridSearch::GenerateAStarPath(
    const double sx, const double sy, const double ex, const double ey,
    const std::vector<double> &XYbounds,
    const std::vector<std::vector<common::math::LineSegment2d>>
        &obstacles_linesegments_vec,
    GridAStartResult *result) {
  std::priority_queue<std::pair<std::string, double>,
                      std::vector<std::pair<std::string, double>>, cmp>
      open_pq;
  std::unordered_map<std::string, std::shared_ptr<Node2d>> open_set;
  std::unordered_map<std::string, std::shared_ptr<Node2d>> close_set;
  XYbounds_ = XYbounds;
  std::shared_ptr<Node2d> start_node =
      std::make_shared<Node2d>(sx, sy, xy_grid_resolution_, XYbounds_); //起点
  std::shared_ptr<Node2d> end_node =
      std::make_shared<Node2d>(ex, ey, xy_grid_resolution_, XYbounds_); //终点
  std::shared_ptr<Node2d> final_node_ = nullptr;
  obstacles_linesegments_vec_ =
      obstacles_linesegments_vec; //参数转为私有变量是为了让类中可以使用该变量
  open_set.emplace(start_node->GetIndex(),
                   start_node); // unorderer_map(x_y索引,node2d node)
  open_pq.emplace(start_node->GetIndex(),
                  start_node->GetCost()); //(string,double)

  // Grid a star begins
  size_t explored_node_num = 0;
  while (!open_pq.empty()) {
    std::string current_id = open_pq.top().first;
    open_pq.pop();
    std::shared_ptr<Node2d> current_node = open_set[current_id];
    // Check destination是否到达终点
    if (*(current_node) == *(end_node)) {
      final_node_ = current_node;
      break;
    }
    close_set.emplace(current_node->GetIndex(), current_node);
    std::vector<std::shared_ptr<Node2d>> next_nodes =
        std::move(GenerateNextNodes(current_node));
    for (auto &next_node : next_nodes) {
      if (!CheckConstraints(next_node)) {
        continue;
      }
      if (close_set.find(next_node->GetIndex()) != close_set.end()) {
        continue;
      }
      if (open_set.find(next_node->GetIndex()) == open_set.end()) {
        ++explored_node_num;
        next_node->SetHeuristic(
            EuclidDistance(next_node->GetGridX(), next_node->GetGridY(),
                           end_node->GetGridX(), end_node->GetGridY()));
        next_node->SetPreNode(current_node);
        open_set.emplace(next_node->GetIndex(), next_node);
        open_pq.emplace(next_node->GetIndex(), next_node->GetCost());
      }
    }
  }

  if (final_node_ == nullptr) {
    std::cout << "Grid A searching return null ptr(open_set ran out)"
              << std::endl;
    return false;
  }
  LoadGridAStarResult(result);
  std::cout << "explored node num is " << explored_node_num;
  return true;
}
//主要
bool GridSearch::GenerateDpMap(
    const double ex, const double ey, const std::vector<double> &XYbounds,
    const std::vector<std::vector<common::math::LineSegment2d>>
        &obstacles_linesegments_vec) {
  std::priority_queue<std::pair<std::string, double>,
                      std::vector<std::pair<std::string, double>>, cmp>
      open_pq;
  std::unordered_map<std::string, std::shared_ptr<Node2d>> open_set;
  dp_map_ = decltype(dp_map_)();
  XYbounds_ = XYbounds;
  // XYbounds with xmin, xmax, ymin, ymax
  max_grid_y_ = std::round((XYbounds_[3] - XYbounds_[2]) / xy_grid_resolution_);
  max_grid_x_ = std::round((XYbounds_[1] - XYbounds_[0]) / xy_grid_resolution_);
  std::shared_ptr<Node2d> end_node =
      std::make_shared<Node2d>(ex, ey, xy_grid_resolution_, XYbounds_);
  obstacles_linesegments_vec_ = obstacles_linesegments_vec;
  open_set.emplace(end_node->GetIndex(), end_node); //终点放入open_set
  open_pq.emplace(end_node->GetIndex(), end_node->GetCost()); //终点放入open_pq

  // Grid a star begins//计算地图中每个点到终点的cost
  size_t explored_node_num = 0;
  while (!open_pq.empty()) {
    const std::string current_id = open_pq.top().first;
    open_pq.pop();
    std::shared_ptr<Node2d> current_node = open_set[current_id];
    dp_map_.emplace(current_node->GetIndex(),
                    current_node); //当前节点节点存入dp_map_
    std::vector<std::shared_ptr<Node2d>> next_nodes =
        std::move(GenerateNextNodes(
            current_node)); //生成扩展节点,并计算其cost（从起点到该点的距离cost）
    for (auto &next_node : next_nodes) {
      if (!CheckConstraints(next_node)) { //检查是否超过边界或者碰撞，则跳过
        continue;
      }
      if (dp_map_.find(next_node->GetIndex()) !=
          dp_map_.end()) { // map如果搜索不到会返回末尾迭代器
        continue;          //如果节点存在于close_set()内，则跳过
      }
      if (open_set.find(next_node->GetIndex()) ==
          open_set.end()) { //如果不在open_set
        ++explored_node_num;
        next_node->SetPreNode(current_node);                //设置父节点
        open_set.emplace(next_node->GetIndex(), next_node); //加入open_set
        open_pq.emplace(next_node->GetIndex(), next_node->GetCost()); //加入pq
      } else { //已存在open_set,如果存储的cost大于当前cost，更新cost
        if (open_set[next_node->GetIndex()]->GetCost() > next_node->GetCost()) {
          open_set[next_node->GetIndex()]->SetCost(next_node->GetCost());
          open_set[next_node->GetIndex()]->SetPreNode(current_node);
        }
      }
    }
  }
  std::cout << "explored node num is " << explored_node_num << std::endl;
  return true;
}

double GridSearch::CheckDpMap(const double sx, const double sy) {
  std::string index = Node2d::CalcIndex(sx, sy, xy_grid_resolution_, XYbounds_);
  if (dp_map_.find(index) != dp_map_.end()) {
    return dp_map_[index]->GetCost() * xy_grid_resolution_;
  } else {
    return std::numeric_limits<double>::infinity();
  }
}

void GridSearch::LoadGridAStarResult(GridAStartResult *result) {
  (*result).path_cost = final_node_->GetPathCost() * xy_grid_resolution_;
  std::shared_ptr<Node2d> current_node = final_node_;
  std::vector<double> grid_a_x;
  std::vector<double> grid_a_y;
  while (current_node->GetPreNode() != nullptr) {
    grid_a_x.push_back(current_node->GetGridX() * xy_grid_resolution_ +
                       XYbounds_[0]);
    grid_a_y.push_back(current_node->GetGridY() * xy_grid_resolution_ +
                       XYbounds_[2]);
    current_node = current_node->GetPreNode();
  }
  std::reverse(grid_a_x.begin(), grid_a_x.end());
  std::reverse(grid_a_y.begin(), grid_a_y.end());
  (*result).x = std::move(grid_a_x);
  (*result).y = std::move(grid_a_y);
}
