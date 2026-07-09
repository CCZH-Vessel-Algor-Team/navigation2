#ifndef NAV2_COLREGS_VO_RRT_STAR_PLANNER__RRT_STAR_HPP_
#define NAV2_COLREGS_VO_RRT_STAR_PLANNER__RRT_STAR_HPP_

#include <functional>
#include <random>
#include <vector>

#include "geometry_msgs/msg/point.hpp"

namespace nav2_costmap_2d { class Costmap2D; }

namespace nav2_colregs_vo_rrt_star_planner
{

struct RRTStarNode
{
  double x, y;
  int parent_idx;           // -1 for root
  double cost_from_root;

  RRTStarNode() : x(0), y(0), parent_idx(-1), cost_from_root(0.0) {}
};

class RRTStar
{
public:
  RRTStar(
    double step_size,
    int max_iterations,
    double goal_bias,
    double goal_threshold,
    double safety_dist,
    double cost_weight,
    int max_optimize_iters,
    double eta);

  bool planPath(
    double start_x, double start_y,
    double goal_x, double goal_y,
    const nav2_costmap_2d::Costmap2D * costmap,
    const std::vector<geometry_msgs::msg::Point> & barriers,
    std::vector<RRTStarNode> & path_nodes);

  void prunePath(
    std::vector<RRTStarNode> & path,
    const nav2_costmap_2d::Costmap2D * costmap,
    const std::vector<geometry_msgs::msg::Point> & barriers);

private:
  void randomSample(
    double & x, double & y,
    double goal_x, double goal_y,
    double min_x, double max_x, double min_y, double max_y);

  int nearestNode(double x, double y);

  std::vector<int> findNear(double x, double y);

  bool collisionFree(
    double x1, double y1, double x2, double y2,
    const nav2_costmap_2d::Costmap2D * costmap,
    const std::vector<geometry_msgs::msg::Point> & barriers);

  static bool segmentsIntersect(
    double ax, double ay, double bx, double by,
    double cx, double cy, double dx, double dy);

  double edgeCost(
    double x1, double y1, double x2, double y2,
    const nav2_costmap_2d::Costmap2D * costmap);

  void steer(int from_idx, double toward_x, double toward_y,
             double & new_x, double & new_y);

  void rewire(
    int new_idx, const std::vector<int> & near,
    const nav2_costmap_2d::Costmap2D * costmap,
    const std::vector<geometry_msgs::msg::Point> & barriers);

  std::vector<RRTStarNode> tree_;
  double step_size_;
  int max_iterations_;
  double goal_bias_;
  double goal_threshold_;
  double safety_dist_;
  double cost_weight_;
  int max_optimize_iters_;
  double eta_;

  std::mt19937 rng_;
  bool goal_reached_;
  int best_goal_node_idx_;
};

}  // namespace nav2_colregs_vo_rrt_star_planner

#endif  // NAV2_COLREGS_VO_RRT_STAR_PLANNER__RRT_STAR_HPP_
