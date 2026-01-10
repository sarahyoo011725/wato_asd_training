#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <stack>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "planner_core.hpp"
#include "node.hpp"

enum State {
  WAITING_FOR_GOAL,
  REACHING_TO_GOAL
};

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

  private:
    robot::PlannerCore planner_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    State state_;
    nav_msgs::msg::OccupancyGrid global_map_;
    geometry_msgs::msg::PointStamped goal_;
    geometry_msgs::msg::PoseWithCovariance robot_pose_;
    bool goal_updated = false;
    bool planning = false;

    void plan_path();
    void publish_path();
    bool is_close(Point point, Point target, double threshold);
    bool is_valid(Point cell_index);
    Point pose_to_index(Point pose);

    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goal_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timer_callback();
};

#endif 
