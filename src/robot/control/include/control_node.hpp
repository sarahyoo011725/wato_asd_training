#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "control_core.hpp"

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

  private:
    robot::ControlCore control_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    nav_msgs::msg::Path path;
    nav_msgs::msg::Odometry odom;
    geometry_msgs::msg::PoseStamped goal;
    geometry_msgs::msg::PoseStamped target;
    double lookahead_distance;
    double heading;
    double linear_speed;
    int64_t dt = 100; //ms

    const double max_speed = 3.0;
    
    void periodic();
    bool is_near(double target_x, double target_y, double tolerance);
};

#endif
