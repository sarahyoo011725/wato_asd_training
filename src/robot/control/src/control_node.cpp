#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) { 
        path = *msg; 
        goal = path.poses.front();
    });
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { odom = *msg; });
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(dt), [this]() { periodic(); });
}

void ControlNode::periodic() {
  if (path.poses.empty()) {
    return;
  }

  target = path.poses.back();

  double dx = target.pose.position.x - odom.pose.pose.position.x;
  double dy = target.pose.position.y - odom.pose.pose.position.y;
  double target_heading = std::atan2(dy, dx); 
  lookahead_distance = std::hypot(dx, dy); 

  if (is_near(goal.pose.position.x, goal.pose.position.y, 0.1)) {
    linear_speed = 0;
  } else {
    linear_speed = std::clamp(lookahead_distance, 0.0, max_speed);
  }

  double qx = odom.pose.pose.orientation.x;
  double qy = odom.pose.pose.orientation.y;
  double qz = odom.pose.pose.orientation.z;
  double qw = odom.pose.pose.orientation.w;
  heading = std::atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));

  double d_heading = target_heading - heading;
  // ensure delta heading to be between -pi and pi
  d_heading = std::atan2(std::sin(d_heading), std::cos(d_heading));

  geometry_msgs::msg::Twist twist_msg;
  twist_msg.linear.x = linear_speed; 
  twist_msg.linear.y = 0; 
  twist_msg.linear.z = 0;
  twist_msg.angular.x = 0;
  twist_msg.angular.y = 0;
  twist_msg.angular.z = d_heading;

  cmd_vel_pub_->publish(twist_msg);

  path.poses.pop_back();
}

bool ControlNode::is_near(double target_x, double target_y, double tolerance) {
  double dx = target_x - odom.pose.pose.position.x;
  double dy = target_y - odom.pose.pose.position.y;
  return std::hypot(dx, dy) <= tolerance;
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
