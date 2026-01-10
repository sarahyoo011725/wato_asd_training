#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, 
    [this](const nav_msgs::msg::Path::SharedPtr msg) { 
      path = *msg; 
      goal = path.poses.back();
  });
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, 
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) { odom = *msg; });
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(dt), [this]() { periodic(); });
}

void ControlNode::periodic() {
  if (path.poses.empty()) {
    return;
  }

  target = path.poses.front();

  const size_t n = path.poses.size();
  double distance = 0.0;
  for (size_t i = 0; i + 1 < n; i++) {
    double x1 = path.poses[i].pose.position.x;
    double y1 = path.poses[i].pose.position.y;
    double x2 = path.poses[i + 1].pose.position.x;
    double y2 = path.poses[i + 1].pose.position.y;
    distance += std::hypot(x1 - x2, y1 - y2);

    if (distance >= lookahead_distance) {
      target = path.poses[i + 1];
      break;
    }
  }

  double dx = target.pose.position.x - odom.pose.pose.position.x;
  double dy = target.pose.position.y - odom.pose.pose.position.y;
  double target_distance = std::hypot(dx, dy);
  double target_heading = std::atan2(dy, dx); 
  
  double qx = odom.pose.pose.orientation.x;
  double qy = odom.pose.pose.orientation.y;
  double qz = odom.pose.pose.orientation.z;
  double qw = odom.pose.pose.orientation.w;
  double heading = std::atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));
  double d_heading = target_heading - heading;

  // ensure change in heading is between -pi and pi
  d_heading = std::atan2(std::sin(d_heading), std::cos(d_heading));

  // TODO: tune speed
  double angular_speed = std::clamp(d_heading * 2.0, -max_angular_speed, max_angular_speed);
  double linear_speed = std::clamp(target_distance * 1.2, 0.0, max_linear_speed); 

  if (is_near(goal.pose.position.x, goal.pose.position.y, 0.5)) {
    linear_speed = 0;
    angular_speed = 0;
  } 

  geometry_msgs::msg::Twist twist_msg;
  twist_msg.linear.x = linear_speed; 
  twist_msg.linear.y = 0; 
  twist_msg.linear.z = 0;
  twist_msg.angular.x = 0;
  twist_msg.angular.y = 0;
  twist_msg.angular.z = angular_speed;

  cmd_vel_pub_->publish(twist_msg);
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
