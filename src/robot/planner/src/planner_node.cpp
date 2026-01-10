#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), 
  planner_(robot::PlannerCore(this->get_logger())), state_(State::WAITING_FOR_GOAL) {
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10, std::bind(&PlannerNode::map_callback, this, std::placeholders::_1));
  goal_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point", 10, std::bind(&PlannerNode::goal_callback, this, std::placeholders::_1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&PlannerNode::odom_callback, this, std::placeholders::_1));
  path_pub_ = create_publisher<nav_msgs::msg::Path>("/path", 10);
  timer_ = create_wall_timer(std::chrono::milliseconds(500), std::bind(&PlannerNode::timer_callback, this));
}

void PlannerNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  global_map_ =  *msg; 
  if (state_ == State::REACHING_TO_GOAL) {
    plan_path();
  }
}

void PlannerNode::goal_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {  
  goal_ = *msg;
  goal_updated = true;
  state_ = State::REACHING_TO_GOAL;
  plan_path();
}

void PlannerNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_pose_ = msg->pose;
}

void PlannerNode::timer_callback() {
  Point robot_p;
  robot_p.x = robot_pose_.pose.position.x;
  robot_p.y = robot_pose_.pose.position.y;
  Point goal_p;
  goal_p.x = goal_.point.x;
  goal_p.y = goal_.point.y;

  if (state_ == State::REACHING_TO_GOAL) {
    if (is_close(robot_p, goal_p, 0.5)) {
      RCLCPP_INFO(this->get_logger(), "Goal reached!");
      state_ = State::WAITING_FOR_GOAL;
    } else {
      RCLCPP_INFO(this->get_logger(), "Replanning due to timeout or progress...");
      plan_path();
    }
  }
}

// find the shortest path to goal using A* search
void PlannerNode::plan_path() {
  if (!goal_updated || global_map_.data.empty()) {
    RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map or goal!");
    return;
  }

  std::priority_queue<aNode*, std::vector<aNode*>, aNode::CompareNode> to_visit;
  std::unordered_map<Point, aNode*, Point::Hash> in_queue; // used to check if a node exists in to_visit queue
  std::unordered_set<Point, Point::Hash> visited;
  std::stack<Point> path;

  // convert start and goal pose into global map cell indices
  Point start;
  start.x = robot_pose_.pose.position.x;
  start.y = robot_pose_.pose.position.y;
  start = pose_to_index(start);
  Point goal;
  goal.x = goal_.point.x;
  goal.y = goal_.point.y;
  goal = pose_to_index(goal);

  //RCLCPP_INFO(this->get_logger(), "Start: (%f, %f), Goal: (%f, %f)", start.x, start.y, goal.x, goal.y);
  
  double g = global_map_.data[start.y * global_map_.info.width + start.x];
  double h = start.distance_to(goal); 
  double f = h + g;
  aNode *start_node = new aNode(start, f, g, h);

  to_visit.push(start_node);
  in_queue.insert({start, start_node});

  while (!to_visit.empty()) {
    // get node with the smallest f value
    aNode *min_node = to_visit.top();
    to_visit.pop();
    visited.insert(min_node->point);

    if (min_node->point == goal) {
      // create path by iterating nodes reversely
      // make sure the end point is the goal point
      aNode *current = min_node;
      while (current != nullptr) {
        path.push(current->point);
        current = current->parent;
      }
      break;
    }

    // check for adjacent nodes
    for (int dx = -1; dx <= 1; dx++) {
      for (int dy = -1; dy <= 1; dy++) {
        // don't check itself
        if (dx == 0 && dy == 0) continue;

        Point adj_point;
        adj_point.x = min_node->point.x + dx;
        adj_point.y = min_node->point.y + dy;

        if (visited.find(adj_point) != visited.end() || !is_valid(adj_point)) {
          continue;
        }

        double g; // a cell is always one unit next to its adjacent cell.

        if (dx != 0 && dy != 0) {
          g = min_node->g + std::sqrt(2.0); // for cell located diagonally
        } else {
          g = min_node->g + 1.0;
        }

        g += global_map_.data[adj_point.y * global_map_.info.width + adj_point.x];

        double h = adj_point.distance_to(goal);
        double f = h + g;

        aNode *adj_node;
        if (in_queue.find(adj_point) == in_queue.end()) {
          // if adj_node is not in to_visit queue
          // create a new adjacent node
          adj_node = new aNode(adj_point, f, g, h);
          adj_node->parent = min_node; 
          to_visit.push(adj_node);
          in_queue.insert({adj_point, adj_node});
        } else {
          // if adj_node exists in to_visit queue
          // override the existing adjacent node
          adj_node = in_queue[adj_point];
          // if found a better path, update
          if (g < adj_node->g) {
            adj_node->g = g;
            adj_node->h = h;
            adj_node->f = f;
            adj_node->parent = min_node;
            // priority queue does not re-adjust automatically
            // so insert it again
            to_visit.push(adj_node); 
          }
        }
      }
    }
  }
  
  // publish path
  nav_msgs::msg::Path p;
  p.header.frame_id = "sim_world";
  p.header.stamp = get_clock()->now();
  while (!path.empty()) {
    Point point = path.top();
    path.pop();
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "sim_world";
    // convert global index into global pose
    pose.pose.position.x = point.x * global_map_.info.resolution + global_map_.info.origin.position.x;
    pose.pose.position.y = point.y * global_map_.info.resolution + global_map_.info.origin.position.y;
    p.poses.push_back(pose);
  }
  path_pub_->publish(p);
}

// converts pose into global map cell index
Point PlannerNode::pose_to_index(Point pose) {
  Point p;
  p.x = std::floor((pose.x - global_map_.info.origin.position.x) / global_map_.info.resolution);
  p.y = std::floor((pose.y - global_map_.info.origin.position.y) / global_map_.info.resolution);
  return p;
}

bool PlannerNode::is_valid(Point cell_index) {
  return cell_index.x < global_map_.info.width && cell_index.y < global_map_.info.height 
    && cell_index.x >= 0 && cell_index.y >= 0;
}

bool PlannerNode::is_close(Point point, Point target, double threshold) {
  double dx = point.x - target.x;
  double dy = point.y - target.y;
  return std::hypot(dx, dy) <= threshold;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}