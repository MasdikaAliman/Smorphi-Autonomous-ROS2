#include "waypoint_navigator/WaypointNavigator.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "icecream.hpp"

WaypointNavigator::WaypointNavigator(const std::string &name) : Node(name)
{

  WP_Property_t wp_prop;

  wp_prop.area_coverage = {{-2.0, -2.0},
                           {-2., 0.},
                           {0., 0.},
                           {0., -2.},
                           {-2., -2.}};

  wp_prop.transform_area = {{-2., -2.},
                            {-2., -1.},
                            {-1.5, -1.},
                            {-1.5, -2.},
                            {-2., -2.}};

  wp_prop.shape = "i";
  wp_prop.target = {1.2, 0.5};
  wp_property_vector.push_back(wp_prop);

  wp_prop.area_coverage =
      {{-2., 0.},
       {-2., 2.},
       {0.5, 0.},
       {0.5, 2.},
       {-2., 0.}};

  wp_prop.transform_area = {
      {-.1, 0},
      {0., 0},
      {0., -0.5},
      {-.1, -0.5},
      {-.1, 0}};

  wp_prop.shape = "o";
  wp_prop.target = {0.47, -1.11};
  wp_property_vector.push_back(wp_prop);

  wp_prop.area_coverage =
      {{0.5, 0},
       {0.5, 2},
       {2, 2},
       {2, 0},
       {0.5, 0}};

  wp_prop.transform_area = {
      {0, 1.2},
      {0, 2},
      {0.5, 2},
      {0.5, 1.2},
      {0, 1.2}};

  wp_prop.shape = "i";
  wp_prop.target = {-1.2, -0.9};
  wp_property_vector.push_back(wp_prop);

  fp_subscription_ = this->create_subscription<geometry_msgs::msg::Polygon>("/footprint", rclcpp::QoS(10).transient_local().reliable(), std::bind(&WaypointNavigator::fp_callback, this, std::placeholders::_1));
  current_shape_sub = this->create_subscription<std_msgs::msg::String>("/current_shape", rclcpp::QoS(10).transient_local().reliable(), std::bind(&WaypointNavigator::current_shape_callback, this, std::placeholders::_1));
  costmap_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::QoS(10).transient_local().reliable(), std::bind(&WaypointNavigator::costmap_callback, this, std::placeholders::_1));
  pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose", rclcpp::QoS(10).transient_local().reliable(), std::bind(&WaypointNavigator::pose_callback, this, std::placeholders::_1));
  path_publisher = this->create_publisher<nav_msgs::msg::Path>("/path", rclcpp::QoS(10));
  shape_publisher = this->create_publisher<std_msgs::msg::String>("shape", 10);

  this->navigate_client_ptr_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");

  planner_ = std::make_unique<WaveFront>();

  // this->timer = create_wall_timer(std::chrono::milliseconds(5), [this]()
  //                                 { this->doWork(); });
}
void WaypointNavigator::current_shape_callback(const std_msgs::msg::String::SharedPtr msg)
{
  current_shape = msg->data;
}
void WaypointNavigator::costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  nav_msgs::msg::OccupancyGrid cost;
  cost.header = msg->header;
  cost.info = msg->info;
  cost.data = msg->data;
  IC();
  convertOccupancyGridToCostmap2D(cost, costmap);
  RCLCPP_INFO(this->get_logger(), "Received costmap of size [%d x %d]",
              msg->info.width, msg->info.height);
}

void WaypointNavigator::convertOccupancyGridToCostmap2D(const nav_msgs::msg::OccupancyGrid &occupancy_grid, nav2_costmap_2d::Costmap2D &costmap_)
{
  // Extract information from OccupancyGrid
  unsigned int width = occupancy_grid.info.width;
  unsigned int height = occupancy_grid.info.height;
  double resolution = occupancy_grid.info.resolution;
  double origin_x = occupancy_grid.info.origin.position.x;
  double origin_y = occupancy_grid.info.origin.position.y;

  // Initialize the costmap with the size and resolution
  costmap_.resizeMap(width, height, resolution, origin_x, origin_y);
  // Convert occupancy grid data to costmap format
  for (unsigned int y = 0; y < height; ++y)
  {
    for (unsigned int x = 0; x < width; ++x)
    {
      int index = x + y * width;
      int8_t occupancy_value = occupancy_grid.data[index];

      // Convert OccupancyGrid values to Costmap values
      unsigned char costmap_value;
      if (occupancy_value == -1)
      {
        costmap_value = nav2_costmap_2d::NO_INFORMATION;
      }
      else if (occupancy_value == 0)
      {
        costmap_value = nav2_costmap_2d::FREE_SPACE;
      }
      else
      {
        costmap_value = nav2_costmap_2d::LETHAL_OBSTACLE;
      }

      costmap_.setCost(x, y, costmap_value);
    }
  }
  map_received = true;
}

void WaypointNavigator::pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  pose.header = msg->header;
  pose.pose = msg->pose;
}
void WaypointNavigator::fp_callback(const geometry_msgs::msg::Polygon::SharedPtr msg)
{
  fp.points = msg->points;

  fp_w = get_fp_width(this->fp) * 100.; // m to cm
  fp_h = get_fp_height(this->fp) * 100.;
  IC(fp_w, fp_h);
  fp_received = true;
}

double WaypointNavigator::get_fp_width(const geometry_msgs::msg::Polygon &polygon)
{
  double min_x = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();

  for (const auto &point : polygon.points)
  {
    if (point.x < min_x)
    {
      min_x = point.x;
    }
    if (point.x > max_x)
    {
      max_x = point.x;
    }
  }

  return max_x - min_x;
}

double WaypointNavigator::get_fp_height(const geometry_msgs::msg::Polygon &polygon)
{
  double min_y = std::numeric_limits<double>::max();
  double max_y = std::numeric_limits<double>::lowest();

  for (const auto &point : polygon.points)
  {
    if (point.y < min_y)
    {
      min_y = point.y;
    }
    if (point.y > max_y)
    {
      max_y = point.y;
    }
  }

  return max_y - min_y;
}

std::vector<geometry_msgs::msg::PoseStamped> WaypointNavigator::wavefront()
{
  const unsigned char *charmap = costmap.getCharMap();
  unsigned int width = costmap.getSizeInCellsX();
  unsigned int height = costmap.getSizeInCellsY();
  std::vector<std::vector<int>> grid(height, std::vector<int>(width));

  // Fill the 2D vector with values from the char map
  double x_w, y_w;
  costmap.mapToWorld(399, 399, x_w, y_w);
  IC(x_w, y_w);
  for (unsigned int y = 0; y < height; ++y)
  {
    for (unsigned int x = 0; x < width; ++x)
    {
      // The charmap is stored in row-major order
      double px, py;
      costmap.mapToWorld(x, y, px, py);
      auto didalam = isPointInPolygon(wp_property_vector.at(0).transform_area, {px, py}) or isPointInPolygon(wp_property_vector.at(0).area_coverage, {px, py});
      //if (x == 0 or y == 0 or x == width - 1 or y == height - 1)
      //  grid[y][x] = 0;
      if (charmap[y * width + x] == nav2_costmap_2d::LETHAL_OBSTACLE)
        grid[y][x] = -1;
      else
        grid[y][x] = 0;
    }
  }

  unsigned int sx, sy;
  unsigned int gx, gy;

  auto start = pose.pose;
  // start.pose.position.x = -1;
  // start.pose.position.y = 1.2;
  geometry_msgs::msg::Pose goal;
  goal.position.x = wp_property_vector.at(0).target[0];
  goal.position.y = wp_property_vector.at(0).target[1];

  costmap.worldToMap(start.pose.position.x, start.pose.position.y, sx, sy);
  costmap.worldToMap(goal.position.x, goal.position.y, gx, gy);

  IC(start.pose.position.x, start.pose.position.y, sx, sy);
  IC(fp_w, fp_h);
  int fp_height = static_cast<int>(round(fp_h));
  int fp_width = static_cast<int>(round(fp_w));
  IC(fp_height, fp_width);

  std::pair<int, int> start_ = std::make_pair(sx / fp_width, sy / fp_height);
  std::pair<int, int> goal_ = std::make_pair(gx / fp_width, gy / fp_height);

  int oH = grid.size();
  int oW = grid.at(0).size();
  int nH = oH / fp_height;
  int nW = oW / fp_width;

  int mod_h = oH % fp_height;
  int mod_w = oW % fp_width;
  IC(mod_h, mod_w, nH, nW, sx, sy);

  std::vector<std::vector<int>> grid_ = std::vector(nH, std::vector<int>(nW, 0));
  IC();
  IC(grid_.size(), grid_.at(0).size(), oH, oW);
  IC(grid[100][299]);
  for (int y = 0; y < oH - mod_h; ++y)
  {
    for (int x = 0; x < oW - mod_w; ++x)
    {
      // IC(y, fp_height, y / fp_height, x, fp_width, x / fp_width);
      grid_.at(y / fp_height).at(x / fp_width) = grid_[y / fp_height][x / fp_width] == -1 ? -1 : grid[y][x];
    }
  }
  IC(grid_);

  IC(grid_.size(), grid_.at(0).size(), start_.first, start_.second, goal_.first, goal_.second);

  auto res = planner_->calculate(grid_, start_, goal_);

  IC(res.size());

  std::vector<geometry_msgs::msg::PoseStamped> waypoints;
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.header.stamp = this->now();

  for (auto &o : res)
  {
    geometry_msgs::msg::PoseStamped pose;
    double wx, wy;
    costmap.mapToWorld(o.first * fp_width, o.second * fp_height, wx, wy);
    pose.header.frame_id = "map";
    pose.header.stamp = this->now();
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    waypoints.push_back(pose);

    path.poses.push_back(pose);
  }
  path_publisher->publish(path);
  IC();
  return waypoints;
}

rclcpp_action::ResultCode WaypointNavigator::send_goal(const geometry_msgs::msg::PoseStamped &goal)
{
  auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
  goal_msg.pose = goal;

  // Send goal and wait for the result
  auto goal_handle_future = this->navigate_client_ptr_->async_send_goal(goal_msg);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Send goal call failed");
    return rclcpp_action::ResultCode::UNKNOWN;
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle)
  {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    return rclcpp_action::ResultCode::UNKNOWN;
  }

  // Wait for the result
  auto result_future = this->navigate_client_ptr_->async_get_result(goal_handle);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Get result call failed");
    return rclcpp_action::ResultCode::UNKNOWN;
  }

  auto result = result_future.get();
  return result.code;
}

void WaypointNavigator::doWork(rclcpp::Node::SharedPtr node_ptr)
{
  if (fp_w > 0 and fp_h > 0 and map_received and fp_received)
  {
    IC(wp_property_vector.size());
    while (wp_property_vector.size() > 0)
    {
      auto tes = isPointInPolygon(wp_property_vector.at(0).area_coverage, {-1, -1});
      IC(wp_property_vector.at(0).area_coverage);
      IC(tes);
      if (current_shape != wp_property_vector.at(0).shape)
      {
        auto didalam = isPointInPolygon(wp_property_vector.at(0).transform_area, {pose.pose.pose.position.x, pose.pose.pose.position.y});
        IC(didalam);
        if (didalam)
        {
          std_msgs::msg::String msg;
          msg.data = wp_property_vector.at(0).shape;
          shape_publisher->publish(msg);
        }
      }

      rclcpp::spin_some(node_ptr);
      waypoint_ = wavefront();
      IC(waypoint_.size());
      if (waypoint_.size() > 1)
      {
        while (waypoint_.size() > 1)
        {
          rclcpp::spin_some(node_ptr);
          this->send_goal(waypoint_.at(0));
          waypoint_.erase(waypoint_.begin());
          IC(waypoint_.size());
        }
        wp_property_vector.erase(wp_property_vector.begin());
      }
    }
  }
}
