#ifndef WAYPOINT_NAVIGATOR_HPP
#define WAYPOINT_NAVIGATOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>

#include "waypoint_navigator/wavefront.hpp"
#include <string>
#include <sstream>
#include <regex>

struct Point
{
    double x;
    double y;
};

class WP_Property_t
{
public:
    WP_Property_t() {}
    std::vector<std::vector<double>> area_coverage;
    std::vector<std::vector<double>> transform_area;
    std::string shape;
    std::vector<double> target;
};

class WaypointNavigator : public rclcpp::Node
{
public:
    WaypointNavigator(const std::string &name);
    void doWork(rclcpp::Node::SharedPtr node_ptr);

private:
    std::vector<geometry_msgs::msg::PoseStamped> wavefront();
    void costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void fp_callback(const geometry_msgs::msg::Polygon::SharedPtr msg);
    void current_shape_callback(const std_msgs::msg::String::SharedPtr msg);

    double get_fp_width(const geometry_msgs::msg::Polygon &polygon);
    double get_fp_height(const geometry_msgs::msg::Polygon &polygon);

    void convertOccupancyGridToCostmap2D(const nav_msgs::msg::OccupancyGrid &occupancy_grid,
                                         nav2_costmap_2d::Costmap2D &costmap);
    rclcpp_action::ResultCode send_goal(const geometry_msgs::msg::PoseStamped &goal);

    std::vector<std::vector<double>> parseAreaString(const std::string &area_str)
    {
        std::vector<std::vector<double>> area;
        std::regex regex("\\[\\s*(-?\\d*\\.?\\d+)\\s*,\\s*(-?\\d*\\.?\\d+)\\s*\\]");
        auto begin = std::sregex_iterator(area_str.begin(), area_str.end(), regex);
        auto end = std::sregex_iterator();

        for (std::sregex_iterator i = begin; i != end; ++i)
        {
            std::smatch match = *i;
            double x = std::stod(match[1].str());
            double y = std::stod(match[2].str());
            area.push_back({x, y});
        }

        return area;
    }
    bool isPointInPolygon(const std::vector<std::vector<double>> &polygon, const std::vector<double> &point)
    {
        int n = polygon.size();
        bool inside = false;

        for (int i = 0, j = n - 1; i < n; j = i++)
        {
            double xi = polygon[i][0], yi = polygon[i][1];
            double xj = polygon[j][0], yj = polygon[j][1];

            bool intersect = ((yi > point[1]) != (yj > point[1])) &&
                             (point[0] < (xj - xi) * (point[1] - yi) / (yj - yi) + xi);

            if (intersect)
                inside = !inside;
        }

        return inside;
    }

private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr fp_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr current_shape_sub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr shape_publisher;

    geometry_msgs::msg::Polygon fp;
    geometry_msgs::msg::PoseWithCovarianceStamped pose;

    nav2_costmap_2d::Costmap2D costmap;

    std::unique_ptr<WaveFront> planner_;
    rclcpp::TimerBase::SharedPtr timer;

    double fp_w;
    double fp_h;

    bool fp_received = false;
    bool map_received = false;

    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigate_client_ptr_;
    std::vector<geometry_msgs::msg::PoseStamped> waypoint_;
    std::vector<WP_Property_t> wp_property_vector;
    std::string current_shape = "";
};



#endif