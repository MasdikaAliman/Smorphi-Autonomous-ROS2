#ifndef ESP_DRIVER_HPP
#define ESP_DRIVER_HPP

#include <cstdio>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <serial/serial.h>
#include "msgpack_ros_interface/MsgPacketizer/MsgPacketizer.h"
#include "msgpack_ros_interface/MsgPackRosInterfaces/MsgPackRosInterfaces.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include <geometry_msgs/msg/polygon.hpp>

#include "esp_driver/smorphi_shape.hpp"

#include "icecream.hpp"

class EspDriver : public rclcpp::Node
{
public:
    explicit EspDriver(const std::string &node_name);

private:
    void odom_result(const MsgPackRosIF::geometry_msgs::msg::Twist &twist);
    void twist_cb(const geometry_msgs::msg::Twist &msg);
    void shape_cb(const std_msgs::msg::String &msg);
    void setShape(const std::string &s);
    void doWork();

private:
    serial::Serial *serial;
    std::string port{"/dev/ttyUSB0"};

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr footprint_pub_global, footprint_pub_local, footprint_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr current_shape_pub;
    
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr  shape_request_sub;
    MsgPackRosIF::geometry_msgs::msg::Twist twist_cmd;
    MsgPackRosIF::std_msgs::msg::String current_shape, shape_req;
    rclcpp::TimerBase::SharedPtr timer;

    SmorphiShape smorphi_shape;

    std::string m_shape{""};



    double current_x = 0.0;
    double current_y = 0.0;
    double current_theta = 0.0;


};

#endif