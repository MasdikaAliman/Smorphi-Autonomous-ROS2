#include "esp_driver/esp_driver.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

EspDriver::EspDriver(const std::string &node_name) : Node(node_name)
{
  this->serial = new serial::Serial();
  this->serial->setBaudrate(115200);
  this->serial->setPort(this->port);
  this->serial->open();
  if (not this->serial->isOpen())
  {
    RCLCPP_ERROR(this->get_logger(), "serial_port %s not found", this->port);
  }

  this->odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  this->footprint_pub_global = this->create_publisher<geometry_msgs::msg::Polygon>("/global_costmap/footprint", rclcpp::QoS(10).transient_local().reliable());
  this->footprint_pub_local = this->create_publisher<geometry_msgs::msg::Polygon>("/local_costmap/footprint", rclcpp::QoS(10).transient_local().reliable());
  this->footprint_pub = this->create_publisher<geometry_msgs::msg::Polygon>("/footprint", rclcpp::QoS(10).transient_local().reliable());
  this->current_shape_pub = this->create_publisher<std_msgs::msg::String>("/current_shape", rclcpp::QoS(10).transient_local().reliable());
  this->tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  // this->laser_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  this->cmd_sub = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&EspDriver::twist_cb, this, std::placeholders::_1));
  this->shape_request_sub = this->create_subscription<std_msgs::msg::String>("shape", 10, std::bind(&EspDriver::shape_cb, this, std::placeholders::_1));
  MsgPacketizer::publish(*this->serial, 0x01, twist_cmd)->setFrameRate(10);
  MsgPacketizer::publish(*this->serial, 0x02, shape_req)->setFrameRate(10);
  MsgPacketizer::subscribe(*this->serial, 0x03, current_shape);
  MsgPacketizer::subscribe(*this->serial, 0x04, [this](const MsgPackRosIF::geometry_msgs::msg::Twist &twist)
                           { this->odom_result(twist); });

  this->timer = create_wall_timer(std::chrono::milliseconds(5), [this]()
                                  { this->doWork(); });
  this->setShape("i");
}
void EspDriver::doWork()
{
  MsgPacketizer::update();
}

void EspDriver::setShape(const std::string &s)
{
  if (s != m_shape)
  {
    m_shape = s;
    smorphi_shape.setShape(m_shape);
    geometry_msgs::msg::Polygon footprint_polygon, fp_scaled;
    auto shape_vec = smorphi_shape.getShape();
    for (auto &sh : shape_vec)
    {
      geometry_msgs::msg::Point32 p;
      p.x = sh[0];
      p.y = sh[1];
      footprint_polygon.points.push_back(p);
    }
    std_msgs::msg::String fp_msg;
    fp_msg.data = m_shape;
    this->current_shape_pub->publish(fp_msg);
    this->footprint_pub_global->publish(footprint_polygon);
    this->footprint_pub_local->publish(footprint_polygon);

    auto shape_scaled = smorphi_shape.getShapeScaled();
    for (auto &sh : shape_scaled)
    {
      geometry_msgs::msg::Point32 p;
      p.x = sh[0];
      p.y = sh[1];
      fp_scaled.points.push_back(p);
    }

    this->footprint_pub->publish(fp_scaled);
  }
}
void EspDriver::odom_result(const MsgPackRosIF::geometry_msgs::msg::Twist &twist)
{
  this->setShape(current_shape.data);

  static rclcpp::Time last_time = this->now();
  rclcpp::Time now = this->now();
  auto dt = (now - last_time).seconds();
  // IC(dt);
  last_time = now;

  auto dx = (twist.linear.x * std::cos(current_theta) - twist.linear.y * std::sin(current_theta)) * dt;
  auto dy = (twist.linear.x * std::sin(current_theta) + twist.linear.y * std::cos(current_theta)) * dt;

  current_x += dx;
  current_y += dy;
  current_theta += twist.angular.z * dt;

  // current_theta = fmod(current_theta, 2*M_PI);
  if (current_theta > 2 * M_PI)
    current_theta -= 2 * M_PI;
  if (current_theta < 0)
    current_theta += 2 * M_PI;
  IC(current_x, current_y, current_theta, current_shape.data);

  // set odom message
  auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();
  odom_msg->header.frame_id = "odom";
  odom_msg->child_frame_id = "base_link";
  odom_msg->header.stamp = now;

  odom_msg->pose.pose.position.x = current_x;
  odom_msg->pose.pose.position.y = current_y;
  odom_msg->pose.pose.position.z = 0;
  tf2::Quaternion q;
  q.setRPY(0, 0, current_theta);
  odom_msg->pose.pose.orientation.x = q.x();
  odom_msg->pose.pose.orientation.y = q.y();
  odom_msg->pose.pose.orientation.z = q.z();
  odom_msg->pose.pose.orientation.w = q.w();

  odom_msg->twist.twist.linear.x = twist.linear.x;
  odom_msg->twist.twist.linear.y = twist.linear.y;
  odom_msg->twist.twist.linear.z = twist.linear.z;
  odom_msg->twist.twist.angular.x = twist.angular.x;
  odom_msg->twist.twist.angular.y = twist.angular.y;
  odom_msg->twist.twist.angular.z = twist.angular.z;

  // publish /odom

  geometry_msgs::msg::TransformStamped odom_tf;
  odom_tf.header.stamp = now;
  odom_tf.header.frame_id = "odom";
  odom_tf.child_frame_id = "base_link";
  odom_tf.transform.translation.x = odom_msg->pose.pose.position.x;
  odom_tf.transform.translation.y = odom_msg->pose.pose.position.y;
  odom_tf.transform.translation.z = odom_msg->pose.pose.position.z;
  odom_tf.transform.rotation.x = odom_msg->pose.pose.orientation.x;
  odom_tf.transform.rotation.y = odom_msg->pose.pose.orientation.y;
  odom_tf.transform.rotation.z = odom_msg->pose.pose.orientation.z;
  odom_tf.transform.rotation.w = odom_msg->pose.pose.orientation.w;

  odom_pub->publish(std::move(odom_msg));
  tf_broadcaster->sendTransform(odom_tf);

  auto laser_offset = smorphi_shape.getLaserOffset();
  geometry_msgs::msg::TransformStamped laser_tf;
  laser_tf.header.stamp = now;
  laser_tf.header.frame_id = "base_link";
  laser_tf.child_frame_id = "laser";
  laser_tf.transform.translation.x = laser_offset.at(0);
  laser_tf.transform.translation.y = laser_offset.at(1);
  laser_tf.transform.translation.z = 0;
  tf2::Quaternion quat;
  quat.setRPY(0, 0, laser_offset.at(2));
  laser_tf.transform.rotation.x = quat.x();
  laser_tf.transform.rotation.y = quat.y();
  laser_tf.transform.rotation.z = quat.z();
  laser_tf.transform.rotation.w = quat.w();
  tf_broadcaster->sendTransform(laser_tf);

  geometry_msgs::msg::TransformStamped fp_tf;
  fp_tf.header.stamp = now;
  fp_tf.header.frame_id = "base_link";
  fp_tf.child_frame_id = "base_footprint";
  fp_tf.transform.translation.x = smorphi_shape.x_offset();
  ;
  fp_tf.transform.translation.y = smorphi_shape.y_offset();
  fp_tf.transform.translation.z = 0;
  tf2::Quaternion quat_;
  quat_.setRPY(0, 0, 0);
  fp_tf.transform.rotation.x = quat_.x();
  fp_tf.transform.rotation.y = quat_.y();
  fp_tf.transform.rotation.z = quat_.z();
  fp_tf.transform.rotation.w = quat_.w();
  tf_broadcaster->sendTransform(fp_tf);
}

void EspDriver::shape_cb(const std_msgs::msg::String &msg)
{
  shape_req.data = msg.data;
}

void EspDriver::twist_cb(const geometry_msgs::msg::Twist &msg)
{
  twist_cmd.linear.x = msg.linear.x;
  twist_cmd.linear.y = msg.linear.y;
  twist_cmd.linear.z = 0.0;
  twist_cmd.angular.x = 0.0;
  twist_cmd.angular.y = 0.0;
  twist_cmd.angular.z = msg.angular.z;
}

int main(int argc, char *argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  auto node = std::make_shared<EspDriver>("msgpack_ros_interface");
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
