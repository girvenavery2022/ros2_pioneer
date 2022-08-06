// MIT License
//
// Copyright (c) 2022 Avery Girven
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once
#include <memory>
#include <chrono>

#include "Aria/Aria.h"
#include <Aria/ArRobotConfigPacketReader.h> // todo remove after ArRobotConfig implemented in AriaCoda

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "rosaria_msgs/msg/bumper_state.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "nav_msgs/msg/odometry.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "tf2/transform_datatypes.h"
#include <tf2_ros/transform_broadcaster.h>


namespace Pioneer
{
class Pioneer: public rclcpp::Node
{
public:
  explicit Pioneer(rclcpp::NodeOptions options);

  /**
   * @brief 
   * 
   */
  void connect();

private:
  /**
   * @brief 
   * 
   */
  void setup();

  /**
   * @brief 
   * 
   */
  void battery_voltage_publisher();

  /**
   * @brief 
   * 
   */
  void sonar_publisher();

  /**
   * @brief 
   * 
   */
  void bumper_publisher();

  /**
   * @brief 
   * 
   */
  void odometry_publisher();
  
  /**
   * @brief 
   * 
   * @param msg geometry_msgs::msg::Twist::SharedPtr
   */
  void cmdvel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  /**
   * @brief 
   * 
   */
  void motor_state_publisher();

  /**
   * @brief 
   * 
   * @param request std_srvs::srv::SetBool::Request
   * @param response std_srvs::srv::SetBool::Response
   */
  void trigger_motors(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
          std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  // publisher and subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sonar_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr voltage_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motors_state_pub;
  rclcpp::Publisher<rosaria_msgs::msg::BumperState>::SharedPtr bumper_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

  // transformation
  std::shared_ptr<tf2_ros::TransformBroadcaster> odom_tf_broadcaster;

  // services
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr motor_service;

  rosaria_msgs::msg::BumperState bumpers;

  // publisher timers
  rclcpp::TimerBase::SharedPtr battery_voltage_publish_timer;
  rclcpp::TimerBase::SharedPtr motors_state_publish_timer;
  rclcpp::TimerBase::SharedPtr bumper_publish_timer;
  rclcpp::TimerBase::SharedPtr sonar_publish_timer;
  rclcpp::TimerBase::SharedPtr odom_publish_timer;

  // Aria sdk attributes
  ArRobot *robot;
  ArRobotConnector *robotConnector;
  ArPose pos;

  // node parameters
  bool publish_sonar{true};
  bool publish_bumper{true};
  bool published_motor_state{false};
  std::string port{"/dev/ttyUSB0"};
  std::string odom_frame_id{};
  std::string base_frame_id{};
  std::string bumper_frame_id{};
  std::string sonar_frame_id{};
  int battery_voltage_update_rate{0};
  int motor_state_update_rate{0};
  int bumper_publish_rate{0};
  int sonar_publish_rate{0};
  int odom_publish_rate{0};
};
}