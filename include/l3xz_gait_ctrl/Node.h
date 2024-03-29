/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_gait_ctrl_gait_ctrl/graphs/contributors.
 */

#ifndef l3xz_gait_ctrl_NODE_H_
#define l3xz_gait_ctrl_NODE_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int64.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <geometry_msgs/msg/twist.hpp>

#include <l3xz/l3xz.h>

#include <ros2_heartbeat/publisher/Publisher.h>
#include <ros2_loop_rate_monitor/Monitor.h>

#include <l3xz_kinematic_engine/l3xz_kinematic_engine.h>

#include <l3xz_gait_ctrl/gait/GaitController.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class Node : public rclcpp::Node
{
public:
  Node();

private:
  kinematic::Engine _kinematic_engine;

  gait::Controller _gait_ctrl;
  gait::ControllerInput _gait_ctrl_input;
  gait::ControllerOutput _gait_ctrl_output;

  std::chrono::steady_clock::time_point const _node_start;

  heartbeat::Publisher::SharedPtr _heartbeat_pub;
  void init_heartbeat();

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _robot_sub;
  std::optional<std::chrono::steady_clock::time_point> _opt_last_robot_msg;
  std::map<LegJointKey,
           rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr> _angle_actual_sub;
  std::map<LegJointKey,
           std::optional<std::chrono::steady_clock::time_point>> _opt_last_angle_actual_msg;
  std::map<Leg,
           rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> _tibia_endpoint_switch_sub;
  std::map<Leg,
           std::optional<std::chrono::steady_clock::time_point>> _opt_last_tibia_endpoint_switch_msg;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _robot_req_up_sub, _robot_req_down_sub;
  void init_sub();

  std::map<LegJointKey,
           rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> _angle_target_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _odom_pub;
  void init_pub();

  static std::chrono::milliseconds constexpr CTRL_LOOP_RATE{10};
  loop_rate::Monitor::SharedPtr _ctrl_loop_rate_monitor;
  rclcpp::TimerBase::SharedPtr _ctrl_loop_timer;
  void ctrl_loop();
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */

#endif /* l3xz_gait_ctrl_NODE_H_ */
