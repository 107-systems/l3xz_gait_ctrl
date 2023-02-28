/**
 * Copyright (c) 2022 LXRobotics GmbH.
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

#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <l3xz_gait_ctrl/kinematic/Engine.h>

#include <l3xz_gait_ctrl/gait/GaitController.h>

#include <l3xz_gait_ctrl/types/LegJoint.h>

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

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _robot_sub;

  std::map<LegJointKey,
           rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr> _angle_actual_sub;

  std::chrono::steady_clock::time_point _prev_ctrl_loop_timepoint;
  static std::chrono::milliseconds constexpr CTRL_LOOP_RATE{10};
  rclcpp::TimerBase::SharedPtr _ctrl_loop_timer;
  void ctrl_loop();
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */

#endif /* l3xz_gait_ctrl_NODE_H_ */
