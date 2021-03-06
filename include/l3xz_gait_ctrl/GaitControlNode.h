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

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>

#include <l3xz_gait_ctrl/kinematic/Engine.h>

#include <l3xz_gait_ctrl/msg/leg_angle.hpp>

#include <l3xz_gait_ctrl/gait/GaitController.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class GaitControlNode : public rclcpp::Node
{
public:
  GaitControlNode();

private:
  kinematic::Engine _kinematic_engine;

  gait::Controller _gait_ctrl;
  gait::ControllerInput _gait_ctrl_input;
  gait::ControllerOutput _gait_ctrl_output;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _robot_sub;
  rclcpp::Publisher<l3xz_gait_ctrl::msg::LegAngle>::SharedPtr _leg_angle_pub;
  rclcpp::Subscription<l3xz_gait_ctrl::msg::LegAngle>::SharedPtr _leg_angle_sub;
  rclcpp::TimerBase::SharedPtr _ctrl_loop_timer;

  void onCtrlLoopTimerEvent();

  void updateGaitControllerInput(l3xz_gait_ctrl::msg::LegAngle::SharedPtr const msg);
  void updateGaitControllerInput(geometry_msgs::msg::Twist::SharedPtr const msg);

  static l3xz_gait_ctrl::msg::LegAngle createOutputMessage(gait::ControllerOutput const & gait_ctrl_output);
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */

#endif /* l3xz_gait_ctrl_NODE_H_ */
