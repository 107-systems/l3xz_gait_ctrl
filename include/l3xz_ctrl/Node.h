/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_ctrl/graphs/contributors.
 */

#ifndef L3XZ_CTRL_NODE_H_
#define L3XZ_CTRL_NODE_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <rclcpp/rclcpp.hpp>

#include <l3xz_teleop/msg/teleop.hpp>

#include <l3xz_ctrl/kinematic/Engine.h>

#include <l3xz_ctrl/msg/input.hpp>
#include <l3xz_ctrl/msg/output.hpp>

#include <l3xz_ctrl/head/HeadController.h>
#include <l3xz_ctrl/gait/GaitController.h>

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

  head::Controller _head_ctrl;
  head::ControllerInput _head_ctrl_input;
  head::ControllerOutput _head_ctrl_output;

  rclcpp::Publisher<l3xz_ctrl::msg::Output>::SharedPtr _output_pub;
  rclcpp::Subscription<l3xz_ctrl::msg::Input>::SharedPtr _input_sub;
  rclcpp::Subscription<l3xz_teleop::msg::Teleop>::SharedPtr _teleop_sub;
  rclcpp::TimerBase::SharedPtr _ctrl_loop_timer;

  void onCtrlLoopTimerEvent();

  void updateGaitControllerInput(l3xz_ctrl::msg::Input const & msg);
  void updateGaitControllerInput(l3xz_teleop::msg::Teleop const & msg);
  void updateHeadControllerInput(l3xz_ctrl::msg::Input const & msg);
  void updateHeadControllerInput(l3xz_teleop::msg::Teleop const & msg);

  static l3xz_ctrl::msg::Output createOutputMessage(gait::ControllerOutput const & gait_ctrl_output, head::ControllerOutput head_ctrl_output);
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */

#endif /* L3XZ_CTRL_NODE_H_ */
