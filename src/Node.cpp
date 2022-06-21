/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_ctrl/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <l3xz_ctrl/Node.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

Node::Node()
: rclcpp::Node("l3xz_ctrl")
, _head_ctrl{}
, _head_ctrl_input{}
, _head_ctrl_output{}
{
  _output_pub = create_publisher<l3xz_ctrl::msg::Output>
    ("/l3xz/ctrl/output", 10);

  _input_sub = create_subscription<l3xz_ctrl::msg::Input>
    ("/l3xz/ctrl/input", 10, [this](l3xz_ctrl::msg::Input const & msg) { this->onInputUpdate(msg); });

  _teleop_sub = create_subscription<l3xz_teleop::msg::Teleop>
    ("/l3xz/cmd_vel", 10, [this](l3xz_teleop::msg::Teleop const & msg) { this->onTeleopUpdate(msg); });

  _ctrl_loop_timer = create_wall_timer
    (std::chrono::milliseconds(50), [this]() { this->onCtrlLoopTimerEvent(); });
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

void Node::onInputUpdate(l3xz_ctrl::msg::Input const & msg)
{
  _head_ctrl_input.set_pan_angle (msg.head_actual.pan_angle_deg);
  _head_ctrl_input.set_tilt_angle(msg.head_actual.tilt_angle_deg);
}

void Node::onTeleopUpdate(l3xz_teleop::msg::Teleop const & msg)
{
  _head_ctrl_input.set_pan_angular_velocity (msg.angular_velocity_head_pan);
  _head_ctrl_input.set_tilt_angular_velocity(msg.angular_velocity_head_tilt);
}

void Node::onCtrlLoopTimerEvent()
{
  _head_ctrl_output = _head_ctrl.update(_head_ctrl_input, _head_ctrl_output);

  l3xz_ctrl::msg::Output msg;
  msg.head_target.pan_angle_deg  = _head_ctrl_output.pan_angle ();
  msg.head_target.tilt_angle_deg = _head_ctrl_output.tilt_angle();
  _output_pub->publish(msg);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* L3XZ_CTRL_NODE_H_ */
