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
, _gait_ctrl{}
, _gait_ctrl_input{}
, _gait_ctrl_output{}
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
  _gait_ctrl_input.set_angle_deg(Leg::LeftFront,   Joint::Coxa,  msg.left_front.coxa_angle_deg);
  _gait_ctrl_input.set_angle_deg(Leg::LeftFront,   Joint::Femur, msg.left_front.femur_angle_deg);
  _gait_ctrl_input.set_angle_deg(Leg::LeftFront,   Joint::Tibia, msg.left_front.tibia_angle_deg);

  _gait_ctrl_input.set_angle_deg(Leg::LeftMiddle,  Joint::Coxa,  msg.left_middle.coxa_angle_deg);
  _gait_ctrl_input.set_angle_deg(Leg::LeftMiddle,  Joint::Femur, msg.left_middle.femur_angle_deg);
  _gait_ctrl_input.set_angle_deg(Leg::LeftMiddle,  Joint::Tibia, msg.left_middle.tibia_angle_deg);

  _gait_ctrl_input.set_angle_deg(Leg::LeftBack,    Joint::Coxa,  msg.left_back.coxa_angle_deg);
  _gait_ctrl_input.set_angle_deg(Leg::LeftBack,    Joint::Femur, msg.left_back.femur_angle_deg);
  _gait_ctrl_input.set_angle_deg(Leg::LeftBack,    Joint::Tibia, msg.left_back.tibia_angle_deg);

  _gait_ctrl_input.set_angle_deg(Leg::RightFront,  Joint::Coxa,  msg.right_front.coxa_angle_deg);
  _gait_ctrl_input.set_angle_deg(Leg::RightFront,  Joint::Femur, msg.right_front.femur_angle_deg);
  _gait_ctrl_input.set_angle_deg(Leg::RightFront,  Joint::Tibia, msg.right_front.tibia_angle_deg);

  _gait_ctrl_input.set_angle_deg(Leg::RightMiddle, Joint::Coxa,  msg.right_middle.coxa_angle_deg);
  _gait_ctrl_input.set_angle_deg(Leg::RightMiddle, Joint::Femur, msg.right_middle.femur_angle_deg);
  _gait_ctrl_input.set_angle_deg(Leg::RightMiddle, Joint::Tibia, msg.right_middle.tibia_angle_deg);

  _gait_ctrl_input.set_angle_deg(Leg::RightBack,   Joint::Coxa,  msg.right_back.coxa_angle_deg);
  _gait_ctrl_input.set_angle_deg(Leg::RightBack,   Joint::Femur, msg.right_back.femur_angle_deg);
  _gait_ctrl_input.set_angle_deg(Leg::RightBack,   Joint::Tibia, msg.right_back.tibia_angle_deg);

  _head_ctrl_input.set_pan_angle (msg.head_actual.pan_angle_deg);
  _head_ctrl_input.set_tilt_angle(msg.head_actual.tilt_angle_deg);
}

void Node::onTeleopUpdate(l3xz_teleop::msg::Teleop const & msg)
{
  _gait_ctrl_input.set_teleop_linear_velocity_x (msg.linear_velocity_x);
  _gait_ctrl_input.set_teleop_linear_velocity_y (msg.linear_velocity_y);
  _gait_ctrl_input.set_teleop_angular_velocity_z(msg.angular_velocity_z);

  _head_ctrl_input.set_pan_angular_velocity (msg.angular_velocity_head_pan);
  _head_ctrl_input.set_tilt_angular_velocity(msg.angular_velocity_head_tilt);
}

void Node::onCtrlLoopTimerEvent()
{
  _gait_ctrl_output = _gait_ctrl.update(_gait_ctrl_input, _gait_ctrl_output);

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
