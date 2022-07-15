/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_ctrl/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <l3xz_ctrl/ControlNode.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

ControlNode::ControlNode()
: rclcpp::Node("l3xz_ctrl")
, _kinematic_engine{}
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
    ("/l3xz/ctrl/input", 10, [this](l3xz_ctrl::msg::Input const & msg)
                             {
                               updateGaitControllerInput(msg);
                             });

  _robot_sub = create_subscription<geometry_msgs::msg::Twist>
    ("/l3xz/cmd_vel_robot", 10, [this](geometry_msgs::msg::Twist::SharedPtr const msg) { updateGaitControllerInput(msg);});

  _head_sub = create_subscription<geometry_msgs::msg::Twist>
    ("/l3xz/cmd_vel_head", 10, [this](geometry_msgs::msg::Twist::SharedPtr const msg) { updateHeadControllerInput(msg);});

  _head_angle_sub = create_subscription<l3xz_ctrl::msg::HeadAngle>
    ("/l3xz/ctrl/head/angle/actual", 10, [this](l3xz_ctrl::msg::HeadAngle::SharedPtr const msg)
                                         {
                                            updateHeadControllerInput(msg);
                                         });

  _head_angle_pub = create_publisher<l3xz_ctrl::msg::HeadAngle>
    ("/l3xz/ctrl/head/angle/target", 10);

  _ctrl_loop_timer = create_wall_timer
    (std::chrono::milliseconds(50), [this]() { this->onCtrlLoopTimerEvent(); });
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

void ControlNode::onCtrlLoopTimerEvent()
{
  _gait_ctrl_output = _gait_ctrl.update(_kinematic_engine, _gait_ctrl_input, _gait_ctrl_output);

  _head_ctrl_output = _head_ctrl.update(_head_ctrl_input, _head_ctrl_output);

  l3xz_ctrl::msg::HeadAngle head_msg;
  head_msg.pan_angle_deg  = _head_ctrl_output.pan_angle ();
  head_msg.tilt_angle_deg = _head_ctrl_output.tilt_angle();
  _head_angle_pub->publish(head_msg);

  l3xz_ctrl::msg::Output const msg = createOutputMessage(_gait_ctrl_output);
  _output_pub->publish(msg);
}

void ControlNode::updateGaitControllerInput(l3xz_ctrl::msg::Input const & msg)
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
}

void ControlNode::updateGaitControllerInput(geometry_msgs::msg::Twist::SharedPtr const msg)
{
  _gait_ctrl_input.set_teleop_linear_velocity_x (msg->linear.x);
  _gait_ctrl_input.set_teleop_angular_velocity_z(msg->angular.z);
}

void ControlNode::updateHeadControllerInput(l3xz_ctrl::msg::HeadAngle::SharedPtr msg)
{
  _head_ctrl_input.set_pan_angle (msg->pan_angle_deg);
  _head_ctrl_input.set_tilt_angle(msg->tilt_angle_deg);
}

void ControlNode::updateHeadControllerInput(geometry_msgs::msg::Twist::SharedPtr const msg)
{
  _head_ctrl_input.set_pan_angular_velocity (msg->angular.y);
  _head_ctrl_input.set_tilt_angular_velocity(msg->angular.z);
}

l3xz_ctrl::msg::Output ControlNode::createOutputMessage(gait::ControllerOutput const & gait_ctrl_output)
{
  l3xz_ctrl::msg::Output msg;

  msg.left_front.coxa_angle_deg    = gait_ctrl_output.get_angle_deg(Leg::LeftFront,   Joint::Coxa);
  msg.left_front.femur_angle_deg   = gait_ctrl_output.get_angle_deg(Leg::LeftFront,   Joint::Femur);
  msg.left_front.tibia_angle_deg   = gait_ctrl_output.get_angle_deg(Leg::LeftFront,   Joint::Tibia);

  msg.left_middle.coxa_angle_deg   = gait_ctrl_output.get_angle_deg(Leg::LeftMiddle,  Joint::Coxa);
  msg.left_middle.femur_angle_deg  = gait_ctrl_output.get_angle_deg(Leg::LeftMiddle,  Joint::Femur);
  msg.left_middle.tibia_angle_deg  = gait_ctrl_output.get_angle_deg(Leg::LeftMiddle,  Joint::Tibia);

  msg.left_back.coxa_angle_deg     = gait_ctrl_output.get_angle_deg(Leg::LeftBack,    Joint::Coxa);
  msg.left_back.femur_angle_deg    = gait_ctrl_output.get_angle_deg(Leg::LeftBack,    Joint::Femur);
  msg.left_back.tibia_angle_deg    = gait_ctrl_output.get_angle_deg(Leg::LeftBack,    Joint::Tibia);

  msg.right_front.coxa_angle_deg   = gait_ctrl_output.get_angle_deg(Leg::RightFront,  Joint::Coxa);
  msg.right_front.femur_angle_deg  = gait_ctrl_output.get_angle_deg(Leg::RightFront,  Joint::Femur);
  msg.right_front.tibia_angle_deg  = gait_ctrl_output.get_angle_deg(Leg::RightFront,  Joint::Tibia);

  msg.right_middle.coxa_angle_deg  = gait_ctrl_output.get_angle_deg(Leg::RightMiddle, Joint::Coxa);
  msg.right_middle.femur_angle_deg = gait_ctrl_output.get_angle_deg(Leg::RightMiddle, Joint::Femur);
  msg.right_middle.tibia_angle_deg = gait_ctrl_output.get_angle_deg(Leg::RightMiddle, Joint::Tibia);

  msg.right_back.coxa_angle_deg    = gait_ctrl_output.get_angle_deg(Leg::RightBack,   Joint::Coxa);
  msg.right_back.femur_angle_deg   = gait_ctrl_output.get_angle_deg(Leg::RightBack,   Joint::Femur);
  msg.right_back.tibia_angle_deg   = gait_ctrl_output.get_angle_deg(Leg::RightBack,   Joint::Tibia);

  return msg;
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* L3XZ_CTRL_ControlNode_H_ */
