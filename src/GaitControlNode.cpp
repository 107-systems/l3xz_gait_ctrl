/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_ctrl/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <l3xz_ctrl/GaitControlNode.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

GaitControlNode::GaitControlNode()
: rclcpp::Node("l3xz_gait_ctrl")
, _kinematic_engine{}
, _gait_ctrl{}
, _gait_ctrl_input{}
, _gait_ctrl_output{}
{
  _robot_sub = create_subscription<geometry_msgs::msg::Twist>
    ("/l3xz/cmd_vel_robot", 10, [this](geometry_msgs::msg::Twist::SharedPtr const msg) { updateGaitControllerInput(msg);});

  _leg_angle_pub = create_publisher<l3xz_ctrl::msg::LegAngle>
    ("/l3xz/ctrl/leg/angle/target", 10);

  _leg_angle_sub = create_subscription<l3xz_ctrl::msg::LegAngle>
    ("/l3xz/ctrl/leg/angle/actual", 10, [this](l3xz_ctrl::msg::LegAngle::SharedPtr const msg) { updateGaitControllerInput(msg); });

  _ctrl_loop_timer = create_wall_timer
    (std::chrono::milliseconds(50), [this]() { this->onCtrlLoopTimerEvent(); });
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

void GaitControlNode::onCtrlLoopTimerEvent()
{
  _gait_ctrl_output = _gait_ctrl.update(_kinematic_engine, _gait_ctrl_input, _gait_ctrl_output);

  l3xz_ctrl::msg::LegAngle const leg_msg = createOutputMessage(_gait_ctrl_output);
  _leg_angle_pub->publish(leg_msg);
}

void GaitControlNode::updateGaitControllerInput(l3xz_ctrl::msg::LegAngle::SharedPtr const msg)
{
  _gait_ctrl_input.set_angle_deg(Leg::LeftFront,   Joint::Coxa,  msg->coxa_angle_deg [0]);
  _gait_ctrl_input.set_angle_deg(Leg::LeftFront,   Joint::Femur, msg->femur_angle_deg[0]);
  _gait_ctrl_input.set_angle_deg(Leg::LeftFront,   Joint::Tibia, msg->tibia_angle_deg[0]);

  _gait_ctrl_input.set_angle_deg(Leg::LeftMiddle,  Joint::Coxa,  msg->coxa_angle_deg [1]);
  _gait_ctrl_input.set_angle_deg(Leg::LeftMiddle,  Joint::Femur, msg->femur_angle_deg[1]);
  _gait_ctrl_input.set_angle_deg(Leg::LeftMiddle,  Joint::Tibia, msg->tibia_angle_deg[1]);

  _gait_ctrl_input.set_angle_deg(Leg::LeftBack,    Joint::Coxa,  msg->coxa_angle_deg [2]);
  _gait_ctrl_input.set_angle_deg(Leg::LeftBack,    Joint::Femur, msg->femur_angle_deg[2]);
  _gait_ctrl_input.set_angle_deg(Leg::LeftBack,    Joint::Tibia, msg->tibia_angle_deg[2]);

  _gait_ctrl_input.set_angle_deg(Leg::RightFront,  Joint::Coxa,  msg->coxa_angle_deg [3]);
  _gait_ctrl_input.set_angle_deg(Leg::RightFront,  Joint::Femur, msg->femur_angle_deg[3]);
  _gait_ctrl_input.set_angle_deg(Leg::RightFront,  Joint::Tibia, msg->tibia_angle_deg[3]);

  _gait_ctrl_input.set_angle_deg(Leg::RightMiddle, Joint::Coxa,  msg->coxa_angle_deg [4]);
  _gait_ctrl_input.set_angle_deg(Leg::RightMiddle, Joint::Femur, msg->femur_angle_deg[4]);
  _gait_ctrl_input.set_angle_deg(Leg::RightMiddle, Joint::Tibia, msg->tibia_angle_deg[4]);

  _gait_ctrl_input.set_angle_deg(Leg::RightBack,   Joint::Coxa,  msg->coxa_angle_deg [5]);
  _gait_ctrl_input.set_angle_deg(Leg::RightBack,   Joint::Femur, msg->femur_angle_deg[5]);
  _gait_ctrl_input.set_angle_deg(Leg::RightBack,   Joint::Tibia, msg->tibia_angle_deg[5]);
}

void GaitControlNode::updateGaitControllerInput(geometry_msgs::msg::Twist::SharedPtr const msg)
{
  _gait_ctrl_input.set_teleop_linear_velocity_x (msg->linear.x);
  _gait_ctrl_input.set_teleop_angular_velocity_z(msg->angular.z);
}

l3xz_ctrl::msg::LegAngle GaitControlNode::createOutputMessage(gait::ControllerOutput const & gait_ctrl_output)
{
  l3xz_ctrl::msg::LegAngle msg;

  msg.coxa_angle_deg    [0] = gait_ctrl_output.get_angle_deg(Leg::LeftFront,   Joint::Coxa);
  msg.femur_angle_deg   [0] = gait_ctrl_output.get_angle_deg(Leg::LeftFront,   Joint::Femur);
  msg.tibia_angle_deg   [0] = gait_ctrl_output.get_angle_deg(Leg::LeftFront,   Joint::Tibia);

  msg.coxa_angle_deg    [1] = gait_ctrl_output.get_angle_deg(Leg::LeftMiddle,  Joint::Coxa);
  msg.femur_angle_deg   [1] = gait_ctrl_output.get_angle_deg(Leg::LeftMiddle,  Joint::Femur);
  msg.tibia_angle_deg   [1] = gait_ctrl_output.get_angle_deg(Leg::LeftMiddle,  Joint::Tibia);

  msg.coxa_angle_deg    [2] = gait_ctrl_output.get_angle_deg(Leg::LeftBack,    Joint::Coxa);
  msg.femur_angle_deg   [2] = gait_ctrl_output.get_angle_deg(Leg::LeftBack,    Joint::Femur);
  msg.tibia_angle_deg   [2] = gait_ctrl_output.get_angle_deg(Leg::LeftBack,    Joint::Tibia);

  msg.coxa_angle_deg    [3] = gait_ctrl_output.get_angle_deg(Leg::RightFront,  Joint::Coxa);
  msg.femur_angle_deg   [3] = gait_ctrl_output.get_angle_deg(Leg::RightFront,  Joint::Femur);
  msg.tibia_angle_deg   [3] = gait_ctrl_output.get_angle_deg(Leg::RightFront,  Joint::Tibia);

  msg.coxa_angle_deg    [4] = gait_ctrl_output.get_angle_deg(Leg::RightMiddle, Joint::Coxa);
  msg.femur_angle_deg   [4] = gait_ctrl_output.get_angle_deg(Leg::RightMiddle, Joint::Femur);
  msg.tibia_angle_deg   [4] = gait_ctrl_output.get_angle_deg(Leg::RightMiddle, Joint::Tibia);

  msg.coxa_angle_deg    [5] = gait_ctrl_output.get_angle_deg(Leg::RightBack,   Joint::Coxa);
  msg.femur_angle_deg   [5] = gait_ctrl_output.get_angle_deg(Leg::RightBack,   Joint::Femur);
  msg.tibia_angle_deg   [5] = gait_ctrl_output.get_angle_deg(Leg::RightBack,   Joint::Tibia);

  return msg;
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */
