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
: rclcpp::Node("l3xz::ctrl")
{
  _output_pub = create_publisher<l3xz_ctrl::msg::Output>
    ("/l3xz/ctrl/output", 10);

  _input_sub = create_subscription<l3xz_ctrl::msg::Input>
    ("/l3xz/ctrl/input", 10, [this](l3xz_ctrl::msg::Input const & msg) { this->onInputUpdate(msg); });

  _cmd_vel_sub = create_subscription<geometry_msgs::msg::Twist>
    ("/l3xz/cmd_vel", 10, [this](geometry_msgs::msg::Twist const & msg) { this->onCmdVelUpdate(msg); });

  _ctrl_loop_timer = create_wall_timer(std::chrono::milliseconds(50), [this]() { this->onCtrlLoopTimerEvent(); });
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

void Node::onInputUpdate(l3xz_ctrl::msg::Input const & /* msg */)
{
  /* TODO */
}

void Node::onCmdVelUpdate(geometry_msgs::msg::Twist const & /* msg */)
{
  /* TODO */
}

void Node::onCtrlLoopTimerEvent()
{
  /* TODO */
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* L3XZ_CTRL_NODE_H_ */
