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
  _cmd_vel_sub = create_subscription<geometry_msgs::msg::Twist>
    ("/l3xz/cmd_vel", 10, [this](geometry_msgs::msg::Twist const & msg) { this->onCmdVelUpdate(msg); });
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

void Node::onCmdVelUpdate(geometry_msgs::msg::Twist const & /* msg */)
{
  /* TODO */
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* L3XZ_CTRL_NODE_H_ */
