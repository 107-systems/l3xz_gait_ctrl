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

#include <geometry_msgs/msg/twist.hpp>

#include <l3xz_ctrl/msg/input.hpp>
#include <l3xz_ctrl/msg/output.hpp>

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
  rclcpp::Publisher<l3xz_ctrl::msg::Output>::SharedPtr _output_pub;
  rclcpp::Subscription<l3xz_ctrl::msg::Input>::SharedPtr _input_sub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_sub;
  rclcpp::TimerBase::SharedPtr _ctrl_loop_timer;

  void onInputUpdate(l3xz_ctrl::msg::Input const & msg);
  void onCmdVelUpdate(geometry_msgs::msg::Twist const & msg);
  void onCtrlLoopTimerEvent();
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */

#endif /* L3XZ_CTRL_NODE_H_ */
