/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_gait_ctrl_gait_ctrl/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <l3xz_gait_ctrl/Node.h>

#include <l3xz_gait_ctrl/const/LegList.h>
#include <l3xz_gait_ctrl/const/JointList.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

Node::Node()
: rclcpp::Node("l3xz_gait_ctrl")
, _kinematic_engine{}
, _gait_ctrl{get_logger()}
, _gait_ctrl_input{}
, _gait_ctrl_output{}
, _prev_ctrl_loop_timepoint{std::chrono::steady_clock::now()}
{
  _robot_sub = create_subscription<geometry_msgs::msg::Twist>
    ("/l3xz/cmd_vel_robot",
     1,
     [this](geometry_msgs::msg::Twist::SharedPtr const msg)
     {
       _gait_ctrl_input.set_teleop_linear_velocity_x (msg->linear.x);
       _gait_ctrl_input.set_teleop_angular_velocity_z(msg->angular.z);
     });

  for (auto leg : LEG_LIST)
    for (auto joint : JOINT_LIST)
    {
      std::stringstream angle_actual_sub_topic, angle_target_pub_topic;
      angle_actual_sub_topic << "/l3xz/leg/" << LegToStr(leg) << "/" << JointToStr(joint) << "/angle/actual";
      angle_target_pub_topic << "/l3xz/leg/" << LegToStr(leg) << "/" << JointToStr(joint) << "/angle/target";

      _angle_actual_sub[make_key(leg, joint)] = create_subscription<std_msgs::msg::Float32>
        (angle_actual_sub_topic.str(),
         1,
         [this, leg, joint](std_msgs::msg::Float32::SharedPtr const msg) { _gait_ctrl_input.set_angle_deg(leg, joint, msg->data * 180.0f / M_PI); }
         );

      _angle_targed_pub[make_key(leg, joint)] = create_publisher<std_msgs::msg::Float32>(angle_target_pub_topic.str(), 1);
    }

  _ctrl_loop_timer = create_wall_timer
    (std::chrono::milliseconds(CTRL_LOOP_RATE.count()), [this]() { this->ctrl_loop(); });
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

void Node::ctrl_loop()
{
  auto const now = std::chrono::steady_clock::now();
  auto const ctrl_loop_rate = (now - _prev_ctrl_loop_timepoint);
  if (ctrl_loop_rate > (CTRL_LOOP_RATE + std::chrono::milliseconds(1)))
    RCLCPP_WARN_THROTTLE(get_logger(),
                         *get_clock(),
                         1000,
                         "ctrl_loop should be called every %ld ms, but is %ld ms instead",
                         CTRL_LOOP_RATE.count(),
                         std::chrono::duration_cast<std::chrono::milliseconds>(ctrl_loop_rate).count());
  _prev_ctrl_loop_timepoint = now;


  _gait_ctrl_output = _gait_ctrl.update(_kinematic_engine, _gait_ctrl_input, _gait_ctrl_output);


  for (auto leg : LEG_LIST)
    for (auto joint : JOINT_LIST)
    {
      /* Publish all target angles. */
      std_msgs::msg::Float32 angle_target_msg;
      angle_target_msg.data = _gait_ctrl_output.get_angle_deg(leg, joint) * M_PI / 180.0f;
      _angle_targed_pub[make_key(leg, joint)]->publish(angle_target_msg);
    }
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */