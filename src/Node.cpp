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
, _gait_ctrl{}
, _gait_ctrl_input{}
, _gait_ctrl_output{}
, _prev_ctrl_loop_timepoint{std::chrono::steady_clock::now()}
{
  _robot_sub = create_subscription<geometry_msgs::msg::Twist>
    ("/l3xz/cmd_vel_robot", 10, [this](geometry_msgs::msg::Twist::SharedPtr const msg) { updateGaitControllerInput(msg);});

  for (auto leg : LEG_LIST)
    for (auto joint : JOINT_LIST)
    {
      std::stringstream topic;
      topic << "/l3xz/leg/" << LegToStr(leg) << "/" << JointToStr(joint) << "/angle/actual";
      _angle_actual_sub[make_key(leg, joint)] = create_subscription<std_msgs::msg::Float32>
        (topic.str(),
         1,
         [this, leg, joint](std_msgs::msg::Float32::SharedPtr const msg) { _gait_ctrl_input.set_angle_deg(leg, joint, msg->data); }
         );
    }

  _leg_angle_pub = create_publisher<l3xz_gait_ctrl::msg::LegAngle>
    ("/l3xz/ctrl/leg/angle/target", 10);

  _leg_angle_sub = create_subscription<l3xz_gait_ctrl::msg::LegAngle>
    ("/l3xz/ctrl/leg/angle/actual", 10, [this](l3xz_gait_ctrl::msg::LegAngle::SharedPtr const msg) { updateGaitControllerInput(msg); });

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

  l3xz_gait_ctrl::msg::LegAngle const leg_msg = createOutputMessage(_gait_ctrl_output);
  _leg_angle_pub->publish(leg_msg);
}

void Node::updateGaitControllerInput(l3xz_gait_ctrl::msg::LegAngle::SharedPtr const msg)
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

void Node::updateGaitControllerInput(geometry_msgs::msg::Twist::SharedPtr const msg)
{
  _gait_ctrl_input.set_teleop_linear_velocity_x (msg->linear.x);
  _gait_ctrl_input.set_teleop_angular_velocity_z(msg->angular.z);
}

l3xz_gait_ctrl::msg::LegAngle Node::createOutputMessage(gait::ControllerOutput const & gait_ctrl_output)
{
  l3xz_gait_ctrl::msg::LegAngle msg;

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
