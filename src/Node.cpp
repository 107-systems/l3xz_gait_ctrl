/**
 * Copyright (c) 2023 LXRobotics GmbH.
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
, _kinematic_engine{get_logger()}
, _gait_ctrl{get_logger(), get_clock()}
, _gait_ctrl_input{}
, _gait_ctrl_output{}
, _node_start{std::chrono::steady_clock::now()}
, _prev_ctrl_loop_timepoint{std::chrono::steady_clock::now()}
{
  init_heartbeat();
  init_sub();
  init_pub();

  _ctrl_loop_timer = create_wall_timer(CTRL_LOOP_RATE, [this]() { this->ctrl_loop(); });

  RCLCPP_INFO(get_logger(), "%s init complete.", get_name());
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

void Node::init_heartbeat()
{
  std::stringstream heartbeat_topic;
  heartbeat_topic << "/l3xz/" << get_name() << "/heartbeat";

  _heartbeat_pub = heartbeat::Publisher::create(*this, heartbeat_topic.str(), HEARTBEAT_LOOP_RATE);
}

void Node::init_sub()
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
      std::stringstream angle_actual_sub_topic;
      angle_actual_sub_topic << "/l3xz/leg/" << LegToStr(leg) << "/" << JointToStr(joint) << "/angle/actual";

      _angle_actual_sub[make_key(leg, joint)] = create_subscription<std_msgs::msg::Float32>(
        angle_actual_sub_topic.str(),
        1,
        [this, leg, joint](std_msgs::msg::Float32::SharedPtr const msg) { _gait_ctrl_input.set_angle_deg(leg, joint, msg->data * 180.0f / M_PI); });
    }

  for (auto leg : LEG_LIST)
  {
    std::stringstream tibia_endpoint_switch_sub_topic;
    tibia_endpoint_switch_sub_topic << "/l3xz/leg/" << LegToStr(leg) << "/tibia_endpoint_switch/actual";

    _tibia_endpoint_switch_sub[leg] = create_subscription<std_msgs::msg::Bool>(
      tibia_endpoint_switch_sub_topic.str(),
      1,
      [this, leg](std_msgs::msg::Bool::SharedPtr const msg) { _gait_ctrl_input.set_is_tibia_endpoint_switch_pressed(leg, msg->data); });
  }
}

void Node::init_pub()
{
  for (auto leg : LEG_LIST)
    for (auto joint : JOINT_LIST)
    {
      std::stringstream angle_target_pub_topic;
      angle_target_pub_topic << "/l3xz/leg/" << LegToStr(leg) << "/" << JointToStr(joint) << "/angle/target";

      _angle_target_pub[make_key(leg, joint)] = create_publisher<std_msgs::msg::Float32>(angle_target_pub_topic.str(), 1);
    }

  _odom_pub = create_publisher<nav_msgs::msg::Odometry>("/l3xz/odom", 1);
}

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


  /* Calculate the joint angle set points based on the
   * current joint position, kinematic constraints and
   * controller input.
   */
  _gait_ctrl_output = _gait_ctrl.update(_kinematic_engine, _gait_ctrl_input, _gait_ctrl_output);

  /* Publish the target angle for all joints of all legs.
   */
  for (auto leg : LEG_LIST)
    for (auto joint : JOINT_LIST)
    {
      std_msgs::msg::Float32 angle_target_msg;
      angle_target_msg.data = _gait_ctrl_output.get_angle_deg(leg, joint) * M_PI / 180.0f;
      _angle_target_pub[make_key(leg, joint)]->publish(angle_target_msg);
    }

  /* Calculate the odometry from the kinematic engine
   * and publish a nav_msgs/Odometry message which is
   * required by the mapping software.
   */
  {
    nav_msgs::msg::Odometry msg;

    /* Setup header. */
    msg.header = std_msgs::msg::Header();
    msg.header.stamp = builtin_interfaces::msg::Time();
    msg.header.frame_id = "odom";

    /* Setup child frame id. */
    msg.child_frame_id = "base_link";

    /* Calculate and set-up odmetry pose. */
    msg.pose = geometry_msgs::msg::PoseWithCovariance(); // TODO

    /* Calculate and set-up odometry twist. */
    msg.twist = geometry_msgs::msg::TwistWithCovariance(); // TODO

    /* Actually publish the message. */
    _odom_pub->publish(msg);
  }
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */
