/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_gait_ctrl/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <l3xz_gait_ctrl/gait/GaitControllerInput.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::gait
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

ControllerInput::ControllerInput(float const teleop_linear_velocity_x,
                                 float const teleop_angular_velocity_z,
                                 float const left_front_coxa_angle_actual,
                                 float const left_front_femur_angle_actual,
                                 float const left_front_tibia_angle_actual,
                                 float const left_middle_coxa_angle_actual,
                                 float const left_middle_femur_angle_actual,
                                 float const left_middle_tibia_angle_actual,
                                 float const left_back_coxa_angle_actual,
                                 float const left_back_femur_angle_actual,
                                 float const left_back_tibia_angle_actual,
                                 float const right_front_coxa_angle_actual,
                                 float const right_front_femur_angle_actual,
                                 float const right_front_tibia_angle_actual,
                                 float const right_middle_coxa_angle_actual,
                                 float const right_middle_femur_angle_actual,
                                 float const right_middle_tibia_angle_actual,
                                 float const right_back_coxa_angle_actual,
                                 float const right_back_femur_angle_actual,
                                 float const right_back_tibia_angle_actual)
: _teleop_linear_velocity_x{teleop_linear_velocity_x}
, _teleop_angular_velocity_z{teleop_angular_velocity_z}
{
  _angle_position_map[make_key(Leg::LeftFront,   Joint::Coxa)]  = left_front_coxa_angle_actual;
  _angle_position_map[make_key(Leg::LeftFront,   Joint::Femur)] = left_front_femur_angle_actual;
  _angle_position_map[make_key(Leg::LeftFront,   Joint::Tibia)] = left_front_tibia_angle_actual;

  _angle_position_map[make_key(Leg::LeftMiddle,  Joint::Coxa)]  = left_middle_coxa_angle_actual;
  _angle_position_map[make_key(Leg::LeftMiddle,  Joint::Femur)] = left_middle_femur_angle_actual;
  _angle_position_map[make_key(Leg::LeftMiddle,  Joint::Tibia)] = left_middle_tibia_angle_actual;

  _angle_position_map[make_key(Leg::LeftBack,    Joint::Coxa)]  = left_back_coxa_angle_actual;
  _angle_position_map[make_key(Leg::LeftBack,    Joint::Femur)] = left_back_femur_angle_actual;
  _angle_position_map[make_key(Leg::LeftBack,    Joint::Tibia)] = left_back_tibia_angle_actual;

  _angle_position_map[make_key(Leg::RightFront,  Joint::Coxa)]  = right_front_coxa_angle_actual;
  _angle_position_map[make_key(Leg::RightFront,  Joint::Femur)] = right_front_femur_angle_actual;
  _angle_position_map[make_key(Leg::RightFront,  Joint::Tibia)] = right_front_tibia_angle_actual;

  _angle_position_map[make_key(Leg::RightMiddle, Joint::Coxa)]  = right_middle_coxa_angle_actual;
  _angle_position_map[make_key(Leg::RightMiddle, Joint::Femur)] = right_middle_femur_angle_actual;
  _angle_position_map[make_key(Leg::RightMiddle, Joint::Tibia)] = right_middle_tibia_angle_actual;

  _angle_position_map[make_key(Leg::RightBack,   Joint::Coxa)]  = right_back_coxa_angle_actual;
  _angle_position_map[make_key(Leg::RightBack,   Joint::Femur)] = right_back_femur_angle_actual;
  _angle_position_map[make_key(Leg::RightBack,   Joint::Tibia)] = right_back_tibia_angle_actual;

  _is_tibia_endpoint_switch_pressed_map[Leg::LeftFront]   = false;
  _is_tibia_endpoint_switch_pressed_map[Leg::LeftMiddle]  = false;
  _is_tibia_endpoint_switch_pressed_map[Leg::LeftBack]    = false;
  _is_tibia_endpoint_switch_pressed_map[Leg::RightFront]  = false;
  _is_tibia_endpoint_switch_pressed_map[Leg::RightMiddle] = false;
  _is_tibia_endpoint_switch_pressed_map[Leg::RightBack]   = false;
}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

float ControllerInput::get_angle_deg(Leg const leg, Joint const joint) const
{
  return _angle_position_map.at(make_key(leg, joint));
}

void ControllerInput::set_angle_deg(Leg const leg, Joint const joint, float const val)
{
  _angle_position_map[make_key(leg, joint)] = val;
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::gait */
