/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_gait_ctrl/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <l3xz_gait_ctrl/gait/state/Init_Femur.h>

#include <l3xz_gait_ctrl/gait/state/Init_Tibia.h>

#include <l3xz_gait_ctrl/const/LegList.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::gait::state
{

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void Init_Femur::onEnter(ControllerInput const & input)
{
  RCLCPP_INFO(_logger, "Init_Femur ENTER");

  /* This is the very first init state so we capture the initial angles
   * of both coxa and tibia to provide a drifting away while the other
   * joints are initialized.
   */
  for (auto leg : LEG_LIST)
  {
    _coxa_initial_angle_deg_map[leg] = input.get_angle_deg(leg, Joint::Coxa);
    _tibia_initial_angle_deg_map[leg] = input.get_angle_deg(leg, Joint::Tibia);
  }
}

void Init_Femur::onExit()
{
  RCLCPP_INFO(_logger, "Init_Femur EXIT");
}

std::tuple<StateBase *, ControllerOutput> Init_Femur::update(kinematic::Engine const & /* engine */, ControllerInput const & input, ControllerOutput const & prev_output)
{
  ControllerOutput next_output = prev_output;

  bool all_target_angles_reached = true;
  std::stringstream leg_not_reached_list;
  for (auto leg : LEG_LIST)
  {
    float const femur_deg_actual = input.get_angle_deg(leg, Joint::Femur);
    float const femur_deg_target = 0.0f;

    next_output.set_angle_deg(leg, Joint::Coxa,  _coxa_initial_angle_deg_map.at(leg));
    next_output.set_angle_deg(leg, Joint::Femur, femur_deg_target);
    next_output.set_angle_deg(leg, Joint::Tibia, _tibia_initial_angle_deg_map.at(leg));

    float const femur_angle_error = fabs(femur_deg_actual - femur_deg_target);
    bool  const femur_is_initial_angle_reached = femur_angle_error < 5.0f;

    if (!femur_is_initial_angle_reached)
    {
      leg_not_reached_list << LegToStr(leg) << " ";
      all_target_angles_reached = false;
    }
  }

  if (!all_target_angles_reached)
  {
    RCLCPP_INFO_THROTTLE(_logger,
                         *_clock,
                         1000,
                         "l3xz::gait::state::Init_Femur::update: target angle not reached for [ %s]", leg_not_reached_list.str().c_str());

    return std::tuple(this, next_output);
  }


  return std::tuple(new Init_Tibia(_logger, _clock), next_output);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::gait::state */
