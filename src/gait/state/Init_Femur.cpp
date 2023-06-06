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

void Init_Femur::onEnter()
{
  RCLCPP_INFO(_logger, "Init_Femur ENTER");
}

void Init_Femur::onExit()
{
  RCLCPP_INFO(_logger, "Init_Femur EXIT");
}

std::tuple<StateBase *, ControllerOutput> Init_Femur::update(kinematic::Engine const & /* engine */, ControllerInput const & input, ControllerOutput const & prev_output)
{
  ControllerOutput next_output = prev_output;

  /* This is the first init state. Prevent sudden movement on
   * femur/tibia by capturing their current state and storing it
   * within next_output.
   */
  for (auto leg : LEG_LIST)
  {
    next_output.set_angle_deg(leg, Joint::Coxa, input.get_angle_deg(leg, Joint::Coxa));
    next_output.set_angle_deg(leg, Joint::Tibia, input.get_angle_deg(leg, Joint::Tibia));
  }

  bool all_target_angles_reached = true;
  std::stringstream leg_not_reached_list;
  for (auto leg : LEG_LIST)
  {
    float const femur_deg_actual = input.get_angle_deg(leg, Joint::Femur);
    float const femur_deg_target = 0.0f;

    next_output.set_angle_deg(leg, Joint::Femur, femur_deg_target);

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
