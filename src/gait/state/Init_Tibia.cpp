/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_gait_ctrl/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <l3xz_gait_ctrl/gait/state/Init_Tibia.h>

#include <l3xz_gait_ctrl/gait/state/Init_Coxa.h>

#include <l3xz_gait_ctrl/const/LegList.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::gait::state
{

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void Init_Tibia::onEnter()
{
  RCLCPP_INFO(_logger, "Init_Tibia ENTER");
}

void Init_Tibia::onExit()
{
  RCLCPP_INFO(_logger, "Init_Tibia EXIT");
}

std::tuple<StateBase *, ControllerOutput> Init_Tibia::update(kinematic::Engine const & /* engine */, ControllerInput const & input, ControllerOutput const & prev_output)
{
  ControllerOutput next_output = prev_output;

  bool all_target_angles_reached = true;
  std::stringstream leg_not_reached_list;
  for (auto leg : LEG_LIST)
  {
    float const tibia_deg_actual = input.get_angle_deg(leg, Joint::Tibia);
    float const tibia_deg_target = 0.0f;

    next_output.set_angle_deg(leg, Joint::Tibia, tibia_deg_target);

    float const tibia_angle_error = fabs(tibia_deg_actual - tibia_deg_target);
    bool const tibia_is_initial_angle_reached = tibia_angle_error < 5.0f;

    if (!tibia_is_initial_angle_reached)
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
                         "l3xz::gait::state::Init_Tibia::update: target angle not reached for [ %s]", leg_not_reached_list.str().c_str());

    return std::tuple(this, next_output);
  }

  return std::tuple(new Init_Coxa(_logger, _clock), next_output);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::gait::state */
