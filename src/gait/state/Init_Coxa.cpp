/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_gait_ctrl/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <l3xz_gait_ctrl/gait/state/Init_Coxa.h>

#include <l3xz_gait_ctrl/gait/state/Sitting.h>

#include <l3xz/l3xz.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::gait::state
{

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void Init_Coxa::onEnter(ControllerInput const & /* input */)
{
  RCLCPP_INFO(_logger, "Init_Coxa ENTER");
}

void Init_Coxa::onExit()
{
  RCLCPP_INFO(_logger, "Init_Coxa EXIT");
}

std::tuple<StateBase *, ControllerOutput> Init_Coxa::update(kinematic::Engine const & /* engine */, ControllerInput const & input, ControllerOutput const & prev_output)
{
  ControllerOutput next_output = prev_output;

  bool all_target_angles_reached = true;
  std::stringstream leg_not_reached_list;
  for (auto leg : LEG_LIST)
  {
    float const coxa_deg_actual = input.get_angle_deg(leg, Joint::Coxa);
    float const coxa_deg_target = 180.0f;

    next_output.set_angle_deg(leg, Joint::Coxa,  coxa_deg_target);

    float const coxa_angle_error = fabs(coxa_deg_actual - coxa_deg_target);
    bool  const coxa_is_initial_angle_reached = coxa_angle_error < 2.5f;

    if (!coxa_is_initial_angle_reached)
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
                         "l3xz::gait::state::Init_Coxa::update: target angle not reached for [ %s]", leg_not_reached_list.str().c_str());

    return std::tuple(this, next_output);
  }

  return std::tuple(new Sitting(_logger, _clock), next_output);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::gait::state */
