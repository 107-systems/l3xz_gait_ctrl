/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_gait_ctrl/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <l3xz_gait_ctrl/gait/state/Standing.h>

#include <l3xz_gait_ctrl/gait/state/SitDown.h>
#include <l3xz_gait_ctrl/gait/state/Turning.h>
#include <l3xz_gait_ctrl/gait/state/Walking.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::gait::state
{

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void Standing::onEnter(ControllerInput const & /* input */)
{
  RCLCPP_INFO(_logger, "Standing ENTER");
}

void Standing::onExit()
{
  RCLCPP_INFO(_logger, "Standing EXIT");
}

std::tuple<StateBase *, ControllerOutput> Standing::update(kinematic::Engine const & /* engine */, ControllerInput const & input, ControllerOutput const & prev_output)
{
  ControllerOutput next_output = prev_output;

  if (input.get_request_down())
    return std::tuple(new SitDown(_logger, _clock), next_output);

  if (std::abs(input.teleop_linear_velocity_x()) > 0.2f)
    return std::tuple(new Walking(_logger, _clock, input.teleop_linear_velocity_x() > 0), next_output);

  if (std::abs(input.teleop_angular_velocity_z()) > 0.2f)
    return std::tuple(new Turning(_logger, _clock, input.teleop_angular_velocity_z() > 0), next_output);

  return std::tuple(this, next_output);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::gait::state */
