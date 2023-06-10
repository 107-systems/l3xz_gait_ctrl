/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_gait_ctrl/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <l3xz_gait_ctrl/gait/state/Sitting.h>

#include <l3xz_gait_ctrl/gait/state/StandUp.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::gait::state
{

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void Sitting::onEnter(ControllerInput const & /* input */)
{
  RCLCPP_INFO(_logger, "Sitting ENTER");
}

void Sitting::onExit()
{
  RCLCPP_INFO(_logger, "Sitting EXIT");
}

std::tuple<StateBase *, ControllerOutput> Sitting::update(kinematic::Engine const & /* engine */, ControllerInput const & input, ControllerOutput const & prev_output)
{
  ControllerOutput next_output = prev_output;

  if (input.get_request_up())
    return std::tuple(new StandUp(_logger, _clock), next_output);
  else
    return std::tuple(this, next_output);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::gait::state */
