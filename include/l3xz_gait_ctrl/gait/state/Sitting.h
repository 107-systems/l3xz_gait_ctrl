/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_gait_ctrl/graphs/contributors.
 */

#pragma once

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include "StateBase.h"

#include "StandUp.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::gait::state
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class Sitting : public StateBase
{
public:
  Sitting(rclcpp::Logger const logger, rclcpp::Clock::SharedPtr const clock) : StateBase(logger, clock) { }
  virtual ~Sitting() { }
  virtual void onEnter(ControllerInput const & /* input */) override
  {
    RCLCPP_INFO(_logger, "Sitting ENTER");
  }
  virtual void onExit() override
  {
    RCLCPP_INFO(_logger, "Sitting EXIT");
  }
  virtual std::tuple<StateBase *, ControllerOutput> update(kinematic::Engine const & /* engine */, ControllerInput const & input, ControllerOutput const & prev_output) override
  {
    ControllerOutput next_output = prev_output;

    if (input.get_request_up())
      return std::tuple(new StandUp(_logger, _clock), next_output);
    else
      return std::tuple(this, next_output);
  }
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::gait::state */
