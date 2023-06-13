/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_gait_ctrl/graphs/contributors.
 */

#ifndef STANDING_STATE_H_
#define STANDING_STATE_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include "StateBase.h"

#include "SitDown.h"
#include "Turning.h"
#include "Walking.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::gait::state
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class Standing : public StateBase
{
public:
  Standing(rclcpp::Logger const logger, rclcpp::Clock::SharedPtr const clock) : StateBase(logger, clock) { }
  virtual ~Standing() { }
  virtual void onEnter(ControllerInput const & /* input */) override
  {
    RCLCPP_INFO(_logger, "Standing ENTER");
  }
  virtual void onExit() override
  {
    RCLCPP_INFO(_logger, "Standing EXIT");
  }
  virtual std::tuple<StateBase *, ControllerOutput> update(kinematic::Engine const & /* engine */, ControllerInput const & input, ControllerOutput const & prev_output) override
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
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::gait::state */

#endif /* STANDING_STATE_H_ */
