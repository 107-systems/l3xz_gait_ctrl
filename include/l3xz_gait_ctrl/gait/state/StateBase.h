/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_gait_ctrl/graphs/contributors.
 */

#ifndef GAIT_CONTROLLER_STATE_H_
#define GAIT_CONTROLLER_STATE_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <rclcpp/rclcpp.hpp>

#include <l3xz_gait_ctrl/kinematic/Engine.h>

#include "../GaitControllerInput.h"
#include "../GaitControllerOutput.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::gait::state
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class StateBase
{
public:
  StateBase(rclcpp::Logger const logger, rclcpp::Clock::SharedPtr const clock) : _logger{logger}, _clock{clock} { }
  virtual ~StateBase() { }
  virtual void onEnter() { }
  virtual void onExit() { }
  virtual std::tuple<StateBase *, ControllerOutput> update(kinematic::Engine const & engine, ControllerInput const & input, ControllerOutput const & prev_output) = 0;

protected:
  rclcpp::Logger const _logger;
  rclcpp::Clock::SharedPtr const _clock;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::gait::state */

#endif /* GAIT_CONTROLLER_STATE_H_ */
