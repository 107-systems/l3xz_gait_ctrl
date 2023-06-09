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

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::gait::state
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class SitDown : public StateBase
{
public:
  SitDown(rclcpp::Logger const logger, rclcpp::Clock::SharedPtr const clock) : StateBase(logger, clock) { }
  virtual ~SitDown() { }
  virtual void onEnter() override;
  virtual void onExit() override;
  virtual std::tuple<StateBase *, ControllerOutput> update(kinematic::Engine const & engine, ControllerInput const & input, ControllerOutput const & prev_output) override;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::gait::state */