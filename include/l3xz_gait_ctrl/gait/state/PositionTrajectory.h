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

#include <l3xz_gait_ctrl/types/Point3D.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::gait::state
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class PositionTrajectory : public StateBase
{
public:
  enum class NextState { Sitting, Standing };
  PositionTrajectory(rclcpp::Logger const logger, rclcpp::Clock::SharedPtr const clock, PointVector const & point_vect, NextState const next_state);
  virtual ~PositionTrajectory() { }
  virtual void onEnter(ControllerInput const & input) override;
  virtual void onExit() override;
  virtual std::tuple<StateBase *, ControllerOutput> update(kinematic::Engine const & engine, ControllerInput const & input, ControllerOutput const & prev_output) override;

private:
  PointVector const _point_vect;
  PointVector::const_iterator _citer;
  NextState const _next_state;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::gait::state */
