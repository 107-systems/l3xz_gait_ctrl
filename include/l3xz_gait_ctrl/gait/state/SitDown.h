/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_gait_ctrl/graphs/contributors.
 */

#pragma once

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include "PositionTrajectory.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::gait::state
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class SitDown : public PositionTrajectory
{
public:
  static Point3D constexpr START = std::make_tuple(-210.0f, 0.0f, -260.0f);
  static Point3D constexpr STOP  = std::make_tuple(-200.0f, 0.0f, -135.0f);

  SitDown(rclcpp::Logger const logger, rclcpp::Clock::SharedPtr const clock)
  : PositionTrajectory(logger,
                       clock,
                       calc_point_vector(START, STOP, 20),
                       PositionTrajectory::NextState::Sitting)
  { }
  virtual ~SitDown()
  { }
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::gait::state */
