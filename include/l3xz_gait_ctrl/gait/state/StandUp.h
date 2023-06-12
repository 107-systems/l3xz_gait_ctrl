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

class StandUp : public PositionTrajectory
{
public:
  static Point3D constexpr START = std::make_tuple(-200.0f, 0.0f, -140.0f);
  static Point3D constexpr STOP  = std::make_tuple(-210.0f, 0.0f, -260.0f);

  StandUp(rclcpp::Logger const logger, rclcpp::Clock::SharedPtr const clock)
  : PositionTrajectory(logger,
                       clock,
                       calc_point_vector(START, STOP, 40),
                       PositionTrajectory::NextState::Standing)
  { }
  virtual ~StandUp()
  { }
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::gait::state */
