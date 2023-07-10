/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_gait_ctrl/graphs/contributors.
 */

#ifndef GAIT_CONTROLLER_H_
#define GAIT_CONTROLLER_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <rclcpp/rclcpp.hpp>

#include <l3xz_kinematic_engine/Engine.h>

#include "state/StateBase.h"
#include "GaitControllerInput.h"
#include "GaitControllerOutput.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::gait
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class Controller
{
public:
   Controller(rclcpp::Logger const logger, rclcpp::Clock::SharedPtr const clock);
  ~Controller();

  ControllerOutput update(kinematic::Engine const & engine, ControllerInput const & input, ControllerOutput const & prev_output);

private:
  rclcpp::Logger const _logger;
  rclcpp::Clock::SharedPtr const _clock;
  state::StateBase * _robot_state;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::gait */

#endif /* GAIT_CONTROLLER_H_ */
