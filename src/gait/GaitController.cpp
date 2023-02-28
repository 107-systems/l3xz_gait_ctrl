/**
 * Copyright (c) 2022 LXControllerics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_gait_ctrl/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <l3xz_gait_ctrl/gait/GaitController.h>

#include <l3xz_gait_ctrl/gait/state/Init.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::gait
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

Controller::Controller(rclcpp::Logger const logger, rclcpp::Clock::SharedPtr const clock)
: _logger{logger}
, _clock{clock}
, _robot_state{new state::Init(_logger, _clock)}
{
  _robot_state->onEnter();
}

Controller::~Controller()
{
  delete _robot_state;
}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

ControllerOutput Controller::update(kinematic::Engine const & engine, ControllerInput const & input, ControllerOutput const & prev_output)
{
  auto [next_robot_state, next_output] = _robot_state->update(engine, input, prev_output);
    
  if (next_robot_state != _robot_state)
  {
    _robot_state->onExit();

    delete _robot_state;
    _robot_state = next_robot_state;
    
    _robot_state->onEnter();
  }

  return next_output;
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::gait */