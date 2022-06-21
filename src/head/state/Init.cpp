/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_ctrl/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <l3xz_ctrl/head/state/Init.h>

#include <cmath>

#include <l3xz_ctrl/head/state/Teleop.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::head::state
{

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static float constexpr INITIAL_PAN_ANGLE_DEG  = 0.0f;
static float constexpr INITIAL_TILT_ANGLE_DEG = 0.0f;

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void Init::onEnter()
{

}

void Init::onExit()
{

}

std::tuple<StateBase *, ControllerOutput> Init::update(ControllerInput const & input, ControllerOutput const & prev_output)
{
  /* Check if we have reached the initial tilt angle. */
  float const tilt_angle_error = fabs(INITIAL_TILT_ANGLE_DEG - input._tilt_angle_actual);
  bool  const tilt_is_initial_angle_reached = tilt_angle_error < INITIAL_ANGLE_EPSILON;

  /* Check if we have reached the initial pan angle. */
  float const pan_angle_error = fabs(INITIAL_PAN_ANGLE_DEG - input._pan_angle_actual);
  bool  const pan_is_initial_angle_reached = pan_angle_error < INITIAL_ANGLE_EPSILON;

  /* If we have reached the initial angles transition
   * into the active state of the header controller.
   */
  if (tilt_is_initial_angle_reached && pan_is_initial_angle_reached)
    return std::tuple(new Teleop(), prev_output);

  return std::tuple(this, ControllerOutput(INITIAL_PAN_ANGLE_DEG, INITIAL_TILT_ANGLE_DEG));
}
 
/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::head::state */
