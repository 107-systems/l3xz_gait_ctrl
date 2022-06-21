/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

#ifndef HEAD_CONTROLLER_INPUT_H_
#define HEAD_CONTROLLER_INPUT_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <l3xz_ctrl/types/TeleopCommandData.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::head
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class ControllerInput
{
public:
  ControllerInput(TeleopCommandData const teleop_cmd,
                  double const pan_angle_actual,
                  double const tilt_angle_actual)
  : _teleop_cmd       {teleop_cmd}
  , _pan_angle_actual {pan_angle_actual}
  , _tilt_angle_actual{tilt_angle_actual}
  { }


  TeleopCommandData const _teleop_cmd;
  double const _pan_angle_actual;
  double const _tilt_angle_actual;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::head */

#endif /* HEAD_CONTROLLER_INPUT_H_ */
