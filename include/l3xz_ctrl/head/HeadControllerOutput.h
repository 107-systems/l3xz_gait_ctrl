/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_ctrl/graphs/contributors.
 */

#ifndef HEAD_CONTROLLER_OUTPUT_H_
#define HEAD_CONTROLLER_OUTPUT_H_

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::head
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class ControllerOutput
{
public:
  ControllerOutput(float const pan_angle,
                   float const tilt_angle)
  : _pan_angle {pan_angle}
  , _tilt_angle{tilt_angle}
  { }

  ControllerOutput() : ControllerOutput(0.0f, 0.0f) { }

  inline float pan_angle() const { return _pan_angle; }
  inline float tilt_angle() const { return _tilt_angle; }

private:
  float _pan_angle,
        _tilt_angle;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::head */

#endif /* HEAD_CONTROLLER_OUTPUT_H_ */
