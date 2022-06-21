/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_ctrl/graphs/contributors.
 */

#ifndef HEAD_CONTROLLER_INPUT_H_
#define HEAD_CONTROLLER_INPUT_H_

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::head
{

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static float constexpr INITIAL_PAN_ANGLE_DEG  = 0.0f;
static float constexpr INITIAL_TILT_ANGLE_DEG = 0.0f;

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class ControllerInput
{
public:
  ControllerInput(float const pan_angular_velocity,
                  float const tilt_angular_velocity,
                  float const pan_angle,
                  float const tilt_angle)
  : _pan_angular_velocity {pan_angular_velocity}
  , _tilt_angular_velocity{tilt_angular_velocity}
  , _pan_angle            {pan_angle}
  , _tilt_angle           {tilt_angle}
  { }

  ControllerInput() : ControllerInput(0.0f, 0.0f, 0.0f, 0.0f)
  { }

  inline float pan_angular_velocity () const { return _pan_angular_velocity; }
  inline float tilt_angular_velocity() const { return _tilt_angular_velocity; }
  inline float pan_angle            () const { return _pan_angle; }
  inline float tilt_angle           () const { return _tilt_angle; }

  inline void set_pan_angular_velocity (float const val) { _pan_angular_velocity = val; }
  inline void set_tilt_angular_velocity(float const val) { _tilt_angular_velocity = val; }
  inline void set_pan_angle            (float const val) { _pan_angle = val; }
  inline void set_tilt_angle           (float const val) { _tilt_angle = val; }


private:
  float _pan_angular_velocity,
        _tilt_angular_velocity,
        _pan_angle,
        _tilt_angle;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::head */

#endif /* HEAD_CONTROLLER_INPUT_H_ */
