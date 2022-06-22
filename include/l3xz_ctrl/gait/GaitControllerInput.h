/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

#ifndef GAIT_CONTROLLER_STATE_INPUT_H_
#define GAIT_CONTROLLER_STATE_INPUT_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <map>

#include <l3xz_ctrl/types/LegJoint.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::gait
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class ControllerInput
{
public:
  ControllerInput(float const teleop_linear_velocity_x,
                  float const teleop_linear_velocity_y,
                  float const teleop_angular_velocity_z,    
                  float const left_front_coxa_angle_actual,
                  float const left_front_femur_angle_actual,
                  float const left_front_tibia_angle_actual,
                  float const left_middle_coxa_angle_actual,
                  float const left_middle_femur_angle_actual,
                  float const left_middle_tibia_angle_actual,
                  float const left_back_coxa_angle_actual,
                  float const left_back_femur_angle_actual,
                  float const left_back_tibia_angle_actual,
                  float const right_front_coxa_angle_actual,
                  float const right_front_femur_angle_actual,
                  float const right_front_tibia_angle_actual,
                  float const right_middle_coxa_angle_actual,
                  float const right_middle_femur_angle_actual,
                  float const right_middle_tibia_angle_actual,
                  float const right_back_coxa_angle_actual,
                  float const right_back_femur_angle_actual,
                  float const right_back_tibia_angle_actual);

  ControllerInput() : ControllerInput(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f) { }


  inline float teleop_linear_velocity_x () const { return _teleop_linear_velocity_x; }
  inline float teleop_linear_velocity_y () const { return _teleop_linear_velocity_y; }
  inline float teleop_angular_velocity_z() const { return _teleop_angular_velocity_z; }

  inline void set_teleop_linear_velocity_x (float const val) { _teleop_linear_velocity_x  = val; }
  inline void set_teleop_linear_velocity_y (float const val) { _teleop_linear_velocity_y  = val; }
  inline void set_teleop_angular_velocity_z(float const val) { _teleop_angular_velocity_z = val; }


  float get_angle_deg(Leg const leg, Joint const joint) const;

private:
  float _teleop_linear_velocity_x,
        _teleop_linear_velocity_y,
        _teleop_angular_velocity_z;
  std::map<LegJointKey, float> _angle_position_map;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::gait */

#endif /* GAIT_CONTROLLER_STATE_INPUT_H_ */
