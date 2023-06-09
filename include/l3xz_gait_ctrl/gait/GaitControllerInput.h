/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_gait_ctrl/graphs/contributors.
 */

#ifndef GAIT_CONTROLLER_STATE_INPUT_H_
#define GAIT_CONTROLLER_STATE_INPUT_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <map>

#include <l3xz_gait_ctrl/types/LegJoint.h>

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

  ControllerInput() : ControllerInput(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f) { }


  inline float teleop_linear_velocity_x () const { return _teleop_linear_velocity_x; }
  inline float teleop_angular_velocity_z() const { return _teleop_angular_velocity_z; }

  inline void set_teleop_linear_velocity_x (float const val) { _teleop_linear_velocity_x  = val; }
  inline void set_teleop_angular_velocity_z(float const val) { _teleop_angular_velocity_z = val; }


  float get_angle_deg(Leg const leg, Joint const joint) const;
  void  set_angle_deg(Leg const leg, Joint const joint, float const val);

  bool is_tibia_endpoint_switch_pressed(Leg const leg) const{
    return _is_tibia_endpoint_switch_pressed_map.at(leg);
  }
  void set_is_tibia_endpoint_switch_pressed(Leg const leg, bool const is_pressed) {
    _is_tibia_endpoint_switch_pressed_map[leg] = is_pressed;
  }

  bool get_request_up() const {
    return _request_up;
  }
  void set_request_up(bool const request_up) {
    _request_up = request_up;
  }

  bool get_request_down() const {
    return _request_down;
  }
  void set_request_down(bool const request_down) {
    _request_down = request_down;
  }

private:
  float _teleop_linear_velocity_x,
        _teleop_angular_velocity_z;
  std::map<LegJointKey, float> _angle_position_map;
  std::map<Leg, bool> _is_tibia_endpoint_switch_pressed_map;
  bool _request_up, _request_down;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::gait */

#endif /* GAIT_CONTROLLER_STATE_INPUT_H_ */
