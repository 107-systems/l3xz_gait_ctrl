/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_gait_ctrl/graphs/contributors.
 */

#ifndef GAIT_CONTROLLER_STATE_OUTPUT_H_
#define GAIT_CONTROLLER_STATE_OUTPUT_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <map>
#include <string>

#include <l3xz/l3xz.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::gait
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class ControllerOutput
{
public:
  ControllerOutput(float const left_front_coxa_angle_target,
                   float const left_front_femur_angle_target,
                   float const left_front_tibia_angle_target,
                   float const left_middle_coxa_angle_target,
                   float const left_middle_femur_angle_target,
                   float const left_middle_tibia_angle_target,
                   float const left_back_coxa_angle_target,
                   float const left_back_femur_angle_target,
                   float const left_back_tibia_angle_target,
                   float const right_front_coxa_angle_target,
                   float const right_front_femur_angle_target,
                   float const right_front_tibia_angle_target,
                   float const right_middle_coxa_angle_target,
                   float const right_middle_femur_angle_target,
                   float const right_middle_tibia_angle_target,
                   float const right_back_coxa_angle_target,
                   float const right_back_femur_angle_target,
                   float const right_back_tibia_angle_target);

  ControllerOutput() : ControllerOutput(180.0f, 0.0f, 0.0f,
                                        180.0f, 0.0f, 0.0f,
                                        180.0f, 0.0f, 0.0f,
                                        180.0f, 0.0f, 0.0f,
                                        180.0f, 0.0f, 0.0f,
                                        180.0f, 0.0f, 0.0f) { }

  float get_angle_deg(Leg const leg, Joint const joint) const;
  void  set_angle_deg(Leg const leg, Joint const joint, float const angle_deg);

  std::string toStr() const;

private:
  std::map<LegJointKey, float> _angle_position_map;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::gait */

#endif /* GAIT_CONTROLLER_STATE_OUTPUT_H_ */
