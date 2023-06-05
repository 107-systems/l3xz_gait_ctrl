/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_gait_ctrl/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <l3xz_gait_ctrl/gait/state/Init.h>

#include <l3xz_gait_ctrl/gait/state/StandUp.h>

#include <l3xz_gait_ctrl/const/LegList.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::gait::state
{

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void Init::onEnter()
{
  RCLCPP_INFO(_logger, "Init ENTER");
}

void Init::onExit()
{
  RCLCPP_INFO(_logger, "Init EXIT");
}

std::tuple<StateBase *, ControllerOutput> Init::update(kinematic::Engine const & /* engine */, ControllerInput const & input, ControllerOutput const & prev_output)
{
  ControllerOutput next_output = prev_output;

  bool all_target_angles_reached = true;
  for (auto leg : LEG_LIST)
  {
    float const coxa_deg_actual = input.get_angle_deg(leg, Joint::Coxa);
    float const coxa_deg_target = 180.0f;

    float const femur_deg_actual = input.get_angle_deg(leg, Joint::Femur);
    float const femur_deg_target = 0.0f;

    float const tibia_deg_actual = input.get_angle_deg(leg, Joint::Tibia);
    float const tibia_deg_target = 0.0f;


    /* Set output to the angle actuators. */
    next_output.set_angle_deg(leg, Joint::Coxa,  coxa_deg_target);
    next_output.set_angle_deg(leg, Joint::Femur, femur_deg_target);
    next_output.set_angle_deg(leg, Joint::Tibia, tibia_deg_target);

    /* Check if target angles have been reached. */
    {
      float const coxa_angle_error = fabs(coxa_deg_actual - coxa_deg_target);
      bool  const coxa_is_initial_angle_reached = coxa_angle_error < 2.5f;

      if (!coxa_is_initial_angle_reached) {
        RCLCPP_INFO_THROTTLE(_logger,
                             *_clock,
                             1000,
                             "l3xz::gait::state::Init::update: %s coxa target angle not reached", LegToStr(leg).c_str());
        all_target_angles_reached = false;
      }
    }

    {
      float const femur_angle_error = fabs(femur_deg_actual - femur_deg_target);
      bool  const femur_is_initial_angle_reached = femur_angle_error < 5.0f;

      if (!femur_is_initial_angle_reached) {
        RCLCPP_INFO_THROTTLE(_logger,
                             *_clock,
                             1000,
                             "l3xz::gait::state::Init::update: %s femur target angle not reached", LegToStr(leg).c_str());
        all_target_angles_reached = false;
      }
    }

    {
      float const tibia_angle_error = fabs(tibia_deg_actual - tibia_deg_target);
      bool  const tibia_is_initial_angle_reached = tibia_angle_error < 5.0f;

      if (!tibia_is_initial_angle_reached) {
        RCLCPP_INFO_THROTTLE(_logger,
                             *_clock,
                             1000,
                             "l3xz::gait::state::Init::update: %s tibia target angle not reached", LegToStr(leg).c_str());
        all_target_angles_reached = false;
      }
    }
  }

  if (!all_target_angles_reached)
    return std::tuple(this, next_output);

  /* If the robot control joystick is moved after
   * initialization is complete then we shall transition
   * into the stand-up state.
   */
  if (std::abs(input.teleop_linear_velocity_x()) > 0.4f ||
      std::abs(input.teleop_angular_velocity_z()) > 0.4f)
    return std::tuple(new StandUp(_logger, _clock), next_output);

  return std::tuple(this, next_output);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::gait::state */
