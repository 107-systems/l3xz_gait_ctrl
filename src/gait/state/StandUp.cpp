/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_gait_ctrl/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <l3xz_gait_ctrl/gait/state/StandUp.h>

#include <l3xz_gait_ctrl/gait/state/Standing.h>
#include <l3xz_gait_ctrl/gait/state/Walking.h>

#include <l3xz_gait_ctrl/const/LegList.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::gait::state
{

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void StandUp::onEnter()
{
  RCLCPP_INFO(_logger, "StandUp ENTER");
}

void StandUp::onExit()
{
  RCLCPP_INFO(_logger, "StandUp EXIT");
}

std::tuple<StateBase *, ControllerOutput> StandUp::update(kinematic::Engine const & engine, ControllerInput const & input, ControllerOutput const & prev_output)
{
  ControllerOutput next_output = prev_output;

  bool all_target_angles_reached = true;
  for (auto leg : LEG_LIST)
  {
    double const coxa_deg_actual  = input.get_angle_deg(leg, Joint::Coxa );
    double const femur_deg_actual = input.get_angle_deg(leg, Joint::Femur);
    double const tibia_deg_actual = input.get_angle_deg(leg, Joint::Tibia);

    /* Calculate required target angles for desired
     * target position and set the output actuators.
     */
    const auto pos = Walking::sampleFootTrajectory(Walking::getLegTraits(leg), 0);
    kinematic::IK_Input const ik_input(pos(0), pos(1), pos(2),
                                               coxa_deg_actual, femur_deg_actual, tibia_deg_actual);
    auto const ik_output = engine.ik_solve(ik_input);

    if (!ik_output.has_value()) {
      RCLCPP_ERROR(_logger, "StandUp::update, engine.ik_solve failed for (%0.2f, %0.2f, %0.2f / %0.2f, %0.2f, %0.2f)",
        pos(0), pos(1), pos(2), coxa_deg_actual, femur_deg_actual, tibia_deg_actual);
      return {this, next_output};
    }

    /* Set output to the angle actuators. */
    next_output.set_angle_deg(leg, Joint::Coxa,  ik_output.value().coxa_angle_deg ());
    next_output.set_angle_deg(leg, Joint::Femur, ik_output.value().femur_angle_deg());
    next_output.set_angle_deg(leg, Joint::Tibia, ik_output.value().tibia_angle_deg());

    /* Check if target angles have been reached. */
    float const coxa_angle_actual = input.get_angle_deg(leg, Joint::Coxa);
    float const coxa_angle_error = fabs(ik_output.value().coxa_angle_deg() - coxa_angle_actual);
    bool  const coxa_is_initial_angle_reached = coxa_angle_error < 5.0f;

    if (!coxa_is_initial_angle_reached) {
      RCLCPP_INFO(_logger, "l3xz::gait::state::StandUp::update: leg %d coxa target angle not reached", int(leg));
      all_target_angles_reached = false;
    }
 
    float const femur_angle_actual = input.get_angle_deg(leg, Joint::Femur);
    float const femur_angle_error = fabs(ik_output.value().femur_angle_deg() - femur_angle_actual);
    bool  const femur_is_initial_angle_reached = femur_angle_error < 5.0f;

    if (!femur_is_initial_angle_reached) {
      RCLCPP_INFO(_logger, "l3xz::gait::state::StandUp::update: leg %d femur target angle not reached", int(leg));
      all_target_angles_reached = false;
    }

    float const tibia_angle_actual = input.get_angle_deg(leg, Joint::Tibia);
    float const tibia_angle_error = fabs(ik_output.value().tibia_angle_deg() - tibia_angle_actual);
    bool  const tibia_is_initial_angle_reached = tibia_angle_error < 5.0f;

    if (!tibia_is_initial_angle_reached) {
      RCLCPP_INFO(_logger, "l3xz::gait::state::StandUp::update: tibia %d target angle not reached", int(leg));
      all_target_angles_reached = false;
    }
  }

  if (!all_target_angles_reached)
    return std::tuple(this, next_output);

  return std::tuple(new Standing(_logger, _clock), next_output);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::gait::state */
