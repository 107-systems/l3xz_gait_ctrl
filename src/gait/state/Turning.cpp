/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_gait_ctrl/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <l3xz_gait_ctrl/gait/state/Turning.h>

#include <l3xz_gait_ctrl/gait/state/Standing.h>
#include <l3xz_gait_ctrl/gait/state/Walking.h>

#include <l3xz_gait_ctrl/const/LegList.h>

#include <l3xz_gait_ctrl/gait/util/Util.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::gait::state
{

/**************************************************************************************
 * CLASS MEMBER FUNCTIONS
 **************************************************************************************/

void Turning::onEnter(ControllerInput const & /* input */)
{
  RCLCPP_INFO(_logger, "Turning ENTER");
}

void Turning::onExit()
{
  RCLCPP_INFO(_logger, "Turning EXIT");
}

std::tuple<StateBase *, ControllerOutput> Turning::update(kinematic::Engine const & engine, ControllerInput const & input, ControllerOutput const & prev_output)
{
  ControllerOutput next_output = prev_output;
  for (const auto leg : LEG_LIST)
  {
    double const coxa_deg_actual  = input.get_angle_deg(leg, Joint::Coxa );
    double const femur_deg_actual = input.get_angle_deg(leg, Joint::Femur);
    double const tibia_deg_actual = input.get_angle_deg(leg, Joint::Tibia);
    const auto lt = Walking::getLegTraits(leg);
    const auto pos = Walking::sampleFootTrajectory(lt, (lt.is_left == _left) ? +_phase : -_phase);
    kinematic::IK_Input const ik_input(pos(0), pos(1), pos(2),
                                               coxa_deg_actual, femur_deg_actual, tibia_deg_actual);
    auto const ik_output = engine.ik_solve(ik_input);
    if (!ik_output.has_value()) {
      RCLCPP_ERROR(_logger, "Turning::update, engine.ik_solve failed for (%0.2f, %0.2f, %0.2f / %0.2f, %0.2f, %0.2f)",
        pos(0), pos(1), pos(2), coxa_deg_actual, femur_deg_actual, tibia_deg_actual);
      return {this, next_output};
    }
    next_output.set_angle_deg(leg, Joint::Coxa,  ik_output.value().coxa_angle_deg ());
    next_output.set_angle_deg(leg, Joint::Femur, ik_output.value().femur_angle_deg());
    next_output.set_angle_deg(leg, Joint::Tibia, ik_output.value().tibia_angle_deg());
  }
  _phase += PHASE_INCREMENT;
  return std::tuple((_phase < 1.0F) ? this : static_cast<StateBase*>(new Standing(_logger, _clock)), next_output);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::gait::state */
