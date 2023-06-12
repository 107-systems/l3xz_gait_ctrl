/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_gait_ctrl/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <l3xz_gait_ctrl/gait/state/PositionTrajectory.h>

#include <l3xz_gait_ctrl/gait/state/Sitting.h>
#include <l3xz_gait_ctrl/gait/state/Standing.h>

#include <l3xz_gait_ctrl/const/LegList.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::gait::state
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

PositionTrajectory::PositionTrajectory(rclcpp::Logger const logger, rclcpp::Clock::SharedPtr const clock, PointVector const & point_vect, NextState const next_state)
: StateBase(logger, clock)
, _point_vect{point_vect}
, _citer{_point_vect.cbegin()}
, _next_state{next_state}
{

}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void PositionTrajectory::onEnter(ControllerInput const & /* input */)
{
  RCLCPP_INFO(_logger, "PositionTrajectory ENTER");
}

void PositionTrajectory::onExit()
{
  RCLCPP_INFO(_logger, "PositionTrajectory EXIT");
}

std::tuple<StateBase *, ControllerOutput> PositionTrajectory::update(kinematic::Engine const & engine, ControllerInput const & input, ControllerOutput const & prev_output)
{
  ControllerOutput next_output = prev_output;

  bool all_target_positions_reached = true;
  std::stringstream target_position_not_reached_list;

  for (auto leg : LEG_LIST)
  {
    double const coxa_deg_actual  = input.get_angle_deg(leg, Joint::Coxa );
    double const femur_deg_actual = input.get_angle_deg(leg, Joint::Femur);
    double const tibia_deg_actual = input.get_angle_deg(leg, Joint::Tibia);

    /* Calculate required target angles for desired
     * target position and set the output actuators.
     */
    Point3D const tip_target = *_citer;
    auto const [target_x, target_y, target_z] = tip_target;

    kinematic::IK_Input const ik_input(target_x,
                                       target_y,
                                       target_z,
                                       coxa_deg_actual,
                                       femur_deg_actual,
                                       tibia_deg_actual);
    auto const ik_output = engine.ik_solve(ik_input);

    if (!ik_output.has_value())
    {
      RCLCPP_ERROR(_logger,
                   "engine.ik_solve failed for (%0.2f, %0.2f, %0.2f / %0.2f, %0.2f, %0.2f)",
                   target_x,
                   target_y,
                   target_z,
                   coxa_deg_actual,
                   femur_deg_actual,
                   tibia_deg_actual);
      return {this, next_output};
    }

    /* Set output to the angle actuators. */
    next_output.set_angle_deg(leg, Joint::Coxa,  ik_output.value().coxa_angle_deg ());
    next_output.set_angle_deg(leg, Joint::Femur, ik_output.value().femur_angle_deg());
    next_output.set_angle_deg(leg, Joint::Tibia, ik_output.value().tibia_angle_deg());

    /* Check if target position has been reached: First calculate
     * the Euclidean distance between the actual and the target position.
     */
    kinematic::FK_Input const fk_input(coxa_deg_actual, femur_deg_actual, tibia_deg_actual);
    auto const fk_output = engine.fk_solve(fk_input);

    if (!fk_output.has_value())
    {
      RCLCPP_ERROR(_logger,
                   "engine.fk_solve failed for (%0.2f, %0.2f, %0.2f)",
                   coxa_deg_actual,
                   femur_deg_actual,
                   tibia_deg_actual);
      return {this, next_output};
    }

    Point3D const tip_actual = std::make_tuple(
      fk_output.value().tibia_tip_x(),
      fk_output.value().tibia_tip_y(),
      fk_output.value().tibia_tip_z());

    float const target_err = euclid_distance(tip_target, tip_actual);

    if (target_err > 20.0f)
    {
      RCLCPP_INFO_THROTTLE(_logger,
                           *_clock,
                           1000,
                           "target = (%0.2f, %0.2f, %0.2f), fk_output = (%0.2f, %0.2f, %0.2f), target_err = %0.2f",
                           target_x,
                           target_y,
                           target_z,
                           fk_output.value().tibia_tip_x(),
                           fk_output.value().tibia_tip_y(),
                           fk_output.value().tibia_tip_z(),
                           target_err);

      all_target_positions_reached = false;
      target_position_not_reached_list << LegToStr(leg) << " ";
    }
  }

  if (!all_target_positions_reached)
  {
    RCLCPP_INFO_THROTTLE(_logger,
                         *_clock,
                         1000,
                         "target position not reached for [ %s]",
                         target_position_not_reached_list.str().c_str());

    return std::tuple(this, next_output);
  }

  RCLCPP_INFO(_logger, "transitioning to next step on sitting down trajectory.");
  _citer++;

  if (_citer != _point_vect.cend())
    return std::tuple(this, next_output);

  if      (_next_state == NextState::Sitting)
    return std::tuple(new Sitting(_logger, _clock), next_output);
  else if (_next_state == NextState::Standing)
    return std::tuple(new Standing(_logger, _clock), next_output);
  else
    return std::tuple(this, next_output); /* This should never happen. */
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::gait::state */
