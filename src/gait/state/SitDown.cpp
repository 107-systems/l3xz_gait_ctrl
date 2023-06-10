/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_gait_ctrl/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <l3xz_gait_ctrl/gait/state/SitDown.h>

#include <l3xz_gait_ctrl/gait/state/Sitting.h>

#include <l3xz_gait_ctrl/const/LegList.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::gait::state
{

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void SitDown::onEnter(ControllerInput const & /* input */)
{
  RCLCPP_INFO(_logger, "SitDown ENTER");
}

void SitDown::onExit()
{
  RCLCPP_INFO(_logger, "SitDown EXIT");
}

std::tuple<StateBase *, ControllerOutput> SitDown::update(kinematic::Engine const & engine, ControllerInput const & input, ControllerOutput const & prev_output)
{
  ControllerOutput next_output = prev_output;

  bool all_target_angles_reached = true;
  std::stringstream coxa_leg_not_reached_list, femur_leg_not_reached_list, tibia_leg_not_reached_list;

  for (auto leg : LEG_LIST)
  {
    double const coxa_deg_actual  = input.get_angle_deg(leg, Joint::Coxa );
    double const femur_deg_actual = input.get_angle_deg(leg, Joint::Femur);
    double const tibia_deg_actual = input.get_angle_deg(leg, Joint::Tibia);

    /* Right now this should simply hold position.
     * TODO: extend the code for a smooth lowering
     * of the legs.
     */
    kinematic::FK_Input const fk_input(180.0f, 0.0f, 0.0f);
    auto const fk_output = engine.fk_solve(fk_input);

    if (!fk_output.has_value())
    {
      RCLCPP_ERROR(_logger,
                   "SitDown::update, engine.fk_solve failed for (%0.2f, %0.2f, %0.2f)",
                   coxa_deg_actual,
                   femur_deg_actual,
                   tibia_deg_actual);
      return {this, next_output};
    }

    RCLCPP_INFO_THROTTLE(_logger,
                         *_clock,
                         1000,
                         "SitDown::update, fk_output = (%0.2f, %0.2f, %0.2f)",
                         fk_output.value().tibia_tip_x(),
                         fk_output.value().tibia_tip_y(),
                         fk_output.value().tibia_tip_z());

    /* Calculate required target angles for desired
     * target position and set the output actuators.
     */
    kinematic::IK_Input const ik_input(fk_output.value().tibia_tip_x(),
                                       fk_output.value().tibia_tip_y(),
                                       fk_output.value().tibia_tip_z(),
                                       coxa_deg_actual,
                                       femur_deg_actual,
                                       tibia_deg_actual);
    auto const ik_output = engine.ik_solve(ik_input);

    if (!ik_output.has_value())
    {
      RCLCPP_ERROR(_logger,
                   "SitDown::update, engine.ik_solve failed for (%0.2f, %0.2f, %0.2f / %0.2f, %0.2f, %0.2f)",
                   fk_output.value().tibia_tip_x(),
                   fk_output.value().tibia_tip_y(),
                   fk_output.value().tibia_tip_z(),
                   coxa_deg_actual,
                   femur_deg_actual,
                   tibia_deg_actual);
      return {this, next_output};
    }

    /* Set output to the angle actuators. */
    next_output.set_angle_deg(leg, Joint::Coxa,  ik_output.value().coxa_angle_deg ());
    next_output.set_angle_deg(leg, Joint::Femur, ik_output.value().femur_angle_deg());
    next_output.set_angle_deg(leg, Joint::Tibia, ik_output.value().tibia_angle_deg());

    /* Check if target angles have been reached. */
    float const coxa_angle_actual = input.get_angle_deg(leg, Joint::Coxa);
    float const coxa_angle_error = fabs(ik_output.value().coxa_angle_deg() - coxa_angle_actual);
    bool  const coxa_is_initial_angle_reached = coxa_angle_error < 2.5f;

    if (!coxa_is_initial_angle_reached)
    {
      coxa_leg_not_reached_list << LegToStr(leg) << " ";
      all_target_angles_reached = false;
    }
 
    float const femur_angle_actual = input.get_angle_deg(leg, Joint::Femur);
    float const femur_angle_error = fabs(ik_output.value().femur_angle_deg() - femur_angle_actual);
    bool  const femur_is_initial_angle_reached = femur_angle_error < 5.0f;

    if (!femur_is_initial_angle_reached)
    {
      femur_leg_not_reached_list << LegToStr(leg) << " ";
      all_target_angles_reached = false;
    }

    float const tibia_angle_actual = input.get_angle_deg(leg, Joint::Tibia);
    float const tibia_angle_error = fabs(ik_output.value().tibia_angle_deg() - tibia_angle_actual);
    bool  const tibia_is_initial_angle_reached = tibia_angle_error < 5.0f;

    if (!tibia_is_initial_angle_reached)
    {
      tibia_leg_not_reached_list << LegToStr(leg) << " ";
      all_target_angles_reached = false;
    }
  }

  if (!all_target_angles_reached)
  {
    RCLCPP_INFO_THROTTLE(_logger,
                         *_clock,
                         1000,
                         "l3xz::gait::state::SitDown::update: target angle not reached for\n\tcoxa  [ %s]\n\tfemur [ %s]\n\ttibia [ %s]",
                         coxa_leg_not_reached_list.str().c_str(),
                         femur_leg_not_reached_list.str().c_str(),
                         tibia_leg_not_reached_list.str().c_str());

    return std::tuple(this, next_output);
  }

  return std::tuple(new Sitting(_logger, _clock), next_output);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::gait::state */
