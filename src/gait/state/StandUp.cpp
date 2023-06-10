/**
 * Copyright (c) 2023 LXRobotics GmbH.
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
 * CONSTANTS
 **************************************************************************************/

static std::vector<std::tuple<float /* x */, float /* y */, float /* z */>> const STAND_UP_TRAJ =
{
  {-201.0f, 0.0f, -145.0f},
  {-201.0f, 0.0f, -150.0f},
  {-202.0f, 0.0f, -155.0f},
  {-202.0f, 0.0f, -160.0f},
  {-203.0f, 0.0f, -165.0f},
  {-204.0f, 0.0f, -170.0f},
  {-204.0f, 0.0f, -175.0f},
  {-205.0f, 0.0f, -180.0f},
  {-205.0f, 0.0f, -185.0f},
  {-205.0f, 0.0f, -190.0f},
  {-206.0f, 0.0f, -195.0f},
  {-207.0f, 0.0f, -200.0f},
  {-207.0f, 0.0f, -205.0f},
  {-208.0f, 0.0f, -210.0f},
  {-208.0f, 0.0f, -215.0f},
  {-209.0f, 0.0f, -220.0f},
  {-209.0f, 0.0f, -225.0f},
  {-209.0f, 0.0f, -230.0f},
  {-210.0f, 0.0f, -235.0f},
  {-209.0f, 0.0f, -240.0f},
  {-210.0f, 0.0f, -245.0f},
  {-209.0f, 0.0f, -250.0f},
  {-210.0f, 0.0f, -255.0f},
  {-210.0f, 0.0f, -260.0f}
};

StandUp::StandUp(rclcpp::Logger const logger, rclcpp::Clock::SharedPtr const clock)
: StateBase(logger, clock)
, _citer{STAND_UP_TRAJ.cbegin()}
{

}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void StandUp::onEnter(ControllerInput const & /* input */)
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
    auto [target_x, target_y, target_z] = *_citer;

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
                   "StandUp::update, engine.ik_solve failed for (%0.2f, %0.2f, %0.2f / %0.2f, %0.2f, %0.2f)",
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
                   "StandUp::update, engine.fk_solve failed for (%0.2f, %0.2f, %0.2f)",
                   coxa_deg_actual,
                   femur_deg_actual,
                   tibia_deg_actual);
      return {this, next_output};
    }

    float const target_err = sqrt(
      pow((fk_output.value().tibia_tip_x() - target_x), 2.0f) +
      pow((fk_output.value().tibia_tip_y() - target_y), 2.0f) +
      pow((fk_output.value().tibia_tip_z() - target_z), 2.0f)
      );

    if (target_err > 20.0f)
    {
      RCLCPP_INFO_THROTTLE(_logger,
                           *_clock,
                           1000,
                           "StandUp::update, target = (%0.2f, %0.2f, %0.2f), fk_output = (%0.2f, %0.2f, %0.2f), target_err = %0.2f",
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
                         "l3xz::gait::state::StandUp::update: target position not reached for [ %s]",
                         target_position_not_reached_list.str().c_str());

    return std::tuple(this, next_output);
  }

  RCLCPP_INFO(_logger, "transitioning to next step on stand up trajectory.");
  _citer++;

  if (_citer != STAND_UP_TRAJ.cend())
    return std::tuple(this, next_output);

  return std::tuple(new Standing(_logger, _clock), next_output);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::gait::state */
