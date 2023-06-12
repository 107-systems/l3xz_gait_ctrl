/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_gait_ctrl/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <l3xz_gait_ctrl/gait/state/Walking.h>

#include <l3xz_gait_ctrl/gait/state/Standing.h>

#include <l3xz_gait_ctrl/const/LegList.h>

#include <l3xz_gait_ctrl/types/Point3D.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::gait::state
{

const float PITCH_MULT  =    0.3F;
const float FOOT_X      = -210.0F;
const float FOOT_Z_UP   = -200.0F;
const float FOOT_Z_DOWN = -260.0F;
const std::vector<KDL::Vector> Walking::FOOT_TRAJECTORY{
  {FOOT_X, +103.5 * PITCH_MULT, FOOT_Z_UP},
  {FOOT_X, +103.5 * PITCH_MULT, FOOT_Z_DOWN},
  {FOOT_X, + 80.5 * PITCH_MULT, FOOT_Z_DOWN},
  {FOOT_X, + 57.5 * PITCH_MULT, FOOT_Z_DOWN},
  {FOOT_X, + 34.5 * PITCH_MULT, FOOT_Z_DOWN},
  {FOOT_X, + 11.5 * PITCH_MULT, FOOT_Z_DOWN},
  {FOOT_X, - 11.5 * PITCH_MULT, FOOT_Z_DOWN},
  {FOOT_X, - 34.5 * PITCH_MULT, FOOT_Z_DOWN},
  {FOOT_X, - 57.5 * PITCH_MULT, FOOT_Z_DOWN},
  {FOOT_X, - 80.5 * PITCH_MULT, FOOT_Z_DOWN},
  {FOOT_X, -103.5 * PITCH_MULT, FOOT_Z_DOWN},
  {FOOT_X, -103.5 * PITCH_MULT, FOOT_Z_UP},
};

void Walking::onEnter(ControllerInput const & /* input */)
{
  RCLCPP_INFO(_logger, "Walking ENTER");
}

void Walking::onExit()
{
  RCLCPP_INFO(_logger, "Walking EXIT");
}

std::tuple<StateBase *, ControllerOutput> Walking::update(kinematic::Engine const & engine, ControllerInput const & input, ControllerOutput const & prev_output)
{
  ControllerOutput next_output = prev_output;

  bool all_target_positions_reached = true;
  std::stringstream target_position_not_reached_list;

  for (const auto leg : LEG_LIST)
  {
    double const coxa_deg_actual  = input.get_angle_deg(leg, Joint::Coxa );
    double const femur_deg_actual = input.get_angle_deg(leg, Joint::Femur);
    double const tibia_deg_actual = input.get_angle_deg(leg, Joint::Tibia);
    const auto pos = sampleFootTrajectory(getLegTraits(leg), _phase);

    /* Calculate required target angles for desired
     * target position and set the output actuators.
     */
    Point3D const tip_target = std::make_tuple(pos(0), pos(1), pos(2));
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

    float const target_err = fabs(fk_output.value().tibia_tip_z() - target_z);

    if (target_err > 10.0f)
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

  _phase += _phase_increment;
  return std::tuple((std::abs(_phase) < 1.0F) ? this : static_cast<StateBase*>(new Standing(_logger, _clock)), next_output);
}

[[nodiscard]] KDL::Vector Walking::sampleFootTrajectory(const LegTraits lt, const float phase)
{
  auto pos = interpolatePiecewiseClosed(wrapPhase(phase + (lt.index / 6.0F)), FOOT_TRAJECTORY.data(), FOOT_TRAJECTORY.size());
  /*
  if (lt.is_front)
  {
    pos[1] += 100;
  }
  if (lt.is_rear)
  {
    pos[1] -= 80;
  }
   */
  pos[1] *= lt.is_left ? -1.0F : +1.0F;
  return pos;
}

[[nodiscard]] LegTraits Walking::getLegTraits(const Leg leg)
{
  static const std::array<Leg, 6> ref{{
    Leg::RightFront,
    Leg::LeftMiddle,
    Leg::RightBack,
    Leg::LeftFront,
    Leg::RightMiddle,
    Leg::LeftBack,
  }};
  std::uint8_t idx = 0;
  while (ref.at(idx) != leg)
  {
    idx++;
  }
  const bool is_left  = (idx % 2) != 0;
  const bool is_front = (idx % 3) == 0;
  const bool is_rear  = (idx % 3) == 2;
  return {idx, is_left, is_front, is_rear};
}

} /* l3xz::gait::state */
