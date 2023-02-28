/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_gait_ctrl/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <l3xz_gait_ctrl/gait/state/Walking.h>

#include <l3xz_gait_ctrl/gait/util/Util.h>
#include <l3xz_gait_ctrl/gait/state/Standing.h>

#include <l3xz_gait_ctrl/const/LegList.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::gait::state
{

const float PITCH_MULT  = 0.8F;
const float FOOT_X      = +180.0F;
const float FOOT_Z_UP   = -130.0F;
const float FOOT_Z_DOWN = -200.0F;
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

void Walking::onEnter()
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
  for (const auto leg : LEG_LIST)
  {
    double const coxa_deg_actual  = input.get_angle_deg(leg, Joint::Coxa );
    double const femur_deg_actual = input.get_angle_deg(leg, Joint::Femur);
    double const tibia_deg_actual = input.get_angle_deg(leg, Joint::Tibia);
    const auto pos = sampleFootTrajectory(getLegTraits(leg), _phase);
    kinematic::IK_Input const ik_input(pos(0), pos(1), pos(2),
                                               coxa_deg_actual, femur_deg_actual, tibia_deg_actual);
    auto const ik_output = engine.ik_solve(ik_input);
    if (!ik_output.has_value()) {
      RCLCPP_INFO(_logger, "Walking::update, engine.ik_solve failed for (%0.2f, %0.2f, %0.2f / %0.2f, %0.2f, %0.2f)",
        pos(0), pos(1), pos(2), coxa_deg_actual, femur_deg_actual, tibia_deg_actual);
      return {this, next_output};
    }
    next_output.set_angle_deg(leg, Joint::Coxa,  ik_output.value().coxa_angle_deg ());
    next_output.set_angle_deg(leg, Joint::Femur, ik_output.value().femur_angle_deg());
    next_output.set_angle_deg(leg, Joint::Tibia, ik_output.value().tibia_angle_deg());
    if (leg == Leg::LeftFront)
    {
      RCLCPP_INFO(_logger, "Walking::update Front/Left foot pos: %f %f %f", pos(0), pos(1), pos(2));
    }
  }
  _phase += _phase_increment;
  return std::tuple((std::abs(_phase) < 1.0F) ? this : static_cast<StateBase*>(new Standing(_logger)), next_output);
}

[[nodiscard]] KDL::Vector Walking::sampleFootTrajectory(const LegTraits lt, const float phase)
{
  auto pos = interpolatePiecewiseClosed(wrapPhase(phase + (lt.index / 6.0F)), FOOT_TRAJECTORY.data(), FOOT_TRAJECTORY.size());
  if (lt.is_front)
  {
    pos[1] += 100;
  }
  if (lt.is_rear)
  {
    pos[1] -= 80;
  }
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
