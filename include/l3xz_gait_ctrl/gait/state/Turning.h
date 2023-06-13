/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_gait_ctrl/graphs/contributors.
 */

#ifndef TURNING_LEFT_H_
#define TURNING_LEFT_H_

#include "StateBase.h"

namespace l3xz::gait::state
{

class Turning : public StateBase
{
public:
  static constexpr float PHASE_INCREMENT = 0.001f;

  explicit Turning(rclcpp::Logger const logger, rclcpp::Clock::SharedPtr const clock, const bool left)
  : StateBase(logger, clock)
  , _left(left)
  , _phase{0.0f}
  { }
  virtual void onEnter(ControllerInput const & input) override;
  virtual void onExit() override;
  virtual std::tuple<StateBase *, ControllerOutput> update(kinematic::Engine const & engine, ControllerInput const & input, ControllerOutput const & prev_output) override;

private:
  const bool _left;
  float _phase;
};

} /* l3xz::gait::state */

#endif /* TURNING_LEFT_H_ */
