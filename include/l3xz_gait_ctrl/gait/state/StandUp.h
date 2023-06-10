/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_gait_ctrl/graphs/contributors.
 */

#ifndef STANDUP_STATE_H_
#define STANDUP_STATE_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include "StateBase.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::gait::state
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class StandUp : public StateBase
{
public:
  StandUp(rclcpp::Logger const logger, rclcpp::Clock::SharedPtr const clock);
  virtual ~StandUp() { }
  virtual void onEnter(ControllerInput const & input) override;
  virtual void onExit() override;
  virtual std::tuple<StateBase *, ControllerOutput> update(kinematic::Engine const & engine, ControllerInput const & input, ControllerOutput const & prev_output) override;

private:
  std::vector<std::tuple<float /* x */, float /* y */, float /* z */>>::const_iterator _citer;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::gait::state */

#endif /* STANDUP_STATE_H_ */
