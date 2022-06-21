/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_ctrl/graphs/contributors.
 */

#ifndef HEAD_TELEOP_STATE_H_
#define HEAD_TELEOP_STATE_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include "StateBase.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::head::state
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class Teleop : public StateBase
{
public:
  virtual ~Teleop() { }
  virtual void onEnter() override;
  virtual void onExit() override;
  virtual std::tuple<StateBase *, ControllerOutput> update(ControllerInput const & input, ControllerOutput const & prev_output) override;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::head::state */

#endif /* HEAD_TELEOP_STATE_H_ */
