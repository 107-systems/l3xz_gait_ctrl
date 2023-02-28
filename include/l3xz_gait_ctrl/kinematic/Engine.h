/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_gait_ctrl_gait_ctrl/graphs/contributors.
 */

#ifndef KINEMATIC_ENGINE_H_
#define KINEMATIC_ENGINE_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <memory>
#include <optional>

#include <rclcpp/rclcpp.hpp>

#include "FK_Input.h"
#include "FK_Output.h"

#include "IK_Input.h"
#include "IK_Output.h"

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::kinematic
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class Engine
{
public:

  Engine(rclcpp::Logger const logger);

  std::optional<FK_Output> fk_solve(FK_Input const & fk_input) const;
  std::optional<IK_Output> ik_solve(IK_Input const & ik_input) const;

private:
  rclcpp::Logger const _logger;
  KDL::Chain _leg_chain;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> _fksolver;
  std::unique_ptr<KDL::ChainIkSolverVel_pinv> _iksolver_vel;
  std::unique_ptr<KDL::ChainIkSolverPos_NR> _iksolver_pos;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::kinematic */

#endif /* KINEMATIC_ENGINE_H_ */
