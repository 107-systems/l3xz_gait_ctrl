/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_gait_ctrl_gait_ctrl/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <l3xz_gait_ctrl/kinematic/FK_Output.h>

#include <sstream>
#include <iomanip>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::kinematic
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

FK_Output::FK_Output(KDL::Frame const & tibia_tip_frame)
:  _pos_vec(tibia_tip_frame.p)
, _tibia_tip_x{_pos_vec(0)}
, _tibia_tip_y{_pos_vec(1)}
, _tibia_tip_z{_pos_vec(2)}
{ }

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

std::string FK_Output::toStr() const
{
  std::stringstream msg;

  msg << "["
      << std::fixed
      << std::setprecision(2) 
      << std::setfill(' ')
      << std::setw(6)
      << _tibia_tip_x
      << ", "
      << std::fixed
      << std::setprecision(2) 
      << std::setfill(' ')
      << std::setw(6)
      << _tibia_tip_y
      << ", "
      << std::fixed
      << std::setprecision(2) 
      << std::setfill(' ')
      << std::setw(6)
      << _tibia_tip_z
      << "]";

  return msg.str();
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::kinematic */
