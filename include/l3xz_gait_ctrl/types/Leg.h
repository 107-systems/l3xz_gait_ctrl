/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_gait_ctrl_gait_ctrl/graphs/contributors.
 */

#ifndef l3xz_gait_ctrl_TYPES_LEG_H_
#define l3xz_gait_ctrl_TYPES_LEG_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <string>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz
{

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

enum class Leg
{
  LeftFront,
  LeftMiddle,
  LeftBack,
  RightFront,
  RightMiddle,
  RightBack
};

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

std::string LegToStr(Leg const leg);

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */

#endif /* l3xz_gait_ctrl_TYPES_LEG_H_ */