/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_gait_ctrl_gait_ctrl/graphs/contributors.
 */

#ifndef L3XZ_GAIT_CTRL_JOINTLIST_H
#define L3XZ_GAIT_CTRL_JOINTLIST_H

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <list>

#include <l3xz_gait_ctrl/types/Joint.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz
{

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

static std::list<Joint> const JOINT_LIST =
{
  Joint::Coxa, Joint::Femur, Joint::Tibia
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */

#endif /* L3XZ_GAIT_CTRL_JOINTLIST_H */
