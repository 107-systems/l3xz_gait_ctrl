/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_gait_ctrl_gait_ctrl/graphs/contributors.
 */

#pragma once

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <tuple>
#include <vector>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz
{

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

typedef std::tuple<float /* x */, float /* y */, float /* z */> Point3D;
typedef std::vector<Point3D> PointVector;

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */