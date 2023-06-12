/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_gait_ctrl_gait_ctrl/graphs/contributors.
 */

#pragma once

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <cmath>

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

[[nodiscard]] inline float euclid_distance(Point3D const & p1, Point3D const & p2)
{
  auto const [x1, y1, z1] = p1;
  auto const [x2, y2, z2] = p2;

  auto const x_diff = x1 - x2;
  auto const y_diff = y1 - y2;
  auto const z_diff = z1 - z2;

  return sqrt(pow(x_diff, 2.0f) + pow(y_diff, 2.0f) + pow(z_diff, 2.0f));
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */
