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

[[nodiscard]] inline PointVector calc_point_vector(Point3D const & p1, Point3D const & p2, size_t const num_steps)
{
  auto const [x1, y1, z1] = p1;
  auto const [x2, y2, z2] = p2;

  auto const x_diff = x2 - x1;
  auto const y_diff = y2 - y1;
  auto const z_diff = z2 - z1;

  PointVector point_vect;

  for (size_t i = 0; i < num_steps; i++)
  {
    float const delta_x = (x_diff * i) / static_cast<float>(num_steps);
    float const delta_y = (y_diff * i) / static_cast<float>(num_steps);
    float const delta_z = (z_diff * i) / static_cast<float>(num_steps);

    Point3D const p_i = std::make_tuple(
      x1 + delta_x,
      y1 + delta_y,
      z1 + delta_z
      );

    point_vect.push_back(p_i);
  }

  return point_vect;
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */
