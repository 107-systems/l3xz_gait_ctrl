/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_ctrl/graphs/contributors.
 */

#ifndef L3XZ_CTRL_TYPES_TELEOP_COMMAND_DATA_H_
#define L3XZ_CTRL_TYPES_TELEOP_COMMAND_DATA_H_

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class TeleopCommandData
{
public:

  TeleopCommandData()
  : linear_velocity_x{0.0f}
  , linear_velocity_y{0.0f}
  , angular_velocity_head_tilt{0.0f}
  , angular_velocity_head_pan{0.0f}
  , angular_velocity_z{0.0f}
  {
  }

  float linear_velocity_x,
        linear_velocity_y,
        angular_velocity_head_tilt,
        angular_velocity_head_pan,
        angular_velocity_z;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */

#endif /* L3XZ_CTRL_TYPES_TELEOP_COMMAND_DATA_H_ */