/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_gait_ctrl_gait_ctrl/graphs/contributors.
 */

#ifndef KINEMATIC_FK_OUTPUT_H_
#define KINEMATIC_FK_OUTPUT_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <string>

#include <kdl/frames.hpp>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::kinematic
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class FK_Output
{
public:
  FK_Output(KDL::Frame const & tibia_tip_frame);

  inline double tibia_tip_x() const { return _tibia_tip_x; }
  inline double tibia_tip_y() const { return _tibia_tip_y; }
  inline double tibia_tip_z() const { return _tibia_tip_z; }

  std::string toStr() const;

private:
  KDL::Vector const _pos_vec;
  double const _tibia_tip_x;
  double const _tibia_tip_y;
  double const _tibia_tip_z;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::kinematic */

#endif /* KINEMATIC_FK_OUTPUT_H_ */
