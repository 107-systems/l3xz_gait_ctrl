/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_ctrl/graphs/contributors.
 */

#ifndef KINEMATIC_IK_OUTPUT_H_
#define KINEMATIC_IK_OUTPUT_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <stdexcept>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::kinematic
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class IK_Output
{
public:
  IK_Output(double const coxa_angle_rad, double const femur_angle_rad, double const tibia_angle_rad);


  inline double coxa_angle_deg () const { return _coxa_angle_deg; }
  inline double femur_angle_deg() const { return _femur_angle_deg; }
  inline double tibia_angle_deg() const { return _tibia_angle_deg; }


  std::string toStr() const;


private:
  double const _coxa_angle_deg;
  double const _femur_angle_deg;
  double const _tibia_angle_deg;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::kinematic */

#endif /* KINEMATIC_IK_OUTPUT_H_ */
