/*********************************************************************
 Autonomous Motion Department
 Max-Planck Intelligent Systems
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         CartesianPDCmd.h

 \author       Alexander Herzog
 \date         Jan 23, 2014

 *********************************************************************/

#ifndef CARTESIANPDCMD_H_
#define CARTESIANPDCMD_H_

#include "string"
#include "KinematicsEigen.h"
//#include "RtMatrixX.h"
//#include "HierarchInverseDynamics.hh"
//#include <SL_user.h>

namespace floating_base_utilities
{

/**
 * generates an impedance control command: cmd = -impedance^-1*f + ddX_ff +
 *       impedance^-1*d_gain * dX_e + impedance^-1*p_gain * X_e
 *       for f=0 and impedance = identity we get a PD controller
 * f = force
 * x_e = position error
 * q_e = orientation error (euler angles computed from two quaternions)
 * dx_e = linear velocity error
 * dq_e = angular velocity error
 * ddx_ff, ddq_ff = feed-forward linear and angular acceleration
 * X_e = [x_e' -q_e'], dX_e = [dx_e' dq_e'], ddX_ff = [ddx_ff' ddq_ff']
 */
class CartesianPDCmd
{
public:
  CartesianPDCmd();
  virtual ~CartesianPDCmd(){};

  void initialize(KinematicsEigen& kinematics, int link_id, std::string config_file=std::string(""));

  void update(const Eigen::Matrix<double, 6, 1>& force,
      const Eigen::Matrix<double, 6, 1>& des_acc,
      const Eigen::Matrix<double, 6, 1>& des_vel,
      const Eigen::Matrix<double, 3, 1>& des_pos,
      const Eigen::Quaterniond& des_orient);

  int linkId() const{return link_id_;};
//  int& linkId(){return link_id_;};
  const Eigen::Matrix<double, 6, 1>& impedance() const{return impedance_;};
  Eigen::Matrix<double, 6, 1>& impedance() {return impedance_;};
  const Eigen::Matrix<double, 6, 6>& pGain() const{return p_gain_;};
  Eigen::Matrix<double, 6, 6>& pGain() {return p_gain_;};
  const Eigen::Matrix<double, 6, 6>& dGain() const{return d_gain_;};
  Eigen::Matrix<double, 6, 6>& dGain() {return d_gain_;};
  const Eigen::Matrix<double, 6, 1>& command() const{return cmd_;};


  Eigen::Matrix<double, 3, 1> des_pos_;
  Eigen::Quaterniond des_orient_;
private:
  std::string config_file_;
  KinematicsEigen* kinematics_;
  Eigen::Matrix<double, 6, 1> cmd_;
  Eigen::Matrix<double, 6, 1> impedance_;
  Eigen::Matrix<double, 6, 6> p_gain_;
  Eigen::Matrix<double, 6, 6> d_gain_;
  int link_id_;
};

} /* namespace floating_base_utilities */
#endif /* CARTESIANPDCMD_H_ */
