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

#include "CartesianPDCmd.h"
#include "GeometryUtils.h"

namespace floating_base_utilities
{

CartesianPDCmd::CartesianPDCmd()
{
  link_id_=-1;
}

void  CartesianPDCmd::initialize(KinematicsEigen& kinematics, int link_id, std::string config_file)
{
  kinematics_ = &kinematics;
  config_file_ = config_file;
  link_id_ = link_id;
  impedance_.setOnes();
  p_gain_.setIdentity();
  d_gain_.setIdentity();
}

void CartesianPDCmd::update(const Eigen::Matrix<double, 6, 1>& force,
    const Eigen::Matrix<double, 6, 1>& des_acc,
    const Eigen::Matrix<double, 6, 1>& des_vel,
    const Eigen::Matrix<double, 3, 1>& des_pos, const Eigen::Quaterniond& des_orient)
{
  Eigen::Matrix<double, 6, 1> impedance_inv = impedance_.array().inverse();
  Eigen::Matrix<double, 6, 1> pos_e;
  Eigen::Vector3d orient_err;
  pos_e.head<3>() = des_pos - kinematics_->linkPosition(link_id_);
  GeometryUtils::computeOrientationError(des_orient,
      Eigen::Quaterniond(kinematics_->getLinkPose(link_id_).topLeftCorner<3,3>()),
      orient_err);
  pos_e.tail<3>() = -orient_err;
  cmd_ = des_acc + impedance_inv.asDiagonal()*(-force +
      d_gain_*(des_vel-kinematics_->linkJacobian(link_id_)*
          kinematics_->generalizedJointVelocities())
          + p_gain_*pos_e);
}

} /* namespace floating_base_utilities */
