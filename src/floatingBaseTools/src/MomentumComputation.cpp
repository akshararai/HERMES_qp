/*
 * MomentumComputation.cpp
 *
 *  Created on: Apr 2, 2013
 *      Author: herzog
 */

#include "MomentumComputation.h"

//#include <geometry_utils.h>
#include "mdefs.h"

namespace floating_base_utilities
{




MomentumComputation::MomentumComputation()
{
  num_deriv_ok_ = false;
};


bool MomentumComputation::initialize()
{
  num_deriv_ok_ = false;

  centroidal_momentum_matrix_.setZero();
  dcentroidal_momentum_matrix_.setZero();
  momentum_.setZero();

  dcog_.setZero();
  ddcog_.setZero();

  return true;
}


bool MomentumComputation::update(const KinematicsEigen& kinematics_eigen)
{
  Eigen::Matrix<double, 2*N_CART, N_DOFS+2*N_CART> prev_centr_mom_mat = centroidal_momentum_matrix_;

  centroidal_momentum_matrix_.setZero();

  //get the full Jacobian
  //  jacobian_creator_.update(j_state, base_position, base_orientation, endeffectors);
  Eigen::Matrix<double, 2*N_CART*N_DOFS+2*N_CART, N_DOFS+2*N_CART> full_joint_jacobian;
  full_joint_jacobian = kinematics_eigen.fullBodyJointJacobian();


  //for each DOF we compute the parts of the centroidal momentum matrix
  //we start counting from the base
  for(int i=0; i<=N_DOFS; ++i)
  {
    Eigen::Matrix3d joint_rotation;
    Eigen::Vector3d joint_position;

    Eigen::Matrix<double, 4,4> joint_pose = Eigen::Matrix<double, 4,4>::Zero();
    kinematics_eigen.getJointPose(i, joint_pose);
    joint_rotation = joint_pose.block(0,0,3,3);


    if(i==0)
      joint_position = joint_pose.block(0,3,3,1);
    else
      joint_position = kinematics_eigen.cartesianJointPositions().block(0,i,3,1);


    //get the inertia tensor of joint in world/cog frame
    Eigen::Matrix3d joint_inertia_tensor;
    for(int j=0; j<N_CART; ++j)
      for(int k=0; k<N_CART; ++k)
        joint_inertia_tensor(j,k) = links[i].inertia[j+1][k+1];
    joint_inertia_tensor = joint_rotation * joint_inertia_tensor * joint_rotation.transpose();

    //compute the position of the com (relative to the joint) in world coordinates
    Eigen::Vector3d joint_mcom_pos(links[i].mcm[_X_], links[i].mcm[_Y_], links[i].mcm[_Z_]);
    joint_mcom_pos = joint_rotation * joint_mcom_pos;

    Eigen::Vector3d robot_com_to_joint = (joint_position - kinematics_eigen.cog());
    Eigen::Matrix3d robot_com_to_joint_cross;
    robot_com_to_joint_cross <<
        0, -robot_com_to_joint(2), robot_com_to_joint(1),
        robot_com_to_joint(2), 0, -robot_com_to_joint(0),
        -robot_com_to_joint(1), robot_com_to_joint(0), 0;

    Eigen::Matrix3d joint_mcom_pos_cross;
    joint_mcom_pos_cross <<
        0, -joint_mcom_pos(2), joint_mcom_pos(1),
        joint_mcom_pos(2), 0, -joint_mcom_pos(0),
        -joint_mcom_pos(1), joint_mcom_pos(0), 0;

    Eigen::Matrix<double, 6, 6> joint_inertia_matrix;

    //linear velocity of joint
    joint_inertia_matrix.block(0,0,3,3) = Eigen::Matrix3d::Identity();
    joint_inertia_matrix.block(0,0,3,3) *= links[i].m;
    //the inertia due to rotation of link + additional lin. velocity of com due to rotation of joint
    joint_inertia_matrix.block(3,3,3,3) = joint_inertia_tensor + robot_com_to_joint_cross*joint_mcom_pos_cross.transpose();

    //get the contribution of linear velocity of joint to the momentum around the com
    joint_inertia_matrix.block(3,0,3,3) = joint_mcom_pos_cross + links[i].m *robot_com_to_joint_cross;

    // minus cross operator to get omega x m_com (additional vel. of com due to rotation of joint)
    joint_inertia_matrix.block(0,3,3,3) = joint_mcom_pos_cross.transpose();

    int row_base;

    row_base = i*2*N_CART;
    centroidal_momentum_matrix_ += joint_inertia_matrix * full_joint_jacobian.block(row_base,0,6, N_DOFS+6);

    Eigen::Matrix<double, 6, 1> btaa_speed = full_joint_jacobian.block(row_base,0,6, N_DOFS+6) * kinematics_eigen.generalizedJointVelocities();

  }

  momentum_ = centroidal_momentum_matrix_ * kinematics_eigen.generalizedJointVelocities();
  dcog_ = momentum_.topRows(N_CART)/kinematics_eigen.robotMass();
  if(num_deriv_ok_)
  {
    dcentroidal_momentum_matrix_ = (centroidal_momentum_matrix_ - prev_centr_mom_mat)*double(task_servo_rate);
  }
  else
  {
    dcentroidal_momentum_matrix_.setZero();
    num_deriv_ok_ = true;
  }
  ddcog_ = (centroidal_momentum_matrix_.topRows(N_CART) * kinematics_eigen.generalizedJointAccelerations() +
      dcentroidal_momentum_matrix_.topRows(N_CART) * kinematics_eigen.generalizedJointVelocities())/kinematics_eigen.robotMass();

  return true;
}



void MomentumComputation::computeCOPTorque(const Eigen::Vector3d& ankle_pos, const Eigen::Matrix4d& foot_sole_frame,
                                                                                    const Eigen::Matrix<double, 2*N_CART, 1>& generalized_forces_world,
                                                                                    double& moment) const
{
  Eigen::Matrix4d foot_sole_frame_inv = foot_sole_frame;
  foot_sole_frame_inv.block(0,0,3,3).transposeInPlace();
  foot_sole_frame_inv.block(0,3,3,1) = -foot_sole_frame_inv.block(0,0,3,3) * foot_sole_frame.block(0,3,3,1);
  Eigen::Matrix<double, 3, 1> ankle_pos_foot = foot_sole_frame_inv.block(0,0,3,3) * ankle_pos +
      foot_sole_frame_inv.block(0,3,3,1);

  Eigen::Matrix<double, 3, 3> a_cross_rot_trans = GeometryUtils::vector_to_skew_matrix(ankle_pos_foot);
  a_cross_rot_trans = a_cross_rot_trans*foot_sole_frame_inv.block(0,0,3,3);

  Eigen::Matrix<double, 6, 3> lambda_quad = Eigen::Matrix<double, 6, 3>::Zero();
  lambda_quad.block(3,0,3,3) = Eigen::Matrix<double, 3, 3>::Identity();
  for(unsigned int i=0; i<3; ++i)
  {
    lambda_quad.block(0,0,3,3) += a_cross_rot_trans.block(i,0,1,3).transpose() * foot_sole_frame_inv.block(i,0,1,3);
  }

  moment = (generalized_forces_world.transpose() * lambda_quad *generalized_forces_world.block(0,0,3,1))[0];
  moment /= (foot_sole_frame_inv.block(1,0,1,3) * generalized_forces_world.block(0,0,3,1))[0];
}



bool MomentumComputation::computeFootCoP(const Eigen::Vector3d& ankle_pos,
                                                                                  const Eigen::Matrix4d& foot_sole_frame,  Eigen::Matrix<double, 2*N_CART, 1>& generalized_forces_world,
                                                                                  Eigen::Vector3d& cop)
{
  Eigen::Matrix4d foot_sole_frame_inv = foot_sole_frame;
  foot_sole_frame_inv.block<3,3>(0,0).transposeInPlace();
  foot_sole_frame_inv.block<3,1>(0,3) = -foot_sole_frame_inv.block<3,3>(0,0) * foot_sole_frame.block<3,1>(0,3);

  Eigen::Vector3d ankle_pos_foot = foot_sole_frame_inv.block<3,3>(0,0) * ankle_pos +
      foot_sole_frame_inv.block<3,1>(0,3);

  Eigen::Matrix<double, 6, 1> gen_frcs_foot;
  gen_frcs_foot.block<3,1>(0,0) = foot_sole_frame.block<3,3>(0,0).transpose() * generalized_forces_world.block<3,1>(0,0);
  gen_frcs_foot.block<3,1>(3,0) = foot_sole_frame.block<3,3>(0,0).transpose() * generalized_forces_world.block<3,1>(3,0);

  Eigen::Matrix<double, 3, 1> ank_mom = Eigen::Matrix<double, 3, 1>::Zero();
  ank_mom = ankle_pos_foot.cross(gen_frcs_foot.block<3,1>(0,0));

  cop[0] = (gen_frcs_foot[5] + ank_mom[0])/gen_frcs_foot[1];
  cop[1] = 0.0;
  cop[2] = -(gen_frcs_foot[3] + ank_mom[2])/gen_frcs_foot[1];

  return true;
}

} /* namespace floating_base_utilities */
