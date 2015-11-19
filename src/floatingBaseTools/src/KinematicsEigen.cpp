/*
 * KinematicsEigen.cpp
 *
 *  Created on: Apr 1, 2013
 *      Author: herzog
 */

#include <iostream>
#include <Eigen/Eigen>
#include "KinematicsEigen.h"

#include "SL.h"
#include "SL_common.h"
#include "SL_kinematics.h"

namespace floating_base_utilities
{


KinematicsEigen::KinematicsEigen()
{
  cog_ = Eigen::Vector3d::Zero();
  robot_mass_ = -1.0;

  full_body_joint_jacobian_ = Eigen::Matrix<double, 2*N_CART*N_DOFS+2*N_CART, N_DOFS+2*N_CART>::Zero();
  joint_poses_ = Eigen::Matrix<double, 4 * (N_DOFS + 1), 4>::Zero();
  link_positions_ = Eigen::Matrix<double, 3, (N_LINKS + 1)>::Zero();
  joint_positions_ = Eigen::Matrix<double, N_DOFS + 6, 1>::Zero();
  link_com_poitions_ = Eigen::Matrix<double, 3, (N_DOFS + 1)>::Zero();
  joint_velocities_ = Eigen::Matrix<double, N_DOFS + 6, 1>::Zero();
  joint_accelerations_ = Eigen::Matrix<double, N_DOFS + 6, 1>::Zero();
  link_poses_ = Eigen::Matrix<double, 4 * (N_LINKS + 1), 4>::Zero();
  joint_cart_poitions_ = Eigen::Matrix<double, 3, (N_DOFS + 1)>::Zero();
  joint_axes_ = Eigen::Matrix<double, 3, (N_DOFS + 1)>::Zero();

  xmcog = my_matrix(0, N_DOFS, 1, 3);
  xaxis = my_matrix(0, N_DOFS, 1, 3);
  xorigin = my_matrix(0, N_DOFS, 1, 3);
  xlink = my_matrix(0, N_LINKS, 1, 3);

  for(int i=0; i<= N_LINKS; ++i)
    ahmat[i] = my_matrix(1,4,1,4);
  for(int i=0; i<= N_DOFS; ++i)
    ahmat_dof[i] = my_matrix(1,4,1,4);
}


KinematicsEigen::~KinematicsEigen()
{
  my_free_matrix(xmcog, 0, N_DOFS, 1, 3);
  my_free_matrix(xaxis, 0, N_DOFS, 1, 3);
  my_free_matrix(xorigin, 0, N_DOFS, 1, 3);
  my_free_matrix(xlink, 0, N_LINKS, 1, 3);

  for(int i=0; i<= N_LINKS; ++i)
     my_free_matrix(ahmat[i], 1,4,1,4);
  for(int i=0; i<= N_DOFS; ++i)
     my_free_matrix(ahmat_dof[i], 1,4,1,4);
}


void KinematicsEigen::initialize(const Eigen::Matrix<double, N_DOFS + 6, 1>& joint_positions,
    const Eigen::Matrix<double, N_DOFS + 6, 1>& joint_velocities,
    const Eigen::Matrix<double, N_DOFS + 6, 1>& joint_accelerations,
    SL_endeff* eff)
{
  initialize();
  update(joint_positions, joint_velocities, joint_accelerations, eff);
  is_link_jacobian_computed_.setConstant(false);
  is_prev_link_jacobian_computed_.setConstant(false);
}

void KinematicsEigen::initialize(SL_Jstate* state, SL_Cstate& basec, SL_quat& baseo, SL_endeff* eff)
{
  initialize();
  update(state, basec, baseo, eff);
  is_link_jacobian_computed_.setConstant(false);
  is_prev_link_jacobian_computed_.setConstant(false);
}

void KinematicsEigen::initialize()
{
  robot_mass_ = 0.0;
  for (int i = 0; i <= N_DOFS; ++i)
    robot_mass_ += links[i].m;
  for(int i=1; i<=N_ENDEFFS;++i)
  {
    double buffer[10];
    char var_name[100];
    std::string tmp_str = std::string("STD_FRAME_") + std::string(cart_names[i]);
    std::strcpy(var_name, tmp_str.c_str());
    if(!read_parameter_pool_double_array("floatingBaseTools.cf", var_name, 9, buffer))
    	assert(false && "error reading parameter");
    standardized_endeff_frames_[i].setIdentity();
    standardized_endeff_frames_[i].topLeftCorner(3,3) = Eigen::Map<
          Eigen::Matrix<double, 3,3,Eigen::RowMajor,3,3> >(&buffer[1]);
//    std::cout << standardized_endeff_frames_[i];
  }

  is_link_jacobian_computed_.setConstant(false);
  is_prev_link_jacobian_computed_.setConstant(false);
}


void KinematicsEigen::standardizedEndeffPose(int id, Eigen::Matrix4d& pose) const
{
  pose = standardizedEndeffPose(id);
}

Eigen::Matrix4d KinematicsEigen::standardizedEndeffPose(int id) const
{
	return endeffPose(id)*standardized_endeff_frames_[id];
}

void KinematicsEigen::getSlState(SL_Jstate* state, SL_Cstate& basec, SL_quat& baseo) const
{
	state[0].th = state[0].thd = state[0].thdd =
			state[0].u = state[0].ufb = state[0].load = 0.0;

  for (unsigned int i = 1; i <= N_DOFS; ++i)
  {
    state[i].th = joint_positions_(i - 1, 0);
    state[i].thd = joint_velocities_(i - 1, 0);
    state[i].thdd = joint_accelerations_(i - 1, 0);
    state[i].u = state[i].ufb = state[i].load = 0.0;
  }

  MY_VECTOR(eul, 1, 3);
  for (unsigned int i = 1; i <= N_CART; ++i)
  {
    basec.x[i] = joint_positions_(N_DOFS + i - 1, 0);
    eul[i] = joint_positions_(N_DOFS + 3 + i - 1, 0);

    basec.xd[i] = joint_velocities_(N_DOFS + i - 1, 0);
    basec.xdd[i] = joint_accelerations_(N_DOFS + i - 1, 0);
    baseo.ad[i] = joint_velocities_(N_DOFS + 3 + i - 1, 0);
    baseo.add[i] = joint_accelerations_(N_DOFS + 3 + i - 1, 0);
  }
  baseo.q[1] =baseo.q[2] =baseo.q[3] =baseo.q[4] =0.0;
  eulerToQuat(eul, &baseo);
  quatDerivatives(&baseo);
}


void KinematicsEigen::eigenGeneralizedJointVelocitiesToSlPose(const Eigen::Matrix<double, 6, 1>& joint_velocities,
                                                              SL_quat& rot, SL_Cstate& lin) const
{
  for (unsigned int i = 1; i <= N_CART; ++i)
  {
    lin.xd[i] = joint_velocities(i - 1, 0);
    rot.ad[i] = joint_velocities(3 + i - 1, 0);
  }
}


void KinematicsEigen::eigenGeneralizedJointPositionsToSlPose(const Eigen::Matrix<double, 6, 1>& joint_positions,
                                                             SL_quat& rot, SL_Cstate& pos) const
{
  MY_VECTOR(eul, 1, 3);
  for (unsigned int i = 1; i <= N_CART; ++i)
  {
    pos.x[i] = joint_positions(i - 1, 0);
    eul[i] = joint_positions(3 + i - 1, 0);
  }
  eulerToQuat(eul, &rot);

}


void KinematicsEigen::slPoseToEigenGeneraliedJointPositions(SL_quat& rot, SL_Cstate& pos,
                                                            Eigen::Matrix<double, 6, 1>& joint_positions) const
{
  MY_VECTOR(eul, 1, 3);
  quatToEuler(&rot, eul);
  for (unsigned int i = 1; i <= N_CART; ++i)
  {
    joint_positions(i - 1, 0) = pos.x[i];
    joint_positions(3 + i - 1, 0) = eul[i];
  }

}


void KinematicsEigen::computeCartesianLinkInformation(SL_Jstate* state, SL_Cstate& basec, SL_quat& baseo,
                                                      SL_endeff* eff, Eigen::Vector3d& center_of_gravity,
                                                      Eigen::Matrix<double, 3, (N_DOFS + 1)>& joint_axes,
                                                      Eigen::Matrix<double, 3, (N_DOFS + 1)>& joint_cart_poitions,
                                                      Eigen::Matrix<double, 4 * (N_LINKS + 1), 4>& link_poses,
                                                      Eigen::Matrix<double, 3, (N_LINKS + 1)>& link_poitions,
                                                      Eigen::Matrix<double, 3, (N_DOFS + 1)>& link_com_poitions,
                                                      Eigen::Matrix<double, 4 * (N_DOFS + 1), 4>& joint_poses)
{
  linkInformation(state, &basec, &baseo, eff, xmcog, xaxis, xorigin, xlink, ahmat, ahmat_dof);

  for (unsigned int i = 0; i <= N_LINKS; ++i)
    for (unsigned int r = 1; r <= 4; ++r)
      for (unsigned int c = 1; c <= 4; ++c)
        link_poses(4 * i + r - 1, c - 1) = ahmat[i][r][c];

  for (unsigned int i = 0; i <= N_LINKS; ++i)
    for (unsigned int r = 1; r <= 3; ++r)
      link_poitions(r - 1, i) = xlink[i][r];

  center_of_gravity = Eigen::Vector3d::Zero();
  for (unsigned int i = 0; i <= N_DOFS; ++i)
  {
    joint_poses.block(4*i,0,4,4).setIdentity();
    for (unsigned int r = 1; r <= 3; ++r)
    {
      joint_cart_poitions(r - 1, i) = xorigin[i][r];
      joint_axes(r - 1, i) = xaxis[i][r];
      link_com_poitions(r - 1, i) = xmcog[i][r] / links[i].m;
      center_of_gravity(r - 1) += xmcog[i][r];

      for (int c = 1; c <= 4; ++c)
        joint_poses(4 * i + r - 1, c - 1) = ahmat_dof[i][r][c];
    }
  }

  center_of_gravity *= 1.0 / robot_mass_;
}


void KinematicsEigen::computeLinkJacobian(int i)
{
  link_jacobian_derivs_[i] = -link_jacobians_[i];

  MY_MATRIX(body_jac, 1, 2*N_CART, 1, N_DOFS + 2*N_CART);
  genJacobian(xlink[i], i, xorigin, xaxis, body_jac );

  for(int r=1; r<=2*N_CART; ++r)
    for(int c=1; c<=N_DOFS; ++c)
      link_jacobians_[i](r-1, c-1) = body_jac[r][c];

  if(0 == i) //Base link
    link_jacobians_[i].setZero();

  link_jacobians_[i].block(0, N_DOFS,2 * N_CART, 2 * N_CART).setIdentity();

  link_jacobians_[i](1, N_DOFS + N_CART) = -(xlink[i][3] - xorigin[0][3]);
  link_jacobians_[i](2, N_DOFS + N_CART) = xlink[i][2] - xorigin[0][2];

  link_jacobians_[i](0, N_DOFS + N_CART + 1) = xlink[i][3] - xorigin[0][3];
  link_jacobians_[i](2, N_DOFS + N_CART + 1) = -(xlink[i][1] - xorigin[0][1]);

  link_jacobians_[i](0, N_DOFS + N_CART + 2) = -(xlink[i][2] - xorigin[0][2]);
  link_jacobians_[i](1, N_DOFS + N_CART + 2) = xlink[i][1] - xorigin[0][1];

  link_jacobian_derivs_[i] += link_jacobians_[i];
  link_jacobian_derivs_[i] *= (double)task_servo_rate;

  if(!is_prev_link_jacobian_computed_[i])
  {
    link_jacobian_derivs_[i].setZero();
  }

  is_link_jacobian_computed_[i] = true;
}


void KinematicsEigen::computeFullBodyLinksJacobian()
{
  for(int i=1; i<=N_LINKS; ++i)
  {
    computeLinkJacobian(i);
  }
}


void KinematicsEigen::computeFullBodyJointsJacobian(
    const Eigen::Matrix<double, 3, (N_DOFS + 1)>& joint_axes,
    const Eigen::Matrix<double, 3, (N_DOFS + 1)>& joint_cart_poitions,
    Eigen::Matrix<double, 2 * N_CART * N_DOFS + 2 * N_CART, N_DOFS + 2 * N_CART>& jac) const
{

  // compute Jacobians
  jac.setZero();
  for (unsigned int l = 1; l <= N_DOFS; ++l)
  {
    for(int link_trav=l;link_trav!=0;link_trav = jointPredecessor[link_trav])
    {

      MY_VECTOR(endff_pos, 1,3);
      MY_VECTOR(j_orig, 1,3);
      MY_VECTOR(j_axis, 1,3);
      MY_VECTOR(jac_col, 1,6);

      for(int i=1;i<=3;++i)
      {
        endff_pos[i] = joint_cart_poitions(i-1, l);
        j_orig[i] = joint_cart_poitions(i-1, link_trav);
        j_axis[i] = joint_axes(i-1, link_trav);
      }

      if (prismatic_joint_flag[link_trav]) {
        prismaticGJacColumn( endff_pos,
                             j_orig,
                             j_axis,
                             jac_col );
      } else {
        revoluteGJacColumn( endff_pos,
                            j_orig,
                            j_axis,
                            jac_col );
      }
      for (int r=1; r<=2*N_CART; ++r)
        jac(l*2*N_CART + r-1, link_trav-1) = jac_col[r];
    }
  }

  //add the base part of the jacobians
  for (unsigned int i = 0; i <= N_DOFS; ++i) //start at BASE
  {
    int row_base;
    row_base = i * 2 * N_CART;

    jac.block(row_base, N_DOFS,2 * N_CART, 2 * N_CART).setIdentity();

    jac(row_base + 1, N_DOFS + N_CART) = -(joint_cart_poitions(2, i) - joint_cart_poitions(2, 0));
    jac(row_base + 2, N_DOFS + N_CART) = joint_cart_poitions(1, i) - joint_cart_poitions(1, 0);

    jac(row_base, N_DOFS + N_CART + 1) = joint_cart_poitions(2, i) - joint_cart_poitions(2, 0);
    jac(row_base + 2, N_DOFS + N_CART + 1) = -(joint_cart_poitions(0, i) - joint_cart_poitions(0, 0));

    jac(row_base, N_DOFS + N_CART + 2) = -(joint_cart_poitions(1, i) - joint_cart_poitions(1, 0));
    jac(row_base + 1, N_DOFS + N_CART + 2) = joint_cart_poitions(0, i) - joint_cart_poitions(0, 0);
  }
}


void KinematicsEigen::computeCartesianLinkInformation(const Eigen::Matrix<double, N_DOFS + 6, 1>& joint_positions,
                                                      SL_endeff* eff, Eigen::Vector3d& center_of_gravity,
                                                      Eigen::Matrix<double, 3, (N_DOFS + 1)>& joint_axes,
                                                      Eigen::Matrix<double, 3, (N_DOFS + 1)>& joint_cart_poitions,
                                                      Eigen::Matrix<double, 4 * (N_LINKS + 1), 4>& link_poses,
                                                      Eigen::Matrix<double, 3, (N_LINKS + 1)>& link_poitions,
                                                      Eigen::Matrix<double, 3, (N_DOFS + 1)>& link_com_poitions,
                                                      Eigen::Matrix<double, 4 * (N_DOFS + 1), 4>& joint_poses)
{
  SL_Jstate state[N_DOFS + 1];
  for (unsigned int i = 1; i <= N_DOFS; ++i)
  {
    state[i].th = joint_positions(i - 1, 0);
  }
  SL_Cstate basec;

  SL_quat baseo;
  baseo.q[_QW_] = 1.0;
  baseo.q[_QX_] = 0.0;
  baseo.q[_QY_] = 0.0;
  baseo.q[_QZ_] = 0.0;
  Eigen::Matrix<double, 6, 1> base_general_joint_pos = joint_positions.block(N_DOFS, 0,6, 1);
  eigenGeneralizedJointPositionsToSlPose(base_general_joint_pos, baseo, basec);

  computeCartesianLinkInformation(state, basec, baseo, eff, center_of_gravity, joint_axes, joint_cart_poitions,
                                  link_poses, link_poitions, link_com_poitions, joint_poses);
}


void KinematicsEigen::update(SL_Jstate* state, SL_Cstate& basec, SL_quat& baseo, const SL_endeff* eff)
{
  Eigen::Matrix<double, N_DOFS + 6, 1> joint_positions;
  Eigen::Matrix<double, N_DOFS + 6, 1> joint_velocities;
  Eigen::Matrix<double, N_DOFS + 6, 1> joint_accelerations;

  Eigen::Matrix<double, 6, 1> gen_joint_positions;
  slPoseToEigenGeneraliedJointPositions(baseo, basec, gen_joint_positions);
  joint_positions.block(N_DOFS, 0,6, 1) = gen_joint_positions;

  for(int i=0;i<3;++i)
  {
	  joint_velocities(N_DOFS + i, 0) = basec.xd[i+1];
	  joint_velocities(N_DOFS + 3 + i, 0) = baseo.ad[i+1];
	  joint_accelerations(N_DOFS + i, 0) = basec.xdd[i+1];
	  joint_accelerations(N_DOFS + 3 + i, 0) = baseo.add[i+1];
  }

  joint_accelerations(N_DOFS, 0) = basec.xdd[_X_];
  joint_accelerations(N_DOFS + 1, 0) = basec.xdd[_Y_];
  joint_accelerations(N_DOFS + 2, 0) = basec.xdd[_Z_];
  joint_accelerations(N_DOFS + 3, 0) = baseo.add[_A_];
  joint_accelerations(N_DOFS + 4, 0) = baseo.add[_B_];
  joint_accelerations(N_DOFS + 5, 0) = baseo.add[_G_];

  for (unsigned int i = 1; i <= N_DOFS; ++i)
  {
    joint_positions(i - 1, 0) = state[i].th;
    joint_velocities(i - 1, 0) = state[i].thd;
    joint_accelerations(i - 1, 0) = state[i].thdd;
  }

  update(joint_positions, joint_velocities, joint_accelerations, eff);
}


void KinematicsEigen::update(const Eigen::Matrix<double, N_DOFS + 6, 1>& joint_positions,
                             const Eigen::Matrix<double, N_DOFS + 6, 1>& joint_velocities,
                             const Eigen::Matrix<double, N_DOFS + 6, 1>& joint_accelerations, const SL_endeff* eff)
{
  memcpy(endeffectors_, eff, (N_ENDEFFS+1)*sizeof(SL_endeff));
  joint_positions_ = joint_positions;
  joint_velocities_ = joint_velocities;
  joint_accelerations_ = joint_accelerations;
  computeCartesianLinkInformation(joint_positions_, endeffectors_, cog_, joint_axes_, joint_cart_poitions_, link_poses_,
                                  link_positions_, link_com_poitions_, joint_poses_);
  computeFullBodyJointsJacobian(joint_axes_, joint_cart_poitions_, full_body_joint_jacobian_);
//  computeFullBodyLinksJacobian();

  is_prev_link_jacobian_computed_ = is_link_jacobian_computed_;
  is_link_jacobian_computed_.setConstant(false);
}


KinematicsEigen::KinematicsEigen(const KinematicsEigen& other)
{
  xmcog = my_matrix(0, N_DOFS, 1, 3);
  xaxis = my_matrix(0, N_DOFS, 1, 3);
  xorigin = my_matrix(0, N_DOFS, 1, 3);
  xlink = my_matrix(0, N_LINKS, 1, 3);

  joint_positions_  = other.joint_positions_;
  joint_velocities_  = other.joint_velocities_;
  joint_accelerations_  = other.joint_accelerations_;

  link_poses_  = other.link_poses_;
  joint_poses_  = other.joint_poses_;
  link_positions_  = other.link_positions_;
  joint_cart_poitions_    = other.joint_cart_poitions_;
  link_com_poitions_  = other.link_com_poitions_;
  joint_axes_  = other.joint_axes_;

  full_body_joint_jacobian_  = other.full_body_joint_jacobian_;
  cog_  = other.cog_;
  robot_mass_  = other.robot_mass_;

  is_link_jacobian_computed_ = other.is_link_jacobian_computed_;
  is_prev_link_jacobian_computed_ = other.is_prev_link_jacobian_computed_;

  for(int l=0; l<=N_LINKS; ++l)
  {
    link_jacobians_[l] = other.link_jacobians_[l];
    link_jacobian_derivs_[l] = other.link_jacobian_derivs_[l];

    ahmat[l] = my_matrix(1,4,1,4);
    for(int r=1; r<=4; ++r)
      for(int c=1; c<=4; ++c)
        ahmat[l][r][c] = other.ahmat[l][r][c];

    for(int c=1; c<=3; ++c)
      xlink[l][c] = other.xlink[l][c];
  }


  for(int j=0; j<= N_DOFS; ++j)
  {
    ahmat_dof[j] = my_matrix(1,4,1,4);
    for(int r=1; r<=4; ++r)
      for(int c=1; c<=4; ++c)
        ahmat_dof[j][r][c] = other.ahmat_dof[j][r][c];


    for(int c=1; c<=3; ++c)
    {
      xlink[j][c] = other.xlink[j][c];
      xmcog[j][c] = other.xmcog[j][c];
      xaxis[j][c] = other.xaxis[j][c];
      xorigin[j][c] = other.xorigin[j][c];
    }
  }

  for(int e=0; e<=N_ENDEFFS; ++e)
    endeffectors_[e] = other.endeffectors_[e];

}



const Eigen::Matrix<double, 2*N_CART, N_DOFS+2*N_CART>& KinematicsEigen::linkJacobian(int link_id)
{
  if(!is_link_jacobian_computed_[link_id])
    computeLinkJacobian(link_id);
  return link_jacobians_[link_id];
}


const Eigen::Matrix<double, 2*N_CART, N_DOFS+2*N_CART>& KinematicsEigen::linkJacobianDerivative(int link_id)
{
  if(!is_link_jacobian_computed_[link_id])
    computeLinkJacobian(link_id);
  return link_jacobian_derivs_[link_id];
}

} /* namespace floating_base_utilities */
