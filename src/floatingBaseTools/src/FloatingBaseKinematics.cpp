/*
 * FloatingBaseKinematic.cpp
 *
 *  Created on: Mar 5, 2012
 *      Author: righetti
 */



#include "FloatingBaseKinematics.h"
#include "SL.h"
#include "SL_common.h"
#include "SL_kinematics.h"

namespace floating_base_utilities
{


FloatingBaseKinematics::FloatingBaseKinematics()
{
  num_constraints_ = 0;

  valid_jacobian_ = false;
  previous_constraints_.setZero();

  Xmcog_ = my_matrix(0, N_DOFS, 1, 3);
  Xaxis_ = my_matrix(0, N_DOFS, 1, 3);
  Xorigin_ = my_matrix(0, N_DOFS, 1, 3);
  Xlink_ = my_matrix(0, N_LINKS, 1, 3);
  for (int i=0; i<=N_LINKS; ++i)
  {
    Ahmat_[i] = my_matrix(1, 4, 1, 4);
  }
  for (int i=0; i<=N_DOFS; ++i)
  {
    Ahmat_dof_[i] = my_matrix(1, 4, 1, 4);
  }
  endeff_joint_jacobians_ = my_matrix(1, 2*N_CART*N_ENDEFFS, 1, N_DOFS);
  endeff_base_jacobians_ = my_matrix(1, 2*N_CART*N_ENDEFFS, 1, 2*N_CART);
}

FloatingBaseKinematics::~FloatingBaseKinematics()
{
  my_free_matrix(Xmcog_, 0, N_DOFS, 1, 3);
  my_free_matrix(Xaxis_, 0, N_DOFS, 1, 3);
  my_free_matrix(Xorigin_, 0, N_DOFS, 1, 3);
  my_free_matrix(Xlink_, 0, N_LINKS, 1, 3);
  for (int i=0; i<=N_LINKS; ++i)
  {
    my_free_matrix(Ahmat_[i], 1, 4, 1, 4);
  }
  for (int i=0; i<=N_DOFS; ++i)
  {
    my_free_matrix(Ahmat_dof_[i], 1, 4, 1, 4);
  }
  my_free_matrix(endeff_joint_jacobians_, 1, 2*N_CART*N_ENDEFFS, 1, N_DOFS);
  my_free_matrix(endeff_base_jacobians_, 1, 2*N_CART*N_ENDEFFS, 1, 2*N_CART);
}


void floating_base_utilities::FloatingBaseKinematics::computeQRDecomposition(bool update_projectors)
{
  //compute the QR decomposition of the Jacobian
  //Q = V ; R = s U^T
  jc_svd_.compute(constrained_jacobian_, Eigen::ComputeFullU | Eigen::ComputeFullV);

  //  j_svd_u_ =  jc_svd_.matrixU();
  //  j_svd_v_ =  jc_svd_.matrixV();
  //compute R inverse
  R_inverse_ = jc_svd_.matrixU().block(0, 0, num_constraints_, num_constraints_) *
      jc_svd_.singularValues().block(0,0,num_constraints_,1).asDiagonal().inverse();


  //update the Jacobian stuff
  if(update_projectors)
  {
    //compute the pseudo inverse
    Jc_pinv_.setZero();
    //get R-1 transpose
    Jc_pinv_.topLeftCorner(num_constraints_, num_constraints_) = R_inverse_.transpose();
    Jc_pinv_ = jc_svd_.matrixV() * Jc_pinv_;

    //compute the nullspace projector
    null_Jc_ = Eigen::Matrix<double, N_DOFS+6, N_DOFS+6>::Identity()- Jc_pinv_*constrained_jacobian_;
  }
  else
  {
    valid_jacobian_ = false;
  }
}

static void my_mat_zero_size(Matrix m, int nrl, int nrh, int ncl, int nch) // mat_zero_size that really works
{
  double* chunk = m[nrl] + ncl;
  int size = (nrh-nrl+1) * (nch-ncl+1);
  memset(chunk, 0, size * sizeof(double));
}

void FloatingBaseKinematics::computeJacobians(SL_Jstate* j_state, SL_Cstate& base_state, SL_quat& base_orient, SL_endeff* end_effectors, int& num_constraints)
{
  // zero everything to be conservative (TODO: check which of these might not be needed)
  my_mat_zero_size(Xmcog_, 0, N_DOFS, 1, 3);
  my_mat_zero_size(Xaxis_, 0, N_DOFS, 1, 3);
  my_mat_zero_size(Xorigin_, 0, N_DOFS, 1, 3);
  my_mat_zero_size(Xlink_, 0, N_LINKS, 1, 3);
  for (int i=0; i<=N_LINKS; ++i) my_mat_zero_size(Ahmat_[i], 1, 4, 1, 4);
  for (int i=0; i<=N_DOFS; ++i) my_mat_zero_size(Ahmat_dof_[i], 1, 4, 1, 4);
  my_mat_zero_size(endeff_joint_jacobians_, 1, 2*N_CART*N_ENDEFFS, 1, N_DOFS);
  my_mat_zero_size(endeff_base_jacobians_, 1, 2*N_CART*N_ENDEFFS, 1, 2*N_CART);

  constrained_jacobian_.setZero(6*N_ENDEFFS, N_DOFS+6);
  unconstrained_jacobian_.setZero(6*N_ENDEFFS, N_DOFS+6);

  // compute the link information for this state
  linkInformation(j_state, &base_state, &base_orient, end_effectors,
                  Xmcog_, Xaxis_, Xorigin_, Xlink_, Ahmat_, Ahmat_dof_);

  // compute Jacobian
  jacobian(Xlink_, Xorigin_, Xaxis_, endeff_joint_jacobians_);

  // create the Jc matrix
  int count = 0;
  int count2 = 0;
  for (int i=1; i<=N_ENDEFFS; ++i)
  {

    end_effectors_[i] = end_effectors[i];

    // compute the Jacobian component due to the base: these results
    // come from cross productcs of the unit vectors of the base coordinate
    // with the endeffector-base position.


    // calculate the jacobian of the end effector velocities with respect to the base
    // velocities.

    // check out Sicliano et al; Robotics: Modelling, planning, and control
    // The angular velocity of the base turns into a translational velocity
    // of the end effector via a cross product. See, e.g. Sicliano eq 3.26
    // p_effectorDot = p_BaseDot + omega_Base cross r_{base, effector}
    // note (a cross b) = -(b cross a)
    // p_effectorDot = p_BaseDot - r_{base, effector} cross omega_Base
    // p_effectorDot = [(eye(3,3), crossProductMatrix(r_{base, effector})] * [p_BaseDot, omega_Base]'
    // so our jacobian for position is [(eye(3,3), crossProductMatrix(r_{base, effector})]
    // for angular velocity, J = [zeros(3,3), eye(3,3)]

    int ej_index = (i - 1) * 2 * N_CART;
    endeff_base_jacobians_[ej_index + _Y_][N_CART+_X_] = -(Xlink_[link2endeffmap[i]][_Z_] - base_state.x[_Z_]);
    endeff_base_jacobians_[ej_index + _Z_][N_CART+_X_] =   Xlink_[link2endeffmap[i]][_Y_] - base_state.x[_Y_];

    endeff_base_jacobians_[ej_index + _X_][N_CART+_Y_] =   Xlink_[link2endeffmap[i]][_Z_] - base_state.x[_Z_];
    endeff_base_jacobians_[ej_index + _Z_][N_CART+_Y_] = -(Xlink_[link2endeffmap[i]][_X_] - base_state.x[_X_]);

    endeff_base_jacobians_[ej_index + _X_][N_CART+_Z_] = -(Xlink_[link2endeffmap[i]][_Y_] - base_state.x[_Y_]);
    endeff_base_jacobians_[ej_index + _Y_][N_CART+_Z_] =   Xlink_[link2endeffmap[i]][_X_] - base_state.x[_X_];


    // all of the diagonal linear and angular velocity terms translate
    // directly from the base to the end effector
    for (int j=1; j<=2*N_CART; ++j)
    {
      endeff_base_jacobians_[ej_index + j][j] = 1.0;
    }

    // these are the contributions of each joint velocity to the end effector velocities.
    // they are split into constrained and unconstrained matrices
    for (int j=1; j<=2*N_CART; ++j)
    {

      if (end_effectors_[i].c[j])
      {
        ++count;

        for (int m=1; m<=N_DOFS; ++m)
          constrained_jacobian_(count-1,m-1) = endeff_joint_jacobians_[ej_index + j][m];

        for (int m=1; m<=2*N_CART; ++m)
          constrained_jacobian_(count-1,N_DOFS+m-1) = endeff_base_jacobians_[ej_index + j][m];

      }
      else
      {
        ++count2;

        for (int m=1; m<=N_DOFS; ++m)
          unconstrained_jacobian_(count2-1,m-1) = endeff_joint_jacobians_[ej_index + j][m];

        for (int m=1; m<=2*N_CART; ++m)
          unconstrained_jacobian_(count2-1,N_DOFS+m-1) = endeff_base_jacobians_[ej_index + j][m];

      }

      // if the endeffector constraint conditions change at all then we can't trust the derivative,
      if(end_effectors_[i].c[j] != previous_constraints_(i-1,j-1))
      {
        valid_jacobian_ = false;
        previous_constraints_(i-1,j-1) = end_effectors[i].c[j];
      }
    }
  }

  // the jacobian derivatives are just numerically differentiated from the jacobians at this minus the last
  // timestep
  if(valid_jacobian_)
  {
    dJc_ = (constrained_jacobian_ - previous_Jc_)*double(task_servo_rate);
    dJunconstr_ = (unconstrained_jacobian_ - previous_Junconstr_)*double(task_servo_rate);
  }
  else
  {
    dJc_.setZero();
    dJunconstr_.setZero();
    valid_jacobian_ = true;
  }

  //save Jc in previous Jc
  previous_Jc_ = constrained_jacobian_;
  previous_Junconstr_ = unconstrained_jacobian_;

  num_constraints_ = count;
  num_unconstrained_dimensions_ = N_DOFS+6 - num_constraints_;

  num_constraints = num_constraints_;
}



void FloatingBaseKinematics::getConstrainedJacobian(Eigen::Matrix<double, 6 * N_ENDEFFS, N_DOFS + 6>& constrained_jacobian)
{
  constrained_jacobian = constrained_jacobian_;
}


void FloatingBaseKinematics::getUnconstrainedJacobian(Eigen::Matrix<double, 6 * N_ENDEFFS, N_DOFS + 6>& unconstrained_jacobian)
{
  unconstrained_jacobian = unconstrained_jacobian_;
}


void FloatingBaseKinematics::getNullspaceProjector
(Eigen::Matrix<double, N_DOFS+6, N_DOFS+6>& null_Jc)
{
  null_Jc = null_Jc_;
}


void FloatingBaseKinematics::getConstraintJacobian
(Eigen::Matrix<double, 6*N_ENDEFFS, N_DOFS+6>& Jc,
 Eigen::Matrix<double, 6*N_ENDEFFS, N_DOFS+6>& dJc,
 Eigen::Matrix<double, N_DOFS+6, 6*N_ENDEFFS>& Jc_pinv,
 Eigen::Matrix<double, N_DOFS+6, N_DOFS+6>& null_Jc)
{
  Jc = constrained_jacobian_;
  dJc = dJc_;
  Jc_pinv = Jc_pinv_;
  null_Jc = null_Jc_;
}


void FloatingBaseKinematics::getConstraintJacobian(Matrix Jc, Matrix dJc, Matrix Jc_pinv, Matrix null_Jc)
{
  for(int i=0; i<6*N_ENDEFFS; ++i)
    for(int j=0; j<N_DOFS+6; ++j)
    {
      Jc[i+1][j+1] = constrained_jacobian_(i,j);
      dJc[i+1][j+1] = dJc_(i,j);
      Jc_pinv[j+1][i+1] = Jc_pinv_(j,i);
    }
  for(int i=0; i<N_DOFS+6; ++i)
    for(int j=0; j<N_DOFS+6; ++j)
    {
      null_Jc[i+1][j+1] = null_Jc_(i,j);
    }
}



}
