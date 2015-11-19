/*
 * OptimalInverseDynamicsEigen.cpp
 *
 *  Created on: Mar 5, 2012
 *      Author: righetti
 */


#include "eiquadprog.hpp"
#include <cassert>
#include <numeric>

#include "OptimalInverseDynamicsEigen.h"


namespace floating_base_utilities
{


OptimalInverseDynamicsEigen::OptimalInverseDynamicsEigen()
{
  initialized_ = false;
}

  // The most general cost funtion is:
  // 0.5*tau^{T}*command_cost_weight_*tau + command_cost_linear_term_^{T}*tau + 0.5*lambda^{T}*contact_cost_weight_*lambda + contact_cost_linear_term_^{T}*lambda
  // By default, this solver minimizes the cost function min_{tau} 0.5||tau||^{2} s.t. (dynamics).
bool OptimalInverseDynamicsEigen::initialize()
{
  initialized_ = true;

  num_constraints_ = 0;

  command_cost_weight_.setIdentity();
  contact_cost_weight_.setZero();
  command_cost_linear_term_.setZero();
  contact_cost_linear_term_.setZero();

  optimal_force_objectives_.setZero();
  optimal_torque_objectives_.setZero();

  use_contact_inequalities_ = false;
  use_torque_saturation_ = false;

  for(int i=0; i<N_DOFS; ++i)
    torque_limits_(i,0) = u_max[i+1];

  return initialized_;
}


void OptimalInverseDynamicsEigen::getPredictedReactionForces(Vector reaction_forces)
{
  assert(initialized_);

  for(int i=0; i<6*N_ENDEFFS; ++i)
  {
    reaction_forces[i+1] = predicted_reaction_forces_(i,0);
  }
}


void OptimalInverseDynamicsEigen::getPredictedReactionForces(std::vector<double>& reaction_forces)
{
  assert(initialized_);

  if(reaction_forces.size() < 6*N_ENDEFFS)
    reaction_forces.resize(6*N_ENDEFFS);

  for(int i=0; i<6*N_ENDEFFS; ++i)
    reaction_forces[i] = predicted_reaction_forces_(i,0);

}


void OptimalInverseDynamicsEigen::setDesiredForces(Vector desired_forces, Matrix force_weight)
{
  assert(initialized_);
  Eigen::Matrix<double, 6*N_ENDEFFS, 1> f;
  Eigen::Matrix<double, 6*N_ENDEFFS, 6*N_ENDEFFS> w;
  for(int i=0; i<6*N_ENDEFFS; ++i)
  {
    f(i) = desired_forces[i+1];
    for(int j=0; j<6*N_ENDEFFS; ++j)
      w(i,j) = force_weight[i+1][j+1];
  }
  setDesiredForces(f, w);
}



void OptimalInverseDynamicsEigen::setDesiredForces(
    const Eigen::Matrix<double, 6*N_ENDEFFS, 1>& desired_forces,
    const Eigen::Matrix<double, 6*N_ENDEFFS, 6*N_ENDEFFS>& force_weight
)
{
  assert(initialized_);

  optimal_force_objectives_ = desired_forces;
  contact_cost_weight_ = force_weight;
  contact_cost_linear_term_ = - desired_forces.transpose() * force_weight;

}


void OptimalInverseDynamicsEigen::setDesiredTorques(Vector desired_torques, Matrix torque_weight)
{
  assert(initialized_);
  Eigen::Matrix<double, N_DOFS, 1> t;
  Eigen::Matrix<double, N_DOFS, N_DOFS> w;
  for(int i=0; i<N_DOFS; ++i)
  {
    t(i) = desired_torques[i+1];
    for(int j=0; j<N_DOFS; ++j)
      w(i,j) = torque_weight[i+1][j+1];
  }
  setDesiredTorques(t, w);
}


void OptimalInverseDynamicsEigen::setDesiredTorques(
    const Eigen::Matrix<double, N_DOFS, 1>& desired_torques,
    const Eigen::Matrix<double, N_DOFS, N_DOFS>& torque_weight
)
{
  assert(initialized_);

  optimal_torque_objectives_ = desired_torques;
  command_cost_weight_ = torque_weight;
  command_cost_linear_term_ = - desired_torques.transpose() * torque_weight;
}


bool OptimalInverseDynamicsEigen::prepareInverseDynamics(FloatingBaseKinematics& kinematics)
{
  assert(initialized_);

  //get the constrained Jacobian
  num_constraints_ = kinematics.getNumConstraints();
  num_unconstrained_dimensions_ = N_DOFS + 6 - num_constraints_;

  if(num_constraints_ < 6)
  {
    printf("ERROR constraints = %d < 6- not implemented yet how to deal with that.", num_constraints_);
    return false;
  }


  //compute the contact projector
  Eigen::Matrix<double, Eigen::Dynamic, N_DOFS+6, 0, 6*N_ENDEFFS, N_DOFS+6> contact_proj;
  contact_proj = kinematics.R_inverse() * kinematics.Q().block(0,0,N_DOFS+6, num_constraints_).transpose();
  contact_projector_.setZero();
  int index = 0;
  for(int i=0; i<N_ENDEFFS; ++i)
  {
    for(int j=0; j<6; ++j)
    {
      //copy the indexed line of contact_proj to contact projector
      if(kinematics.end_effectors_[i+1].c[j+1])
      {
        contact_projector_.row(i*6 + j) = contact_proj.row(index);
        ++index;
      }
    }
  }

  //compute the pseudo inverse of the constraint Jac
  //compute R^-1 * Q^T
  Eigen::Matrix<double, N_DOFS+6, N_DOFS+6> invRQT = Eigen::Matrix<double, N_DOFS+6, N_DOFS+6>::Identity();
  invRQT.topLeftCorner(num_constraints_, num_constraints_) = kinematics.R_inverse().topLeftCorner(num_constraints_, num_constraints_);
  invRQT = invRQT * kinematics.Q().transpose();
  if(!updatePseudoInverseWeight(kinematics.end_effectors_, invRQT))
  {
    printf("cannot update weights.");
    return false;
  }

  Eigen::Matrix<double, N_DOFS, Eigen::Dynamic, 0, N_DOFS, N_DOFS + 6> weight_SQu =
      cost_weight_inverse_
      * kinematics.Q().block(0,num_constraints_,N_DOFS,num_unconstrained_dimensions_);


  //compute the pseudo inverse
  Eigen::Matrix<double, N_DOFS, Eigen::Dynamic, 0, N_DOFS, N_DOFS+6> pseudo_inverse =
      weight_SQu *
      (kinematics.Q().block(0,num_constraints_,N_DOFS,num_unconstrained_dimensions_).transpose()
          * weight_SQu).inverse();

  //it is the nullspace projector - to be multiplied by tau_0
  internal_torque_projector_ = cost_weight_inverse_ - pseudo_inverse * weight_SQu.transpose();

  //it is the projector used to compute the normal ID (P* Mddq+h)
  torque_projector_ =
      pseudo_inverse
      * kinematics.Q().block(0,num_constraints_,N_DOFS+6, num_unconstrained_dimensions_).transpose();

  return true;
}


void OptimalInverseDynamicsEigen::computeTorques(
    FloatingBaseKinematics& kinematics,
    SL_DJstate* des_joint_state, SL_endeff* end_effectors , SL_Cstate& base_state, SL_quat& base_orient
)
{
  assert(initialized_);

  Eigen::Matrix<double, N_DOFS+6, 1> Mddq_h;
  MY_VECTOR(fbase, 1, 6);
  SL_InvDynNEBase(NULL, des_joint_state, end_effectors, &base_state, &base_orient, fbase);

  //get the full Mddq + h without local contributions
  for(int i=1; i<=N_DOFS; ++i)
  {
    Mddq_h(i-1) = des_joint_state[i].uff
        - des_joint_state[i].thd*links[i].vis
        - COULOMB_FUNCTION(des_joint_state[i].thd,coulomb_slope)*links[i].coul
        - des_joint_state[i].th * links[i].stiff
        - links[i].cons;
  }
  for(int i=1; i<=6; ++i)
  {
    Mddq_h(i+N_DOFS-1) = fbase[i];
  }

  //if QP does not do the job, use least square
  if(use_contact_inequalities_ || use_torque_saturation_)
  {
    if(!computeTorquesQP(kinematics, Mddq_h))
      computeTorquesLeastSquare(Mddq_h);
  }
  else
  {
    computeTorquesLeastSquare(Mddq_h);
  }

  //set the ID with local contributions
  for(int i=1; i<=N_DOFS; ++i)
  {
    des_joint_state[i].uff = desired_torques_(i-1) + des_joint_state[i].thd*links[i].vis
        + COULOMB_FUNCTION(des_joint_state[i].thd,coulomb_slope)*links[i].coul
        + des_joint_state[i].th * links[i].stiff
        + links[i].cons;
  }

  //compute the predicted reaction forces
  Mddq_h.topRows(N_DOFS).noalias() -= desired_torques_;
  predicted_reaction_forces_ = contact_projector_ * Mddq_h;
  predicted_constraint_cost_ =
      0.5 * predicted_reaction_forces_.transpose() * contact_cost_weight_ * predicted_reaction_forces_ +
      contact_cost_linear_term_.dot(predicted_reaction_forces_);

  predicted_command_cost_ = 0.5 * desired_torques_.transpose() * command_cost_weight_ * desired_torques_ +
      command_cost_linear_term_.dot(desired_torques_);
}


bool OptimalInverseDynamicsEigen::computeInverseDynamics(SL_DJstate* des_joint_state, SL_endeff* end_effectors, SL_Cstate& base_state, SL_quat& base_orient)
{
  assert(initialized_);

  SL_Jstate joint_st[N_DOFS+1];
  for(int i=1; i<=N_DOFS; ++i)
  {
    //don't copy the rest since we are only interested in the kinematics
    joint_st[i].th = des_joint_state[i].th;
  }

  FloatingBaseKinematics kinematics;
  kinematics.computeJacobians(joint_st, base_state, base_orient, end_effectors, num_constraints_);
  kinematics.computeQRDecomposition(false);

  if(!prepareInverseDynamics(kinematics))
  {
    return false;
  }

  computeTorques(kinematics, des_joint_state, end_effectors, base_state, base_orient);

  return true;
}


bool OptimalInverseDynamicsEigen::updatePseudoInverseWeight(SL_endeff* end_effectors, const Eigen::Matrix<double, N_DOFS+6, N_DOFS+6>& invRQT)
{
  assert(initialized_);

  full_constraint_weight_.setIdentity();
  int constrained_dim[6*N_ENDEFFS + 1];
  int index = 1;
  for(int i=1; i<=N_ENDEFFS; ++i)
  {
    for(int j=1; j<=6; ++j)
    {
      //check which dimensions get constrained
      if(end_effectors[i].c[j])
      {
        constrained_dim[6*(i-1) + j] = index;
        ++index;
      }
      else
      {
        constrained_dim[6*(i-1) + j] = 0;
      }
    }
  }
  //now we can create Wc and bc
  for(int i=1; i<=6*N_ENDEFFS; ++i)
  {
    if(constrained_dim[i] > 0)
    {
      for(int j=1; j<=6*N_ENDEFFS; ++j)
      {
        if(constrained_dim[j] > 0)
        {
          full_constraint_weight_(constrained_dim[i]-1,constrained_dim[j]-1) = contact_cost_weight_(i-1,j-1);
        }
      }
      full_constraint_linear_term_(constrained_dim[i]-1) = contact_cost_linear_term_(i-1);
    }
  }

  full_constraint_linear_term_ = invRQT.transpose() * full_constraint_linear_term_; //bc
  full_constraint_weight_ = invRQT.transpose() * full_constraint_weight_ * invRQT; //Wc
  cost_weight_ = command_cost_weight_ + full_constraint_weight_.topLeftCorner(N_DOFS,N_DOFS); //W
  cost_weight_inverse_ = cost_weight_.inverse(); // W^-1

  return true;
}


bool OptimalInverseDynamicsEigen::computeTorquesLeastSquare(const Eigen::Matrix<double, N_DOFS+6, 1>& Mddq_h)
{
  desired_torques_ =
      torque_projector_ * Mddq_h +
      internal_torque_projector_ *
      (full_constraint_weight_.topRows(N_DOFS) * Mddq_h + full_constraint_linear_term_.topRows(N_DOFS) - command_cost_linear_term_);

  return true;
}


bool OptimalInverseDynamicsEigen::computeTorquesQP(
    FloatingBaseKinematics& kinematics,
    const Eigen::Matrix<double, N_DOFS+6, 1>& Mddq_h
)
{
  //the weight
  Eigen::MatrixXd G1 = cost_weight_;

  //the linear part
  Eigen::VectorXd g0 = command_cost_linear_term_ - full_constraint_linear_term_.topRows(N_DOFS) - (full_constraint_weight_*Mddq_h).topRows(N_DOFS);

  //CE and ce0 - CE = QuTST
  Eigen::MatrixXd CE = kinematics.Q().block(0,num_constraints_,N_DOFS,num_unconstrained_dimensions_);
  Eigen::VectorXd ce0 = - kinematics.Q().block(0,num_constraints_,N_DOFS+6,num_unconstrained_dimensions_).transpose() * Mddq_h;

  //CI and ci0
  int num_ineq = 0;
  if(use_torque_saturation_)
    num_ineq += 2*N_DOFS;
  if(use_contact_inequalities_)
    num_ineq += contact_ineq_matrix_.rows();

  Eigen::MatrixXd CI;
  Eigen::VectorXd ci0;
  CI.resize(N_DOFS, num_ineq);
  ci0.resize(num_ineq);

  if(use_torque_saturation_)
  {
    Eigen::MatrixXd torque_sat = Eigen::MatrixXd::Zero(N_DOFS, 2*N_DOFS);
    Eigen::VectorXd torque_sat_limit = Eigen::VectorXd::Zero(2*N_DOFS);
    for(int i=0; i<N_DOFS; ++i)
    {
      torque_sat_limit(i) = torque_limits_(i,0);
      torque_sat(i,i) = -1.0;
      torque_sat_limit[i+N_DOFS] = torque_limits_(i,0);
      torque_sat(i,i+N_DOFS) = 1.0;
    }

    CI.leftCols(torque_sat.cols()).matrix()= torque_sat;
    ci0.topRows(torque_sat_limit.rows()).matrix()= torque_sat_limit;
  }

  if(use_contact_inequalities_)
  {
    Eigen::MatrixXd temp_ineq_contact1 = contact_ineq_matrix_ * contact_projector_.leftCols(N_DOFS);
    Eigen::VectorXd temp_ineq_contact2 = contact_ineq_vector_ - contact_ineq_matrix_ * contact_projector_ *Mddq_h;

    CI.rightCols(temp_ineq_contact1.rows()).matrix() = temp_ineq_contact1.transpose();
    ci0.bottomRows(temp_ineq_contact2.rows()).matrix() = temp_ineq_contact2;
  }
  Eigen::VectorXd des_torque = Eigen::VectorXd::Zero(N_DOFS);

  double result = Eigen::solve_quadprog(G1, g0, CE, ce0, CI, ci0, des_torque);

  //check if a solution was found
  if(result == std::numeric_limits<double>::infinity())
    return false;

  for(int i=0; i<N_DOFS; ++i)
    desired_torques_[i] = des_torque[i];

  return true;
}


void OptimalInverseDynamicsEigen::setTorqueSaturation(Vector saturation)
{
  assert(initialized_);

  for(int i=0; i<N_DOFS; ++i)
    torque_limits_(i,0) = saturation[i+1];

  use_torque_saturation_ = true;
}


void OptimalInverseDynamicsEigen::setTorqueSaturation(const Eigen::Matrix<double, N_DOFS, 1>& saturation)
{
  assert(initialized_);

  torque_limits_ = saturation;
  use_torque_saturation_ = true;
}



void OptimalInverseDynamicsEigen::unsetTorqueSaturation()
{
  use_torque_saturation_ = false;
}


void OptimalInverseDynamicsEigen::setContactInequalities(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, int num_ineq)
{
  assert(initialized_);

  if(num_ineq == 0)
  {
    use_contact_inequalities_ = false;
  }
  else
  {
    use_contact_inequalities_ = true;
    contact_ineq_vector_ = b;
    contact_ineq_matrix_ = A;
  }
}


void OptimalInverseDynamicsEigen::setContactInequalities(Matrix A, Vector b, int num_ineq)
{
  assert(initialized_);

  if(num_ineq == 0)
  {
    use_contact_inequalities_ = false;
  }
  else
  {
    use_contact_inequalities_ = true;
    contact_ineq_vector_.resize(num_ineq,1);
    contact_ineq_matrix_.resize(num_ineq, 6*N_ENDEFFS);
    for(int i=0; i<num_ineq; ++i)
    {
      contact_ineq_vector_(i) = b[i+1];
      for(int j=0; j<6*N_ENDEFFS; ++j)
        contact_ineq_matrix_(i,j) = A[i+1][j+1];
    }
  }
}

void OptimalInverseDynamicsEigen::unsetContactInequalities()
{
  use_contact_inequalities_ = false;
}






















}
