/*********************************************************************
 Autonomous Motion Department
 Max-Planck Intelligent Systems
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         HInvDynExample.h

 \author       Alexander Herzog
 \date         Aug 18, 2014

 *********************************************************************/

#pragma once

#include <string>
#include <Eigen/Eigen>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include "DynamicsConstraint.h"
#include "CartesianPositionCtrl.h"
#include "CartesianForceCtrl.h"
#include "MomentumRateCtrl.h"
#include "JointSpaceCtrl.h"
#include "HierarchInverseDynamics.h"
#include "KinematicsEigen.h"
#include "MomentumComputation.h"
#include "FloatingBaseKinematics.h"
#include "FootContactHandlerHermes.h"

namespace hierarchical_inverse_dynamics_example {

class HInvDynExample {
 public:
  HInvDynExample();
  virtual ~HInvDynExample(){};

  int run();

 private:
  /* Unfortunately, right now we need to know the maximum size of our
   * tasks.
   */
  // We have upper and lower torque limit constraints
  static const int Max_Ineq_Rows = 2*N_DOFS;
  // We will do momentum control (6 rows) and constrain
  // the feet not to move (6 each)
  static const int Max_Eq_Rows = 6+2*6+N_DOFS+6+2*6;

  // we will use a config file to tune parameters conveniently
  std::string config_file_;

  // this is to set endeffectors as un-/constrained
  SL_endeff endeff_constraints_[N_ENDEFFS+1];

  // our helper classes to construct various quanitties with eigen
  floating_base_utilities::KinematicsEigen kinematics_;
  floating_base_utilities::FloatingBaseKinematics endeff_kinematics_;
  floating_base_utilities::MomentumComputation momentum_helper_;
  momentum_balance_control::FootContactHandlerHermes contact_helper_;

  // the hierarchical inverse dynamics solver and task composers
  floating_base_utilities::HierarchInverseDynamics<Max_Ineq_Rows,
      Max_Eq_Rows> hinvdyn_solver_;
  floating_base_utilities::DynamicsConstraint<Max_Ineq_Rows,
      Max_Eq_Rows> dyn_eqs_;
  floating_base_utilities::CartesianPositionCtrl<Max_Ineq_Rows,
      Max_Eq_Rows> left_foot_constr_, right_foot_constr_;
  floating_base_utilities::MomentumRateCtrl<Max_Ineq_Rows,
      Max_Eq_Rows> cog_ctrl_;
  floating_base_utilities::JointSpaceCtrl<Max_Ineq_Rows,
      Max_Eq_Rows> joint_ctrl_;
  floating_base_utilities::CartesianForceCtrl<Max_Ineq_Rows,
      Max_Eq_Rows> left_frc_reg_, right_frc_reg_;

  // ranks of our tasks
  int foot_constr_rank_, joint_ctrl_rank_, cog_ctrl_rank_, frc_reg_rank_;

  // this is for PD control
  Eigen::Vector3d cog_des_, cog_p_gains_, cog_d_gains_;
  Eigen::Matrix<double, 6, 1> cog_ref_;
  Eigen::Matrix<double, N_DOFS+6,1> default_posture_;
  Eigen::Matrix<double, N_DOFS+6,1> posture_p_gains_, posture_d_gains_;

  // weights
  Eigen::Matrix<double, 6, 1> foot_constr_weight_;
  Eigen::Matrix<double, 6, 1> frc_reg_weight_;
  Eigen::Matrix<double, 6, 1> cog_ctrl_weight_;
  Eigen::Matrix<double, N_DOFS+6, 1> joint_ctrl_weight_;

  // parameters for the push simulation
  double push_force_, push_dur_;

  // some info about task
  double task_start_time_;
  Eigen::Matrix<double, N_DOFS,1> init_joint_state_uff_, init_joint_state_th_, init_joint_state_thd_, init_joint_state_thdd_;
};

}  // Namespace
