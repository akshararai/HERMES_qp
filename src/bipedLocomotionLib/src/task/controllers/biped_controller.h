/*
 * biped_controller.h
 *
 *  Created on: Jul 7, 2014
 *      Author: righetti
 */

#ifndef BIPED_CONTROLLER_H_
#define BIPED_CONTROLLER_H_

#include <Eigen/Eigen>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <string>

#include "HierarchInverseDynamics.hh"
#include "BipedalTaskComposer.h"
#include "KinematicsEigen.h"
#include "MomentumComputation.h"
#include "FloatingBaseKinematics.h"
#include "OptimalInverseDynamicsEigen.h"
#include "FootContactHandlerHermes.h"

class BipedController {
public:
  BipedController();
  virtual ~BipedController();

  bool initialize(std::string config_file);
  virtual bool run() = 0;

protected:
  bool computeDesiredComAcc();
  bool computeDesiredSwingAcc(bool swinging_leg = false);
  bool computeDesiredBaseAcc();
  void computeInvariantReferenceFrame();
  void sendOpenGL();

  void transformInInvariantFrame(const Eigen::Vector3d& wrld, Eigen::Vector3d& invariant);
  void transformInWorldFrame(const Eigen::Vector3d& invariant, Eigen::Vector3d& wrld);

  void rotateInInvariantFrame(const Eigen::Vector3d& wrld, Eigen::Vector3d& invariant);
  void rotateInWorldFrame(const Eigen::Vector3d& invariant, Eigen::Vector3d& wrld);
  void rotateInInvariantFrame(const Eigen::Quaterniond& wrld, Eigen::Quaterniond& invariant);
  void rotateInWorldFrame(const Eigen::Quaterniond& invariant, Eigen::Quaterniond& wrld);

  void computeLegOpSpInertia(int leg, Eigen::Matrix<double, 7, 7>& inertia);
  void computeJointOpSpInertia(Eigen::Matrix<double, N_DOFS, N_DOFS>& inertia);

  void update();

  bool initialized_;
  int leading_leg_, swing_leg_;

  //reference com and momentum in wrld and invariant frames
  Eigen::Vector3d reference_com_pos_, reference_com_vel_, reference_com_acc_;
  Eigen::Vector3d reference_ang_mom_, reference_ang_mom_rate_;
  Eigen::Vector3d reference_com_pos_wrld_, reference_com_vel_wrld_, reference_com_acc_wrld_;

  //generalized forces and cops
  Eigen::Matrix<double, 6, 1> generalized_force_[N_ENDEFFS+1]; //world frame
  Eigen::Vector3d cop_wrld_[N_ENDEFFS+1]; //world frame
  Eigen::Vector3d std_cop_[N_ENDEFFS+1]; //in std frame
  double std_cop_moment_[N_ENDEFFS+1];
  Eigen::Vector3d cog_, dcog_, ddcog_;
  Eigen::Vector3d cog_inv_, dcog_inv_;
  Eigen::Vector3d cop_[N_ENDEFFS+1];

  Eigen::Matrix<double, 6, 1> reference_contact_forces_[N_ENDEFFS+1];

  Eigen::Matrix4d endeff_frm_[N_ENDEFFS+1];

  Eigen::Vector4d foot_center_local_[N_ENDEFFS+1];

  //helper classes and hierarchical QP stuff
  boost::shared_ptr<floating_base_utilities::KinematicsEigen> kinematics_eigen_;
  boost::shared_ptr<floating_base_utilities::MomentumComputation> momentum_helper_;
  boost::shared_ptr<floating_base_utilities::FloatingBaseKinematics> floating_base_kin_;
  boost::shared_ptr<momentum_balance_control::FootContactHandlerHermes> foot_contact_handler_;
  boost::shared_ptr<momentum_balance_control::BipedalTaskComposer::HierarchInverseDynamicsHermes> hierarch_inv_dyn_;
  boost::shared_ptr<momentum_balance_control::BipedalTaskComposer> biped_task_comp_;

  //control of the base orientation
  boost::shared_ptr<momentum_balance_control::BipedalTaskComposer::CartesianPositionCtrlHermes> base_controller_;

  //control the swing foot
  //cartesian control
  boost::shared_ptr<momentum_balance_control::BipedalTaskComposer::CartesianPositionCtrlHermes> swing_foot_controller_[N_ENDEFFS+1];
  Eigen::Matrix<double, 6, 6> swing_foot_weight_;

  //joint position control
  boost::shared_ptr<momentum_balance_control::BipedalTaskComposer::JointPositionCtrlHermes> joint_position_controller_;
  boost::shared_ptr<momentum_balance_control::BipedalTaskComposer::LegJointPositionCtrlHermes> leg_joint_position_controller_[N_ENDEFFS+1];
  Eigen::Matrix<double, N_DOFS, 1> joint_ctrl_weight_;

  //these are given in the invariant frame
  Eigen::Vector3d reference_foot_position_[N_ENDEFFS+1];
  Eigen::Quaterniond reference_foot_orientation_[N_ENDEFFS+1];
  Eigen::Vector3d reference_foot_lin_velocity_[N_ENDEFFS+1];
  Eigen::Vector3d reference_foot_ang_velocity_[N_ENDEFFS+1];
  Eigen::Vector3d reference_foot_lin_acceleration_[N_ENDEFFS+1];
  Eigen::Vector3d reference_foot_ang_acceleration_[N_ENDEFFS+1];

  //and their equivalent in world frame
  Eigen::Vector3d reference_foot_position_wrld_[N_ENDEFFS+1];
  Eigen::Quaterniond reference_foot_orientation_wrld_[N_ENDEFFS+1];
  Eigen::Vector3d reference_foot_lin_velocity_wrld_[N_ENDEFFS+1];
  Eigen::Vector3d reference_foot_ang_velocity_wrld_[N_ENDEFFS+1];
  Eigen::Vector3d reference_foot_lin_acceleration_wrld_[N_ENDEFFS+1];
  Eigen::Vector3d reference_foot_ang_acceleration_wrld_[N_ENDEFFS+1];


  //these are in world frame
  Eigen::Vector3d foot_position_wrld_[N_ENDEFFS+1];
  Eigen::Quaterniond foot_orientation_wrld_[N_ENDEFFS+1];
  Eigen::Vector3d foot_lin_velocity_wrld_[N_ENDEFFS+1];
  Eigen::Vector3d foot_ang_velocity_wrld_[N_ENDEFFS+1];

  //and in invariant frame
  Eigen::Vector3d foot_position_[N_ENDEFFS+1];
  Eigen::Quaterniond foot_orientation_[N_ENDEFFS+1];
  Eigen::Vector3d foot_lin_velocity_[N_ENDEFFS+1];
  Eigen::Vector3d foot_ang_velocity_[N_ENDEFFS+1];

  SL_endeff endeff_constraints_[N_ENDEFFS+1];

  Eigen::Matrix<double, 6*N_ENDEFFS, 1> desired_foot_acc_;
  Eigen::Matrix<double, 6, 12> foot_gains_;

  Eigen::Matrix<double, 6, 9> momentum_gains_stance_;
  Eigen::Matrix<double, 6, 9> momentum_gains_swing_;
  Eigen::Matrix<double, 6, 9> momentum_gains_;

  Eigen::Matrix<double, 6, 1> desired_momentum_rate_;
  Eigen::Vector3d ang_momentum_wrld_, ang_momentum_;

  Eigen::Matrix<double, 6, 1> base_P_gains_, base_D_gains_;
  Eigen::Matrix<double, 6, 1> desired_base_acceleration_;

  //  double des_com_height_;
  Eigen::Matrix<double, N_DOFS, 1> reference_joint_posture_, reference_joint_vel_, reference_joint_acc_;
  Eigen::Matrix<double, N_DOFS, 1> default_posture_;
  Eigen::Matrix<double, N_DOFS, 1> null_posture_P_gains_;
  Eigen::Matrix<double, N_DOFS, 1> null_posture_D_gains_;

  Eigen::Matrix4d invariant_reference_frame_;
  double gravity_contribution_;

  //in wrld frame
  Eigen::Vector3d zmp_, desired_zmp_;

  Eigen::Matrix<double, N_DOFS, 1> initial_u_;

  //opengl stuff
  int opengl_counter_;
  int frame_number_;
  int make_video_;
};

#endif /* BIPED_CONTROLLER_H_ */
