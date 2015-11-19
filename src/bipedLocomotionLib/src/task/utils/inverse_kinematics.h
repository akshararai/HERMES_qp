/*
 * inverseKinematics.h
 *
 *  Created on: Feb 22, 2012
 *      Author: righetti
 */

#ifndef INVERSEKINEMATICS_H_
#define INVERSEKINEMATICS_H_

#include <eigen3/Eigen/Eigen>
#include <boost/shared_ptr.hpp>

#include "eiquadprog.hpp"

#include "OptimalInverseDynamicsEigen.h"
#include "FloatingBaseKinematics.h"
#include "KinematicsEigen.h"
#include "MomentumComputation.h"

#include "SL_user.h"
#include "SL.h"


namespace inverse_kinematics
{

struct RobotState
{
  std::vector<SL_Cstate> constraint_position;   // 0 = com, rest is indexed by endeffectors
  std::vector<SL_quat> constraint_orientation;  // ""
  std::vector<SL_Jstate> joints;                //the associated joint posture
  std::vector<SL_endeff> is_constrained;        //1 if constrained, 0 otherwise (starts at base/com 0)
  SL_Cstate base_position;                      //used if 0 is com
  SL_quat base_orientation;                     //same
};

void save_robot_state(const char* file_name, const std::vector<inverse_kinematics::RobotState>& robot_state);
void save_joint_positions(const char* file_name, const inverse_kinematics::RobotState& robot_state);
void read_robot_state(const char* file_name, inverse_kinematics::RobotState& robot_state);
void save_robot_state(const char* file_name, const RobotState& robot_state);
void create_robot_state(SL_Cstate& basec, SL_quat& baseo, SL_Cstate* cart_state, SL_quat* cart_orient,
                        SL_Jstate* jstate, SL_endeff* endeff, RobotState& robot_state);

class InverseKinematics
{
public:
  InverseKinematics();
  InverseKinematics(const InverseKinematics& copy);
  InverseKinematics& operator=(const InverseKinematics& copy);
  virtual ~InverseKinematics();

  /*!
   * Computes the IK trajectory as well as the followed path and predicted contact forces
   * @param base_position [in] the desired base position path [out] the one actually tracked by IK
   * @param base_orientation [in] the desired [out] the actual
   * @param foot_position [in] the desired [out] the actual (if empty [in] stays empty [out])
   * @param foot_orientation [in] the desired [out] the actual (if empty [in] stays empty [out])
   * @param contact_forces (if empty [in] stays empty [out])
   * @param joints 1 element for the initial state of the robot [out] the whole body trajectory
   * @param posture a default posture
   * @param delta_time the delta t between each point
   * @return
   */
  bool computeInverseKinematics(std::vector<RobotState>& robot_state, double delta_time);

  bool inverseKinematicsStepQP(RobotState& desired_robot_state, RobotState& current_robot_state);

  void setIntegrationStep(double integration_step) {integration_step_ = integration_step;};


  void setGains(Eigen::Matrix<double, 2*N_CART*(N_ENDEFFS+1), 1> p_base_gain,
                Eigen::Matrix<double, N_DOFS, 1> p_posture_gain);

  void computeTaskPseudoInverse(Eigen::Matrix<double, Eigen::Dynamic, N_DOFS+6>& input_matrix,
                             Eigen::Matrix<double, N_DOFS+6, Eigen::Dynamic>& pseudo_inverse,
                             int max_rank, bool compute_nullspace, Eigen::Matrix<double, N_DOFS+6, N_DOFS+6>& nullspace);

  void useCom(bool use){use_com_ = use;};

private:

  bool use_com_;

  int task_size_;

  bool jointLimitCheck(SL_Jstate* current_joint_state, Eigen::Matrix<double, N_DOFS+6, 1>& fullstate_velocity,
                       Eigen::Matrix<double, Eigen::Dynamic, 1> desired_velocity);

  //used for the CoM IK
  floating_base_utilities::KinematicsEigen kinematics_eigen_;
  floating_base_utilities::MomentumComputation momentum_helper_;

  static const double condition_nb_cutoff_ = 100.0;

  double integration_step_;

  floating_base_utilities::FloatingBaseKinematics floating_base_kinematics_;

  Eigen::Matrix<double, 6*N_ENDEFFS, N_DOFS+6> unconstrained_endeff_jacobian_; //jacobian of the not-constrained foot
  Eigen::Matrix<double, 6*N_ENDEFFS, N_DOFS+6> constrained_endeff_jacobian_; //jacobian of the constrained foot
  Eigen::Matrix<double, N_DOFS+6, N_DOFS+6> constrained_nullspace_; //the constrained nullspace
  Eigen::Matrix<double, Eigen::Dynamic, N_DOFS+6> task_jacobian_; //the constraint consistent Jacobian (base + endeff)
  Eigen::Matrix<double, N_DOFS+6, N_DOFS+6> task_nullspace_;
  Eigen::Matrix<double, N_DOFS+6, Eigen::Dynamic> pseudo_inverse_;


  Eigen::Matrix<double, 2*N_CART*(N_ENDEFFS + 1), 1> p_task_gain_; //we don't control the orientation of the CoM
  Eigen::Matrix<double, N_DOFS, 1> p_posture_gain_;

};

} /* namespace inverse_kinematics */
#endif /* INVERSEKINEMATICS_H_ */
