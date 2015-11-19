/*
 * OptimalInverseDynamicsEigen.h
 *
 *  This class implements a floating-based inverse dynamics able to optimize torque and contact forces
 *  in the nullspace of the desired accelerations
 *
 *  Created on: Nov 27, 2011
 *      Author: righetti
 */

#ifndef OPTIMALINVERSEDYNAMICSEIGEN_H_
#define OPTIMALINVERSEDYNAMICSEIGEN_H_

//using Eigen3
#include <vector>
#include <iostream>
#include <Eigen/Eigen>

#include "FloatingBaseKinematics.h"

#include <SL.h>
#include <utility.h>
#include <utility_macros.h>
#include <SL_kinematics.h>
#include <SL_dynamics.h>
#include <SL_task_servo.h>
#include <SL_user.h>


//the definition is put in a .cpp file where we use template specialization (1 specialization per robot is needed)
//this is not very clean but allows to compile the library before hand and thus speeds up compilation time afterwards
//we need to use templates because we want to use fixed size Matrices as much as possible to avoid memory allocation to be Xenomai friendly

namespace floating_base_utilities
{

class OptimalInverseDynamicsEigen
{
public:
  OptimalInverseDynamicsEigen();
  virtual ~OptimalInverseDynamicsEigen(){};

  /*!
   * initialize function (sets up a few default parameters)
   * @return
   */
  bool initialize();

  /*!
   * Prepare things for the inverse dynamics: the various projectors
   * @return
   */
  bool prepareInverseDynamics(FloatingBaseKinematics& kinematics);


  /*!
   * Compute the feedforward ID torques given a desired joint state and predict contact forces
   * it uses the projectors computed in prepareInverseDynamics()
   * @param des_joint_state
   * @param end_effectors
   * @param base_state
   * @param base_orient
   */
  void computeTorques(FloatingBaseKinematics& kinematics,
                      SL_DJstate* des_joint_state, SL_endeff* end_effectors, SL_Cstate& base_state, SL_quat& base_orient);


  /*!
   * combines prepareInverseDynamics and computeTorques for a simpler call
   * computes the full inverse dynamics given a desired joint state
   * (uses the desired joint state to compute the jacobians and full ID)
   * @param des_joint_state
   * @param end_effectors
   * @param base_state
   * @param base_orient
   * @return
   */
  bool computeInverseDynamics(SL_DJstate* des_joint_state, SL_endeff* end_effectors,
                              SL_Cstate& base_state, SL_quat& base_orient);

  /*!
   * get an SL-type Vector of predicted reaction forces (computed through computeTorques)
   * @param reaction_forces
   */
  void getPredictedReactionForces(Vector reaction_forces);
  void getPredictedReactionForces(std::vector<double>& reaction_forces);

  /*!
   * Set desired force to optimize, i.e. min a cost of the form (lambda - desired_forces)^T force_weight (lambda-desired_forces)
   * @param desired_forces
   * @param force_weight
   */
  void setDesiredForces(const Eigen::Matrix<double, 6*N_ENDEFFS, 1>& desired_forces,
                        const Eigen::Matrix<double, 6*N_ENDEFFS, 6*N_ENDEFFS>& force_weight);
  void setDesiredForces(Vector desired_forces, Matrix force_weight);
  /*!
   * Set desired torque to optimize, i.e. min a cost of the form (tau - desired_torques)^T torque_weight (tau - desired_torques)
   * @param desired_torques
   * @param torque_weight
   */
  void setDesiredTorques(const Eigen::Matrix<double, N_DOFS, 1>& desired_torques,
                         const Eigen::Matrix<double, N_DOFS, N_DOFS>& torque_weight);
  void setDesiredTorques(Vector desired_torques, Matrix torque_weight);

  /*!
   * get the sum of predicted command and constraint costs
   * @return
   */
  double getPredictedCost() { return predicted_command_cost_ + predicted_constraint_cost_; };

  /*!
   * get the predicted command cost
   * @return
   */
  double getPredictedCommandCost() { return predicted_command_cost_; };

  /*!
   * get the predicted constraint cost
   * @return
   */
  double getPredictedConstraintCost() { return predicted_constraint_cost_; };

  void setTorqueSaturation(Vector saturation);
  void setTorqueSaturation(const Eigen::Matrix<double, N_DOFS, 1>& saturation);
  void unsetTorqueSaturation();
  //Assume we want to have A lambda < b
  void setContactInequalities(Matrix A, Vector b, int num_ineq);
  void setContactInequalities(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, int num_ineq);
  void unsetContactInequalities();


private:
  bool updatePseudoInverseWeight(SL_endeff* end_effectors, const Eigen::Matrix<double, N_DOFS+6, N_DOFS+6>& invRQT);
  bool computeTorquesQP(FloatingBaseKinematics& kinematics,
                        const Eigen::Matrix<double, N_DOFS+6, 1>& Mddq_h);
  bool computeTorquesLeastSquare(const Eigen::Matrix<double, N_DOFS+6, 1>& Mddq_h);


  bool initialized_;//true if init was called

  int num_constraints_;
  int num_unconstrained_dimensions_;

  Eigen::Matrix<double, N_DOFS, N_DOFS+6> torque_projector_;
  Eigen::Matrix<double, 6*N_ENDEFFS, N_DOFS+6> contact_projector_;
  Eigen::Matrix<double, N_DOFS, N_DOFS> internal_torque_projector_;

  Eigen::Matrix<double, 6*N_ENDEFFS, 1> predicted_reaction_forces_; //predicted reaction forces from the commands
  Eigen::Matrix<double, N_DOFS, 1> desired_torques_; // desired torques to be sent to the robot

  //optimization related
  Eigen::Matrix<double, N_DOFS, N_DOFS> cost_weight_;
  Eigen::Matrix<double, N_DOFS, N_DOFS> cost_weight_inverse_; //inverse of W
  Eigen::Matrix<double, N_DOFS+6, N_DOFS+6> full_constraint_weight_; //Wc
  Eigen::Matrix<double, N_DOFS+6, 1> full_constraint_linear_term_; //b_c

  Eigen::Matrix<double, N_DOFS, N_DOFS> command_cost_weight_; //W tau
  Eigen::Matrix<double, N_DOFS, 1> command_cost_linear_term_; //b_tau
  Eigen::Matrix<double, 6*N_ENDEFFS, 6*N_ENDEFFS> contact_cost_weight_; //Wlambda
  Eigen::Matrix<double, 6*N_ENDEFFS, 1> contact_cost_linear_term_; //b_lambda

  Eigen::Matrix<double, N_DOFS, 1> optimal_torque_objectives_; //as used in the cost
  Eigen::Matrix<double, 6*N_ENDEFFS, 1> optimal_force_objectives_; //as used in the cost


  //related to method of optimization (QP or least-square)
  bool use_contact_inequalities_;
  bool use_torque_saturation_;
  Eigen::Matrix<double, N_DOFS, 1> torque_limits_;
  Eigen::Matrix<double, Eigen::Dynamic, 6*N_ENDEFFS> contact_ineq_matrix_;
  Eigen::Matrix<double, Eigen::Dynamic, 1> contact_ineq_vector_;


  double predicted_command_cost_;
  double predicted_constraint_cost_;

};


} /* namespace inverse_dynamics */
#endif /* OPTIMALINVERSEDYNAMICSEIGEN_H_ */
