/*********************************************************************
 Autonomous Motion Department
 Max-Planck Intelligent Systems
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         BipedalTaskComposer.h

 \author       Alexander Herzog
 \date         Jul 18, 2013

 *********************************************************************/

#ifndef BIPEDALTASKCOMPOSER_H_
#define BIPEDALTASKCOMPOSER_H_

#include <eigen3/Eigen/Dense>

#include "RtMatrixX.h"

#include "HierarchInverseDynamics.hh"
#include "HierarchAffineCost.h"
#include "CartesianForceCtrl.hh"
#include "CartesianPositionCtrl.hh"
#include "MomentumRateCtrl.hh"
#include "FloatingBaseImpedanceCtrl.hh"
#include "JointPositionCtrl.hh"

#include "ContactHelper.h"
#include "FloatingBaseKinematics.h"
#include "KinematicsEigen.h"
#include "MomentumComputation.h"

#include <SL.h>
#include <utility.h>
#include <utility_macros.h>
#include <SL_kinematics.h>
#include <SL_dynamics.h>
#include <SL_task_servo.h>

namespace momentum_balance_control
{

class BipedalTaskComposer : public floating_base_utilities::HierarchAffineCost
{
public:
  static const int max_eq_constraints_ = 6+N_DOFS + (4*N_ENDEFFS)*6;
  static const int max_ineq_constraints_ = 12+8+8+2+2*N_DOFS+2*N_DOFS;

  typedef floating_base_utilities::CartesianPositionCtrl< max_ineq_constraints_, max_eq_constraints_> CartesianPositionCtrlHermes;
  typedef floating_base_utilities::CartesianForceCtrl< max_ineq_constraints_, max_eq_constraints_> CartesianForceCtrlHermes;
  typedef floating_base_utilities::MomentumRateCtrl< max_ineq_constraints_, max_eq_constraints_> MomentumRateCtrlHermes;
  typedef floating_base_utilities::MomentumRateCtrl< max_ineq_constraints_, max_eq_constraints_, true> MomentumRateCtrlHermesAccBased;
  typedef floating_base_utilities::HierarchAffineCost BaseClass;
  typedef floating_base_utilities::HierarchInverseDynamics< max_ineq_constraints_, max_eq_constraints_> HierarchInverseDynamicsHermes;

  typedef floating_base_utilities::JointPositionCtrl< max_ineq_constraints_, max_eq_constraints_, N_DOFS> JointPositionCtrlHermes;
  typedef floating_base_utilities::JointPositionCtrl< max_ineq_constraints_, max_eq_constraints_, 7> LegJointPositionCtrlHermes;

  BipedalTaskComposer(HierarchInverseDynamicsHermes& hierarch_inv_dyn);
  virtual ~BipedalTaskComposer();

  void addCostToHierarchy(int rank) const;
  void addCostToHierarchyAfterReduction(int rank) const;
  void updateAfterSolutionFound();
  int maxRank() const {return max_rank_;};

  void initialize();

  void update(const Eigen::Matrix<double, 6, 1>& des_mom_rate,
                const Eigen::Matrix<double, N_DOFS, 1>& pose_ref,
                const Eigen::Matrix<double, 6*N_ENDEFFS, 1>& des_foot_acc);

  std::vector<double> cop_weight_sqrt_;
  Eigen::Matrix<double, 12, 12> sqrt_stat_feet_weight_;
  Eigen::Matrix<double, 6, 6> sqrt_frc_reg_weight_;

  HierarchInverseDynamicsHermes& hierarch_inv_dyn_;
  CartesianForceCtrlHermes frc_reg_R_;
  CartesianForceCtrlHermes frc_reg_L_;
  MomentumRateCtrlHermes momentum_ctrl_;
//  Eigen::Matrix<double, N_DOFS, 1> sqrt_posture_tr_weight_;
private:
  Eigen::Matrix<double, N_DOFS+6, 1> is_joint_used_;
  int max_rank_;
  // costs and constraints
  floating_base_utilities::RtMatrixX<6*N_ENDEFFS, N_DOFS+6>::d stat_feet_mat_;
  floating_base_utilities::RtVectorX<6*N_ENDEFFS>::d stat_feet_vec_;
  double stat_feet_slack_;
  floating_base_utilities::RtMatrixX<2*N_DOFS, N_DOFS+6>::d jacc_lim_mat_;
  floating_base_utilities::RtVectorX<2*N_DOFS>::d jacc_lim_vec_;
  double jacc_lim_slack_;
//  floating_base_utilities::RtMatrixX<N_DOFS, N_DOFS+6+6*N_ENDEFFS>::d posture_track_mat_;
//  floating_base_utilities::RtVectorX<N_DOFS>::d posture_track_vec_;
//  double posture_track_slack_;
  floating_base_utilities::RtMatrixX<floating_base_utilities::ContactHelper::max_endeff_corners_,6>::d cop_ineq_mats_[N_ENDEFFS+1];
  double cop_ineq_slack_;
  Eigen::Matrix<double, 4, 6> frict_ineq_mats_[N_ENDEFFS+1];
  double frict_ineq_slack_;

#ifdef OPT_OVER_TRQS
  floating_base_utilities::RtMatrixX<N_DOFS+6, N_DOFS+6+6*N_ENDEFFS+N_DOFS>::d dyn_eq_mat_;
  floating_base_utilities::RtVectorX<N_DOFS+6>::d dyn_eq_vec_;
#else
  floating_base_utilities::RtMatrixX<6, N_DOFS+6+6*N_ENDEFFS>::d dyn_eq_mat_;
  floating_base_utilities::RtVectorX<6>::d dyn_eq_vec_;
#endif
  double dyn_eq_slack_;
#ifdef OPT_OVER_TRQS
  floating_base_utilities::RtMatrixX<2*N_DOFS, N_DOFS+6+6*N_ENDEFFS+N_DOFS>::d torque_lim_mat_;
#else
  floating_base_utilities::RtMatrixX<2*N_DOFS, N_DOFS+6+6*N_ENDEFFS>::d torque_lim_mat_;
#endif
  floating_base_utilities::RtVectorX<2*N_DOFS>::d torque_lim_vec_;
  double torque_lim_slack_;

  Eigen::Matrix<double, 6, 6> rate_tr_weight_;
  Eigen::Matrix<double, 6, 6> frc_reg_weight_;
  Eigen::Matrix<double, 6, 6> sqrt_rate_tr_weight_;


  int feet_const_rank_;
  int force_reg_rank_;
//  int posture_rank_;
  int cop_const_rank_;
  int friction_cone_rank_;

  std::vector<double> j_padding_;
  std::vector<double> max_jacc_;
  std::vector<double> jacc_lim_slope_;
  Eigen::Matrix<double, N_DOFS, 1> torque_limits_;
  Eigen::Matrix<double, N_DOFS, 1> min_accels_;
  Eigen::Matrix<double, N_DOFS, 1> max_accels_;
  Eigen::Matrix<double, N_DOFS, 1> jacc_constr_activation_;


  floating_base_utilities::RtMatrixX<6*N_ENDEFFS, N_DOFS+6+6*N_ENDEFFS>::d var_to_lambda_;
  floating_base_utilities::RtMatrixX<N_DOFS+6, N_DOFS+6+6*N_ENDEFFS>::d var_to_qdd_;
  bool is_endeff_const[N_ENDEFFS+1];
  int num_stance_feet_;
  Eigen::Matrix<double, 3, 1> adms_cops_[N_ENDEFFS+1];
  std::vector<double> adms_cop_torques_;
  const floating_base_utilities::ContactHelper* foot_contact_handler_;


  void computeSlacks();
};

} /* namespace floating_base_utilities */

/** make sure that templated classes are not compiled in this translation unit
 *  This safes compile time.
*/
extern template class floating_base_utilities::CartesianPositionCtrl< momentum_balance_control::BipedalTaskComposer::max_ineq_constraints_, momentum_balance_control::BipedalTaskComposer::max_eq_constraints_>;
extern template class floating_base_utilities::FloatingBaseImpedanceCtrl< momentum_balance_control::BipedalTaskComposer::max_ineq_constraints_, momentum_balance_control::BipedalTaskComposer::max_eq_constraints_>;
extern template class floating_base_utilities::CartesianForceCtrl< momentum_balance_control::BipedalTaskComposer::max_ineq_constraints_, momentum_balance_control::BipedalTaskComposer::max_eq_constraints_>;
extern template class floating_base_utilities::MomentumRateCtrl< momentum_balance_control::BipedalTaskComposer::max_ineq_constraints_, momentum_balance_control::BipedalTaskComposer::max_eq_constraints_>;
extern template class floating_base_utilities::MomentumRateCtrl< momentum_balance_control::BipedalTaskComposer::max_ineq_constraints_, momentum_balance_control::BipedalTaskComposer::max_eq_constraints_, true>;
extern template class floating_base_utilities::HierarchInverseDynamics< momentum_balance_control::BipedalTaskComposer::max_ineq_constraints_, momentum_balance_control::BipedalTaskComposer::max_eq_constraints_>;
extern template class floating_base_utilities::JointPositionCtrl< momentum_balance_control::BipedalTaskComposer::max_ineq_constraints_, momentum_balance_control::BipedalTaskComposer::max_eq_constraints_, 7>;

#endif /* BIPEDALTASKCOMPOSER_H_ */
