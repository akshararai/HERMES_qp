/*********************************************************************
 Autonomous Motion Department
 Max-Planck Intelligent Systems
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         BipedalTaskComposer.cpp

 \author       Alexander Herzog
 \date         Jul 18, 2013

 *********************************************************************/

#include <boost/thread.hpp>
#include "BipedalTaskComposer.h"
#include "GeometryUtils.h"
#include "quaternions.h"


using namespace floating_base_utilities;
namespace momentum_balance_control
{


BipedalTaskComposer::BipedalTaskComposer(HierarchInverseDynamicsHermes& hierarch_inv_dyn) :
                        hierarch_inv_dyn_(hierarch_inv_dyn),
                        frc_reg_R_(hierarch_inv_dyn_), frc_reg_L_(hierarch_inv_dyn_),
                        momentum_ctrl_(hierarch_inv_dyn_), max_rank_(4),
                        j_padding_(N_DOFS+1, 0.0), max_jacc_(N_DOFS+1, 0.0), jacc_lim_slope_(N_DOFS+1, 0.0),
                        cop_weight_sqrt_(N_ENDEFFS+1, 1.0),
                        adms_cop_torques_(N_ENDEFFS+1, 0.0)
{
  rate_tr_weight_ = Eigen::Matrix<double, 6, 6>::Zero();
  frc_reg_weight_ = Eigen::Matrix<double, 6, 6>::Zero();
  foot_contact_handler_ = &hierarch_inv_dyn_.contactHelper();
}


BipedalTaskComposer::~BipedalTaskComposer()
{
}

void BipedalTaskComposer::initialize()
{
  RtMatrixXUtils::setZero(jacc_lim_mat_,2*N_DOFS, N_DOFS+6);
  jacc_lim_mat_.block(0, 0, N_DOFS, N_DOFS).setIdentity();
  jacc_lim_mat_.block(N_DOFS, 0, N_DOFS, N_DOFS).setIdentity();
  jacc_lim_mat_.block(N_DOFS, 0, N_DOFS, N_DOFS) *= -1.0;
  RtVectorXUtils::resize(jacc_lim_vec_, 2 * N_DOFS);
//  RtMatrixXUtils::setIdentity(posture_track_mat_, N_DOFS);
  RtVectorXUtils::setZero(jacc_constr_activation_);

  hierarch_inv_dyn_.sub_cost_composers_.push_back(this);

  hierarch_inv_dyn_.setDiagMatFromFile<6>(rate_tr_weight_, "RATE_TRACK_WEIGHT");
  hierarch_inv_dyn_.setDiagMatFromFile<6>(frc_reg_weight_, "FORCE_REG_WEIGHT");
  hierarch_inv_dyn_.setDiagMatFromFile<12>(sqrt_stat_feet_weight_, "STAT_FEET_WEIGHT");
  sqrt_rate_tr_weight_ = rate_tr_weight_.array().sqrt();
  sqrt_frc_reg_weight_ = frc_reg_weight_.array().sqrt();
  sqrt_stat_feet_weight_ = sqrt_stat_feet_weight_.array().sqrt();

  hierarch_inv_dyn_.setArrayFromFile(j_padding_, "JOINT_PADDING");
  hierarch_inv_dyn_.setArrayFromFile(max_jacc_, "MAX_JOINT_ACC");
  hierarch_inv_dyn_.setArrayFromFile(jacc_lim_slope_, "JACC_LIM_SLOPE");

//  std::vector<double> buffer(N_DOFS+1, 0.0);
//  hierarch_inv_dyn_.setArrayFromFile(buffer, "POSTURE_TRACK_WEIGHT");
//  sqrt_posture_tr_weight_ = Eigen::Map<Eigen::Matrix<double, N_DOFS,1> >(buffer.data());
////  hierarch_inv_dyn_.setDiagMatFromFile<N_DOFS>(sqrt_posture_tr_weight_, "POSTURE_TRACK_WEIGHT");
//  sqrt_posture_tr_weight_ = sqrt_posture_tr_weight_.array().sqrt();
  hierarch_inv_dyn_.setArrayFromFile(cop_weight_sqrt_, "cop_weight");
  for(int i=0; i<(int)cop_weight_sqrt_.size(); ++i)
    cop_weight_sqrt_[i] = sqrt(cop_weight_sqrt_[i]);

  min_accels_ = -100*Eigen::Matrix<double, N_DOFS, 1>::Ones();
  max_accels_ = 100*Eigen::Matrix<double, N_DOFS, 1>::Ones();
  torque_limits_ = Eigen::Matrix<double, N_DOFS, 1>(&(u_max[1]));

  if(!read_parameter_pool_int(hierarch_inv_dyn_.config_file_, "feet_const_rank", &feet_const_rank_))
  {
    printf("CANNOT READ FEET CONST RANK. Default value\n");
    feet_const_rank_ = 2;
  }
  if(!read_parameter_pool_int(hierarch_inv_dyn_.config_file_, "force_reg_rank", &force_reg_rank_))
  {
    printf("CANNOT READ FORCE REG RANK. Default value\n");
    force_reg_rank_ = 4;
  }
//  if(!read_parameter_pool_int(hierarch_inv_dyn_.config_file_, "posture_rank", &posture_rank_))
//  {
//    printf("CANNOT READ POSTURE RANK. Default value\n");
//    posture_rank_ = 3;
//  }

  if(!read_parameter_pool_int(hierarch_inv_dyn_.config_file_, "cop_const_rank", &cop_const_rank_))
  {
    printf("CANNOT READ COP CONST RANK. Default value\n");
    cop_const_rank_ = 2;
  }
  if(!read_parameter_pool_int(hierarch_inv_dyn_.config_file_, "friction_cone_rank", &friction_cone_rank_))
  {
    printf("CANNOT READ FRICTION CONE RANK. Default value\n");
    friction_cone_rank_ = 2;
  }

  frc_reg_R_.initialize(force_reg_rank_, RIGHT_FOOT, true);
  frc_reg_L_.initialize(force_reg_rank_, LEFT_FOOT, true);
  frc_reg_R_.weightingMat() = sqrt_frc_reg_weight_;
  frc_reg_L_.weightingMat() = sqrt_frc_reg_weight_;
  double down_frcs[] = {0.0, -250.0, 0.0, 0.0, 0.0, 0.0};
  frc_reg_R_.update(Eigen::Map<Eigen::Matrix<double, 6, 1> >(down_frcs));
  frc_reg_L_.update(Eigen::Map<Eigen::Matrix<double, 6, 1> >(down_frcs));

  momentum_ctrl_.initialize(2);
  momentum_ctrl_.weightingMat() = sqrt_rate_tr_weight_;
  int mom_ctrl_ranks[7];
  if(read_parameter_pool_int_array(hierarch_inv_dyn_.config_file_, "momentum_ranks", 6, mom_ctrl_ranks))
  {
    momentum_ctrl_.ranks_ = Eigen::Map<Eigen::Matrix<int, 6, 1> >(
        &mom_ctrl_ranks[1]);
  }


  hierarch_inv_dyn_.sub_cost_composers_.push_back(static_cast<HierarchAffineCost*>(
      &frc_reg_R_));
  hierarch_inv_dyn_.sub_cost_composers_.push_back(static_cast<HierarchAffineCost*>(
      &frc_reg_L_));
  hierarch_inv_dyn_.sub_cost_composers_.push_back(static_cast<HierarchAffineCost*>(
      &momentum_ctrl_));

  is_joint_used_.setConstant(1.0);
  is_joint_used_.block(B_TR-1, 0, 3, 1).setConstant(0.0);
  hierarch_inv_dyn_.joint_select_ = (is_joint_used_.array() -0.5).array() > 0.0;
  std::cout << "joint_select_" << std::endl << hierarch_inv_dyn_.joint_select_.transpose() << std::endl;

  // publish variables for data collection
  addVarToCollect((char*)&cop_ineq_slack_, "cop_slck", "-", DOUBLE, TRUE);
  addVarToCollect((char*)&frict_ineq_slack_, "friction_slck", "-", DOUBLE, TRUE);
  addVarToCollect((char*)&dyn_eq_slack_, "dynamics_eq_slck", "-", DOUBLE, TRUE);
  addVarToCollect((char*)&torque_lim_slack_, "torque_sat_slck", "-", DOUBLE, TRUE);
//  addVarToCollect((char*)&posture_track_slack_, "posture_slck", "-", DOUBLE, TRUE);
  addVarToCollect((char*)&stat_feet_slack_, "stat_feet_slck", "-", DOUBLE, TRUE);       //
  addVarToCollect((char*)&jacc_lim_slack_, "jacc_lim_slck", "-", DOUBLE, TRUE);

  for(int i=0; i<N_DOFS;++i)
  {
    char vname[50];
    std::strcpy(vname, (std::string("jacc_constr_")+std::string(joint_names[i+1])).c_str());
    addVarToCollect((char*)&jacc_constr_activation_[i], vname, "-", DOUBLE, TRUE);

    std::strcpy(vname, (std::string("admis_acc_")+std::string(joint_names[i+1])).c_str());
    addVarToCollect((char*)&hierarch_inv_dyn_.admis_accels_[i], vname, "-", DOUBLE, TRUE);
    //          std::cout << vname << std::endl;
  }

  addVarToCollect((char*)&hierarch_inv_dyn_.getAdmisForceAtEndeff(LEFT_FOOT)[0], "admis_grf_left_x", "N", DOUBLE, TRUE);
  addVarToCollect((char*)&hierarch_inv_dyn_.getAdmisForceAtEndeff(LEFT_FOOT)[1], "admis_grf_left_y", "N", DOUBLE, TRUE);
  addVarToCollect((char*)&hierarch_inv_dyn_.getAdmisForceAtEndeff(LEFT_FOOT)[2], "admis_grf_left_z", "N", DOUBLE, TRUE);
  addVarToCollect((char*)&hierarch_inv_dyn_.getAdmisForceAtEndeff(LEFT_FOOT)[3], "admis_grf_left_a", "N*m", DOUBLE, TRUE);
  addVarToCollect((char*)&hierarch_inv_dyn_.getAdmisForceAtEndeff(LEFT_FOOT)[4], "admis_grf_left_b", "N*m", DOUBLE, TRUE);
  addVarToCollect((char*)&hierarch_inv_dyn_.getAdmisForceAtEndeff(LEFT_FOOT)[5], "admis_grf_left_g", "N*m", DOUBLE, TRUE);
  addVarToCollect((char*)&hierarch_inv_dyn_.getAdmisForceAtEndeff(RIGHT_FOOT)[0], "admis_grf_right_x", "N", DOUBLE, TRUE);
  addVarToCollect((char*)&hierarch_inv_dyn_.getAdmisForceAtEndeff(RIGHT_FOOT)[1], "admis_grf_right_y", "N", DOUBLE, TRUE);
  addVarToCollect((char*)&hierarch_inv_dyn_.getAdmisForceAtEndeff(RIGHT_FOOT)[2], "admis_grf_right_z", "N", DOUBLE, TRUE);
  addVarToCollect((char*)&hierarch_inv_dyn_.getAdmisForceAtEndeff(RIGHT_FOOT)[3], "admis_grf_right_a", "N*m", DOUBLE, TRUE);
  addVarToCollect((char*)&hierarch_inv_dyn_.getAdmisForceAtEndeff(RIGHT_FOOT)[4], "admis_grf_right_b", "N*m", DOUBLE, TRUE);
  addVarToCollect((char*)&hierarch_inv_dyn_.getAdmisForceAtEndeff(RIGHT_FOOT)[5], "admis_grf_right_g", "N*m", DOUBLE, TRUE);


  addVarToCollect((char*)&adms_cops_[LEFT_FOOT][0], "admis_cop_l_x", "m", DOUBLE, TRUE);
  addVarToCollect((char*)&adms_cops_[LEFT_FOOT][1], "admis_cop_l_y", "m", DOUBLE, TRUE);
  addVarToCollect((char*)&adms_cops_[LEFT_FOOT][2], "admis_cop_l_z", "m", DOUBLE, TRUE);
  addVarToCollect((char*)&adms_cops_[RIGHT_FOOT][0], "admis_cop_r_x", "m", DOUBLE, TRUE);
  addVarToCollect((char*)&adms_cops_[RIGHT_FOOT][1], "admis_cop_r_y", "m", DOUBLE, TRUE);
  addVarToCollect((char*)&adms_cops_[RIGHT_FOOT][2], "admis_cop_r_z", "m", DOUBLE, TRUE);
}


void BipedalTaskComposer::addCostToHierarchy(int rank) const
{

  hierarch_inv_dyn_.hierarch_solver_.apply_ineq_slacks_ = true;

  switch(rank)
  {
    //add equation of motion and torque limits
    case 0:
      hierarch_inv_dyn_.hierarch_solver_.apply_ineq_slacks_ = false;
      floating_base_utilities::RtAffineUtils::append(hierarch_inv_dyn_.next_eq_cost_mat_, hierarch_inv_dyn_.next_eq_cost_vec_,
                                                     dyn_eq_mat_, dyn_eq_vec_);
      floating_base_utilities::RtAffineUtils::append(hierarch_inv_dyn_.next_ineq_cost_mat_, hierarch_inv_dyn_.next_ineq_cost_vec_,
                                                     torque_lim_mat_, torque_lim_vec_);

      break;
      //joint acceleration limit + CoP constraints + friction //+ static feet
    case 1:
      //      floating_base_utilities::RtAffineUtils::appendAtColumn(hierarch_inv_dyn_.next_eq_cost_mat_, hierarch_inv_dyn_.next_eq_cost_vec_,
      //                                                                   stat_feet_mat_, stat_feet_vec_, 0);

      floating_base_utilities::RtAffineUtils::appendAtColumn(hierarch_inv_dyn_.next_ineq_cost_mat_, hierarch_inv_dyn_.next_ineq_cost_vec_,
                                                             jacc_lim_mat_, jacc_lim_vec_, 0);

      break;
    default:
      break;
  }
  if(rank == cop_const_rank_)
  {
    for(int i =1; i<= N_ENDEFFS; ++i)
      if(is_endeff_const[i])
      {
        floating_base_utilities::RtAffineUtils::appendAtColumn(hierarch_inv_dyn_.next_ineq_cost_mat_, hierarch_inv_dyn_.next_ineq_cost_vec_,
                                                               cop_ineq_mats_[i], N_DOFS+6+6*(i-1));
      }
  }
  if(rank == friction_cone_rank_)
  {
    for(int i =1; i<= N_ENDEFFS; ++i)
      if(is_endeff_const[i])
      {
        floating_base_utilities::RtAffineUtils::appendAtColumn(hierarch_inv_dyn_.next_ineq_cost_mat_, hierarch_inv_dyn_.next_ineq_cost_vec_,
                                                               frict_ineq_mats_[i], N_DOFS+6+6*(i-1));
      }
  }

  //both feet
  if(rank == feet_const_rank_)
    floating_base_utilities::RtAffineUtils::appendAtColumn(hierarch_inv_dyn_.next_eq_cost_mat_, hierarch_inv_dyn_.next_eq_cost_vec_,
                                                           stat_feet_mat_, stat_feet_vec_, 0);
  //posture
//  if(rank == posture_rank_)
//  {
//    floating_base_utilities::RtAffineUtils::appendAtColumn(hierarch_inv_dyn_.next_eq_cost_mat_, hierarch_inv_dyn_.next_eq_cost_vec_,
//                                                           posture_track_mat_, posture_track_vec_, 0);
//  }

}

void BipedalTaskComposer::addCostToHierarchyAfterReduction(int rank) const
{

  switch(rank)
  {
    case 0:
      break;
    case 1:
      break;
    case 2:
      break;
    case 3:
      break;
    default:
      break;
  }
}

void BipedalTaskComposer::updateAfterSolutionFound()
{

#ifndef RTEIG_NO_ASSERTS
  for (unsigned int i = 0; i < N_DOFS; ++i)
  {
    if (!(/*qp_found_solution_ && */(std::abs(hierarch_inv_dyn_.admis_torques_[i]) <= torque_limits_[i] + 0.001)
        && "computed torques exceed saturation"))
      std::cout << "joint " << i << " exceeds torque limits: " << std::abs(hierarch_inv_dyn_.admis_torques_[i]) << " < " << torque_limits_[i]
                                                                                                                                           << std::endl;
  }
#endif

  /*
   * compute admissible COP's
   */
  for (int eff_i = 1; eff_i <= N_ENDEFFS; ++eff_i)
  {
    if (is_endeff_const[eff_i])
    {
      foot_contact_handler_->computeEndeffCoP(eff_i, hierarch_inv_dyn_.getAdmisForceAtEndeff(eff_i),
                                            adms_cops_[eff_i]);
      foot_contact_handler_->computeCOPTorque(eff_i, hierarch_inv_dyn_.getAdmisForceAtEndeff(eff_i),
                                              adms_cop_torques_[eff_i]);
    }
    else
    {
      adms_cops_[eff_i].setZero();
    }
  }

  computeSlacks();
}


void BipedalTaskComposer::update(const Eigen::Matrix<double, 6, 1>& des_mom_rate,
                                 const Eigen::Matrix<double, N_DOFS, 1>& pose_ref, const Eigen::Matrix<double, N_ENDEFFS*6, 1>& des_foot_acc)
{

  /**
   * assert that feet are either fully constrained or fully unconstrained
   */
  for (int i = 1; i <= N_ENDEFFS; ++i)
  {
    is_endeff_const[i] = hierarch_inv_dyn_.kinematics().endeffectors()[i].c[1];
    for (int j = 2; j <= 6; ++j)
    {
      assert(
          is_endeff_const[i] == hierarch_inv_dyn_.floatingBaseKinematics().getEndEffectors()[i].c[j]
                                                                                                  && "each endeffector has to be fully constrained or fully unconstrained");
    }
  }
  const int n_lambdas = hierarch_inv_dyn_.floatingBaseKinematics().getNumConstraints();
  const int n_qdds = N_DOFS+6;
  const int n_vars = n_qdds + n_lambdas;
  num_stance_feet_ = n_lambdas/6;

  /**
   * compute tasks and constraints
   */
  // variable substitutions
  RtMatrixXUtils::setZero(var_to_lambda_, n_lambdas, n_vars);
  var_to_lambda_.block(0, n_qdds, n_lambdas, n_lambdas).setIdentity();

  RtMatrixXUtils::setZero(var_to_qdd_, n_qdds, n_vars);
  var_to_qdd_.block(0, 0, n_qdds, n_qdds).setIdentity();

  // momentum tracking
  Eigen::Matrix<double, N_DOFS + 6, 1> j_select = Eigen::Matrix<double, N_DOFS + 6, 1>::Ones();

  // dynamics inequalities
  //  Eigen::Matrix<double, 4 * N_ENDEFFS, 6 * N_ENDEFFS> tmp_cop_mat_;
  //  Eigen::Matrix<double, 4 * N_ENDEFFS, 6 * N_ENDEFFS> tmp_fric_mat_;
  //  foot_contact_handler_->createCOPIneqConstraints(tmp_cop_mat_);
  //  foot_contact_handler_->createFrictionForceInequalities(tmp_fric_mat_);

  for (int eff_i = 1; eff_i <= N_ENDEFFS; ++eff_i)
  {
    if (is_endeff_const[eff_i])
    {
      foot_contact_handler_->getCoPInequality(eff_i, cop_ineq_mats_[eff_i]);
      foot_contact_handler_->getFrictionInequality(eff_i, frict_ineq_mats_[eff_i]);
      //      cop_ineq_mats_[eff_i] = -tmp_cop_mat_.block((eff_i - 1) * 4, (eff_i - 1) * 6, 4, 6);
      //      frict_ineq_mats_[eff_i] = -tmp_fric_mat_.block((eff_i - 1) * 4, (eff_i - 1) * 6, 4, 6);
      cop_ineq_mats_[eff_i] = cop_weight_sqrt_[eff_i] * cop_ineq_mats_[eff_i];
    }
  }



  // dynamics equality (last six rows of eq of motion)
  RtMatrixXUtils::setZero(dyn_eq_mat_);
#ifdef OPT_OVER_TRQS
  RtVectorXUtils::resize(dyn_eq_vec_, N_DOFS+6);
#else
  RtVectorXUtils::resize(dyn_eq_vec_, 6);
#endif
  // set left part
#ifdef OPT_OVER_TRQS
  const int dyneq_row_off = N_DOFS;
  dyn_eq_mat_.topLeftCorner(N_DOFS, N_DOFS+6) = hierarch_inv_dyn_.inertiaMatrix().topRows(N_DOFS);
  dyn_eq_mat_.topRightCorner(N_DOFS, N_DOFS) = -Eigen::Matrix<double, N_DOFS, N_DOFS>::Identity();
#else
  const int dyneq_row_off = 0;
#endif

  //#ifdef DONT_USE_MOMENTUM_MAT
  dyn_eq_mat_.bottomLeftCorner(6, n_qdds) = hierarch_inv_dyn_.inertiaMatrix().bottomLeftCorner(6, N_DOFS+6);
  //#else
  //  dyn_eq_mat_.bottomLeftCorner(6, n_qdds) = hierarch_inv_dyn_.momentumComputation().getCentroidalMomentumMatrix();
  //#endif

  // set right part
  for (int eff_i = 1; eff_i <= N_ENDEFFS; ++eff_i)
  {
    if (is_endeff_const[eff_i])
    {

#ifdef OPT_OVER_TRQS
      dyn_eq_mat_.block(0, N_DOFS+6 + 6 * (eff_i-1), N_DOFS, 6) = -hierarch_inv_dyn_.kinematics().linkJacobian(link2endeffmap[eff_i]).transpose().topRows(N_DOFS);
#endif
      //#ifdef DONT_USE_MOMENTUM_MAT
      dyn_eq_mat_.block(dyneq_row_off, N_DOFS+6 + 6 * (eff_i-1), 6, 6) = -hierarch_inv_dyn_.kinematics().linkJacobian(link2endeffmap[eff_i]).transpose().bottomRows(6);
      //#else
      //      dyn_eq_mat_.block(dyneq_row_off, n_qdds + 6 * (eff_i-1), 6, 6) = -Eigen::Matrix<double, 6, 6>::Identity();
      //      Eigen::Matrix<double, 3, 1> cog_ankle = foot_contact_handler_->endeffPose(eff_i).block<3, 1>(0, 3) - hierarch_inv_dyn_.kinematics().cog();
      //      dyn_eq_mat_.block(dyneq_row_off+3, n_qdds + 6 * (eff_i-1), 3, 3) = -GeometryUtils::vector_to_skew_matrix(cog_ankle);
      //#endif
    }
  }

  // set equality vector
#ifdef OPT_OVER_TRQS
  dyn_eq_vec_.topRows(N_DOFS) = hierarch_inv_dyn_.nonlinearTerms().topRows(N_DOFS);

  dyn_eq_mat_ = is_joint_used_.asDiagonal()*dyn_eq_mat_;
  dyn_eq_vec_ = is_joint_used_.asDiagonal()*dyn_eq_vec_;
#endif

  //#ifdef DONT_USE_MOMENTUM_MAT
  dyn_eq_vec_.bottomRows(6) = hierarch_inv_dyn_.nonlinearTerms().bottomRows(6);
  //#else
  //  dyn_eq_vec_.bottomRows(6) = hierarch_inv_dyn_.momentumComputation().getdCentroidalMomentumMatrix() * hierarch_inv_dyn_.kinematics().generalizedJointVelocities();
  //  dyn_eq_vec_.segment(dyneq_row_off, 3) -= hierarch_inv_dyn_.kinematics().robotMass() * Eigen::Matrix<double, 3, 1>(0.0, 0.0, -G);
  //#endif

  // torque limits
  RtMatrixXUtils::setZero(torque_lim_mat_);
  RtVectorXUtils::resize(torque_lim_vec_, 2 * N_DOFS);

#ifdef OPT_OVER_TRQS
  torque_lim_mat_.topRightCorner(N_DOFS, N_DOFS) = Eigen::Matrix<double, N_DOFS, N_DOFS>::Identity();
  torque_lim_vec_.topRows(N_DOFS) = - torque_limits_;
  torque_lim_vec_.bottomRows(N_DOFS) = - torque_limits_;
#else
  torque_lim_mat_.block(0, 0, N_DOFS, n_qdds) = hierarch_inv_dyn_.inertiaMatrix().block(0, 0, N_DOFS, n_qdds);

  for (int eff_i = 1; eff_i <= N_ENDEFFS; ++eff_i)
  {
    if (is_endeff_const[eff_i])
    {
      torque_lim_mat_.block(0, n_qdds + 6 * (eff_i-1), N_DOFS, 6) = -hierarch_inv_dyn_.kinematics().linkJacobian(link2endeffmap[eff_i]).transpose().block(
          0, 0, N_DOFS, 6);
    }
  }
  torque_lim_vec_.topRows(N_DOFS) = is_joint_used_.topRows<N_DOFS>().asDiagonal() *(hierarch_inv_dyn_.nonlinearTerms().topRows(N_DOFS) - torque_limits_);
  torque_lim_vec_.bottomRows(N_DOFS) = is_joint_used_.topRows<N_DOFS>().asDiagonal() *(-hierarch_inv_dyn_.nonlinearTerms().topRows(N_DOFS) - torque_limits_);
#endif


  //we don't care about fingers and eyes, etc.
  torque_lim_mat_.topRows<N_DOFS>() = is_joint_used_.topRows<N_DOFS>().asDiagonal() * torque_lim_mat_.topRows<N_DOFS>();
  torque_lim_mat_.bottomRows(N_DOFS) = -torque_lim_mat_.topRows(N_DOFS);

//  RtMatrixXUtils::setIdentity(posture_track_mat_, N_DOFS);
//  posture_track_vec_ = - pose_ref;
//  posture_track_mat_ = is_joint_used_.topRows<N_DOFS>().asDiagonal() * (sqrt_posture_tr_weight_.asDiagonal() * posture_track_mat_);
//  posture_track_vec_ = is_joint_used_.topRows<N_DOFS>().asDiagonal() * (sqrt_posture_tr_weight_.asDiagonal() * posture_track_vec_);

  stat_feet_mat_.setZero(6*N_ENDEFFS, N_DOFS + 6);
  stat_feet_vec_.setZero(6*N_ENDEFFS, 1);
  for(int i=0; i<N_ENDEFFS; ++i)
  {
    stat_feet_mat_.block<6,N_DOFS+6>(i*6,0) = hierarch_inv_dyn_.kinematics().linkJacobian(link2endeffmap[i+1]);
    stat_feet_vec_.block<6,1>(i*6,0) = hierarch_inv_dyn_.kinematics().linkJacobianDerivative(link2endeffmap[i+1]) *
        hierarch_inv_dyn_.kinematics().generalizedJointVelocities();
  }
  stat_feet_vec_ -= des_foot_acc;
  stat_feet_mat_ = sqrt_stat_feet_weight_ * stat_feet_mat_;
  stat_feet_vec_ = sqrt_stat_feet_weight_ * stat_feet_vec_;


  // joint acceleration limits
  for (int i = 1; i <= N_DOFS; ++i)
  {
    //    std::cout << "joint_range[" << joint_names[i] << "]: "<< joint_range[i][MIN_THETA] <<
    //        ", "<<joint_range[i][THETA_OFFSET] << ", "<<joint_range[i][MAX_THETA] << std::endl;
    const double scale = j_padding_[i];  //0.1;
    const double max_j = scale * joint_range[i][MIN_THETA] + (1.0 - scale) * joint_range[i][MAX_THETA];
    const double min_j = (1.0 - scale) * joint_range[i][MIN_THETA] + scale * joint_range[i][MAX_THETA];

    min_accels_[i - 1] = -max_jacc_[i]
                                   * tanh(jacc_lim_slope_[i] * (hierarch_inv_dyn_.kinematics().generalizedJointPositions()[i - 1] - min_j));
    max_accels_[i - 1] = -max_jacc_[i]
                                   * tanh(jacc_lim_slope_[i] * (hierarch_inv_dyn_.kinematics().generalizedJointPositions()[i - 1] - max_j));
  }

  jacc_lim_vec_.block(0, 0, N_DOFS, 1) = -max_accels_;
  jacc_lim_vec_.block(N_DOFS, 0, N_DOFS, 1) = min_accels_;

  momentum_ctrl_.update(des_mom_rate);
}

void BipedalTaskComposer::computeSlacks()
{
  RtVectorX<N_DOFS+6 + 6*N_ENDEFFS>::d sol;
  RtVectorXUtils::resize(sol, N_DOFS+6 + 6*num_stance_feet_);
  sol.block(0,0, N_DOFS+6, 1) = hierarch_inv_dyn_.admis_accels_;
  sol.block(N_DOFS+6,0, 6*num_stance_feet_, 1) = hierarch_inv_dyn_.admis_forces_;

//  posture_track_slack_ = (posture_track_mat_*sol.head<N_DOFS>() + posture_track_vec_).squaredNorm();

  stat_feet_slack_ = (stat_feet_mat_*sol.topRows<N_DOFS+6>() + stat_feet_vec_).squaredNorm();

  RtVectorX<8+2*N_DOFS>::d slack;
  cop_ineq_slack_ = 0.0;
  frict_ineq_slack_ = 0.0;
  RtVectorX<2*N_DOFS>::d trq_lim_tmp;
#ifdef OPT_OVER_TRQS
  RtVectorX<N_DOFS+6>::d dyneq_lim_tmp;
#else
  RtVectorX<6>::d dyneq_lim_tmp;
#endif
  trq_lim_tmp = torque_lim_mat_.leftCols(N_DOFS+6)*hierarch_inv_dyn_.admis_accels_ +  torque_lim_vec_;
  //  torque_lim_slack_ =(torque_lim_mat_.leftCols(N_DOFS+6)*hierarch_inv_dyn_.admis_accels_ +  torque_lim_vec_).squaredNorm();
  dyneq_lim_tmp = dyn_eq_mat_.leftCols(N_DOFS+6)*hierarch_inv_dyn_.admis_accels_ +  dyn_eq_vec_;
  for(int i =1; i<=N_ENDEFFS; ++i)
  {
    if(is_endeff_const[i])
    {
      slack = cop_ineq_mats_[i]*hierarch_inv_dyn_.getAdmisForceAtEndeff(i);
      cop_ineq_slack_ += hierarch_inv_dyn_.inequalitySlack(slack);
      slack = frict_ineq_mats_[i]*hierarch_inv_dyn_.getAdmisForceAtEndeff(i);
      frict_ineq_slack_ += hierarch_inv_dyn_.inequalitySlack(slack);
      trq_lim_tmp += torque_lim_mat_.block(0,N_DOFS+6+6*(i-1), 2*N_DOFS, 6) *
          hierarch_inv_dyn_.getAdmisForceAtEndeff(i);
#ifdef OPT_OVER_TRQS
      dyneq_lim_tmp += dyn_eq_mat_.block(0,N_DOFS+6+6*(i-1), N_DOFS+6, 6) *
          hierarch_inv_dyn_.getAdmisForceAtEndeff(i);
#else
      dyneq_lim_tmp += dyn_eq_mat_.block(0,N_DOFS+6+6*(i-1), 6, 6) *
          hierarch_inv_dyn_.getAdmisForceAtEndeff(i);
#endif
    }
  }
  torque_lim_slack_ = hierarch_inv_dyn_.inequalitySlack(trq_lim_tmp);
  dyn_eq_slack_ = hierarch_inv_dyn_.inequalitySlack(dyneq_lim_tmp);
  slack = jacc_lim_mat_*sol.topRows<N_DOFS+6>() + jacc_lim_vec_;
  RtVectorXUtils::setZero(jacc_constr_activation_);
  jacc_constr_activation_ = (slack.head<N_DOFS>().array() >= 0.0-1e-6 ).select(1.0, jacc_constr_activation_);
  jacc_constr_activation_ = (slack.tail<N_DOFS>().array() >= 0.0-1e-6 ).select(-1.0, jacc_constr_activation_);
  jacc_lim_slack_ = hierarch_inv_dyn_.inequalitySlack(slack);
}

} /* namespace floating_base_utilities */
