/*********************************************************************
 Autonomous Motion Department
 Max-Planck Intelligent Systems
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         MomentumRateCtrl.h

 \author       Alexander Herzog
 \date         Jul 18, 2013

 *********************************************************************/

#ifndef MOMENTUMRATECTRL_H_
#define MOMENTUMRATECTRL_H_

#include <iostream>
#include <sstream>
#include "MomentumRateCtrl.hh"
#include "HierarchInverseDynamics.h"

#include "SL_collect_data.h"

namespace floating_base_utilities
{

template< int Max_Ineq_Rows, int Max_Eq_Rows,  bool use_momentum_rate_mat>
const unsigned int MomentumRateCtrl< Max_Ineq_Rows, Max_Eq_Rows, use_momentum_rate_mat>::num_variables_;

template< int Max_Ineq_Rows, int Max_Eq_Rows,  bool use_momentum_rate_mat>
MomentumRateCtrl< Max_Ineq_Rows, Max_Eq_Rows, use_momentum_rate_mat>::MomentumRateCtrl(
    HierarchInverseDynamics< Max_Ineq_Rows, Max_Eq_Rows>& opt_prob_composer) : opt_prob_composer_(&opt_prob_composer),
    kinematics_(&opt_prob_composer.kinematics())
{
//#ifdef DONT_USE_MOMENTUM_MAT
  RtMatrixXUtils::setZero(mat_);
//#else
  momentum_comp_ = &opt_prob_composer.momentumComputation();
//  mat_.setZero();
//#endif
  slack_.setZero();
  weight_.setIdentity();
  ranks_ = -Eigen::Matrix<int, 6, 1>::Ones();
}

template< int Max_Ineq_Rows, int Max_Eq_Rows,  bool use_momentum_rate_mat>
void MomentumRateCtrl< Max_Ineq_Rows, Max_Eq_Rows, use_momentum_rate_mat>::addCostToHierarchy(int rank) const
{
  const int append_col = (use_momentum_rate_mat)?0:N_DOFS+6;
  opt_prob_composer_->template appendRowsOfRank<6, typename RtMatrixX<6, num_variables_>::d, Eigen::Matrix<double, 6, 1> >(rank, ranks_,
       mat_, vec_, true, append_col);
}

template< int Max_Ineq_Rows, int Max_Eq_Rows,  bool use_momentum_rate_mat>
void MomentumRateCtrl< Max_Ineq_Rows, Max_Eq_Rows, use_momentum_rate_mat>::initialize(int rank)
{
  ranks_ = rank*Eigen::Matrix<int, 6, 1>::Ones();

  name_ = std::string("MomRate");

  std::stringstream ss;
  std::vector<std::string> var_names;
  ss << name_ << "_slck_x";
  var_names.push_back(std::string());
  ss >> var_names.back();
  ss.clear();
  ss << name_ << "_slck_y";
  var_names.push_back(std::string());
  ss >> var_names.back();
  ss.clear();
  ss << name_ << "_slck_z";
  var_names.push_back(std::string());
  ss >> var_names.back();
  ss.clear();
  ss << name_ << "_slck_a";
  var_names.push_back(std::string());
  ss >> var_names.back();
  ss.clear();
  ss << name_ << "_slck_b";
  var_names.push_back(std::string());
  ss >> var_names.back();
  ss.clear();
  ss << name_ << "_slck_g";
  var_names.push_back(std::string());
  ss >> var_names.back();
  ss.clear();

  for(unsigned int i=0; i<var_names.size(); ++i)
    if(var_names[i].length() > 20)
      std::cout << "WARNING: Long Variable Name: " << var_names[i] << ". Can SL deal with that?" << std::endl;

  std::cout << "Adding Variables to Data Collection:" << std::endl;
  char tmp_c_str[50];
  for(unsigned int i=0; i<var_names.size(); ++i)
  {
    std::strcpy(tmp_c_str,var_names[i].c_str());
    addVarToCollect((char*)&slack_(i), tmp_c_str, "-", DOUBLE, TRUE);
    std::cout << tmp_c_str << std::endl;
  }
}

template< int Max_Ineq_Rows, int Max_Eq_Rows,  bool use_momentum_rate_mat>
void MomentumRateCtrl< Max_Ineq_Rows, Max_Eq_Rows, use_momentum_rate_mat>::update(
    const Eigen::Matrix<double, 6, 1>& des_mom_rate)
{
  if(use_momentum_rate_mat){
    mat_ = momentum_comp_->getCentroidalMomentumMatrix();
    vec_ = momentum_comp_->getdCentroidalMomentumMatrix()*kinematics_->generalizedJointVelocities();
  }else{
    opt_prob_composer_->contactHelper().constructMomrateMap(mat_, vec_);
  }
  vec_ -= des_mom_rate;
  vec_ = weight_*vec_;
  mat_ = weight_*mat_;
}

template< int Max_Ineq_Rows, int Max_Eq_Rows,  bool use_momentum_rate_mat>
void MomentumRateCtrl< Max_Ineq_Rows, Max_Eq_Rows, use_momentum_rate_mat>::updateAfterSolutionFound()
{
//#ifdef DONT_USE_MOMENTUM_MAT

  if(use_momentum_rate_mat){
    slack_ = mat_*opt_prob_composer_->admis_accels_ + vec_;
  }else{
    slack_.setZero();
    for (int eff_i = 1; eff_i <= N_ENDEFFS; ++eff_i)
    {
      slack_ += mat_.template block<6,floatingBaseToolsRobotInfo::max_contact_force_dim_ >(
          0, floatingBaseToolsRobotInfo::max_contact_force_dim_*(eff_i-1)) * opt_prob_composer_->getAdmisForceAtEndeff(eff_i);
    }
    slack_ += vec_;
  }
//#else
//  slack_ = mat_*opt_prob_composer_->admis_accels_ + vec_;
//#endif
}

} /* namespace floating_base_utilities */
#endif /* MOMENTUMRATECTRL_H_ */
