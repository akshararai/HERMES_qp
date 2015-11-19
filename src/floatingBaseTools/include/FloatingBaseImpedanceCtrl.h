/*********************************************************************
 Autonomous Motion Department
 Max-Planck Intelligent Systems
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         FloatingBaseImpedanceCtrl.h

 \author       Alexander Herzog
 \date         Jul 11, 2013

 *********************************************************************/

#ifndef FLOATINGBASEIMPEDANCECTRL_H_
#define FLOATINGBASEIMPEDANCECTRL_H_

#include <iostream>
#include <sstream>
#include "FloatingBaseImpedanceCtrl.hh"
#include "HierarchInverseDynamics.h"

#include "SL_collect_data.h"

namespace floating_base_utilities
{

template< int Max_Ineq_Rows, int Max_Eq_Rows>
FloatingBaseImpedanceCtrl< Max_Ineq_Rows, Max_Eq_Rows>::FloatingBaseImpedanceCtrl(
    HierarchInverseDynamics<Max_Ineq_Rows, Max_Eq_Rows>& opt_prob_composer) : opt_prob_composer_(&opt_prob_composer),
    kinematics_(&opt_prob_composer.kinematics()), rank_(-1)
{
  RtMatrixXUtils::setZero(mat_);
  slack_.setZero();
  vec_.setZero();
  weight_.setIdentity();

}

template< int Max_Ineq_Rows, int Max_Eq_Rows>
void FloatingBaseImpedanceCtrl< Max_Ineq_Rows, Max_Eq_Rows>::addCostToHierarchy(int rank) const
{
  if(rank != rank_)
    return;

  RtAffineUtils::appendAtColumn(opt_prob_composer_->next_eq_cost_mat_, opt_prob_composer_->next_eq_cost_vec_,
                                                 mat_, vec_, 0);
}

template< int Max_Ineq_Rows, int Max_Eq_Rows>
void FloatingBaseImpedanceCtrl< Max_Ineq_Rows, Max_Eq_Rows>::initialize(int rank)
{
  rank_ = rank;
  name_ = std::string("impedance");

  for(int eff_i = 1; eff_i <= N_ENDEFFS; ++eff_i)
    impedance_cmds_[eff_i].initialize(*kinematics_, link2endeffmap[eff_i]);

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

template< int Max_Ineq_Rows, int Max_Eq_Rows>
void FloatingBaseImpedanceCtrl< Max_Ineq_Rows, Max_Eq_Rows>::update(const bool* is_eff_constr)
{
  // update affine mapping
  vec_.setZero();
  vec_(2) = -kinematics_->robotMass()*G;
  mat_.setZero();
  Eigen::Matrix<double,6,N_DOFS+6> djac_sum = Eigen::Matrix<double,6,N_DOFS+6>::Zero();
  for(int eff_i=1;eff_i<=N_ENDEFFS;++eff_i)
  {
    if(is_eff_constr[eff_i])
    {
      mat_ += impedance_cmds_[eff_i].impedance().asDiagonal()*kinematics_->linkJacobian(link2endeffmap[eff_i]);
      vec_ -= impedance_cmds_[eff_i].impedance().asDiagonal()*impedance_cmds_[eff_i].command();
      djac_sum += impedance_cmds_[eff_i].impedance().asDiagonal()*kinematics_->linkJacobianDerivative(link2endeffmap[eff_i]);
    }
  }
  vec_ += djac_sum*kinematics_->generalizedJointVelocities();
  mat_ = weight_*mat_;
  vec_ = weight_*vec_;
}

template< int Max_Ineq_Rows, int Max_Eq_Rows>
void FloatingBaseImpedanceCtrl< Max_Ineq_Rows, Max_Eq_Rows>::updateAfterSolutionFound()
{

  slack_ = mat_*opt_prob_composer_->admis_accels_ + vec_;
}

} /* namespace floating_base_utilities */
#endif /* FLOATINGBASEIMPEDANCECTRL_H_ */
