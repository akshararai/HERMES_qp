/*********************************************************************
 Autonomous Motion Department
 Max-Planck Intelligent Systems
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         DynamicsConstraint.h

 \author       Alexander Herzog
 \date         Jul 18, 2013

 *********************************************************************/

#pragma once

#include <iostream>
#include <sstream>
#include "DynamicsConstraint.hh"
#include "HierarchInverseDynamics.h"

#include "SL_collect_data.h"

namespace floating_base_utilities
{

template< int Max_Ineq_Rows, int Max_Eq_Rows,  bool use_momentum_rate_mat>
DynamicsConstraint< Max_Ineq_Rows, Max_Eq_Rows, use_momentum_rate_mat>::DynamicsConstraint(
    HierarchInverseDynamics< Max_Ineq_Rows, Max_Eq_Rows>& opt_prob_composer) : opt_prob_composer_(opt_prob_composer),
    kinematics_(opt_prob_composer.kinematics())
{
  mat_.setZero();
  vec_.setZero();
  slack_.setZero();
  rank_ = -1;
}

template< int Max_Ineq_Rows, int Max_Eq_Rows,  bool use_momentum_rate_mat>
void DynamicsConstraint< Max_Ineq_Rows, Max_Eq_Rows, use_momentum_rate_mat>::addCostToHierarchy(int rank) const
{
  if(rank != rank_)
    return;

  RtAffineUtils::append(opt_prob_composer_.next_eq_cost_mat_,
      opt_prob_composer_.next_eq_cost_vec_, mat_, vec_);
}

template< int Max_Ineq_Rows, int Max_Eq_Rows,  bool use_momentum_rate_mat>
void DynamicsConstraint< Max_Ineq_Rows, Max_Eq_Rows, use_momentum_rate_mat>::initialize(int rank)
{
  rank_ = rank;
  name_ = std::string("DynEq");

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
void DynamicsConstraint< Max_Ineq_Rows, Max_Eq_Rows, use_momentum_rate_mat>::update()
{
  mat_.template leftCols<N_DOFS+6>() = opt_prob_composer_.inertiaMatrix().template bottomRows<6>();
  for (int eff_i = 1; eff_i <= N_ENDEFFS; ++eff_i)
  {
    if (kinematics_.endeffector(eff_i).c[1])
    {
      for(int d=2; d<=6; ++d)
        assert(kinematics_.endeffector(eff_i).c[1] == kinematics_.endeffector(eff_i).c[d]);
      mat_.block<6,6>(0, N_DOFS+6 + 6 * (eff_i-1)) = -kinematics_.linkJacobian(link2endeffmap[eff_i]).transpose().template bottomRows<6>();
    }
  }

  vec_ = opt_prob_composer_.nonlinearTerms().template bottomRows<6>();
}

template< int Max_Ineq_Rows, int Max_Eq_Rows,  bool use_momentum_rate_mat>
void DynamicsConstraint< Max_Ineq_Rows, Max_Eq_Rows, use_momentum_rate_mat>::updateAfterSolutionFound()
{
  slack_ = mat_.leftCols<N_DOFS+6>()*opt_prob_composer_.admis_accels_ + vec_;
  for (int eff_i = 1; eff_i <= N_ENDEFFS; ++eff_i){
    if (kinematics_.endeffector(eff_i).c[1]){
      slack_ += mat_.block<6,6>(0,N_DOFS+6+6*(eff_i-1)) * opt_prob_composer_.getAdmisForceAtEndeff(eff_i);
    }
  }
}

} /* namespace floating_base_utilities */
