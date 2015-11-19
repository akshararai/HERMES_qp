/*********************************************************************
 Autonomous Motion Department
 Max-Planck Intelligent Systems
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         JointSpaceCtrl.h

 \author       Alexander Herzog
 \date         Aug 18, 2014

 *********************************************************************/

#pragma once

#include <iostream>
#include <sstream>
#include "JointSpaceCtrl.hh"
#include "HierarchInverseDynamics.h"

#include "SL_collect_data.h"

namespace floating_base_utilities
{

template< int Max_Ineq_Rows, int Max_Eq_Rows>
JointSpaceCtrl< Max_Ineq_Rows, Max_Eq_Rows>::JointSpaceCtrl(
    HierarchInverseDynamics<Max_Ineq_Rows, Max_Eq_Rows>& opt_prob_composer) : opt_prob_composer_(&opt_prob_composer),
    kinematics_(&opt_prob_composer.kinematics())
{
  slack_.setZero();
  vec_.setZero();
  weight_.setIdentity();
  ranks_ = -Eigen::Matrix<int, N_DOFS+6, 1>::Ones();

}

template< int Max_Ineq_Rows, int Max_Eq_Rows>
void JointSpaceCtrl< Max_Ineq_Rows, Max_Eq_Rows>::addCostToHierarchy(int rank) const
{
  opt_prob_composer_->template appendRowsOfRank
      <N_DOFS+6,Eigen::Matrix<double, N_DOFS+6, N_DOFS+6> , Eigen::Matrix<double, N_DOFS+6, 1> >
      (rank, ranks_, weight_, vec_,true , 0);
}

template< int Max_Ineq_Rows, int Max_Eq_Rows>
void JointSpaceCtrl< Max_Ineq_Rows, Max_Eq_Rows>::initialize(int rank)
{
  ranks_ = rank*Eigen::Matrix<int, N_DOFS+6, 1>::Ones();

  name_ = std::string("JCtrl");

  std::stringstream ss;
  std::vector<std::string> var_names;
  for(int i=1; i<=N_DOFS; ++i){
    ss << name_ << "_slck_";
    ss << joint_names[i];
    var_names.push_back(std::string());
    ss >> var_names.back();
    ss.clear();

  }

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
void JointSpaceCtrl< Max_Ineq_Rows, Max_Eq_Rows>::update(
    const Eigen::Matrix<double, N_DOFS+6, 1>& des_qdd)
{
  vec_ = -weight_*des_qdd;
}

template< int Max_Ineq_Rows, int Max_Eq_Rows>
void JointSpaceCtrl< Max_Ineq_Rows, Max_Eq_Rows>::updateAfterSolutionFound()
{

  slack_ = weight_*opt_prob_composer_->admis_accels_ + vec_;
}

} /* namespace floating_base_utilities */
