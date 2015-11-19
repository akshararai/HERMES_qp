/*********************************************************************
 Autonomous Motion Department
 Max-Planck Intelligent Systems
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         CartesianPositionCtrl.h

 \author       Alexander Herzog
 \date         Jul 11, 2013

 *********************************************************************/

#ifndef CARTESIANPOSITIONCTRL_H_
#define CARTESIANPOSITIONCTRL_H_

#include <iostream>
#include <sstream>
#include "CartesianPositionCtrl.hh"
#include "HierarchInverseDynamics.h"

#include "SL_collect_data.h"

namespace floating_base_utilities
{

template< int Max_Ineq_Rows, int Max_Eq_Rows>
CartesianPositionCtrl< Max_Ineq_Rows, Max_Eq_Rows>::CartesianPositionCtrl(
    HierarchInverseDynamics<Max_Ineq_Rows, Max_Eq_Rows>& opt_prob_composer) : opt_prob_composer_(&opt_prob_composer),
    kinematics_(&opt_prob_composer.kinematics())
{
  mat_.setZero();
  slack_.setZero();
  vec_.setZero();
  weight_.setIdentity();
  ranks_ = -Eigen::Matrix<int, 6, 1>::Ones();

}

template< int Max_Ineq_Rows, int Max_Eq_Rows>
void CartesianPositionCtrl< Max_Ineq_Rows, Max_Eq_Rows>::addCostToHierarchy(int rank) const
{
  opt_prob_composer_->template appendRowsOfRank
      <6,Eigen::Matrix<double, 6, N_DOFS+6> , Eigen::Matrix<double, 6, 1> >
      (rank, ranks_, mat_, vec_,true , 0);
}

template< int Max_Ineq_Rows, int Max_Eq_Rows>
void CartesianPositionCtrl< Max_Ineq_Rows, Max_Eq_Rows>::initialize(int rank, int link_id)
{
  ranks_ = rank*Eigen::Matrix<int, 6, 1>::Ones();
  link_id_ = link_id;
  name_ = std::string(link_names[link_id_]);

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
void CartesianPositionCtrl< Max_Ineq_Rows, Max_Eq_Rows>::update(
    const Eigen::Matrix<double, 6, 1>& des_acc)
{
  // update affine mapping
  mat_ = weight_*kinematics_->linkJacobian(link_id_);
  vec_ = weight_*(kinematics_->linkJacobianDerivative(link_id_) *
        kinematics_->generalizedJointVelocities() - des_acc);
}

template< int Max_Ineq_Rows, int Max_Eq_Rows>
void CartesianPositionCtrl< Max_Ineq_Rows, Max_Eq_Rows>::updateAfterSolutionFound()
{

  slack_ = mat_*opt_prob_composer_->admis_accels_ + vec_;
}

} /* namespace floating_base_utilities */
#endif /* CARTESIANPOSITIONCTRL_H_ */
