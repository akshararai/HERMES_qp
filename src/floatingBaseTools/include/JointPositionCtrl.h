/*
 * JointPositionCtrl.h
 *
 *  Created on: Aug 15, 2014
 *      Author: righetti
 */

#ifndef JOINTPOSITIONCTRL_H_
#define JOINTPOSITIONCTRL_H_

#include <iostream>
#include <sstream>
#include "JointPositionCtrl.hh"
#include "HierarchInverseDynamics.h"

#include "SL_collect_data.h"

namespace floating_base_utilities
{

template< int Max_Ineq_Rows, int Max_Eq_Rows, int Num_Joints>
JointPositionCtrl< Max_Ineq_Rows, Max_Eq_Rows, Num_Joints>::JointPositionCtrl(
    HierarchInverseDynamics<Max_Ineq_Rows, Max_Eq_Rows>& opt_prob_composer) : opt_prob_composer_(&opt_prob_composer),
    kinematics_(&opt_prob_composer.kinematics())
{
  mat_.setZero();
  slack_.setZero();
  vec_.setZero();
  weight_.setOnes();
  ranks_ = -Eigen::Matrix<int, Num_Joints, 1>::Ones();
  P_gains_.setZero();
  D_gains_.setZero();
  variable_gain_.setIdentity();
}

template< int Max_Ineq_Rows, int Max_Eq_Rows, int Num_Joints>
void JointPositionCtrl< Max_Ineq_Rows, Max_Eq_Rows, Num_Joints>::addCostToHierarchy(int rank) const
{
  opt_prob_composer_->template appendRowsOfRank
  <Num_Joints,Eigen::Matrix<double, Num_Joints, Num_Joints> , Eigen::Matrix<double, Num_Joints, 1> >
  (rank, ranks_, mat_, vec_,true , starting_joint_-1);
}

template< int Max_Ineq_Rows, int Max_Eq_Rows, int Num_Joints>
void JointPositionCtrl< Max_Ineq_Rows, Max_Eq_Rows, Num_Joints>::initialize(int rank, int starting_joint, std::string ctrl_name)
{
  starting_joint_ = starting_joint;
  ranks_ = rank*Eigen::Matrix<int, Num_Joints, 1>::Ones();
  name_ = ctrl_name;

  std::stringstream ss;
  std::vector<std::string> var_names;
  for(int i=0; i<Num_Joints; ++i)
  {
    ss << name_ << "_slck_" << std::string(joint_names[starting_joint_+i]);
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

  for(int i=0; i<Num_Joints; ++i)
  {
    std::string varname = name_ + "_desacc_" + std::string(joint_names[starting_joint_+i]);
    addVarToCollect((char *)&des_acc_(i), varname.c_str(), "m/s2", DOUBLE, TRUE);
  }
}

template< int Max_Ineq_Rows, int Max_Eq_Rows, int Num_Joints>
void JointPositionCtrl< Max_Ineq_Rows, Max_Eq_Rows, Num_Joints>::update(
    const Eigen::Matrix<double, Num_Joints, 1>& ref_pos,
    const Eigen::Matrix<double, Num_Joints, 1>& ref_vel,
    const Eigen::Matrix<double, Num_Joints, 1>& ref_acc)
{
  //compute desired acceleration
  Eigen::Matrix<double, Num_Joints, 1> actual_pos, actual_vel;
  actual_pos = kinematics_->generalizedJointPositions().block<Num_Joints, 1>(starting_joint_-1,0);
  actual_vel = kinematics_->generalizedJointVelocities().block<Num_Joints, 1>(starting_joint_-1,0);

  des_acc_ = variable_gain_*ref_acc + (P_gains_.asDiagonal() * (ref_pos - actual_pos) + D_gains_.asDiagonal() * (ref_vel - actual_vel));

  // update affine mapping
  Eigen::Matrix<double, Num_Joints, Num_Joints> ATW = variable_gain_.transpose() * weight_.asDiagonal();
  mat_ = ATW * variable_gain_;
  vec_ = -1.0 * ATW *des_acc_;
}

template< int Max_Ineq_Rows, int Max_Eq_Rows, int Num_Joints>
void JointPositionCtrl< Max_Ineq_Rows, Max_Eq_Rows, Num_Joints>::updateAfterSolutionFound()
{

  slack_ = mat_*opt_prob_composer_->admis_accels_.block(starting_joint_-1,0,Num_Joints,1) + vec_;
}

} /* namespace floating_base_utilities */
#endif /* JOINTPOSITIONCTRL_H_ */
