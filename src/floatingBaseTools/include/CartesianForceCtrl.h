/*********************************************************************
 Autonomous Motion Department
 Max-Planck Intelligent Systems
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         CartesianForceCtrl.h

 \author       Alexander Herzog
 \date         Jul 16, 2013

 *********************************************************************/

#ifndef CARTESIANFORCECTRL_H_
#define CARTESIANFORCECTRL_H_
#include <iostream>
#include <sstream>
#include "CartesianForceCtrl.hh"
#include "HierarchInverseDynamics.h"

#include "SL_collect_data.h"



namespace floating_base_utilities
{

template< int Max_Ineq_Rows, int Max_Eq_Rows>
CartesianForceCtrl< Max_Ineq_Rows, Max_Eq_Rows>::CartesianForceCtrl(
    HierarchInverseDynamics<Max_Ineq_Rows, Max_Eq_Rows>& opt_prob_composer) : opt_prob_composer_(&opt_prob_composer),
    kinematics_(&opt_prob_composer.kinematics()), contact_helper_(&opt_prob_composer.contactHelper()), rank_(-1)
{
  mat_.setZero();
  slack_.setZero();
  vec_.setZero();
  weight_.setIdentity();
  world_local_trans_.setIdentity();

}

template< int Max_Ineq_Rows, int Max_Eq_Rows>
void CartesianForceCtrl< Max_Ineq_Rows, Max_Eq_Rows>::addCostToHierarchy(int rank) const
{
  if(rank != rank_)
    return;
  switch(control_type_)
  {
    case eCT_TRACK:
      RtAffineUtils::appendAtColumn(
          opt_prob_composer_->next_eq_cost_mat_, opt_prob_composer_->next_eq_cost_vec_,
            mat_, vec_, N_DOFS+6 + 6*(endeff_id_-1));
      break;
    case eCT_MIN_FRC:
    case eCT_MAX_FRC:
    case eCT_FRC_SAT:
      RtAffineUtils::appendAtColumn(
          opt_prob_composer_->next_ineq_cost_mat_, opt_prob_composer_->next_ineq_cost_vec_,
            mat_, vec_, N_DOFS+6 + 6*(endeff_id_-1));
      break;
    default:
      assert(false && "Invalid control type!");
      break;
  }
}

template< int Max_Ineq_Rows, int Max_Eq_Rows>
void CartesianForceCtrl< Max_Ineq_Rows, Max_Eq_Rows>::initialize(int rank, int endeff_id, bool in_local_frame,
                                                                                                         ControlType ctrl_type)
{
  rank_ = rank;

  endeff_id_ = endeff_id;
  in_local_frame_ = in_local_frame;
  control_type_ = ctrl_type;

  name_ = std::string(link_names[link2endeffmap[endeff_id_]]).substr(0,3);//TODO: THIS IS A HACK, FIND BETTER SOLUTION TO ABREVIATE NAME

  switch(control_type_)
  {
    case eCT_TRACK:
      ctrl_type_mat_sign_ = 1;
      ctrl_type_vec_sign_ = -1;
      mat_.resize(6,6);
      vec_.resize(6,1);
      break;
    case eCT_MIN_FRC:
      ctrl_type_mat_sign_ = -1;
      ctrl_type_vec_sign_ = 1;
      mat_.resize(6,6);
      vec_.resize(6,1);
      name_.append("_min");
      break;
    case eCT_MAX_FRC:
      ctrl_type_mat_sign_ = 1;
      ctrl_type_vec_sign_ = -1;
      mat_.resize(6,6);
      vec_.resize(6,1);
      name_.append("_max");
      break;
    case eCT_FRC_SAT:
      ctrl_type_mat_sign_ = 1;
      ctrl_type_vec_sign_ = -1;
      mat_.resize(12,6);
      vec_.resize(12,1);
      name_.append("_sat");
      break;
    default:
      assert(false && "Invalid control type!");
      break;
  }

  std::stringstream ss;
  std::vector<std::string> var_names;
  ss << name_ << "_frc_slck_x";
  var_names.push_back(std::string());
  ss >> var_names.back();
  ss.clear();
  ss << name_ << "_frc_slck_y";
  var_names.push_back(std::string());
  ss >> var_names.back();
  ss.clear();
  ss << name_ << "_frc_slck_z";
  var_names.push_back(std::string());
  ss >> var_names.back();
  ss.clear();
  ss << name_ << "_frc_slck_a";
  var_names.push_back(std::string());
  ss >> var_names.back();
  ss.clear();
  ss << name_ << "_frc_slck_b";
  var_names.push_back(std::string());
  ss >> var_names.back();
  ss.clear();
  ss << name_ << "_frc_slck_g";
  var_names.push_back(std::string());
  ss >> var_names.back();
  ss.clear();
  ss << name_ << "_frc_slck";
  var_names.push_back(std::string());
  ss >> var_names.back();
  ss.clear();

  for(unsigned int i=0; i<var_names.size(); ++i)
    if(var_names[i].length() > 20)
      std::cout << "WARNING: Long Variable Name: " << var_names[i] << ". Can SL deal with that?" << std::endl;

  std::cout << "Adding Variables to Data Collection:" << std::endl;
  char tmp_c_str[50];
  for(unsigned int i=0; i<slack_.size(); ++i)
  {
    std::strcpy(tmp_c_str,var_names[i].c_str());
    addVarToCollect((char*)&slack_(i), tmp_c_str, "-", DOUBLE, TRUE);
    std::cout << tmp_c_str << std::endl;
  }
  std::strcpy(tmp_c_str,var_names[6].c_str());
  addVarToCollect((char*)&slack_norm_, tmp_c_str, "-", DOUBLE, TRUE);
  std::cout << tmp_c_str << std::endl;
}

template< int Max_Ineq_Rows, int Max_Eq_Rows>
void CartesianForceCtrl< Max_Ineq_Rows, Max_Eq_Rows>::update(
    const Eigen::Matrix<double, 6, 1>& des_frc)
{
  // update affine mapping
  if(in_local_frame_)
  {
    Eigen::Matrix4d std_endeff_p;
    kinematics_->standardizedEndeffPose(endeff_id_, std_endeff_p);
//    const Eigen::Matrix4d& std_endeff_p = contact_helper_->projected_foot_frames_[endeff_id_];

    world_local_trans_.topLeftCorner<3,3>() = std_endeff_p.topLeftCorner<3,3>().transpose();
    world_local_trans_.bottomRightCorner<3,3>() = world_local_trans_.topLeftCorner<3,3>();
  }
  else
    world_local_trans_.setIdentity();

  mat_.topRows<6>() = ctrl_type_mat_sign_*weight_*world_local_trans_;
  vec_.head<6>() = ctrl_type_vec_sign_*weight_*des_frc;

  if(control_type_ == eCT_FRC_SAT){
    vec_.head<6>() -= weight_*saturation_;
    mat_.bottomRows<6>() = -weight_*world_local_trans_;
    vec_.tail<6>() = weight_*(des_frc-saturation_);
  }

}

template< int Max_Ineq_Rows, int Max_Eq_Rows>
void CartesianForceCtrl< Max_Ineq_Rows, Max_Eq_Rows>::updateAfterSolutionFound()
{
  opt_prob_composer_->getAdmisForceAtEndeff(endeff_id_);
  slack_ = mat_.topRows<6>()*opt_prob_composer_->getAdmisForceAtEndeff(endeff_id_) + vec_.head<6>();
  if(control_type_ == eCT_MIN_FRC || control_type_ == eCT_MAX_FRC)
  {
    opt_prob_composer_->inequalitySlack(slack_);
  }
  if(control_type_ == eCT_FRC_SAT){
    opt_prob_composer_->inequalitySlack(slack_);
    Eigen::Matrix<double, 6, 1> slck_bottom = mat_.bottomRows<6>()*opt_prob_composer_->getAdmisForceAtEndeff(endeff_id_) + vec_.tail<6>();
    opt_prob_composer_->inequalitySlack(slck_bottom);
    slack_ -= slck_bottom;
  }
  slack_norm_ = slack_.squaredNorm();
}

} /* namespace floating_base_utilities */
#endif /* CARTESIANFORCECTRL_H_ */
