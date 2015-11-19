/*
 * biped_controller.cpp
 *
 *  Created on: Jul 7, 2014
 *      Author: righetti
 */

#include "biped_controller.h"
#include <SL.h>
#include <SL_collect_data.h>
#include <SL_common.h>
#include "drawUtils.h"
#include <utility_macros.h>
#include <quaternions.h>

using namespace floating_base_utilities;
using namespace momentum_balance_control;

BipedController::BipedController():initialized_(false)
{
}

BipedController::~BipedController() {
}

bool BipedController::initialize(std::string config_file)
{
  initialized_ = true;
  //stop data collection otherwise might crash
  stopcd();

  for(int i=1;i<=N_DOFS;++i){
    initial_u_[i-1] = joint_state[i].u;
  }

  kinematics_eigen_.reset(new KinematicsEigen());
  floating_base_kin_.reset(new FloatingBaseKinematics());
  momentum_helper_.reset(new MomentumComputation());
  foot_contact_handler_.reset(new FootContactHandlerHermes(config_file));
  hierarch_inv_dyn_.reset(new BipedalTaskComposer::HierarchInverseDynamicsHermes(
      *kinematics_eigen_, *momentum_helper_, *foot_contact_handler_, *floating_base_kin_, config_file));
  biped_task_comp_.reset(new BipedalTaskComposer(*hierarch_inv_dyn_));

  base_controller_.reset(new BipedalTaskComposer::CartesianPositionCtrlHermes(*hierarch_inv_dyn_));
  //  swing_foot_controller_[LEFT_FOOT].reset(new BipedalTaskComposer::CartesianPositionCtrlHermes(*hierarch_inv_dyn_));
  //  swing_foot_controller_[RIGHT_FOOT].reset(new BipedalTaskComposer::CartesianPositionCtrlHermes(*hierarch_inv_dyn_));

  leading_leg_ = LEFT_FOOT;
  swing_leg_ = RIGHT_FOOT;

  //initialization of helpers
  kinematics_eigen_->initialize(joint_state, base_state, base_orient,endeff);
  momentum_helper_->initialize();
  foot_contact_handler_->initialize(kinematics_eigen_.get());
  hierarch_inv_dyn_->initialize();
  biped_task_comp_->initialize();


  //initialize of base controller
  int base_ctrl_rank;
  if(!read_parameter_pool_int(config_file.c_str(), "base_ctrl_rank", &base_ctrl_rank))
  {
    printf("ERROR cannot read base_ctrl_rank\n");
    return (initialized_ = false);
  }
  base_controller_->initialize(base_ctrl_rank, BASE);
  double base_wght[6+1];
  if(!read_parameter_pool_double_array(config_file.c_str(), "BASE_TRACK_WEIGHT", 6, base_wght))
  {
    printf("ERROR cannot read BASE_TRACK_WEIGHT\n");
    return (initialized_ = false);
  }
  base_controller_->weightingMat() = Eigen::Map<Eigen::Matrix<double, 6, 1> >(&(base_wght[1])).asDiagonal();
  base_controller_->weightingMat() = base_controller_->weightingMat().array().sqrt();
  double base_gains[6+1];
  if(!read_parameter_pool_double_array(config_file.c_str(), "base_P_gains", 6, base_gains))
  {
    printf("ERROR cannot read base_P_gains\n");
    return (initialized_ = false);
  }
  base_P_gains_ = Eigen::Map<Eigen::Matrix<double, 6, 1> >(&(base_gains[1]));
  if(!read_parameter_pool_double_array(config_file.c_str(), "base_D_gains", 6, base_gains))
  {
    {
      printf("ERROR cannot read base_D_gains\n");
      return (initialized_ = false);
    }
  }
  base_D_gains_ = Eigen::Map<Eigen::Matrix<double, 6, 1> >(&(base_gains[1]));

  hierarch_inv_dyn_->sub_cost_composers_.push_back(static_cast<HierarchAffineCost*>(&(*base_controller_)));



  joint_position_controller_.reset(new BipedalTaskComposer::JointPositionCtrlHermes(*hierarch_inv_dyn_));
  leg_joint_position_controller_[LEFT_FOOT].reset(new BipedalTaskComposer::LegJointPositionCtrlHermes(*hierarch_inv_dyn_));
  leg_joint_position_controller_[RIGHT_FOOT].reset(new BipedalTaskComposer::LegJointPositionCtrlHermes(*hierarch_inv_dyn_));
  //initialize the swing leg joint pos controllers
  int joint_ctrl_rank;
  if(!read_parameter_pool_int(config_file.c_str(), "posture_rank", &joint_ctrl_rank))
  {
    printf("ERROR cannot read joint_pos_ctrl_rank\n");
    return (initialized_ = false);
  }
  joint_position_controller_->initialize(joint_ctrl_rank, L_HFE, std::string("JPC"));
  leg_joint_position_controller_[LEFT_FOOT]->initialize(joint_ctrl_rank, L_HFE, std::string("LPC"));
  leg_joint_position_controller_[RIGHT_FOOT]->initialize(joint_ctrl_rank, R_HFE, std::string("RPC"));

  double joint_ctrl_weight[N_DOFS+1];
  if(!read_parameter_pool_double_array(config_file.c_str(), "POSTURE_TRACK_WEIGHT", N_DOFS, joint_ctrl_weight))
  {
    printf("ERROR cannot read POSTURE_TRACK_WEIGHT\n");
    return (initialized_ = false);
  }
  joint_ctrl_weight_ = Eigen::Map<Eigen::Matrix<double, N_DOFS, 1> >(&(joint_ctrl_weight[1]));
  joint_ctrl_weight_ = joint_ctrl_weight_.array().sqrt();
  joint_position_controller_->weightingMat() = joint_ctrl_weight_;
  leg_joint_position_controller_[LEFT_FOOT]->weightingMat() = joint_ctrl_weight_.block<7,1>(0,0);
  leg_joint_position_controller_[RIGHT_FOOT]->weightingMat() = joint_ctrl_weight_.block<7,1>(7,0);

  //  hierarch_inv_dyn_->sub_cost_composers_.push_back(static_cast<HierarchAffineCost*>(&(*joint_position_controller_)));
  hierarch_inv_dyn_->sub_cost_composers_.push_back(static_cast<HierarchAffineCost*>(&(*leg_joint_position_controller_[LEFT_FOOT])));
  hierarch_inv_dyn_->sub_cost_composers_.push_back(static_cast<HierarchAffineCost*>(&(*leg_joint_position_controller_[RIGHT_FOOT])));


  //initialize the swing foot controllers
  //  int swing_foot_rank[7];
  //  if(!read_parameter_pool_int_array(config_file.c_str(), "swing_track_rank", 6, swing_foot_rank))
  //  {
  //    printf("ERROR cannot read swing_track_rank\n");
  //    return (initialized_ = false);
  //  }
  //  for(int i=1; i<=N_ENDEFFS; ++i)
  //  {
  //          swing_foot_controller_[i]->initialize(swing_foot_rank[1], link2endeffmap[i]);
  //          swing_foot_controller_[i]->ranks_ = Eigen::Map<Eigen::Matrix<int, 6, 1> >(&swing_foot_rank[1]);
  //  }
  //  double swing_wght[6+1];
  //  if(!read_parameter_pool_double_array(config_file.c_str(), "SWING_TRACK_WEIGHT", 6, swing_wght))
  //  {
  //    printf("ERROR cannot read SWING_TRACK_WEIGHT\n");
  //    return (initialized_ = false);
  //  }
  //  swing_foot_weight_ = Eigen::Map<Eigen::Matrix<double, 6, 1> >(&(swing_wght[1])).asDiagonal();
  //  swing_foot_weight_ = swing_foot_weight_.array().sqrt();
  //  swing_foot_controller_[LEFT_FOOT]->weightingMat().setZero();
  //  swing_foot_controller_[RIGHT_FOOT]->weightingMat().setZero();
  //  hierarch_inv_dyn_->sub_cost_composers_.push_back(static_cast<HierarchAffineCost*>(&(*swing_foot_controller_[LEFT_FOOT])));
  //  hierarch_inv_dyn_->sub_cost_composers_.push_back(static_cast<HierarchAffineCost*>(&(*swing_foot_controller_[RIGHT_FOOT])));



  //  //foot control gains
  //  foot_gains_.setZero();
  //  if(!read_parameter_pool_int(config_file.c_str(), "use_foot_lqr_gains", &use_lqr_gains))
  //  {
  //    printf("ERROR cannot read use_foot_lqr_gains\n");
  //    return (initialized_ = false);
  //  }
  //  else
  //  {
  //    if(use_lqr_gains)
  //    {
  //      std::cout << "using foot lqr gains" << std::endl;
  //      double lqr_gains[6*12+1];
  //      if(!read_parameter_pool_double_array(config_file.c_str(), "foot_lqr_gains", 6*12, lqr_gains)) assert(false && "reading parameters failed");
  //      foot_gains_ = Eigen::Map<Eigen::Matrix<double, 6, 12> >(&(lqr_gains[1]));
  //    }
  //    else
  //    {
  //      if(!read_parameter_pool_double_array(config_file.c_str(), "foot_P_gains", 6, gains)) assert(false && "reading parameters failed");
  //      foot_gains_.block<6,6>(0,0) = Eigen::Map<Eigen::Matrix<double, 6, 1> >(&(gains[1])).asDiagonal();
  //      if(!read_parameter_pool_double_array(config_file.c_str(), "foot_D_gains", 6, gains)) assert(false && "reading parameters failed");
  //      foot_gains_.block<6,6>(0,6) = Eigen::Map<Eigen::Matrix<double, 6, 1> >(&(gains[1])).asDiagonal();
  //    }
  //  }

  //momentum control gains
  momentum_gains_.setZero();
  momentum_gains_stance_.setZero();
  momentum_gains_swing_.setZero();
  int use_lqr_gains;
  double gains[N_DOFS +1];
  if(!read_parameter_pool_int(config_file.c_str(), "use_momentum_lqr_gains", &use_lqr_gains))
  {
    printf("ERROR cannot read use_momentum_lqr_gains\n");
    return (initialized_ = false);
  }
  if(use_lqr_gains)
  {
    double mom_gains[6*9+1];
    if(!read_parameter_pool_double_array(config_file.c_str(), "momentum_lqr_gains_stance", 6*9, mom_gains))
    {
      printf("ERROR cannot read momentum_lqr_gains_stance");
      return false;
    }
    momentum_gains_stance_ = Eigen::Map<Eigen::Matrix<double, 6, 9> >(&(mom_gains[1]));
    if(!read_parameter_pool_double_array(config_file.c_str(), "momentum_lqr_gains_swing", 6*9, mom_gains))
    {
      {
        printf("ERROR cannot read momentum_lqr_gains_swing\n");
        return (initialized_ = false);
      }
    }
    momentum_gains_swing_ = Eigen::Map<Eigen::Matrix<double, 6, 9> >(&(mom_gains[1]));
  }
  else
  {
    if(!read_parameter_pool_double_array(config_file.c_str(), "momentum_P_gains_stance", 3, gains))
    {
      {
        printf("ERROR cannot read momentum_P_gains_stance\n");
        return (initialized_ = false);
      }
    }
    momentum_gains_stance_.block<3,3>(0,0) = Eigen::Map<Eigen::Matrix<double, 3, 1> >(&(gains[1])).asDiagonal();
    if(!read_parameter_pool_double_array(config_file.c_str(), "momentum_D_gains_stance", 6, gains))
    {
      {
        printf("ERROR cannot read momentum_D_gains_stance\n");
        return (initialized_ = false);
      }
    }
    momentum_gains_stance_.block<6,6>(0,3) = Eigen::Map<Eigen::Matrix<double, 6, 1> >(&(gains[1])).asDiagonal();
    if(!read_parameter_pool_double_array(config_file.c_str(), "momentum_P_gains_swing", 3, gains))
    {
      {
        printf("ERROR cannot read momentum_P_gains_swing\n");
        return (initialized_ = false);
      }
    }
    momentum_gains_swing_.block<3,3>(0,0) = Eigen::Map<Eigen::Matrix<double, 3, 1> >(&(gains[1])).asDiagonal();
    if(!read_parameter_pool_double_array(config_file.c_str(), "momentum_D_gains_swing", 6, gains))
    {
      {
        printf("ERROR cannot read momentum_D_gains_swing\n");
        return (initialized_ = false);
      }
    }
    momentum_gains_swing_.block<6,6>(0,3) = Eigen::Map<Eigen::Matrix<double, 6, 1> >(&(gains[1])).asDiagonal();
  }
  momentum_gains_ = momentum_gains_stance_;

  if(!read_parameter_pool_double_array(config_file.c_str(), "MCG_POSE_P_GAIN", N_DOFS, gains))
  {
    {
      printf("ERROR cannot read MCG_POSE_P_GAIN\n");
      return (initialized_ = false);
    }
  }
  null_posture_P_gains_ = Eigen::Map<Eigen::Matrix<double, N_DOFS, 1> >(&(gains[1]));
  joint_position_controller_->P_gains_ = null_posture_P_gains_;
  leg_joint_position_controller_[LEFT_FOOT]->P_gains_ = null_posture_P_gains_.block<7,1>(0,0);
  leg_joint_position_controller_[RIGHT_FOOT]->P_gains_ = null_posture_P_gains_.block<7,1>(7,0);

  if(!read_parameter_pool_double_array(config_file.c_str(), "MCG_POSE_D_GAIN", N_DOFS, gains))
  {
    {
      printf("ERROR cannot read MCG_POSE_D_GAIN\n");
      return (initialized_ = false);
    }
  }
  null_posture_D_gains_ = Eigen::Map<Eigen::Matrix<double, N_DOFS, 1> >(&(gains[1]));
  joint_position_controller_->D_gains_ = null_posture_D_gains_;
  leg_joint_position_controller_[LEFT_FOOT]->D_gains_ = null_posture_D_gains_.block<7,1>(0,0);
  leg_joint_position_controller_[RIGHT_FOOT]->D_gains_ = null_posture_D_gains_.block<7,1>(7,0);


  // null posture
  for(int i=0; i<N_DOFS; ++i)
    default_posture_(i,0) = joint_default_state[i+1].th;
  reference_joint_posture_ = default_posture_;
  reference_joint_vel_.setZero();
  reference_joint_acc_.setZero();

  if(!read_parameter_pool_int(config_file.c_str(),"make_video", &make_video_))
  {
    {
      printf("make_video missing - disabling video making\n");
      make_video_ = 0;
    }
  }
  opengl_counter_ = 0;
  frame_number_ = 0;

  //initialize all the variables
  //set the constraints for double support
  for(int i=1; i<=N_ENDEFFS; ++i)
    for(int j=1; j<=2*N_CART; ++j)
      endeff_constraints_[i].c[j] = 1;

  update();

  foot_center_local_[RIGHT_FOOT].setOnes();
  foot_center_local_[LEFT_FOOT].setOnes();
  foot_center_local_[RIGHT_FOOT].topRows<3>() = 0.25*(kinematics_eigen_->linkPosition(R_IN_HEEL)+kinematics_eigen_->linkPosition(R_OUT_HEEL))+
      0.25*(kinematics_eigen_->linkPosition(R_OUT_METATARSAL)+kinematics_eigen_->linkPosition(R_IN_METATARSAL));
  foot_center_local_[LEFT_FOOT].topRows<3>() = 0.25*(kinematics_eigen_->linkPosition(L_IN_HEEL)+kinematics_eigen_->linkPosition(L_OUT_HEEL))+
      0.25*(kinematics_eigen_->linkPosition(L_OUT_METATARSAL)+kinematics_eigen_->linkPosition(L_IN_METATARSAL));
  foot_center_local_[RIGHT_FOOT] = kinematics_eigen_->endeffPose(RIGHT_FOOT).inverse()*foot_center_local_[RIGHT_FOOT];
  foot_center_local_[LEFT_FOOT] = kinematics_eigen_->endeffPose(LEFT_FOOT).inverse()*foot_center_local_[LEFT_FOOT];

  transformInInvariantFrame(cog_, reference_com_pos_);
  reference_com_vel_.setZero();
  reference_com_acc_.setZero();
  reference_ang_mom_.setZero();
  reference_ang_mom_rate_.setZero();

  for(int i=1; i<=N_ENDEFFS; ++i)
  {
    transformInInvariantFrame(foot_position_wrld_[i],reference_foot_position_[i]);
    rotateInInvariantFrame(foot_orientation_wrld_[i], reference_foot_orientation_[i]);
    reference_foot_lin_velocity_[i].setZero();
    reference_foot_lin_acceleration_[i].setZero();
    reference_foot_ang_acceleration_[i].setZero();
    reference_foot_ang_velocity_[i].setZero();
    reference_contact_forces_[i].setZero();
  }
  gravity_contribution_ = - 9.81 * kinematics_eigen_->robotMass();

  reference_contact_forces_[leading_leg_](1) = 0.5*gravity_contribution_;
  reference_contact_forces_[swing_leg_](1) = 0.5*gravity_contribution_;

  //some data collection
  addVarToCollect((char *)&(reference_com_pos_wrld_[0]), "des_com_x","m",DOUBLE,TRUE);
  addVarToCollect((char *)&(reference_com_pos_wrld_[1]), "des_com_y","m",DOUBLE,TRUE);
  addVarToCollect((char *)&(reference_com_pos_wrld_[2]), "des_com_z","m",DOUBLE,TRUE);

  addVarToCollect((char *)&(reference_com_vel_wrld_[0]), "des_com_xd","m",DOUBLE,TRUE);
  addVarToCollect((char *)&(reference_com_vel_wrld_[1]), "des_com_yd","m",DOUBLE,TRUE);
  addVarToCollect((char *)&(reference_com_vel_wrld_[2]), "des_com_zd","m",DOUBLE,TRUE);

  addVarToCollect((char *)&(reference_ang_mom_[0]), "ref_ang_mom_x","-",DOUBLE,TRUE);
  addVarToCollect((char *)&(reference_ang_mom_[1]), "ref_ang_mom_y","-",DOUBLE,TRUE);
  addVarToCollect((char *)&(reference_ang_mom_[2]), "ref_ang_mom_z","-",DOUBLE,TRUE);

  addVarToCollect((char *)&(desired_momentum_rate_[0]), "des_mom_xd","m/s2",DOUBLE,TRUE);
  addVarToCollect((char *)&(desired_momentum_rate_[1]), "des_mom_yd","m/s2",DOUBLE,TRUE);
  addVarToCollect((char *)&(desired_momentum_rate_[2]), "des_mom_zd","m/s2",DOUBLE,TRUE);
  addVarToCollect((char *)&(desired_momentum_rate_[3]), "des_mom_ad","rad/s2",DOUBLE,TRUE);
  addVarToCollect((char *)&(desired_momentum_rate_[4]), "des_mom_bd","rad/s2",DOUBLE,TRUE);
  addVarToCollect((char *)&(desired_momentum_rate_[5]), "des_mom_gd","rad/s2",DOUBLE,TRUE);

  addVarToCollect((char *)&(std_cop_[LEFT_FOOT][0]), "measured_cop_l_x","m",DOUBLE,TRUE);
  addVarToCollect((char *)&(std_cop_[LEFT_FOOT][1]), "measured_cop_l_y","m",DOUBLE,TRUE);
  addVarToCollect((char *)&(std_cop_[LEFT_FOOT][2]), "measured_cop_l_z","m",DOUBLE,TRUE);
  addVarToCollect((char *)&(std_cop_[RIGHT_FOOT][0]), "measured_cop_r_x","m",DOUBLE,TRUE);
  addVarToCollect((char *)&(std_cop_[RIGHT_FOOT][1]), "measured_cop_r_y","m",DOUBLE,TRUE);
  addVarToCollect((char *)&(std_cop_[RIGHT_FOOT][2]), "measured_cop_r_z","m",DOUBLE,TRUE);

  addVarToCollect((char *)&(cop_[LEFT_FOOT][0]), "left_cop_x","m",DOUBLE,TRUE);
  addVarToCollect((char *)&(cop_[LEFT_FOOT][1]), "left_cop_y","m",DOUBLE,TRUE);
  addVarToCollect((char *)&(cop_[LEFT_FOOT][2]), "left_cop_z","m",DOUBLE,TRUE);
  addVarToCollect((char *)&(cop_[RIGHT_FOOT][0]), "right_cop_x","m",DOUBLE,TRUE);
  addVarToCollect((char *)&(cop_[RIGHT_FOOT][1]), "right_cop_y","m",DOUBLE,TRUE);
  addVarToCollect((char *)&(cop_[RIGHT_FOOT][2]), "right_cop_z","m",DOUBLE,TRUE);

  addVarToCollect((char *)&(std_cop_moment_[RIGHT_FOOT]), "adms_cop_mom_r","m",DOUBLE,TRUE);
  addVarToCollect((char *)&(std_cop_moment_[LEFT_FOOT]), "adms_cop_mom_l","m",DOUBLE,TRUE);

  addVarToCollect((char *)&(dcog_[0]), "com_xd","m/s2",DOUBLE,TRUE);
  addVarToCollect((char *)&(dcog_[1]), "com_yd","m/s2",DOUBLE,TRUE);
  addVarToCollect((char *)&(dcog_[2]), "com_zd","m/s2",DOUBLE,TRUE);
  addVarToCollect((char *)&(ddcog_[0]), "com_xdd","m/s2",DOUBLE,TRUE);
  addVarToCollect((char *)&(ddcog_[1]), "com_ydd","m/s2",DOUBLE,TRUE);
  addVarToCollect((char *)&(ddcog_[2]), "com_zdd","m/s2",DOUBLE,TRUE);

  addVarToCollect((char *)&(desired_base_acceleration_[0]), "des_base_xdd","m/s2",DOUBLE,TRUE);
  addVarToCollect((char *)&(desired_base_acceleration_[1]), "des_base_ydd","m/s2",DOUBLE,TRUE);
  addVarToCollect((char *)&(desired_base_acceleration_[2]), "des_base_zdd","m/s2",DOUBLE,TRUE);
  addVarToCollect((char *)&(desired_base_acceleration_[3]), "des_base_add","rad/s2",DOUBLE,TRUE);
  addVarToCollect((char *)&(desired_base_acceleration_[4]), "des_base_bdd","rad/s2",DOUBLE,TRUE);
  addVarToCollect((char *)&(desired_base_acceleration_[5]), "des_base_gdd","rad/s2",DOUBLE,TRUE);

  for(int i=0; i<N_DOFS; ++i)
  {
    std::string var_name = std::string(joint_names[i+1])+"_ref_th";
    addVarToCollect((char *)&(reference_joint_posture_[i]), (char *)var_name.c_str(), "m", DOUBLE, TRUE);
    var_name = std::string(joint_names[L_HFE+i])+"_ref_thd";
    addVarToCollect((char *)&(reference_joint_vel_[i]), (char *)var_name.c_str(), "m", DOUBLE, TRUE);
    var_name = std::string(joint_names[L_HFE+i])+"_ref_thdd";
    addVarToCollect((char *)&(reference_joint_acc_[i]), (char *)var_name.c_str(), "m", DOUBLE, TRUE);
  }

  std::string dim_names(" XYZABG");
  std::vector<std::string> cart;
  cart.push_back(std::string(""));
  cart.push_back(std::string("RF"));
  cart.push_back(std::string("LF"));
  std::string varname;
  for(int i=1; i<=N_ENDEFFS; ++i)
  {
    for(int j=1; j<=N_CART; ++j)
    {
      varname = cart[i] + "_ref_pos_" + dim_names[j];
      addVarToCollect((char *)&(reference_foot_position_wrld_[i][j-1]), (char *)varname.c_str(),"m",DOUBLE,TRUE);
      varname = cart[i] + "_ref_lvel_" + dim_names[j];
      addVarToCollect((char *)&(reference_foot_lin_velocity_wrld_[i][j-1]), (char *)varname.c_str(),"m/s",DOUBLE,TRUE);
      varname = cart[i] + "_ref_lacc_" + dim_names[j];
      addVarToCollect((char *)&(reference_foot_lin_acceleration_wrld_[i][j-1]), (char *)varname.c_str(),"m/s2",DOUBLE,TRUE);
      varname = cart[i] + "_ref_avel_" + dim_names[j];
      addVarToCollect((char *)&(reference_foot_ang_velocity_wrld_[i][j-1]), (char *)varname.c_str(),"rad/s",DOUBLE,TRUE);
      varname = cart[i] + "_ref_aacc_" + dim_names[j];
      addVarToCollect((char *)&(reference_foot_ang_acceleration_wrld_[i][j-1]), (char *)varname.c_str(),"rad/s2",DOUBLE,TRUE);
    }

    varname = cart[i] + "_ref_orien_W";
    addVarToCollect((char *)&(reference_foot_orientation_wrld_[i].w()), (char *)varname.c_str(),"-",DOUBLE,TRUE);
    varname = cart[i] + "_ref_orien_X";
    addVarToCollect((char *)&(reference_foot_orientation_wrld_[i].x()), (char *)varname.c_str(),"-",DOUBLE,TRUE);
    varname = cart[i] + "_ref_orien_Y";
    addVarToCollect((char *)&(reference_foot_orientation_wrld_[i].y()), (char *)varname.c_str(),"-",DOUBLE,TRUE);
    varname = cart[i] + "_ref_orien_Z";
    addVarToCollect((char *)&(reference_foot_orientation_wrld_[i].z()), (char *)varname.c_str(),"-",DOUBLE,TRUE);
  }

  for(int i=1; i<=N_ENDEFFS; ++i)
    for(int j=1; j<=2*N_CART; ++j)
    {
      varname = cart[i] + "_des_acc_" + dim_names[j];
      addVarToCollect((char *)&(desired_foot_acc_[6*(i-1)+j-1]), (char *)varname.c_str(),"-",DOUBLE,TRUE);
      varname = cart[i] + "_ref_frc_" + dim_names[j];
      addVarToCollect((char *)&(reference_contact_forces_[i][j-1]), (char *)varname.c_str(),"N",DOUBLE,TRUE);
    }


  for(int i=1; i<=N_CART; ++i)
  {
    std::string dummy_string;
    dummy_string = std::string("ang_mom_") + dim_names[i+3];
    addVarToCollect((char *)&(ang_momentum_wrld_[i-1]), (char *)dummy_string.c_str(),"-",DOUBLE,TRUE);
  }

  for(int i=1; i<=N_CART; ++i)
  {
    std::string dummy_string;
    dummy_string = std::string("zmp_") + dim_names[i];
    addVarToCollect((char *)&(zmp_[i-1]), (char *)dummy_string.c_str(),"-",DOUBLE,TRUE);
    dummy_string = std::string("des_zmp_") + dim_names[i];
    addVarToCollect((char *)&(desired_zmp_[i-1]), (char *)dummy_string.c_str(),"-",DOUBLE,TRUE);
  }


  updateDataCollectScript();
  return initialized_;
}



bool BipedController::computeDesiredComAcc()
{
  //compute the tracking error (com + lin and ang momentum)
  Eigen::Matrix<double, 9, 1> tracking_error;
  tracking_error.block<3,1>(0,0) = kinematics_eigen_->robotMass()*(reference_com_pos_ - cog_inv_);
  tracking_error.block<3,1>(3,0) = kinematics_eigen_->robotMass()*(reference_com_vel_ - dcog_inv_);
  tracking_error.block<3,1>(6,0) = reference_ang_mom_-ang_momentum_;

  //compute the des mom rate in world frame
  desired_momentum_rate_ = momentum_gains_ * tracking_error;
  desired_momentum_rate_.block<3,1>(0,0) += kinematics_eigen_->robotMass() * reference_com_acc_;
  desired_momentum_rate_.segment<3>(3) += reference_ang_mom_rate_;
  Eigen::Vector3d tmp;
  rotateInWorldFrame(desired_momentum_rate_.block<3,1>(0,0), tmp);
  desired_momentum_rate_.block<3,1>(0,0) = tmp;
  rotateInWorldFrame(desired_momentum_rate_.block<3,1>(3,0), tmp);
  desired_momentum_rate_.block<3,1>(3,0) = tmp;


  //put the ref com in world frame
  transformInWorldFrame(reference_com_pos_, reference_com_pos_wrld_);
  rotateInWorldFrame(reference_com_vel_, reference_com_vel_wrld_);
  rotateInWorldFrame(reference_com_acc_, reference_com_acc_wrld_);

  desired_zmp_ = reference_com_pos_wrld_ - reference_com_pos_wrld_(2)/9.81 * reference_com_acc_wrld_;

  return true;
}

bool BipedController::computeDesiredSwingAcc(bool swinging_leg)
{
  desired_foot_acc_.setZero();

  for(int i=1; i<=N_ENDEFFS; ++i)
  {
    //reference in world frame for data collection
    rotateInWorldFrame(reference_foot_lin_velocity_[i], reference_foot_lin_velocity_wrld_[i]);
    rotateInWorldFrame(reference_foot_lin_acceleration_[i], reference_foot_lin_acceleration_wrld_[i]);
    transformInWorldFrame(reference_foot_position_[i], reference_foot_position_wrld_[i]);
    rotateInWorldFrame(reference_foot_ang_velocity_[i], reference_foot_ang_velocity_wrld_[i]);
    rotateInWorldFrame(reference_foot_ang_acceleration_[i], reference_foot_ang_acceleration_wrld_[i]);
    rotateInWorldFrame(reference_foot_orientation_[i], reference_foot_orientation_wrld_[i]);

    if(swinging_leg && (i==swing_leg_))
    {
      Eigen::Matrix<double, 12, 1> error;
      error.setZero();

      //linear position and velocity error
      error.block<3,1>(0,0) = reference_foot_position_[i] - foot_position_[i];
      error.block<3,1>(6,0) = reference_foot_lin_velocity_[i] - foot_lin_velocity_[i];

      //compute orientation error
      Eigen::Vector3d orientation_error;
      inverse_kinematics::quatLogError(reference_foot_orientation_[i], foot_orientation_[i], orientation_error);

      error.block<3,1>(3,0) = orientation_error;
      error.block<3,1>(9,0) = reference_foot_ang_velocity_[i] - foot_ang_velocity_[i];


      desired_foot_acc_.block<6,1>(6*(i-1),0) = foot_gains_ * error;

      desired_foot_acc_.block<3,1>(6*(i-1),0) += reference_foot_lin_acceleration_[i];
      desired_foot_acc_.block<3,1>(6*(i-1)+3,0) += reference_foot_ang_acceleration_[i];

      //put the desired foot acc in world frame
      Eigen::Vector3d tmp;
      rotateInWorldFrame(desired_foot_acc_.block<3,1>(6*(i-1),0), tmp);
      desired_foot_acc_.block<3,1>(6*(i-1),0) = tmp;
      rotateInWorldFrame(desired_foot_acc_.block<3,1>(6*(i-1)+3,0), tmp);
      desired_foot_acc_.block<3,1>(6*(i-1)+3,0) = tmp;
    }
  }
  return true;
}

bool BipedController::computeDesiredBaseAcc()
{
  desired_base_acceleration_.setZero();

  Eigen::Matrix<double, 6, 1> p_error, d_error;
  p_error.setZero();
  d_error.setZero();

  Eigen::Quaterniond baseq;
  Eigen::Vector3d base_ang_vel;

  Eigen::Quaterniond base_des_q(invariant_reference_frame_.topLeftCorner<3,3>());
  Eigen::Vector3d base_des_ang_vel = Eigen::Vector3d::Zero();

  Eigen::Vector3d base_pos_error;

  GeometryUtils::SLQuaternionToEigen(base_orient.q, baseq);
  base_ang_vel = Eigen::Map<Eigen::Vector3d>(&base_orient.ad[1]);
  inverse_kinematics::quatLogError(base_des_q, baseq, base_pos_error);

  p_error.tail<3>() = base_pos_error;
  d_error.tail<3>() = - base_ang_vel;

  desired_base_acceleration_ = base_P_gains_.asDiagonal() * p_error + base_D_gains_.asDiagonal() * d_error;

  return true;
}

void BipedController::computeInvariantReferenceFrame()
{
  //we compute a frame that is located step_width from the leading leg in the middle of the robot
  //x is step width away
  //y is pointing forward
  //z is pointing up

  //get the pose of the leading leg
  Eigen::Matrix4d foot_pose = kinematics_eigen_->endeffPose(leading_leg_);

  invariant_reference_frame_ = Eigen::Matrix4d::Identity();

  double sign = (leading_leg_ == RIGHT_FOOT) ? 1.0 : -1.0;

  //the translation part
  invariant_reference_frame_.topRightCorner<3,1>() =
      foot_pose.topLeftCorner<3,3>()*Eigen::Vector3d(0,0.09,0) + foot_pose.topRightCorner<3,1>();

  //y is pointing forward
  invariant_reference_frame_.block<3,1>(0,1) = sign*foot_pose.block<3,1>(0,2);

  invariant_reference_frame_(2,1) = 0.0; //the z component needs to be 0
  invariant_reference_frame_.block<3,1>(0,1).normalize();
  //z is up
  invariant_reference_frame_.block<3,1>(0,2) = Eigen::Vector3d(0,0,1);
  //x = y cross z
  invariant_reference_frame_.block<3,1>(0,0) = invariant_reference_frame_.block<3,1>(0,1).cross(invariant_reference_frame_.block<3,1>(0,2));

}


void BipedController::transformInInvariantFrame(const Eigen::Vector3d& wrld, Eigen::Vector3d& invariant)
{
  invariant = invariant_reference_frame_.topLeftCorner<3,3>().transpose() * (wrld - invariant_reference_frame_.topRightCorner<3,1>());
}

void BipedController::transformInWorldFrame(const Eigen::Vector3d& invariant, Eigen::Vector3d& wrld)
{
  wrld = invariant_reference_frame_.topLeftCorner<3,3>() * invariant + invariant_reference_frame_.topRightCorner<3,1>();
}

void BipedController::rotateInInvariantFrame(const Eigen::Vector3d& wrld, Eigen::Vector3d& invariant)
{
  invariant = invariant_reference_frame_.topLeftCorner<3,3>().transpose() * wrld;
}

void BipedController::rotateInWorldFrame(const Eigen::Vector3d& invariant, Eigen::Vector3d& wrld)
{
  wrld = invariant_reference_frame_.topLeftCorner<3,3>() * invariant;
}

void BipedController::rotateInInvariantFrame(const Eigen::Quaterniond& wrld, Eigen::Quaterniond& invariant)
{
  invariant = Eigen::Quaterniond( invariant_reference_frame_.topLeftCorner<3,3>().transpose() * wrld.toRotationMatrix());
}

void BipedController::rotateInWorldFrame(const Eigen::Quaterniond& invariant, Eigen::Quaterniond& wrld)
{
  wrld = Eigen::Quaterniond(invariant_reference_frame_.topLeftCorner<3,3>() * invariant.toRotationMatrix());
}

void BipedController::update()
{
  //update various kinematic and dynamic quantities
  kinematics_eigen_->update(joint_state, base_state, base_orient, endeff_constraints_);
  int num_constraints;
  floating_base_kin_->computeJacobians(joint_state, base_state, base_orient, endeff_constraints_, num_constraints);
  momentum_helper_->update(*kinematics_eigen_);
  foot_contact_handler_->update();

  hierarch_inv_dyn_->update();

  //cog and capture points
  cog_ = kinematics_eigen_->cog();
  dcog_ = momentum_helper_->getdCog();
  ddcog_ = momentum_helper_->getddCog();
  ang_momentum_wrld_ = momentum_helper_->getMomentum().bottomRows<3>();



  //generalized forces from misc sensors
  generalized_force_[RIGHT_FOOT] = foot_contact_handler_->getMeasuredForceAtEndeff(RIGHT_FOOT);//Eigen::Matrix<double, 6, 1>(&(misc_sensor[R_CFx]));
  generalized_force_[LEFT_FOOT] = foot_contact_handler_->getMeasuredForceAtEndeff(LEFT_FOOT);//Eigen::Matrix<double, 6, 1>(&(misc_sensor[L_CFx]));

  for(int i=1; i<=N_ENDEFFS; ++i)
  {
    //get endeff information
    kinematics_eigen_->getLinkPose(link2endeffmap[i], endeff_frm_[i]);
    foot_position_wrld_[i] = Eigen::Map<Eigen::Vector3d>(&(cart_state[i].x[_X_]));
    foot_lin_velocity_wrld_[i] = Eigen::Map<Eigen::Vector3d>(&(cart_state[i].xd[_X_]));
    foot_orientation_wrld_[i] = Eigen::Quaterniond(cart_orient[i].q[_QW_],
                                                   cart_orient[i].q[_QX_],
                                                   cart_orient[i].q[_QY_],
                                                   cart_orient[i].q[_QZ_]);
    foot_ang_velocity_wrld_[i] = Eigen::Map<Eigen::Vector3d>(&(cart_orient[i].ad[_X_]));

    //put in world frame (assumes that the misc sensor is given in endEff frame
    generalized_force_[i].block<3,1>(0,0) = kinematics_eigen_->endeffPose(i).topLeftCorner<3,3>() * generalized_force_[i].block<3,1>(0,0);
    generalized_force_[i].block<3,1>(3,0) = kinematics_eigen_->endeffPose(i).topLeftCorner<3,3>() * generalized_force_[i].block<3,1>(3,0);

    //compute COP
    foot_contact_handler_->computeCOPTorque(i, generalized_force_[i], std_cop_moment_[i]);
    foot_contact_handler_->computeEndeffCoP(i, generalized_force_[i], std_cop_[i]);

    //cop in wrld frame
    endeff_frm_[i].block<3,3>(0,0) *kinematics_eigen_->standardizedToRealEndeffTransform(i).block<3,3>(0,0) * std_cop_[i] +
        endeff_frm_[i].block<3,1>(0,3);
  }
  computeInvariantReferenceFrame();

  zmp_ = cog_ - cog_(2)/9.81 * ddcog_;

  //put the feet in invariant frame
  for(int i=1; i<=N_ENDEFFS; ++i)
  {
    transformInInvariantFrame(foot_position_wrld_[i], foot_position_[i]);
    rotateInInvariantFrame(foot_lin_velocity_wrld_[i], foot_lin_velocity_[i]);
    rotateInInvariantFrame(foot_ang_velocity_wrld_[i], foot_ang_velocity_[i]);
    rotateInInvariantFrame(foot_orientation_wrld_[i], foot_orientation_[i]);
  }

  //put cog in invariant frame
  transformInInvariantFrame(cog_, cog_inv_);
  rotateInInvariantFrame(dcog_, dcog_inv_);
  rotateInInvariantFrame(ang_momentum_wrld_, ang_momentum_);
}

void BipedController::computeJointOpSpInertia(Eigen::Matrix<double, N_DOFS, N_DOFS>& inertia)
{
  const Eigen::Matrix<double, N_DOFS+6, N_DOFS+6>& M = hierarch_inv_dyn_->inertiaMatrix();

  inertia =
      M.topLeftCorner<N_DOFS,N_DOFS>() -
      M.topRightCorner<N_DOFS,6>() * M.bottomRightCorner<6,6>().inverse() * M.bottomLeftCorner<6,N_DOFS>();
}

void BipedController::computeLegOpSpInertia(int leg, Eigen::Matrix<double, 7, 7>& inertia)
{
  //compute the variable gain defined as (S M^-1 S^T)
  const Eigen::Matrix<double, N_DOFS + 6, N_DOFS + 6>& M =  hierarch_inv_dyn_->inertiaMatrix();

  Eigen::Matrix<double, 6, 6> D2_tilde;
  Eigen::Matrix<double, 7, 6> Btilde;
  Eigen::Matrix<double, 6, 6> D3;
  Eigen::Matrix<double, 7, 7> A;

  int index_swing, index_stance;
  int index_torso = B_TR-1;
  if(leg == LEFT_FOOT)
  {
    index_swing = L_HFE-1;
    index_stance = R_HFE-1;
  }
  else
  {
    index_swing = R_HFE-1;
    index_stance = L_HFE-1;
  }

  A = M.block<7,7>(index_swing,index_swing);
  D3 = M.bottomRightCorner<6,6>();
  Btilde = M.block<7, 6>(index_swing,N_DOFS);

  D2_tilde =
      M.block<6,7>(N_DOFS, index_stance) * M.block<7,7>(index_stance,index_stance).inverse() * M.block<7,6>(index_stance, N_DOFS) +
      M.block<6,3>(N_DOFS, index_torso) * M.block<3,3>(index_torso, index_torso).inverse() * M.block<3,6>(index_torso, N_DOFS);

  inertia = A - Btilde * (D3 - D2_tilde).inverse() * Btilde.transpose();
}

void BipedController::sendOpenGL()
{
  BallsInfo data;
  data.numBalls = 4;

  Eigen::Vector3d des_capt_wrld;
  //  transformInWorldFrame(reference_capture_point_, des_capt_wrld);

  for(int i=0; i<3; ++i)
  {
    //    data.balls[0][i] = inst_capture_point_(i);
    //    data.balls[1][i] = des_capt_wrld(i);
    data.balls[2][i] = cop_wrld_[LEFT_FOOT](i);
    data.balls[3][i] = cop_wrld_[RIGHT_FOOT](i);
  }

  //color in rgb + alpha

  //blue
  data.balls[0][3] = 0.0;
  data.balls[0][4] = 0.0;
  data.balls[0][5] = 1.0;
  data.balls[0][6] = 1.0;

  //green
  data.balls[1][3] = 0.0;
  data.balls[1][4] = 1.0;
  data.balls[1][5] = 0.0;
  data.balls[1][6] = 1.0;

  //orange
  data.balls[2][3] = 1.0;
  data.balls[2][4] = 0.5;
  data.balls[2][5] = 0.0;
  data.balls[2][6] = 1.0;

  //pink
  data.balls[3][3] = 1.0;
  data.balls[3][4] = 0.0;
  data.balls[3][5] = 1.0;
  data.balls[3][6] = 1.0;


  sendUserGraphics("draw_ball", &data, sizeof(BallsInfo));


  //draw the invariant frame
  AxesInfo axes_info;
  axes_info.length = 0.1;
  sprintf(axes_info.name, "INV_FRAME");
  axes_info.width = 0.01;
  for(int i=0; i<4; ++i)
    for(int j=0; j<4; ++j)
      axes_info.transform[4*i+1+j] = invariant_reference_frame_(i,j);
  sendUserGraphics("draw_axes", &axes_info, sizeof(AxesInfo));

  if(make_video_)
  {
    if(opengl_counter_>double(task_servo_rate)/25.)
    {
      char filename[300];
      sprintf(filename,"test_%05d",frame_number_);
      sendUserGraphics("capture_frame",filename,sizeof(filename));
      frame_number_++;
      opengl_counter_ = 0;
    }
    else
      opengl_counter_++;
  }
}
