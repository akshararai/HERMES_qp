/*
 * walking_controller.cpp
 *
 *  Created on: Apr 28, 2013
 *      Author: righetti
 */

#include <ctime>
#ifdef __XENO__
#include <native/timer.h>
#endif

#include <string>
#include <algorithm>

#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/lexical_cast.hpp>

#include "RtMinJerkGenerator.h"
//#include "RecordedTrajectory.h"
#include <walking_controller.h>
#include "drawUtils.h"
#include <SL_collect_data.h>
#include <SL_common.h>
#include <SL_dynamics.h>
#include <quaternions.h>

using namespace floating_base_utilities;
using namespace momentum_balance_control;
const int WalkingController::joint_trajs_n_states_;

WalkingController::WalkingController()
{
  cycle_duration_ = 0.0;
}

WalkingController::~WalkingController()
{
}

void WalkingController::increment_walking_transition_counter(int &counter) const
{
  counter = (++counter - 1)%(joint_trajs_n_states_-1)+1; // 0, 1, 2, 3, 4, 5, 6, 1, 2, ...
}

bool WalkingController::computeKinematicStep(const Eigen::Matrix<double, 3, 1>& x_des,
                                             const Eigen::Matrix<double, 3, 1>& dx_des,
                                             const Eigen::Matrix<double, 3, 1>& x,
                                             Eigen::Matrix<double, N_DOFS, 1>& q_pos,
                                             Eigen::Matrix<double, N_DOFS, 1>& q_vel,
                                             Eigen::Matrix<double, N_DOFS, 1>& q_acc)
{
  Eigen::Matrix<double, N_DOFS+6, 1> weights;
  weights.setConstant(1);
  weights.block<7,1>(swing_hipFE_idx_,0).setConstant(1000);

  //compute the ref velocity
  Eigen::Matrix<double, 3, 1> P_gains;
  P_gains << 0.1, 0.1, 0.1;

  Eigen::Matrix<double, 3, 1> ref_vel = dx_des + P_gains.asDiagonal() * (x_des - x);

  Eigen::Matrix<double, 6, N_DOFS+6> J_stance = des_pose_floating_base_kin_->getConstraintJacobian().topRows<6>();
  Eigen::Matrix<double, N_DOFS+6, N_DOFS+6> N_stance =
      Eigen::Matrix<double, N_DOFS+6, N_DOFS+6>::Identity() -
      J_stance.transpose()*(J_stance * J_stance.transpose()).inverse() * J_stance;

  Eigen::Matrix<double, 3, N_DOFS+6> J_swing = des_pose_floating_base_kin_->getUnconstrainedJacobian().topRows<3>() * N_stance * weights.asDiagonal();
  Eigen::Matrix<double, N_DOFS+6, 3> J_swing_pseudo = J_swing.transpose() * (J_swing * J_swing.transpose()).inverse();
  Eigen::Matrix<double, N_DOFS, 1> q_vel_old = q_vel;
  q_vel =  (weights.asDiagonal()*(J_swing_pseudo * ref_vel + (N_stance - J_swing_pseudo * J_swing).leftCols<N_DOFS>() * (default_posture_ - q_pos))).topRows<N_DOFS>();

  q_acc = (q_vel - q_vel_old)*double(task_servo_rate);
  q_pos += q_vel/double(task_servo_rate);

  for(int i=0; i<N_DOFS; ++i)
  {
    if(q_pos(i) > joint_range[i+1][MAX_THETA])
    {
      q_pos(i) = joint_range[i+1][MAX_THETA];
      q_vel(i) = 0.0;
      q_acc(i) = 0.0;
    }
    else if(q_pos(i) < joint_range[i+1][MIN_THETA])
    {
      q_pos(i) = joint_range[i+1][MIN_THETA];
      q_vel(i) = 0.0;
      q_acc(i) = 0.0;
    }
  }

  return true;
}

bool WalkingController::initialize()
{
  //read a parameter file without joint friction for stability
  if(!real_robot_flag)
  {
    printf("loading different LinkParameters\n");
    read_link_parameters("LinkParameters.cf");
  }

  sprintf(config_file_,"%s","WalkingController.cf");

  //initialize the parent class
  if(!BipedController::initialize(std::string(config_file_)))
  {
    std::cout << "BipedController could not be initialized" << std::endl;
    return (initialized_ = false);
  }

  //control gains
  double gains[N_DOFS +1];
  int use_lqr_gains;

  //swing leg weight
  double swing_leg_weight[7+1];
  if(!read_parameter_pool_double_array(config_file_, "LEG_POS_CTRL_WEIGHT", 7, swing_leg_weight))
  {
    printf("ERROR cannot read LEG_POS_CTRL_WEIGHT\n");
    return (initialized_ = false);
  }
  swing_leg_joint_ctrl_weight_ = Eigen::Map<Eigen::Matrix<double, 7, 1> >(&(swing_leg_weight[1]));
  swing_leg_joint_ctrl_weight_ = swing_leg_joint_ctrl_weight_.array().sqrt();

  //swing leg control gains
  //  double sw_leg_gains[7+1];
  //  if(!read_parameter_pool_double_array(config_file_, "swing_leg_P_gains", 7, sw_leg_gains))
  //  {
  //    printf("ERROR cannot read swing leg P gains\n");
  //    return (initialized_ = false);
  //  }
  //  swing_leg_joint_controller_->P_gains_ = Eigen::Map<Eigen::Matrix<double, 7, 1> >(&(sw_leg_gains[1]));
  //  if(!read_parameter_pool_double_array(config_file_, "swing_leg_D_gains", 7, sw_leg_gains))
  //  {
  //    printf("ERROR cannot read swing leg D gains\n");
  //    return (initialized_ = false);
  //  }
  //  swing_leg_joint_controller_->D_gains_ = Eigen::Map<Eigen::Matrix<double, 7, 1> >(&(sw_leg_gains[1]));

  current_num_steps_ = 0;

  //stepping time
  if(!read_parameter_pool_int(config_file_, "number_of_steps", &number_of_steps_))
  {
    printf("ERROR cannot read number_of_steps\n");
    return (initialized_ = false);
  }
  if(!read_parameter_pool_double(config_file_,"stepping_time",&stepping_time_))
  {
    printf("ERROR cannot read stepping_time\n");
    return (initialized_ = false);
  }
  if(!read_parameter_pool_double(config_file_,"init_time",&init_time_))
  {
    printf("ERROR cannot read init_time\n");
    return (initialized_ = false);
  }
  if(!read_parameter_pool_double(config_file_,"stance_duration",&stance_duration_))
  {
    printf("ERROR cannot read stance_duration\n");
    return (initialized_ = false);
  }
  if(!read_parameter_pool_double(config_file_,"step_length", &step_length_))
  {
    printf("ERROR cannot read step_length\n");
    return (initialized_ = false);
  }
  if(!read_parameter_pool_double(config_file_,"step_height", &step_height_))
  {
    printf("ERROR cannot read step_height\n");
    return (initialized_ = false);
  }
  if(!read_parameter_pool_double(config_file_,"step_width", &step_width_))
  {
    printf("ERROR cannot read step_width\n");
    return (initialized_ = false);
  }
  if(!read_parameter_pool_double(config_file_,"force_contact_switch", &force_contact_switch_))
  {
    printf("ERROR cannot read force_contact_switch\n");
    return (initialized_ = false);
  }

  //unloading related params
  if(!read_parameter_pool_double(config_file_, "unloading_leg_weight", &unloading_leg_weight_))
  {
    printf("ERROR cannot read unloading_leg_weight\n");
    return (initialized_ = false);
  }
  if(!read_parameter_pool_double(config_file_, "unloading_leg_time", &unloading_leg_time_))
  {
    printf("ERROR cannot read unloading_leg_time\n");
    return (initialized_ = false);
  }
  if(!read_parameter_pool_double(config_file_, "loading_leg_time", &loading_leg_time_))
  {
    printf("ERROR cannot read loading_leg_time\n");
    return (initialized_ = false);
  }


  double tmp_bias[3+1];
  if(!read_parameter_pool_double_array(config_file_, "com_centered_bias", 3, tmp_bias))
  {
    printf("ERROR cannot read com_centered_bias\n");
    return (initialized_ = false);
  }
  com_centered_bias_ = Eigen::Map<Eigen::Matrix<double, 3, 1> >(&(tmp_bias[1]));

  double tmp_posture[N_DOFS+1];
  if(!read_parameter_pool_double_array(config_file_, "posture_1", N_DOFS, tmp_posture))
  {
    printf("ERROR cannot read posture_1\n");
    return (initialized_ = false);
  }
  posture_1_ = Eigen::Map<Eigen::Matrix<double, N_DOFS, 1> >(&(tmp_posture[1]));

  if(!read_parameter_pool_double_array(config_file_, "posture_2", N_DOFS, tmp_posture))
  {
    printf("ERROR cannot read posture_2\n");
    return (initialized_ = false);
  }
  posture_2_ = Eigen::Map<Eigen::Matrix<double, N_DOFS, 1> >(&(tmp_posture[1]));




  //  if(!read_parameter_pool_double(config_file_, "z_com_D_gains", &z_com_D_gains_))
  //    assert(false && "reading parameters z_com_D_gains failed");

  joint_trajs_it_ = 0;
//  joint_trajs_.resize(joint_trajs_n_states_);
  Eigen::Matrix<double, joint_trajs_n_states_, 1> durs = stance_duration_*
      Eigen::Matrix<double, joint_trajs_n_states_, 1>::Ones();
  durs(2) = stepping_time_/2.0;
  durs(5) = stepping_time_/2.0;
  for(int i=0; i<joint_trajs_n_states_; ++i)
  {
    std::string file_name = std::string("data/stepping_posture_")+boost::lexical_cast<std::string>(i);
//    if(!joint_trajs_[i].initialize(file_name, durs(i)))
//    {
//      std::cout << "could not initialize posture trajectories" << std::endl;
//      return false;
//    }
  }

  // null posture
  postures_it_ = WCPostureIterator();
  postures_it_.postures_.resize(5, Eigen::Matrix<double, N_DOFS, 1>::Zero());
  postures_it_.postures_[2]=default_posture_;
  postures_it_.postures_[3]=posture_1_;
  postures_it_.postures_[4]=posture_2_;
  if(!read_parameter_pool_double_array(config_file_, "posture_3", N_DOFS, tmp_posture))
  {
    printf("ERROR cannot read posture_3\n");
    return (initialized_ = false);
  }
  postures_it_.postures_[1]=Eigen::Map<Eigen::Matrix<double, N_DOFS, 1> >(&(tmp_posture[1]));
  if(!read_parameter_pool_double_array(config_file_, "posture_4", N_DOFS, tmp_posture))
  {
    printf("ERROR cannot read posture_4\n");
    return (initialized_ = false);
  }
  postures_it_.postures_[0]=Eigen::Map<Eigen::Matrix<double, N_DOFS, 1> >(&(tmp_posture[1]));;
  std::cout << "so far so good";


  //compute desired trajectory for CoM
  Eigen::Vector3d leading_foot_center_wrld = kinematics_eigen_->endeffPose(leading_leg_).topRows<3>()*foot_center_local_[leading_leg_];
  Eigen::Vector3d leading_foot_center;
  transformInInvariantFrame(leading_foot_center_wrld, leading_foot_center);
  Eigen::Vector3d swing_foot_center_wrld = kinematics_eigen_->endeffPose(swing_leg_).topRows<3>()*foot_center_local_[swing_leg_];
  Eigen::Vector3d swing_foot_center;
  transformInInvariantFrame(swing_foot_center_wrld, swing_foot_center);
  double dummy_pos;
  Eigen::Matrix2d pos,vel,acc;
  vel << 0.0, 0.0, init_time_, 0.0;
  acc = vel;
  Eigen::Vector3d bias = com_centered_bias_;
  bias(0) = 0.0; //no bias in x direction

  for(int i=0; i<3; ++i)
  {
    pos << 0.0, reference_com_pos_(i), init_time_, (leading_foot_center(i) + swing_foot_center(i))/2.0 + bias(i);
    com_trajectory_[i].initialize(pos,vel,acc);
  }

  reference_unloading_forces_.setZero();
  reference_unloading_forces_(1) = gravity_contribution_/2.0;


  des_pose_kinematics_.reset(new KinematicsEigen());
  des_pose_floating_base_kin_.reset(new FloatingBaseKinematics());
  des_pose_kinematics_->initialize(joint_state, base_state, base_orient,endeff);

  timer_ = 0.0;
  current_state_ = INIT;

  //some data collection
  addVarToCollect((char *)&(cycle_duration_), "cycle_dur","s",DOUBLE,TRUE);

  addVarToCollect((char *)&(current_state_),"walking_state","-",INT,TRUE);


  for(int i=0; i<dummy_variables_.rows(); ++i)
  {
    char dummyname[20];
    sprintf(dummyname, "dummy_%d",i);
    addVarToCollect((char *)&(dummy_variables_[i]), dummyname,"-",DOUBLE,TRUE);
  }

  updateDataCollectScript();

  return initialized_;
}



bool WalkingController::run()
{
  if(!initialized_)
  {
    printf("ERROR controller not initialized\n");
    return false;
  }
#ifdef __XENO__
  SRTIME run_start = rt_timer_ticks2ns(rt_timer_read());
#else
  boost::posix_time::ptime run_start(boost::posix_time::microsec_clock::local_time());
#endif


  WalkingState next_state = current_state_;

  update();
  swing_hipFE_idx_ = (swing_leg_ == LEFT_FOOT) ? (L_HFE-1) : (R_HFE-1);
  stance_hipFE_idx_ = (swing_leg_ == LEFT_FOOT) ? (R_HFE-1) : (L_HFE-1);

  //STATE MACHINE
  switch(current_state_)
  {
    //move the com in the center
    case INIT:
    {
      for(int i=0; i<3; ++i)
        com_trajectory_[i].query(timer_, reference_com_pos_(i), reference_com_vel_(i), reference_com_acc_(i));

      computeDesiredComAcc();
      computeDesiredSwingAcc();
      computeDesiredBaseAcc();

      reference_contact_forces_[leading_leg_](1) = 0.5*gravity_contribution_;
      reference_contact_forces_[swing_leg_](1) = 0.5*gravity_contribution_;

      if(timer_ > init_time_)
      {
        //compute desired trajectory for CoM
        Eigen::Vector3d leading_foot_center_wrld = kinematics_eigen_->endeffPose(leading_leg_).topRows<3>()*foot_center_local_[leading_leg_];
        Eigen::Vector3d leading_foot_center;
        transformInInvariantFrame(leading_foot_center_wrld, leading_foot_center);
        double dummy_pos;
        Eigen::Matrix2d pos,vel,acc;
        vel << 0.0, 0.0, stance_duration_, 0.0;
        acc = vel;
        Eigen::Vector3d bias = com_centered_bias_;
        bias(0) *= (leading_leg_ == RIGHT_FOOT) ? 1.0 : -1.0;
        for(int i=0; i<3; ++i)
        {
          pos << 0.0, reference_com_pos_(i), stance_duration_, leading_foot_center(i) + bias(i);
          com_trajectory_[i].initialize(pos,vel,acc);
        }
        next_state = DOUBLE_SUPPORT;
        timer_ = 0.0;
        printf("DOUBLE SUPPORT\n");
      }

      break;
    }

    //move the capture point to the leading leg using the com ctrl law
    case DOUBLE_SUPPORT:
    {
      for(int i=0; i<3; ++i)
        com_trajectory_[i].query(timer_, reference_com_pos_(i), reference_com_vel_(i), reference_com_acc_(i));

      computeDesiredComAcc();
      computeDesiredSwingAcc();
      computeDesiredBaseAcc();

      if(current_num_steps_ < number_of_steps_)
      {

        //desired contact forces -> weight balancing
        reference_contact_forces_[leading_leg_](1) = (reference_unloading_forces_(1) - gravity_contribution_) *(1.0-timer_/stance_duration_) + gravity_contribution_;
        reference_contact_forces_[swing_leg_](1) = gravity_contribution_ - reference_contact_forces_[leading_leg_](1);

        //        reference_joint_posture_ = timer_/stance_duration_ * postures_it_.posture(1) + (1.0-timer_/stance_duration_)*postures_it_.posture();
        //        joint_trajs_[joint_trajs_it_].update(timer_);
        //        reference_joint_posture_.head<14>() = joint_trajs_[joint_trajs_it_].trajectory_current_;
        reference_joint_posture_ = default_posture_;
        reference_joint_vel_.setZero();
        reference_joint_acc_.setZero();

        //we can lift the swing leg - unloading mode

        if(generalized_force_[swing_leg_](2) < force_contact_switch_)
        {
          current_num_steps_++;
          next_state = UNLOAD_LEG;
          reference_unloading_forces_ = reference_contact_forces_[swing_leg_];
          //the capture point trajectory
          Eigen::Matrix2d pos,vel,acc;
          for(int i=0; i<3; ++i)
          {
            pos(0,0) = vel(0,0) = acc(0,0) = 0.0;
            pos(1,0) = vel(1,0) = acc(1,0) = unloading_leg_time_;
            com_trajectory_[i].query(stance_duration_, pos(1,1), vel(1,1), acc(1,1));
            com_trajectory_[i].query(timer_, pos(0,1), vel(0,1), acc(0,1));
            com_trajectory_[i].initialize(pos, vel, acc);
          }
          //initialize desired trajectory for swing leg (move from default to current)
          legs_initial_position_ = kinematics_eigen_->generalizedJointPositions().topRows<N_DOFS>();
          legs_initial_position_(swing_hipFE_idx_) = 0.05; //HFE
          legs_initial_position_(stance_hipFE_idx_) = 0.05;
          legs_initial_position_(swing_hipFE_idx_+1) = -0.02; //HAA
          legs_initial_position_(stance_hipFE_idx_+1) = 0.02;
          legs_initial_position_(swing_hipFE_idx_+2) = -0.2; //HFR
          legs_initial_position_(stance_hipFE_idx_+2) = 0.2;
          legs_initial_position_(swing_hipFE_idx_+3) = 0.3; //KFE
          legs_initial_position_(stance_hipFE_idx_+3) = 0.3;
          legs_initial_position_(swing_hipFE_idx_+4) = 0.2; //AR
          legs_initial_position_(stance_hipFE_idx_+4) = -0.2;
          legs_initial_position_(swing_hipFE_idx_+5) = 0.25; //AFE
          legs_initial_position_(stance_hipFE_idx_+5) = 0.25;
          legs_initial_position_(swing_hipFE_idx_+6) = 0.2; //AAA
          legs_initial_position_(stance_hipFE_idx_+6) = -0.2;
          for(int i=0; i<N_DOFS; ++i)
          {
            pos.setZero();
            vel.setZero();
            acc.setZero();
            //time
            pos(1,0) = vel(1,0) = acc(1,0) = unloading_leg_time_;
            pos(0,1) = default_posture_(i,0);
            pos(1,1) = legs_initial_position_(i,0);
            legs_joint_trajectory_[i].initialize(pos,vel,acc);
          }

          timer_ = 0.0;
          printf("UNLOAD_LEG\n");
        }
      }
      break;
    }
    //unload the leg while still moving the capture point above the foot
    case UNLOAD_LEG:
    {
      for(int i=0; i<3; ++i)
        com_trajectory_[i].query(timer_, reference_com_pos_(i), reference_com_vel_(i), reference_com_acc_(i));

      for(int i=0; i<N_DOFS; ++i)
        legs_joint_trajectory_[i].query(timer_, reference_joint_posture_(i), reference_joint_vel_(i), reference_joint_acc_(i));


      momentum_gains_ = (1.0 - timer_/unloading_leg_time_)*momentum_gains_stance_ + timer_/unloading_leg_time_*momentum_gains_swing_;

      //      reference_joint_posture_ += 0.1 * (postures_it_.posture(1) - reference_joint_posture_);
      //      const double tscaling= 0.01;
      //      joint_trajs_[joint_trajs_it_].update(joint_trajs_[joint_trajs_it_].duration_*tscaling + (1.0 - tscaling)*joint_trajs_[joint_trajs_it_].time_);
      //      reference_joint_posture_.head<14>() = joint_trajs_[joint_trajs_it_].trajectory_current_;

      computeDesiredComAcc();
      computeDesiredSwingAcc();
      computeDesiredBaseAcc();

      //add the unload constraint
      double desired_force_weight[2+1];
      desired_force_weight[swing_leg_] = (timer_/unloading_leg_time_)*(unloading_leg_weight_-1.0) + 1.0;
      desired_force_weight[leading_leg_] = 1.0;

      biped_task_comp_->frc_reg_L_.weightingMat() = desired_force_weight[LEFT_FOOT] * biped_task_comp_->sqrt_frc_reg_weight_;
      biped_task_comp_->frc_reg_R_.weightingMat() = desired_force_weight[RIGHT_FOOT] * biped_task_comp_->sqrt_frc_reg_weight_;

      //desired contact forces -> weight balancing
      reference_contact_forces_[swing_leg_] = (1.0 - timer_/unloading_leg_time_) *reference_unloading_forces_;
      reference_contact_forces_[leading_leg_](1) = gravity_contribution_ - reference_contact_forces_[swing_leg_](1);

      //swing foot controller is turning on
      //swing_foot_controller_[swing_leg_]->weightingMat() = swing_foot_weight_*timer_/unloading_leg_time_;

      joint_position_controller_->weightingMat().block<7,1>(swing_hipFE_idx_,0) =
          joint_ctrl_weight_.block<7,1>(swing_hipFE_idx_,0)*(1.0-timer_/unloading_leg_time_) + swing_leg_joint_ctrl_weight_*timer_/unloading_leg_time_;

      leg_joint_position_controller_[swing_leg_]->weightingMat() =
          joint_ctrl_weight_.block<7,1>(swing_hipFE_idx_,0)*(1.0-timer_/unloading_leg_time_) + swing_leg_joint_ctrl_weight_*timer_/unloading_leg_time_;


      //      Eigen::Matrix<double, 6, 6> sqrt_stat_swing = (1.0 - timer_/unloading_leg_time_) *
      //          biped_task_comp_->sqrt_stat_feet_weight_.block<6,6>((leading_leg_-1)*6,(leading_leg_-1)*6);
      biped_task_comp_->sqrt_stat_feet_weight_.block<6,6>((swing_leg_-1)*6, (swing_leg_-1)*6) *= (1.0 - timer_/unloading_leg_time_);

      //prepare for SWING1
      if(timer_ >= unloading_leg_time_)
      {
        increment_walking_transition_counter(joint_trajs_it_);
        ++postures_it_;
        reference_foot_orientation_[swing_leg_] = foot_orientation_[swing_leg_];//Eigen::Quaterniond(rot);

        //set the constraints for single support
        for(int j=1; j<=2*N_CART; ++j)
          endeff_constraints_[swing_leg_].c[j] = 0;
        timer_ = 0.0;

        //compute desired trajectory for swing foot
        double sig = (swing_leg_ == RIGHT_FOOT) ? 1.0 : -1.0;
        Eigen::Vector3d sw_motion;
        sw_motion << -foot_position_[swing_leg_](0) + sig * step_width_ ,  -foot_position_[swing_leg_](1) + step_length_/2.0, step_height_;
        Eigen::Matrix2d vel,acc,pos;
        vel << 0.0, 0.0, stepping_time_/2.0, 0.0;
        acc = vel;
        for(int i=0; i<3; ++i)
        {
          pos << 0.0, foot_position_[swing_leg_](i), stepping_time_/2.0, foot_position_[swing_leg_](i) + sw_motion(i);
          swing_foot_trajectory_[i].initialize(pos,vel,acc);
        }

        //the orientation of the swing foot
        Eigen::Matrix3d dummy_ref_rot;
        if(swing_leg_ == RIGHT_FOOT)
        {
          dummy_ref_rot <<
              0, -1, 0,
              0, 0, 1,
              -1, 0, 0;

        }
        else
        {
          dummy_ref_rot <<
              0, 1, 0,
              0, 0, -1,
              -1, 0, 0;

        }
        Eigen::Quaterniond dummy_ref(dummy_ref_rot);
        inverse_kinematics::fixQuaternionSign(foot_orientation_[swing_leg_], dummy_ref);
        pos << 0.0, foot_orientation_[swing_leg_].w(), stepping_time_/2.0, dummy_ref.w();
        swing_foot_traj_orientation_[0].initialize(pos,vel,acc);
        pos << 0.0, foot_orientation_[swing_leg_].x(), stepping_time_/2.0, dummy_ref.x();
        swing_foot_traj_orientation_[1].initialize(pos,vel,acc);
        pos << 0.0, foot_orientation_[swing_leg_].y(), stepping_time_/2.0, dummy_ref.y();
        swing_foot_traj_orientation_[2].initialize(pos,vel,acc);
        pos << 0.0, foot_orientation_[swing_leg_].z(), stepping_time_/2.0, dummy_ref.z();
        swing_foot_traj_orientation_[3].initialize(pos,vel,acc);

        Eigen::Matrix<double, N_DOFS, 1> des_leg_pos = legs_initial_position_;
        des_leg_pos(swing_hipFE_idx_,0) += 1.2;//hip FE
        des_leg_pos(swing_hipFE_idx_+3,0) += 1.2;//knee
        //        des_leg_pos(swing_hipFE_idx_+5,0) += 0.2;//AFE
        //initialize desired trajectory for swing leg (move from default to current)
        for(int i=0; i<N_DOFS; ++i)
        {
          pos.setZero();
          vel.setZero();
          acc.setZero();
          //time
          pos(1,0) = vel(1,0) = acc(1,0) = stepping_time_/2.0;
          //get current position
          pos(0,1) = reference_joint_posture_(i,0);
          vel(0,1) = reference_joint_vel_(i,0);
          acc(0,1) = reference_joint_acc_(i,0);
          pos(1,1) = des_leg_pos(i,0);
          legs_joint_trajectory_[i].initialize(pos,vel,acc);
        }

        //compute desired trajectory for capture point
        for(int i=0; i<3; ++i)
        {
          pos(0,0) = vel(0,0) = acc(0,0) = 0.0;
          pos(1,0) = vel(1,0) = acc(1,0) = stepping_time_/2.0;
          pos(0,1) = reference_com_pos_(i);
          vel(0,1) = reference_com_vel_(i);
          acc(0,1) = reference_com_acc_(i);
          com_trajectory_[i].query(unloading_leg_time_, pos(1,1), vel(1,1), acc(1,1));
          com_trajectory_[i].initialize(pos, vel, acc);
        }

        printf("SWING1\n");
        next_state = SWING1;
      }
      break;
    }
    case SWING1:
    {
      for(int i=0; i<3; ++i)
      {
        com_trajectory_[i].query(timer_, reference_com_pos_(i), reference_com_vel_(i), reference_com_acc_(i));
        swing_foot_trajectory_[i].query(timer_, reference_foot_position_[swing_leg_][i], reference_foot_lin_velocity_[swing_leg_][i], reference_foot_lin_acceleration_[swing_leg_][i]);
      }

      for(int i=0; i<N_DOFS; ++i)
      {
        legs_joint_trajectory_[i].query(timer_, reference_joint_posture_(i), reference_joint_vel_(i), reference_joint_acc_(i));
      }


      //posture
      //      double alpha = std::min(timer_/stance_duration_, 1.0);
      //      reference_joint_posture_ = alpha * postures_it_.posture(1) + (1.0-alpha)*postures_it_.posture();
      //      joint_trajs_[joint_trajs_it_].update(timer_);
      //      reference_joint_posture_.head<14>() = joint_trajs_[joint_trajs_it_].trajectory_current_;

      //      double dummy1, dummy2;
      //      swing_foot_traj_orientation_[0].query(timer_, reference_foot_orientation_[swing_leg_].w(), dummy1, dummy2);
      //      swing_foot_traj_orientation_[1].query(timer_, reference_foot_orientation_[swing_leg_].x(), dummy1, dummy2);
      //      swing_foot_traj_orientation_[2].query(timer_, reference_foot_orientation_[swing_leg_].y(), dummy1, dummy2);
      //      swing_foot_traj_orientation_[3].query(timer_, reference_foot_orientation_[swing_leg_].z(), dummy1, dummy2);
      //      reference_foot_orientation_[swing_leg_].normalize();
      reference_foot_ang_velocity_[swing_leg_].setZero();
      reference_foot_ang_acceleration_[swing_leg_].setZero();

      computeDesiredComAcc();
      computeDesiredSwingAcc(true);
      computeDesiredBaseAcc();

//      transformInWorldFrame(prev_des_endeff_pos_, prev_des_endeff_pos_);
//      computeKinematicStep(
//          reference_foot_position_wrld_[swing_leg_], reference_foot_lin_velocity_wrld_[swing_leg_], prev_des_endeff_pos_,
//          reference_joint_posture_, reference_joint_vel_, reference_joint_acc_);

      //      if(timer_ > 5.0 && timer_ < 5.1)
      //      {
      //        uext_sim[L_HAA].f[_Y_] = 80.0;
      //        uext_sim[R_HAA].f[_Y_] = 80.0;
      //        sendUextSim();
      //      }

      if(timer_ >= stepping_time_/2.0)
      {
        increment_walking_transition_counter(joint_trajs_it_);
        ++postures_it_;
        printf("SWING2\n");
        next_state = SWING2;
        timer_ = 0.0;

        //compute desired trajectory for swing foot
        double sig = (swing_leg_ == RIGHT_FOOT) ? 1.0 : -1.0;
        Eigen::Vector3d sw_motion;
        sw_motion = reference_foot_position_[swing_leg_];
        sw_motion[1] += step_length_/2.0;
        sw_motion[2] = foot_position_[leading_leg_](2) - 0.01;
        Eigen::Matrix2d vel,acc,pos;
        vel << 0.0, 0.0, stepping_time_/2.0, 0.0;
        acc = vel;
        for(int i=0; i<3; ++i)
        {
          pos << 0.0, reference_foot_position_[swing_leg_](i), stepping_time_/2.0, sw_motion(i);
          swing_foot_trajectory_[i].initialize(pos,vel,acc);
        }

        //the orientation of the swing foot
        Eigen::Matrix3d dummy_ref_rot;
        if(swing_leg_ == RIGHT_FOOT)
        {
          dummy_ref_rot <<
              0, -1, 0,
              0, 0, 1,
              -1, 0, 0;

        }
        else
        {
          dummy_ref_rot <<
              0, 1, 0,
              0, 0, -1,
              -1, 0, 0;

        }
        Eigen::Quaterniond dummy_ref(dummy_ref_rot);
        inverse_kinematics::fixQuaternionSign(reference_foot_orientation_[swing_leg_], dummy_ref);
        pos << 0.0, reference_foot_orientation_[swing_leg_].w(), stepping_time_/2.0, dummy_ref.w();
        swing_foot_traj_orientation_[0].initialize(pos,vel,acc);
        pos << 0.0, reference_foot_orientation_[swing_leg_].x(), stepping_time_/2.0, dummy_ref.x();
        swing_foot_traj_orientation_[1].initialize(pos,vel,acc);
        pos << 0.0, reference_foot_orientation_[swing_leg_].y(), stepping_time_/2.0, dummy_ref.y();
        swing_foot_traj_orientation_[2].initialize(pos,vel,acc);
        pos << 0.0, reference_foot_orientation_[swing_leg_].z(), stepping_time_/2.0, dummy_ref.z();
        swing_foot_traj_orientation_[3].initialize(pos,vel,acc);


        Eigen::Vector3d leading_foot_center_wrld = kinematics_eigen_->endeffPose(leading_leg_).topRows<3>()*foot_center_local_[leading_leg_];
        Eigen::Vector3d leading_foot_center;
        transformInInvariantFrame(leading_foot_center_wrld, leading_foot_center);
        double dummy_pos;
        vel << 0.0, 0.0, stepping_time_/2.0, 0.0;
        acc = vel;
        Eigen::Vector3d bias = com_centered_bias_;
        bias(0) *= (leading_leg_ == RIGHT_FOOT) ? 1.0 : -1.0;
        for(int i=0; i<3; ++i)
        {
          pos << 0.0, reference_com_pos_(i), stance_duration_, leading_foot_center(i) + bias(i);
          com_trajectory_[i].initialize(pos,vel,acc);
        }

        Eigen::Matrix<double, N_DOFS, 1> des_swing_height = legs_initial_position_;
        //        des_swing_height(swing_hipFE_idx_) = kinematics_eigen_->generalizedJointPositions()(stance_hipFE_idx_); //HFE
        ////        des_swing_height(stance_hipFE_idx_) = 0.05;
        //        des_swing_height(swing_hipFE_idx_+1) = -kinematics_eigen_->generalizedJointPositions()(stance_hipFE_idx_+1); //HAA
        ////        des_swing_height(stance_hipFE_idx_+1) = 0.02;
        //        des_swing_height(swing_hipFE_idx_+2) = -kinematics_eigen_->generalizedJointPositions()(stance_hipFE_idx_+2); //HFR
        ////        des_swing_height(stance_hipFE_idx_+2) = 0.2;
        //        des_swing_height(swing_hipFE_idx_+3) = kinematics_eigen_->generalizedJointPositions()(stance_hipFE_idx_+3); //KFE
        ////        des_swing_height(stance_hipFE_idx_+3) = 0.3;
        //        des_swing_height(swing_hipFE_idx_+4) = -kinematics_eigen_->generalizedJointPositions()(stance_hipFE_idx_+4); //AR
        ////        des_swing_height(stance_hipFE_idx_+4) = -0.2;
        //        des_swing_height(swing_hipFE_idx_+5) = kinematics_eigen_->generalizedJointPositions()(stance_hipFE_idx_+5); //AFE
        ////        des_swing_height(stance_hipFE_idx_+5) = 0.25;
        //        des_swing_height(swing_hipFE_idx_+6) = -kinematics_eigen_->generalizedJointPositions()(stance_hipFE_idx_+6); //AAA
        ////        des_swing_height(stance_hipFE_idx_+6) = -0.2;
        //initialize desired trajectory for swing leg
        for(int i=0; i<N_DOFS; ++i)
        {
          pos.setZero();
          vel.setZero();
          acc.setZero();
          //time
          pos(1,0) = vel(1,0) = acc(1,0) = stepping_time_/2.0;
          //get current position
          pos(0,1) = reference_joint_posture_(i,0);
          vel(0,1) = reference_joint_vel_(i,0);
          acc(0,1) = reference_joint_acc_(i,0);
          pos(1,1) = des_swing_height(i,0);
          legs_joint_trajectory_[i].initialize(pos,vel,acc);
        }
      }
      break;
    }
    case LOAD_LEG:
    {
      for(int i=0; i<3; ++i)
      {
        com_trajectory_[i].query(timer_, reference_com_pos_(i), reference_com_vel_(i), reference_com_acc_(i));
      }

      momentum_gains_ = (1.0 - timer_/loading_leg_time_)*momentum_gains_swing_ + timer_/loading_leg_time_*momentum_gains_stance_;

      for(int i=0; i<N_DOFS; ++i)
      {
        legs_joint_trajectory_[i].query(timer_, reference_joint_posture_(i), reference_joint_vel_(i), reference_joint_acc_(i));
      }

      computeDesiredComAcc();
      computeDesiredSwingAcc();
      computeDesiredBaseAcc();

      //swing foot controller
      //      swing_foot_controller_[swing_leg_]->weightingMat() = swing_foot_weight_*(1.0-timer_/loading_leg_time_);
      joint_position_controller_->weightingMat().block<7,1>(swing_hipFE_idx_ ,0) =
          joint_ctrl_weight_.block<7,1>(swing_hipFE_idx_, 0)*(timer_/loading_leg_time_) + swing_leg_joint_ctrl_weight_*(1.0 - timer_/loading_leg_time_);
      leg_joint_position_controller_[swing_leg_]->weightingMat() =
          joint_ctrl_weight_.block<7,1>(swing_hipFE_idx_, 0)*(timer_/loading_leg_time_) + swing_leg_joint_ctrl_weight_*(1.0 - timer_/loading_leg_time_);

      Eigen::Matrix<double, 6, 6> sqrt_stat_swing = (timer_/loading_leg_time_) *
          biped_task_comp_->sqrt_stat_feet_weight_.block<6,6>((leading_leg_-1)*6,(leading_leg_-1)*6);
      biped_task_comp_->sqrt_stat_feet_weight_.block<6,6>((swing_leg_-1)*6, (swing_leg_-1)*6) = sqrt_stat_swing;

      //get the desired foot acc to 0
      desired_foot_acc_ *= (1.0 - timer_/loading_leg_time_);

      Eigen::Matrix<double, 6, 1> ref_force;
      ref_force.setZero();
      ref_force(1) = -250 + 250 * (1.0 - timer_/loading_leg_time_);
      //      force_unloading_[swing_leg_]->update(ref_force);

      Eigen::Matrix<double, 6, 1> desired_force[2+1];
      desired_force[1].setZero();
      desired_force[2].setZero();
      desired_force[swing_leg_](1) = -60.0 * timer_/loading_leg_time_;
      desired_force[leading_leg_](1) = gravity_contribution_ + 60*timer_/loading_leg_time_;
      biped_task_comp_->frc_reg_R_.update(desired_force[RIGHT_FOOT]);
      biped_task_comp_->frc_reg_L_.update(desired_force[LEFT_FOOT]);


      //add the unload constraint
      double desired_force_weight[2+1];
      desired_force_weight[swing_leg_] = (1.0 - timer_/loading_leg_time_)*(unloading_leg_weight_-1.0)+1.0;
      desired_force_weight[leading_leg_] = 1.0;

      biped_task_comp_->frc_reg_L_.weightingMat() = desired_force_weight[LEFT_FOOT] * biped_task_comp_->sqrt_frc_reg_weight_;
      biped_task_comp_->frc_reg_R_.weightingMat() = desired_force_weight[RIGHT_FOOT] * biped_task_comp_->sqrt_frc_reg_weight_;

      //desired contact forces -> weight balancing
      reference_contact_forces_[swing_leg_].setZero();
      reference_contact_forces_[swing_leg_](1) = -(timer_/loading_leg_time_) *force_contact_switch_;
      reference_contact_forces_[leading_leg_](1) = gravity_contribution_ - reference_contact_forces_[swing_leg_](1);

      //      biped_task_comp_->sqrt_posture_tr_weight_.segment<2>((swing_leg_==LEFT_FOOT)?L_AFE:R_AFE) = Eigen::Matrix<double, 2, 1>::Ones()*(timer_/loading_leg_time_)*biped_task_comp_->sqrt_posture_tr_weight_[(leading_leg_==LEFT_FOOT)?L_AFE:R_AFE];
      //      joint_trajs_[joint_trajs_it_].update(joint_trajs_[joint_trajs_it_].duration_);
      //      reference_joint_posture_.head<14>() = joint_trajs_[joint_trajs_it_].trajectory_current_;


      if(timer_ >= loading_leg_time_)
      {
        timer_ = 0.0;
        increment_walking_transition_counter(joint_trajs_it_);
        ++postures_it_;
        next_state = DOUBLE_SUPPORT;
        printf("DOUBLE SUPPORT\n");

        reference_unloading_forces_ = reference_contact_forces_[swing_leg_];

        //switch the legs
        int dummy = swing_leg_;
        swing_leg_ = leading_leg_;
        leading_leg_ = dummy;

        //recompute the invariant frame
        computeInvariantReferenceFrame();

        if(current_num_steps_ <= number_of_steps_)
        {
          transformInInvariantFrame(reference_com_pos_wrld_, reference_com_pos_);

          //compute desired trajectory for CoM
          Eigen::Vector3d leading_foot_center_wrld = kinematics_eigen_->endeffPose(leading_leg_).topRows<3>()*foot_center_local_[leading_leg_];
          Eigen::Vector3d leading_foot_center;
          transformInInvariantFrame(leading_foot_center_wrld, leading_foot_center);
          double dummy_pos;
          Eigen::Matrix2d pos,vel,acc;
          vel << 0.0, 0.0, stance_duration_, 0.0;
          acc = vel;
          Eigen::Vector3d bias = com_centered_bias_;
          bias(0) *= (leading_leg_ == RIGHT_FOOT) ? 1.0 : -1.0;
          for(int i=0; i<3; ++i)
          {
            pos << 0.0, reference_com_pos_(i), stance_duration_, leading_foot_center(i) + bias(i);
            vel << 0.0, reference_com_vel_(i), stance_duration_, 0.0;
            acc << 0.0, reference_com_acc_(i), stance_duration_, 0.0;
            com_trajectory_[i].initialize(pos,vel,acc);
          }
        }
        else
        {
          //compute desired trajectory for CoM
          Eigen::Vector3d leading_foot_center_wrld = kinematics_eigen_->endeffPose(leading_leg_).topRows<3>()*foot_center_local_[leading_leg_];
          Eigen::Vector3d leading_foot_center;
          transformInInvariantFrame(leading_foot_center_wrld, leading_foot_center);
          Eigen::Vector3d swing_foot_center_wrld = kinematics_eigen_->endeffPose(swing_leg_).topRows<3>()*foot_center_local_[swing_leg_];
          Eigen::Vector3d swing_foot_center;
          transformInInvariantFrame(swing_foot_center_wrld, swing_foot_center);
          double dummy_pos;
          Eigen::Matrix2d pos,vel,acc;
          vel << 0.0, 0.0, stance_duration_, 0.0;
          acc = vel;
          Eigen::Vector3d bias = com_centered_bias_;
          bias(0) = 0.0; //no bias in x direction

          for(int i=0; i<3; ++i)
          {
            pos << 0.0, reference_com_pos_(i), stance_duration_, (leading_foot_center(i) + swing_foot_center(i))/2.0 + bias(i);
            com_trajectory_[i].initialize(pos,vel,acc);
          }
        }
      }

      break;
    }
    case SWING2:
    {
      for(int i=0; i<3; ++i)
      {
        com_trajectory_[i].query(timer_, reference_com_pos_(i), reference_com_vel_(i), reference_com_acc_(i));
        swing_foot_trajectory_[i].query(timer_, reference_foot_position_[swing_leg_][i], reference_foot_lin_velocity_[swing_leg_][i], reference_foot_lin_acceleration_[swing_leg_][i]);
      }
//      for(int i=0; i<N_DOFS; ++i)
//      {
//        legs_joint_trajectory_[i].query(timer_, reference_joint_posture_(i), reference_joint_vel_(i), reference_joint_acc_(i));
//      }

      double dummy1, dummy2;
      swing_foot_traj_orientation_[0].query(timer_, reference_foot_orientation_[swing_leg_].w(), dummy1, dummy2);
      swing_foot_traj_orientation_[1].query(timer_, reference_foot_orientation_[swing_leg_].x(), dummy1, dummy2);
      swing_foot_traj_orientation_[2].query(timer_, reference_foot_orientation_[swing_leg_].y(), dummy1, dummy2);
      swing_foot_traj_orientation_[3].query(timer_, reference_foot_orientation_[swing_leg_].z(), dummy1, dummy2);
      reference_foot_orientation_[swing_leg_].normalize();
      reference_foot_ang_velocity_[swing_leg_].setZero();
      reference_foot_ang_acceleration_[swing_leg_].setZero();

      computeDesiredComAcc();
      computeDesiredSwingAcc(true);
      computeDesiredBaseAcc();

//      transformInWorldFrame(prev_des_endeff_pos_, prev_des_endeff_pos_);
//      computeKinematicStep(
//          reference_foot_position_wrld_[swing_leg_], reference_foot_lin_velocity_wrld_[swing_leg_], prev_des_endeff_pos_,
//          reference_joint_posture_, reference_joint_vel_, reference_joint_acc_);

      double amp = 0.3;
      double freq = .5;
      double offset = 1.2 - amp;
      reference_joint_posture_(swing_hipFE_idx_) = amp*cos(timer_*M_PI*freq) + legs_initial_position_(swing_hipFE_idx_) + offset;
      reference_joint_vel_(swing_hipFE_idx_) = -amp*M_PI*freq*sin(timer_*M_PI*freq);
      reference_joint_acc_(swing_hipFE_idx_) = -amp*M_PI*freq*freq*M_PI*cos(timer_*M_PI*freq);

      reference_joint_posture_(swing_hipFE_idx_+3) = amp*cos(timer_*M_PI*freq)  + legs_initial_position_(swing_hipFE_idx_+3) + offset;
      reference_joint_vel_(swing_hipFE_idx_+3) = -amp*M_PI*freq*sin(timer_*M_PI*freq);
      reference_joint_acc_(swing_hipFE_idx_+3) = -amp*M_PI*M_PI*freq*freq*cos(timer_*M_PI*freq);

      //      reference_joint_posture_ = (timer_/(stepping_time_/2.0)) * postures_it_.posture(1) + (1.0-timer_/(stepping_time_/2.0))*postures_it_.posture();
      //      joint_trajs_[joint_trajs_it_].update(timer_);
      //      reference_joint_posture_.head<14>() = joint_trajs_[joint_trajs_it_].trajectory_current_;


//      if(timer_ >= stepping_time_/2.0)
//      if(generalized_force_[swing_leg_](2) > force_contact_switch_)
      if(0)
      {
        printf("LOAD_LEG\n");
        next_state = LOAD_LEG;
        timer_ = 0.0;

        //set the constraints for double support
        for(int j=1; j<=2*N_CART; ++j)
          endeff_constraints_[swing_leg_].c[j] = 1;

        //reset force weighting
        biped_task_comp_->frc_reg_L_.weightingMat() = biped_task_comp_->sqrt_frc_reg_weight_;
        biped_task_comp_->frc_reg_R_.weightingMat() = biped_task_comp_->sqrt_frc_reg_weight_;


        Eigen::Matrix2d vel,acc,pos;
        //initialize desired trajectory for swing leg (move from default to current)
        for(int i=0; i<N_DOFS; ++i)
        {
          pos.setZero();
          vel.setZero();
          acc.setZero();
          //time
          pos(1,0) = vel(1,0) = acc(1,0) = loading_leg_time_;
          //get current position
          pos(0,1) = reference_joint_posture_(i,0);
          vel(0,1) = reference_joint_vel_(i,0);
          acc(0,1) = reference_joint_acc_(i,0);
          pos(1,1) = default_posture_(i,0);
          legs_joint_trajectory_[i].initialize(pos,vel,acc);
        }

        //compute CoM motion to start loading the leg
        transformInInvariantFrame(reference_com_pos_wrld_, reference_com_pos_);
//        reference_com_vel_.setZero();
//        reference_com_acc_.setZero();

        //compute desired trajectory for CoM
        Eigen::Vector3d leading_foot_center_wrld = kinematics_eigen_->endeffPose(swing_leg_).topRows<3>()*foot_center_local_[swing_leg_];
        Eigen::Vector3d leading_foot_center;
        transformInInvariantFrame(leading_foot_center_wrld, leading_foot_center);
        double dummy_pos;
        vel << 0.0, 0.0, stance_duration_, 0.0;
        acc = vel;
        Eigen::Vector3d bias = com_centered_bias_;
        bias(0) *= (swing_leg_ == RIGHT_FOOT) ? 1.0 : -1.0;
        for(int i=0; i<3; ++i)
        {
          pos << 0.0, reference_com_pos_(i), stance_duration_, leading_foot_center(i) + bias(i);
          vel << 0.0, reference_com_vel_(i), stance_duration_, 0.0;
          acc << 0.0, reference_com_acc_(i), stance_duration_, 0.0;
          com_trajectory_[i].initialize(pos,vel,acc);
        }
      }

      break;
    }
  }


//  int jt_read_size = std::min(int(dummy_variables_.size()), int(joint_trajs_[joint_trajs_it_].trajectory_current_.size()));
//  dummy_variables_.head(jt_read_size) = joint_trajs_[joint_trajs_it_].trajectory_current_.head(jt_read_size);

  timer_ += 1/double(task_servo_rate);


#ifndef __XENO__
  //  std::cout <<"t: " << timer_ << std::endl;
#endif

  {
    SL_Jstate st[N_DOFS+1];
    for(int i=1; i<=N_DOFS; ++i)
    {
      st[i].th = reference_joint_posture_(i-1);
      st[i].thd = reference_joint_vel_(i-1);
      st[i].thdd = reference_joint_acc_(i-1);
    }
    des_pose_kinematics_->update(st, base_state, base_orient, endeff_constraints_);
    Eigen::Matrix<double, 4, 4> st_pose = des_pose_kinematics_->endeffPose(leading_leg_);
    Eigen::Quaterniond st_orien(st_pose.topLeftCorner<3,3>());
    Eigen::Vector3d st_position(st_pose.topRightCorner<3,1>());
    Eigen::Vector3d base_p;
    Eigen::Quaterniond base_o;
    SL_Cstate prev_base_pose;
    SL_quat prev_base_orien;
    //prev pos in stance foot frame
    prev_des_endeff_pos_ = st_orien.matrix().transpose()*(des_pose_kinematics_->endeffPosition(swing_leg_) - st_position);
    GeometryUtils::SLQuaternionToEigen(base_orient.q, base_o);
    base_p = st_orien.matrix().transpose()*(Eigen::Map<Eigen::Vector3d>(&(base_state.x[1])) - st_position);
    base_o = st_orien.inverse() * base_o;
    //prev pos in real world frame
    prev_des_endeff_pos_ = foot_orientation_wrld_[leading_leg_].matrix()*prev_des_endeff_pos_ + foot_position_wrld_[leading_leg_];
    base_p = foot_orientation_wrld_[leading_leg_].matrix()*base_p + foot_position_wrld_[leading_leg_];
    base_o = foot_orientation_wrld_[leading_leg_] * base_o;
    GeometryUtils::EigQuaternionToSL(base_o, prev_base_orien.q);
    Eigen::Map<Eigen::Vector3d>(&(prev_base_pose.x[1])) = base_p;
    int du;
    des_pose_floating_base_kin_->computeJacobians(st, prev_base_pose,prev_base_orien, endeff_constraints_, du);
    dummy_variables_.topRows<3>() = prev_des_endeff_pos_;
    transformInInvariantFrame(prev_des_endeff_pos_, prev_des_endeff_pos_);
  }
  //update base controller
  base_controller_->update(desired_base_acceleration_);
  //  for(int i=1; i<=2; ++i)
  //    swing_foot_controller_[i]->update(desired_foot_acc_.block<6,1>((i-1)*6,0));

  //  computeLegOpSpInertia(LEFT_FOOT, leg_joint_position_controller_[LEFT_FOOT]->variable_gain_);
  //  computeLegOpSpInertia(RIGHT_FOOT, leg_joint_position_controller_[RIGHT_FOOT]->variable_gain_);
  leg_joint_position_controller_[RIGHT_FOOT]->variable_gain_.setIdentity();
  leg_joint_position_controller_[LEFT_FOOT]->variable_gain_.setIdentity();

  joint_position_controller_->update(reference_joint_posture_,
                                     reference_joint_vel_,
                                     reference_joint_acc_);

  leg_joint_position_controller_[LEFT_FOOT]->update(reference_joint_posture_.block<7,1>(L_HFE-1,0),
                                                    reference_joint_vel_.block<7,1>(L_HFE-1,0),
                                                    reference_joint_acc_.block<7,1>(L_HFE-1,0));
  leg_joint_position_controller_[RIGHT_FOOT]->update(reference_joint_posture_.block<7,1>(R_HFE-1,0),
                                                     reference_joint_vel_.block<7,1>(R_HFE-1,0),
                                                     reference_joint_acc_.block<7,1>(R_HFE-1,0));


  //update force controllers
  biped_task_comp_->frc_reg_R_.update(reference_contact_forces_[RIGHT_FOOT]);
  biped_task_comp_->frc_reg_L_.update(reference_contact_forces_[LEFT_FOOT]);

  // compute torques and accelerations
  //  Eigen::Matrix<double, N_DOFS, 1> posture_ref = null_posture_P_gains_.asDiagonal() *
  //      (reference_null_posture_ - kinematics_eigen_->generalizedJointPositions().topRows<N_DOFS>()) -
  //      null_posture_D_gains_.asDiagonal()*kinematics_eigen_->generalizedJointVelocities().topRows<N_DOFS>();

  Eigen::Matrix<double, N_DOFS, 1> posture_ref;
  posture_ref.setZero();
  biped_task_comp_->update(desired_momentum_rate_, posture_ref, desired_foot_acc_);
  hierarch_inv_dyn_->solve();
  for(int i=1; i<=R_AAA; ++i)
  {
    joint_des_state[i].uff = hierarch_inv_dyn_->admis_torques_[i-1];
    joint_des_state[i].thdd = 0.0; //hierarch_inv_dyn_->admis_accels_(i-1);
    joint_des_state[i].th = joint_state[i].th;
    joint_des_state[i].thd = joint_state[i].thd; //0.0;
  }

#ifdef __XENO__
  SRTIME run_end = rt_timer_ticks2ns(rt_timer_read());
  SRTIME run_duration = run_end - run_start;
  cycle_duration_ = rt_timer_ticks2ns(run_duration)*0.000001;
#else
  boost::posix_time::ptime run_end(boost::posix_time::microsec_clock::local_time());
  boost::posix_time::time_duration run_duration = run_end - run_start;
  cycle_duration_ = run_duration.total_microseconds()/1000000.0;
#endif

  //#ifndef __XENO__
  //  std::cout << "full cycle: " <<
  //      cycle_duration_ << std::endl;
  //#endif

  sendOpenGL();
  current_state_ = next_state;

  return true;
}

