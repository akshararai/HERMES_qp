/*
 * balance_controller.cpp
 *
 *  Created on: Aug 28, 2013
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
#include "RtMinJerkGenerator.h"

#include <balance_controller.h>
#include "drawUtils.h"
#include <SL_collect_data.h>
#include <SL_common.h>
#include <SL_dynamics.h>
#include <quaternions.h>

using namespace floating_base_utilities;
using namespace momentum_balance_control;


class BasePerturbSim{

 template<typename T>
 class VariableBuffer{
  public:
   VariableBuffer(T& original) : original_(original), var_(original){};
   ~VariableBuffer(){
     original_ = var_;
   }
   T& operator ()(){return original_;};
   T& original_;
   T var_;
 };

 public:
  BasePerturbSim(bool initialize, double time) : base_state_(base_state),
      base_orient_(base_orient){
    if(initialize){
      std::cout << "reinitializing BasePerturbSim" << std::endl;
      if(!read_parameter_pool_double("BalanceController.cf","base_perturb_frq",&base_perturb_frq_))
        assert(false && "reading parameter base_perturb_frq failed");
      if(!read_parameter_pool_double("BalanceController.cf","base_perturb_amp",&base_perturb_amp_))
        assert(false && "reading parameter base_perturb_amp failed");
      return;
    }
    if(!real_robot_flag){
      base_state_().x[_X_] += delta_pos(time);
      base_state_().xd[_X_] += delta_vel(time);
      base_state_().xdd[_X_] += delta_acc(time);

//      base_orient_().q[_Q3_] += delta_pos(time);
//      base_orient_().q[_G_] += delta_vel(time);
    }
  }

  double delta_pos(double time){
    return sin(2*M_PI*base_perturb_frq_*time) * base_perturb_amp_;
  }
  double delta_vel(double time){
    return cos(2*M_PI*base_perturb_frq_*time) * base_perturb_amp_*2*M_PI*base_perturb_frq_;
  }
  double delta_acc(double time){
    return -sin(2*M_PI*base_perturb_frq_*time) * base_perturb_amp_*2*M_PI*base_perturb_frq_*2*M_PI*base_perturb_frq_;
  }

  static double base_perturb_frq_, base_perturb_amp_;
  VariableBuffer<SL_Cstate> base_state_;
  VariableBuffer<SL_quat> base_orient_;
};
double BasePerturbSim::base_perturb_frq_ = 0.;
double BasePerturbSim::base_perturb_amp_ = 0.;

BalanceController::BalanceController()
{
  cycle_duration_ = 0.0;
}

BalanceController::~BalanceController()
{
}

bool BalanceController::initialize()
{

  BasePerturbSim base_perturber(true, timer_);

  //read a parameter file without joint friction for stability
  if(!real_robot_flag)
  {
    printf("loading different LinkParameters\n");
    read_link_parameters("LinkParameters.cf");
  }

  sprintf(config_file_,"%s","BalanceController.cf");

  //initialize the parent class
  if(!BipedController::initialize(std::string(config_file_)))
  {
    std::cout << "BipedController could not be initialized" << std::endl;
    return (initialized_ = false);
  }

  initialized_ = true;

  if(!read_parameter_pool_double(config_file_,"push_force",&push_force_))
    assert(false && "reading parameter push_force failed");
  if(!read_parameter_pool_double(config_file_,"push_dur",&push_dur_))
    assert(false && "reading parameter push_dur failed");
  //sine motion parameters
  if(!read_parameter_pool_double(config_file_,"com_sine_frequency",&com_sine_frequency_))
    assert(false && "reading parameter com_sine_frequency failed");
  if(!read_parameter_pool_double(config_file_,"init_duration",&init_duration_))
    assert(false && "reading parameter init_duration failed");
  double dummy_amp[3+1];
  if(!read_parameter_pool_double_array(config_file_,"com_sine_amplitude", 3, dummy_amp))
    assert(false && "reading parameter com_sine_amplitude failed");
  com_sine_amplitude_ = Eigen::Map<Eigen::Vector3d>(&(dummy_amp[1]));

  double tmp_bias[3+1];
  if(!read_parameter_pool_double_array(config_file_, "com_centered_bias", 3, tmp_bias)) assert(false && "reading parameters com_centered_bias failed");
  com_centered_bias_ = Eigen::Map<Eigen::Matrix<double, 3, 1> >(&(tmp_bias[1]));


  //compute desired trajectory for CoM
  Eigen::Vector3d leading_foot_center_wrld = kinematics_eigen_->endeffPose(leading_leg_).topRows<3>()*foot_center_local_[leading_leg_];
  Eigen::Vector3d leading_foot_center;
  transformInInvariantFrame(leading_foot_center_wrld, leading_foot_center);
  Eigen::Vector3d swing_foot_center_wrld = kinematics_eigen_->endeffPose(swing_leg_).topRows<3>()*foot_center_local_[swing_leg_];
  Eigen::Vector3d swing_foot_center;
  transformInInvariantFrame(swing_foot_center_wrld, swing_foot_center);
  double dummy_pos;
  Eigen::Matrix2d pos,vel,acc;
  vel << 0.0, 0.0, init_duration_, 0.0;
  acc = vel;
  Eigen::Vector3d bias = com_centered_bias_ + com_sine_amplitude_;

  for(int i=0; i<3; ++i)
  {
    pos << 0.0, reference_com_pos_(i), init_duration_, (leading_foot_center(i) + swing_foot_center(i))/2.0 + bias(i);
    com_trajectory_[i].initialize(pos,vel,acc);
  }

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


bool BalanceController::run()
{
  if(!initialized_)
  {
    printf("ERROR controller not initialized\n");
    freeze();
    return false;
  }
  BasePerturbSim base_perturber(!initialized_, timer_);
#ifdef __XENO__
  SRTIME run_start = rt_timer_ticks2ns(rt_timer_read());
#else
  boost::posix_time::ptime run_start(boost::posix_time::microsec_clock::local_time());
#endif



  BalancingState next_state = current_state_;

  update();

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

      if(timer_ > init_duration_)
      {
        next_state = BALANCE;
        timer_ = 0.0;
        printf("BALANCING\n");
      }

      break;
    }

    //move the capture point to the leading leg using the com ctrl law
    case BALANCE:
    {
      //compute desired trajectory for CoM
      Eigen::Vector3d leading_foot_center_wrld = kinematics_eigen_->endeffPose(leading_leg_).topRows<3>()*foot_center_local_[leading_leg_];
      Eigen::Vector3d leading_foot_center;
      transformInInvariantFrame(leading_foot_center_wrld, leading_foot_center);
      Eigen::Vector3d swing_foot_center_wrld = kinematics_eigen_->endeffPose(swing_leg_).topRows<3>()*foot_center_local_[swing_leg_];
      Eigen::Vector3d swing_foot_center;
      transformInInvariantFrame(swing_foot_center_wrld, swing_foot_center);


      //COG position from relative position
      reference_com_pos_ = (leading_foot_center + swing_foot_center)/2.0 + com_centered_bias_ + cos(2*M_PI*com_sine_frequency_*timer_) * com_sine_amplitude_;
      reference_com_vel_ = -2*M_PI*com_sine_frequency_ * sin(2*M_PI*com_sine_frequency_*timer_) * com_sine_amplitude_;
      reference_com_acc_ = - 2*M_PI*com_sine_frequency_*2*M_PI*com_sine_frequency_ *
          cos(2*M_PI*com_sine_frequency_*timer_) * com_sine_amplitude_;


      computeDesiredComAcc();
      computeDesiredSwingAcc();
      computeDesiredBaseAcc();


      if(!real_robot_flag){
        if(timer_ > 4. && timer_ < 4. + push_dur_)
        {
          uext_sim[L_HAA].f[_Y_] = .5*push_force_;
          uext_sim[R_HAA].f[_Y_] = .5*push_force_;
          sendUextSim();
        }
      }
      break;
    }
  }

  for(int endeff_id=1; endeff_id<=N_ENDEFFS; ++endeff_id){
		reference_contact_forces_[endeff_id][1] = -1./N_DOFS * 9.81 * kinematics_eigen_->robotMass();
  }

  timer_ += 1/double(task_servo_rate);


#ifndef __XENO__
  //  std::cout <<"t: " << timer_ << std::endl;
#endif

  //update base controller
  base_controller_->update(desired_base_acceleration_);

  //update force controllers
  biped_task_comp_->frc_reg_R_.update(reference_contact_forces_[RIGHT_FOOT]);
  biped_task_comp_->frc_reg_L_.update(reference_contact_forces_[LEFT_FOOT]);

  //update position control of joints
  leg_joint_position_controller_[RIGHT_FOOT]->variable_gain_.setIdentity();
  leg_joint_position_controller_[LEFT_FOOT]->variable_gain_.setIdentity();
//
//  joint_position_controller_->update(reference_joint_posture_,
//                                     reference_joint_vel_,
//                                     reference_joint_acc_);

  leg_joint_position_controller_[LEFT_FOOT]->update(reference_joint_posture_.block<7,1>(L_HFE-1,0),
                                                    reference_joint_vel_.block<7,1>(L_HFE-1,0),
                                                    reference_joint_acc_.block<7,1>(L_HFE-1,0));
  leg_joint_position_controller_[RIGHT_FOOT]->update(reference_joint_posture_.block<7,1>(R_HFE-1,0),
                                                     reference_joint_vel_.block<7,1>(R_HFE-1,0),
                                                     reference_joint_acc_.block<7,1>(R_HFE-1,0));

  // compute torques and accelerations
  //  Eigen::Matrix<double, N_DOFS, 1> posture_ref = null_posture_P_gains_.asDiagonal() *
  //      (reference_joint_posture_ - kinematics_eigen_->generalizedJointPositions().topRows<N_DOFS>()) -
  //      null_posture_D_gains_.asDiagonal()*kinematics_eigen_->generalizedJointVelocities().topRows<N_DOFS>();

  Eigen::Matrix<double, N_DOFS, 1> posture_ref;
  posture_ref.setZero();
  biped_task_comp_->update(desired_momentum_rate_, posture_ref, desired_foot_acc_);
  hierarch_inv_dyn_->solve();

  for(int i=1; i<=R_AAA; ++i)
  {
    if(current_state_ == INIT && timer_ <= 1.){
      joint_des_state[i].uff = timer_*(hierarch_inv_dyn_->admis_torques_[i-1]) + (1.-timer_)*initial_u_[i-1];
    }else{
      joint_des_state[i].uff = hierarch_inv_dyn_->admis_torques_[i-1];
      joint_des_state[i].thdd = hierarch_inv_dyn_->admis_accels_(i-1);
    }joint_des_state[i].th = joint_state[i].th;
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





