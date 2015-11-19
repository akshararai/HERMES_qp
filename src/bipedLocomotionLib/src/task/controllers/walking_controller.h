/*
 * walking_controller.h
 *
 *  Created on: Apr 28, 2013
 *      Author: righetti
 */

#ifndef WALKING_CONTROLLER_H_
#define WALKING_CONTROLLER_H_

#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include "RtMinJerkGenerator.hh"
//#include "RecordedTrajectory.hh"
#include "min_jerk_generator.h"

#include "biped_controller.h"

#include <SL.h>
#include <SL_user.h>
#include "SL_tasks.h"
#include "SL_task_servo.h"
#include "SL_collect_data.h"
#include "SL_objects.h"
#include "utility_macros.h"

class WCPostureIterator
{
public:
  WCPostureIterator()
  {
    iterator_=0;
    count_dir_=1;
  }

  WCPostureIterator& operator++()
  {
    increaseIterator(iterator_, count_dir_);
    return *this;
  }
  int postureIndex(int offset=0) const
  {
    offset=offset%postures_.size();

    int it=iterator_;
    int dir=count_dir_;
    for(; offset>0; --offset)
      increaseIterator(it, dir);
    return it+int(postures_.size()/2);
  }

  const Eigen::Matrix<double, N_DOFS,1>& posture(int offset=0) const{return postures_[postureIndex(offset)];};

  std::vector<Eigen::Matrix<double, N_DOFS,1>, Eigen::aligned_allocator<Eigen::Matrix<double, N_DOFS,1> > > postures_; // left_n, ..., left_1, default, right_1, ..., right_n
private:
  int count_dir_;
  int iterator_;

  void increaseIterator(int& it, int& dir) const
  {
    do
    {
      it+=dir;
      if(abs(it) == int(postures_.size()/2))
        dir*= -1;
    }while(0==it);// for postures_.size()=5 we get 0,1,2,1,-1,-2,-1,1...
  }
};

class WalkingController : public BipedController
{
public:
  WalkingController();
  virtual ~WalkingController();

  bool initialize();
  bool run();

private:
  double cycle_duration_;


  int number_of_steps_;
  int current_num_steps_;

  enum WalkingState {DOUBLE_SUPPORT, SWING1, SWING2, LOAD_LEG, UNLOAD_LEG, INIT};
  WalkingState current_state_;


  Eigen::Vector3d com_centered_bias_;


  double stepping_time_, stance_duration_;
  double init_time_;
  double timer_;

  int swing_hipFE_idx_; //HFE-1 (and not HFE)
  int stance_hipFE_idx_;

  boost::shared_ptr<floating_base_utilities::KinematicsEigen> des_pose_kinematics_;
  boost::shared_ptr<floating_base_utilities::FloatingBaseKinematics> des_pose_floating_base_kin_;
  Eigen::Matrix<double, 3, 1> prev_des_endeff_pos_;

  Eigen::Matrix<double, 6, 1> reference_unloading_forces_;

  double force_contact_switch_;
  WCPostureIterator postures_it_;
//  floating_base_utilities::RtMinJerkGenerator<3> capture_point_trajectory_[3];
  floating_base_utilities::RtMinJerkGenerator<3> com_trajectory_[3];
  floating_base_utilities::RtMinJerkGenerator<3> swing_foot_trajectory_[3];
  floating_base_utilities::RtMinJerkGenerator<3> swing_foot_traj_orientation_[4];

  floating_base_utilities::RtMinJerkGenerator<3> legs_joint_trajectory_[N_DOFS];
  Eigen::Matrix<double, N_DOFS, 1> legs_initial_position_;

  //unloading related parameters
//  int unloading_leg_rank_;
  double unloading_leg_weight_;
  double unloading_leg_time_, loading_leg_time_;


  double step_length_, step_height_, step_width_;

  Eigen::Matrix<double, 7, 1> swing_leg_joint_ctrl_weight_;


  Eigen::Matrix<double, N_DOFS, 1> posture_1_;
  Eigen::Matrix<double, N_DOFS, 1> posture_2_;

//  std::vector<floating_base_utilities::RecordedTrajectory<5010,N_DOFS>,
//      Eigen::aligned_allocator<floating_base_utilities::RecordedTrajectory<5010,N_DOFS> > >
//      joint_trajs_;
  int joint_trajs_it_;
  static const int joint_trajs_n_states_ = 7;

  char config_file_[30];

  Eigen::Matrix<double, 40, 1> dummy_variables_;

  void increment_walking_transition_counter(int &counter) const;

  bool computeKinematicStep(const Eigen::Matrix<double, 3, 1>& x_des,
                            const Eigen::Matrix<double, 3, 1>& dx_des,
                            const Eigen::Matrix<double, 3, 1>& x,
                            Eigen::Matrix<double, N_DOFS, 1>& q_pos,
                            Eigen::Matrix<double, N_DOFS, 1>& q_vel,
                            Eigen::Matrix<double, N_DOFS, 1>& q_acc);
};

#endif /* WALKING_CONTROLLER_H_ */
