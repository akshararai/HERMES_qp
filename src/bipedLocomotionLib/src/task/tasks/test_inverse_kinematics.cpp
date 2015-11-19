/*
 * test_ik_task.cpp
 *
 *  Created on: Feb 28, 2012
 *      Author: righetti
 */


//#include <boost/chrono.hpp>


#include "inverse_kinematics.h"
#include "min_jerk_generator.h"


#include "SL_system_headers.h"

#include "SL.h"
#include "SL_tasks.h"
#include "SL_collect_data.h"
#include "SL_man.h"
#include "SL_common.h"

#include "unistd.h"
#include <vector>
#include <fstream>

#include "display_kinematics.h"

#include "SL_unix_common.h"

extern "C" {
void add_test_ik_task();
static void test_ik();
static void save_robot_state();
}


void add_test_ik_task()
{
  addToMan("test_ik", "test ik", test_ik);
  addToMan("robot_state_save", "save robot state for ik", save_robot_state);
}

static void save_robot_state()
{
  char name[20];
  sprintf(name, "robot_state");
  get_string("Save in which file?", name, name);

  int com = 1;
  get_int("Use CoM as target?", com, &com);
  SL_Cstate dummy_cart[N_ENDEFFS+1];
  SL_quat dummy_orient[N_ENDEFFS+1];
  for(int i=1; i<=N_ENDEFFS;++i)
  {
    dummy_cart[i] = cart_state[i];
    dummy_orient[i] = cart_orient[i];
  }
  dummy_orient[0] = base_orient;
  if(com)
    dummy_cart[0] = cog;
  else
    dummy_cart[0] = base_state;
  inverse_kinematics::RobotState robot_state;
  inverse_kinematics::create_robot_state(base_state, base_orient, dummy_cart, dummy_orient, joint_state, endeff, robot_state);
  inverse_kinematics::save_robot_state(name, robot_state);
}

static void test_ik()
{
  inverse_kinematics::InverseKinematics ik;


  std::vector<inverse_kinematics::RobotState> robot_state_trajectory;
  double delta_time = 0.005;


  inverse_kinematics::RobotState robot_state;
  robot_state.constraint_position.resize(N_ENDEFFS+1);
  robot_state.constraint_orientation.resize(N_ENDEFFS+1);
  robot_state.is_constrained.resize(N_ENDEFFS+1);
  robot_state.joints.resize(N_DOFS+1);

  inverse_kinematics::read_robot_state("robot_state", robot_state);

  ik.useCom(true);
//  SL_Cstate carts[N_ENDEFFS+1];
//  //  carts[0] = base_state;
//  carts[0] = cog;
//  Eigen::Map<Eigen::Vector3d>(&(carts[0].xd[1])).setZero();
//  Eigen::Map<Eigen::Vector3d>(&(carts[0].xdd[1])).setZero();
//
//  SL_quat carto[N_ENDEFFS+1];
//  carto[0] = base_orient;
//  for(int i=1; i<=N_ENDEFFS; ++i)
//  {
//    carts[i] = cart_state[i];
//    carto[i] = cart_orient[i];
//  }
//
//
//  inverse_kinematics::create_robot_state(base_state, base_orient, carts, cart_orient, joint_state, endeff, robot_state);

  //create a new position where the feet are in contact
  for(int i=0; i<=N_ENDEFFS; ++i)
  {
    for(int j=1; j<=2*N_CART; ++j)
    {
      if(i==LEFT_FOOT || i==RIGHT_FOOT)
        robot_state.is_constrained[i].c[j] = 1;
      else
        robot_state.is_constrained[i].c[j] = 0;
    }
  }

  double trajectory_duration = 5.0;

  std::vector<inverse_kinematics::MinJerkGenerator> trajectory;
  Eigen::Matrix2d pos[3*(N_ENDEFFS+1)],vel[3*(N_ENDEFFS+1)],acc[3*(N_ENDEFFS+1)];//pos vel acc constraint at time t

  //fill a dummy desired traj which does not move for each possible goal
  for(int i=0; i<=N_ENDEFFS; ++i)
  {
    inverse_kinematics::MinJerkGenerator dummy;
    for(int j=0; j<N_CART; ++j)
    {
      pos[3*i+j] << 0.0, robot_state.constraint_position[i].x[_X_ + j],
          trajectory_duration, robot_state.constraint_position[i].x[_X_ + j];
      vel[3*i+j] << 0.0, 0.0, trajectory_duration, 0.0;
      acc[3*i+j] = vel[3*i+j];
      dummy.initialize(pos[3*i+j],vel[3*i+j],acc[3*i+j]);
      trajectory.push_back(dummy);
    }
  }

  //right leg up motion
  //  Eigen::Matrix<double , 3, 2> rl_pos;
  //  rl_pos << 0.0, pos[5](0,1),
  //      trajectory_duration/2.0, pos[5](0,1)+0.1,
  //      trajectory_duration, pos[5](0,1);
  //  trajectory[5].initialize(rl_pos,vel[5],acc[5]);

  //  Eigen::Matrix<double , 2, 2> rl_pos;
  //  rl_pos << 0.0, pos[5](0,1),
  //      trajectory_duration, pos[5](0,1)+0.1;
  //  trajectory[5].initialize(rl_pos,vel[5],acc[5]);

  //com motion
  Eigen::Matrix<double, 2, 2> base_pos;
  base_pos << 0.0, pos[0](0,1), trajectory_duration, pos[0](0,1);
  trajectory[0].initialize(base_pos, vel[0], acc[0]);

  base_pos << 0.0, pos[1](0,1), trajectory_duration, pos[1](0,1);
  trajectory[1].initialize(base_pos, vel[1], acc[1]);

  base_pos << 0.0, pos[2](0,1), trajectory_duration, pos[2](0,1)-0.07;
  trajectory[2].initialize(base_pos, vel[2], acc[2]);

  //  //base motion
  //  Eigen::Matrix<double, 2, 2> base_pos;
  //  base_pos << 0.0, pos[0](0,1), trajectory_duration, -0.17;
  //  trajectory[0].initialize(base_pos, vel[0], acc[0]);
  //
  //  base_pos << 0.0, pos[1](0,1), trajectory_duration, -0.0;
  //  trajectory[1].initialize(base_pos, vel[1], acc[1]);
  //
  //  base_pos << 0.0, pos[2](0,1), trajectory_duration, -0.152;
  //  trajectory[2].initialize(base_pos, vel[2], acc[2]);


  //create the robot state trajectory
  int num_way_points = int(trajectory_duration/delta_time);
  for(int i=0; i<num_way_points; i++)
  {
    robot_state_trajectory.push_back(robot_state);
    double pos, vel, acc;

    for(int j=0; j<=N_ENDEFFS; ++j)
    {
      for(int k=0; k<N_CART; ++k)
      {
        trajectory[j*N_CART+k].query(delta_time*i,pos,vel,acc);
        robot_state.constraint_position[j].x[_X_ + k] = pos;
        robot_state.constraint_position[j].xd[_X_ + k] = vel;
      }
    }
  }

  Eigen::Matrix<double, 2*N_CART*(N_ENDEFFS+1), 1> p_task_gain;
  Eigen::Matrix<double, N_DOFS, 1> p_posture_gain;

  p_task_gain.block(0,0,3,1).setConstant(200.0);
  //  p_task_gain(1,0) = 0.0;
  //  p_task_gain(2,0) = 0.0;
//  p_task_gain(3,0) = 0.0;
//  p_task_gain(4,0) = 0.0;
//  p_task_gain(5,0) = 0.0;
  p_task_gain.block(3,0,3,1).setConstant(20.0);
  p_task_gain.block(6,0,2*N_CART*N_ENDEFFS,1).setConstant(20.0);

  //  p_task_gain.block(6,0,3,1).setConstant(2.0);
  //  p_task_gain.block(12,0,3,1).setConstant(2.0);
  //  p_task_gain.block(18,0,3,1).setConstant(2.0);
  //  p_task_gain.block(24,0,3,1).setConstant(2.0);

  p_posture_gain.setConstant(N_DOFS, 1, 1.0);
  ik.setGains(p_task_gain, p_posture_gain);


  inverse_kinematics::save_robot_state("initial_data.txt", robot_state_trajectory);

  ik.computeInverseKinematics(robot_state_trajectory, delta_time);

  inverse_kinematics::save_robot_state("final_data.txt", robot_state_trajectory);
  inverse_kinematics::save_joint_positions("final_joint_positions", robot_state_trajectory.back());


  //replay what happened
  kinematicState kin_state;

  for(int i=0; i<(int)robot_state_trajectory.size(); ++i)
  {

    kin_state.base_orient = robot_state_trajectory[i].base_orientation;
    kin_state.base_state = robot_state_trajectory[i].base_position;

    for(int j=1; j<=N_DOFS; ++j)
      kin_state.joints[j] = robot_state_trajectory[i].joints[j];

    kin_state.enabled = TRUE;


    sendUserGraphics("display_kinematics", &kin_state, sizeof(kin_state));
    usleep(delta_time * 1000000);
  }

  char bla;
  printf("done\n");
  scanf("%c\n",&bla);
  kin_state.enabled = FALSE;
  sendUserGraphics("display_kinematics", &kin_state, sizeof(kin_state));


}

