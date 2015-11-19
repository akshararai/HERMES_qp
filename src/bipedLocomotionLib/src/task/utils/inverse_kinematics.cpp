/*
 * inverseKinematics.cpp
 *
 *  Created on: Feb 22, 2012
 *      Author: righetti
 */

#include <iostream>
#include <fstream>

#include "inverse_kinematics.h"


#include "quaternions.h"
#include "SL_common.h"
#include <utility_macros.h>


namespace inverse_kinematics
{

InverseKinematics::InverseKinematics()
{
  integration_step_ = 0.001; //1ms default

  p_task_gain_.setConstant(1.0);
  p_posture_gain_.setConstant(1.0);

  use_com_ = false;
}


InverseKinematics::InverseKinematics(const InverseKinematics& copy)
{
  *this = copy;
}

InverseKinematics& InverseKinematics::operator=( const InverseKinematics& copy)
{
  integration_step_ =copy.integration_step_;

  use_com_ = copy.use_com_;

  p_task_gain_  = copy.p_task_gain_;
  p_posture_gain_ = copy.p_posture_gain_;

  //setup the basic inverse dynamics
  floating_base_kinematics_ = copy.floating_base_kinematics_;

  return *this;
}



InverseKinematics::~InverseKinematics()
{
}

void InverseKinematics::setGains(Eigen::Matrix<double, 2*N_CART*(N_ENDEFFS+1), 1> p_task_gain,
                                 Eigen::Matrix<double, N_DOFS, 1> p_posture_gain)
{
  p_task_gain_ = p_task_gain;
  p_posture_gain_ = p_posture_gain;
}


bool InverseKinematics::computeInverseKinematics(std::vector<RobotState>& robot_state, double delta_time)
{
  //  save_robot_state("desired_robot_state.txt", robot_state);
  int internal_ik_steps = int(delta_time/integration_step_);

  kinematics_eigen_.initialize(&(robot_state[0].joints[0]),
                               robot_state[0].base_position,
                               robot_state[0].base_orientation,
                               &(robot_state[0].is_constrained[0]));
  momentum_helper_.initialize();

  //for each way point
  for(int i = 1; i<(int)robot_state.size(); ++i)
  {
    RobotState current_robot_state = robot_state[i-1];

    //compute the IK
    RobotState intermediate_robot_state = robot_state[i];
    for(int j=1; j<=internal_ik_steps; j++)
    {
      //interpolate for each cartesian element
      for(int k=0; k<(int)robot_state[i].constraint_position.size(); ++k)
      {
        for(int l=1; l<=N_CART; ++l)
        {
          intermediate_robot_state.constraint_position[k].x[l] = robot_state[i-1].constraint_position[k].x[l] +
              j*(robot_state[i].constraint_position[k].x[l] - robot_state[i-1].constraint_position[k].x[l])/double(internal_ik_steps);

          intermediate_robot_state.constraint_position[k].xd[l] = robot_state[i-1].constraint_position[k].xd[l] +
              j*(robot_state[i].constraint_position[k].xd[l] - robot_state[i-1].constraint_position[k].xd[l])/double(internal_ik_steps);

          intermediate_robot_state.constraint_orientation[k].ad[l] = robot_state[i-1].constraint_orientation[k].ad[l] +
              j*(robot_state[i].constraint_orientation[k].ad[l] - robot_state[i-1].constraint_orientation[k].ad[l])/double(internal_ik_steps);
        }
        for(int l=1; l<=4; ++l)
        {
          intermediate_robot_state.constraint_orientation[k].q[l] = robot_state[i-1].constraint_orientation[k].q[l] +
              j*(robot_state[i].constraint_orientation[k].q[l] - robot_state[i-1].constraint_orientation[k].q[l])/double(internal_ik_steps);
        }

      }
      inverseKinematicsStepQP(intermediate_robot_state, current_robot_state);
    }

    //update the actual motion
    std::vector<SL_endeff> constraint = robot_state[i].is_constrained;
    robot_state[i] = current_robot_state;
    robot_state[i].is_constrained = constraint;
  }

  //  save_robot_state("actual_robot_state.txt", robot_state);

  return true;
}


bool InverseKinematics::inverseKinematicsStepQP(RobotState& desired_robot_state, RobotState& current_robot_state)
{
  //compute the constrained Jacobian and its nullspace and the free end effector Jacobian
  int num_constraints;
  floating_base_kinematics_.computeJacobians(&current_robot_state.joints[0], current_robot_state.constraint_position[BASE],
                                             current_robot_state.constraint_orientation[BASE], &desired_robot_state.is_constrained[0],
                                             num_constraints);

  floating_base_kinematics_.getConstrainedJacobian(constrained_endeff_jacobian_);
  floating_base_kinematics_.getUnconstrainedJacobian(unconstrained_endeff_jacobian_);


  //construct the task Jacobian (base + endeff)
  task_size_ = 2*N_CART*(N_ENDEFFS+1) - num_constraints;
  task_jacobian_.setZero(task_size_, N_DOFS+6);

  //the base part
  if(use_com_)
  {
    kinematics_eigen_.update(&(current_robot_state.joints[0]),
                             current_robot_state.base_position,
                             current_robot_state.base_orientation,
                             &(current_robot_state.is_constrained[0]));
    momentum_helper_.update(kinematics_eigen_);
    task_jacobian_.block<3, N_DOFS+6>(0,0) = momentum_helper_.getCentroidalMomentumMatrix().topRows<3>();
    task_jacobian_.block<3,3>(3,N_DOFS+3) = Eigen::Matrix3d::Identity();
  }
  else
  {
    task_jacobian_.block<6,6>(0, N_DOFS) = Eigen::Matrix<double, 6, 6>::Identity();
  }


  //for each possibly constraint endeffector fill in the jacobian and compute the task error
  int constraint_counter = 0;
  for(int i=1; i<=N_ENDEFFS; ++i)
  {
    for(int j=1; j<=N_CART; ++j)
      if(desired_robot_state.is_constrained[i].c[j] == 0)
      {
        task_jacobian_.block(constraint_counter+6, 0, 1, N_DOFS+6) = unconstrained_endeff_jacobian_.block(constraint_counter,0,1,N_DOFS+6);
        ++constraint_counter;
      }
    //orientation constraint is 3 dim.
    if(desired_robot_state.is_constrained[i].c[N_CART+1] == 0)
    {
      task_jacobian_.block(constraint_counter+6, 0, 3, N_DOFS+6) = unconstrained_endeff_jacobian_.block(constraint_counter,0,3,N_DOFS+6);
      constraint_counter +=3;
    }
  }


  //compute pseudo inverse and nullspace
  computeTaskPseudoInverse(task_jacobian_, pseudo_inverse_, task_size_, true, task_nullspace_);


  //compute the task error
  Eigen::Matrix<double, Eigen::Dynamic, 1> desired_velocity;
  desired_velocity.setZero(task_size_, 1);

  constraint_counter = 0;
  for(int i=0; i<=N_ENDEFFS; ++i) //this time we count from 0
  {
    for(int j=1; j<=N_CART; ++j)
      if(desired_robot_state.is_constrained[i].c[j] == 0)
      {
        desired_velocity(constraint_counter,0) +=
            p_task_gain_(2*N_CART*i+j-1,0) *
            (desired_robot_state.constraint_position[i].x[j] - current_robot_state.constraint_position[i].x[j]) +
            desired_robot_state.constraint_position[i].xd[j];
        ++constraint_counter;
      }
    //orientation constraint is 3 dim.
    if(desired_robot_state.is_constrained[i].c[N_CART+1] == 0)
    {
      MY_VECTOR(cart_orient_error, 1, N_CART);
      fixQuaternionSign(current_robot_state.constraint_orientation[i], desired_robot_state.constraint_orientation[i]);
      quatErrorVector(desired_robot_state.constraint_orientation[i].q, current_robot_state.constraint_orientation[i].q, cart_orient_error);
      desired_velocity.block(constraint_counter,0,3,1) +=
          Eigen::Map<Eigen::Matrix<double, 3, 1> >(&desired_robot_state.constraint_orientation[i].ad[1]) -
          p_task_gain_.block(2*N_CART*i+3,0,3,1).asDiagonal() * Eigen::Map<Eigen::Matrix<double, 3, 1> >(&cart_orient_error[1]);
      constraint_counter +=3;
    }
  }
  //
  //  std::cout << desired_velocity << std::endl << std::endl;
  //    std::cin >> constraint_counter;

  //compute the nullspace stuff
  Eigen::Matrix<double, N_DOFS+6, 1> nullspace_velocity;
  nullspace_velocity.setZero(N_DOFS+6, 1);
  for(int i=0; i<N_DOFS; ++i)
    nullspace_velocity(i,0) = p_posture_gain_(i) * (desired_robot_state.joints[i+1].th - current_robot_state.joints[i+1].th);

  //QP stuff
  //  int nc = 2*N_CART;
  //  if(swing_foot_==0)
  //  {
  //    nc += 2*N_CART;
  //  }
  Eigen::MatrixXd hessian;
  Eigen::VectorXd gradient_vector;
  Eigen::MatrixXd constraint_eq_matrix;
  Eigen::Matrix<double, N_DOFS+6, 2*N_DOFS> constraint_ineq_matrix;
  Eigen::VectorXd constraint_eq_vector;
  Eigen::Matrix<double, 2*N_DOFS, 1> constraint_ineq_vector;

  //compute the hessian matrix
  hessian = task_nullspace_ + task_jacobian_.transpose()*task_jacobian_;
  //compute the gradient vector
  gradient_vector = - (task_nullspace_ * nullspace_velocity + task_jacobian_.transpose() * desired_velocity);
  //set constraint matrix
  constraint_ineq_matrix.setZero();
  constraint_ineq_matrix.block<N_DOFS,N_DOFS>(0,0) = -1.0 * Eigen::Matrix<double, N_DOFS, N_DOFS>::Identity();
  constraint_ineq_matrix.block<N_DOFS,N_DOFS>(0,N_DOFS) = Eigen::Matrix<double, N_DOFS, N_DOFS>::Identity();;
  constraint_eq_matrix = constrained_endeff_jacobian_.topRows(num_constraints).transpose();
  //set constraint vector
  constraint_eq_vector.setZero(num_constraints);
  //set bounds
  for(int i=0; i<N_DOFS; ++i)
  {
    constraint_ineq_vector(i) = -100*(0.5*tanh(50*(current_robot_state.joints[i+1].th+0.1-joint_range[i+1][MAX_THETA]))-0.5);
    constraint_ineq_vector(i+N_DOFS) = 100*(0.5*tanh(50*(current_robot_state.joints[i+1].th-0.1-joint_range[i+1][MIN_THETA]))+0.5);
  }
  Eigen::VectorXd full_base_velocity;

  Eigen::solve_quadprog(hessian, gradient_vector, constraint_eq_matrix,
                        constraint_eq_vector, constraint_ineq_matrix,
                        constraint_ineq_vector, full_base_velocity);


  //update the desired joint states
  for(int i=0; i<N_DOFS; ++i)
  {
    current_robot_state.joints[i+1].th += full_base_velocity[i]*integration_step_; //we integrate
    current_robot_state.joints[i+1].thdd = (full_base_velocity[i] - current_robot_state.joints[i+1].thd)/integration_step_; //we derive
    current_robot_state.joints[i+1].thd = full_base_velocity[i];//we get the current state
  }


  //update the base
  for(int i=0; i<N_CART; ++i)
  {
    current_robot_state.base_position.xdd[i+1] = (full_base_velocity[i+N_DOFS] - current_robot_state.base_position.xd[i+1])/integration_step_; //we derive
    current_robot_state.base_position.xd[i+1] = full_base_velocity[i+N_DOFS];//we get the current state
    current_robot_state.base_position.x[i+1] += full_base_velocity[i+N_DOFS]*integration_step_; //we integrate

    current_robot_state.base_orientation.add[i+1] = (full_base_velocity[i+N_DOFS+3] - current_robot_state.base_orientation.ad[i+1])/integration_step_; //we derive
    current_robot_state.base_orientation.ad[i+1] = full_base_velocity[i+N_DOFS+3]; //we get the current state
  }
  //we compute the resulting quaternion (need to get the quaternion velocity and then integrate)
  quatDerivatives(&current_robot_state.base_orientation); //we update the quaternion velocity
  current_robot_state.base_orientation.q[_Q0_] += current_robot_state.base_orientation.qd[_Q0_]*integration_step_;
  current_robot_state.base_orientation.q[_Q1_] += current_robot_state.base_orientation.qd[_Q1_]*integration_step_;
  current_robot_state.base_orientation.q[_Q2_] += current_robot_state.base_orientation.qd[_Q2_]*integration_step_;
  current_robot_state.base_orientation.q[_Q3_] += current_robot_state.base_orientation.qd[_Q3_]*integration_step_;

  normalizeQuaternion(current_robot_state.base_orientation);

  kinematics_eigen_.update(&(current_robot_state.joints[0]),
                           current_robot_state.base_position,
                           current_robot_state.base_orientation,
                           &(current_robot_state.is_constrained[0]));

  if(use_com_)
  {
    current_robot_state.constraint_orientation[0] = current_robot_state.base_orientation;
    Eigen::Map<Eigen::Vector3d>(&(current_robot_state.constraint_position[0].x[1])) = kinematics_eigen_.cog();
    momentum_helper_.update(kinematics_eigen_);
    Eigen::Vector3d prev_cog = Eigen::Map<Eigen::Vector3d>(&(current_robot_state.constraint_position[0].xdd[1]));
    Eigen::Map<Eigen::Vector3d>(&(current_robot_state.constraint_position[0].xd[1])) = momentum_helper_.getdCog();
    Eigen::Map<Eigen::Vector3d>(&(current_robot_state.constraint_position[0].xdd[1])) =
        (1/integration_step_) * (Eigen::Map<Eigen::Vector3d>(&(current_robot_state.constraint_position[0].xd[1])) - prev_cog);
  }
  else
  {
    current_robot_state.constraint_position[0] = current_robot_state.base_position;
    current_robot_state.constraint_orientation[0] = current_robot_state.base_orientation;
  }

  MY_MATRIX(Xmcog,0,N_DOFS,1,3);
  MY_MATRIX(Xaxis,0,N_DOFS,1,3);
  MY_MATRIX(Xorigin,0,N_DOFS,1,3);
  MY_MATRIX(Xlink,0,N_LINKS,1,3);
  MY_MATRIX_ARRAY(Ahmat,1,4,1,4,N_LINKS);
  MY_MATRIX_ARRAY(Adofs,1,4,1,4,N_DOFS);
  linkInformation(&current_robot_state.joints[0], &current_robot_state.base_position,
                  &current_robot_state.base_orientation, &(desired_robot_state.is_constrained[0]),
                  Xmcog, Xaxis, Xorigin, Xlink, Ahmat,Adofs);

  //update the new foot positions
  desired_velocity = task_jacobian_ * full_base_velocity;
  constraint_counter = 0;
  for(int i=1; i<=N_ENDEFFS; ++i)
  {
    for(int j=1; j<=N_CART; ++j)
    {
      current_robot_state.constraint_position[i].x[j] = Xlink[link2endeffmap[i]][j];
      if(desired_robot_state.is_constrained[i].c[j] == 0)
      {
        current_robot_state.constraint_position[i].xd[j] = desired_velocity[constraint_counter];
        ++constraint_counter;
      }
      else
      {
        current_robot_state.constraint_position[i].xd[j] = 0.0;
      }
    }
    linkQuat(Ahmat[link2endeffmap[i]], &current_robot_state.constraint_orientation[i]);
    for(int j=1; j<=N_CART; ++j)
    {
      if(desired_robot_state.is_constrained[i].c[N_CART+1] == 0)
      {
        current_robot_state.constraint_orientation[i].ad[j] = desired_velocity[constraint_counter];
        ++constraint_counter;
      }
      else
        current_robot_state.constraint_orientation[i].ad[j] = 0.0;
    }
  }

  return true;
}

void InverseKinematics::computeTaskPseudoInverse(Eigen::Matrix<double, Eigen::Dynamic, N_DOFS+6>& input_matrix,
                                                 Eigen::Matrix<double, N_DOFS+6, Eigen::Dynamic>& pseudo_inverse,
                                                 int max_rank, bool compute_nullspace,
                                                 Eigen::Matrix<double, N_DOFS+6, N_DOFS+6>& nullspace)
{
  Eigen::JacobiSVD<Eigen::Matrix<double, Eigen::Dynamic, N_DOFS+6> > svd(input_matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);

  int num_rows = input_matrix.rows();


  Eigen::Matrix<double, N_DOFS+6, Eigen::Dynamic> sigma;
  sigma.setZero(N_DOFS+6, num_rows);


  for(int i=0; i<max_rank; ++i)
  {
    double max_sv = svd.singularValues()(0,0);
    double sv = svd.singularValues()(i,0);
    if((sv + 1.e-10) / max_sv < 1.0/condition_nb_cutoff_)
    {
      //      printf("Warning reached singularity - cond number %f\n",(sv + 1.e-10) / max_sv);
      double sc = max_sv / condition_nb_cutoff_;
      sigma(i,i) = -2.0 * (sv * sv) / (sc*sc*sc) + 3.0 * sv / (sc*sc);
    }
    else
    {
      sigma(i,i) = 1.0/sv;
    }
  }
  pseudo_inverse = svd.matrixV() * sigma * svd.matrixU().transpose();

  if(compute_nullspace)
  {
    //compute the nullspace as N = Nc - V [I 0; 0 0] V^T
    Eigen::Matrix<double, N_DOFS+6, N_DOFS+6> partial_id;
    partial_id.setZero(N_DOFS+6, N_DOFS+6);
    partial_id.block(0, 0, max_rank, max_rank).setIdentity(max_rank, max_rank);
    nullspace = Eigen::MatrixXd::Identity(N_DOFS+6,N_DOFS+6) - svd.matrixV() * partial_id * svd.matrixV().transpose();
  }
}

void save_joint_positions(const char* file_name, const inverse_kinematics::RobotState& robot_state)
{
  std::ofstream file;
  file.open(file_name);
  for(int j=1; j<=N_DOFS; ++j)
  {
    file << robot_state.joints[j].th << " ";
  }
  file << std::endl;
  file.close();
}

void read_robot_state(const char* file_name, inverse_kinematics::RobotState& robot_state)
{
  std::ifstream file;
  file.open(file_name);

  for(int j=0; j<=N_ENDEFFS; ++j)
  {

    file >> robot_state.constraint_position[j].x[_X_] >>
    robot_state.constraint_position[j].x[_Y_] >>
    robot_state.constraint_position[j].x[_Z_] >>
    robot_state.constraint_position[j].xd[_X_] >>
    robot_state.constraint_position[j].xd[_Y_] >>
    robot_state.constraint_position[j].xd[_Z_] >>
    robot_state.constraint_position[j].xdd[_X_] >>
    robot_state.constraint_position[j].xdd[_Y_] >>
    robot_state.constraint_position[j].xdd[_Z_];



    file >>
    robot_state.constraint_orientation[j].q[_Q0_] >>
    robot_state.constraint_orientation[j].q[_Q1_] >>
    robot_state.constraint_orientation[j].q[_Q2_] >>
    robot_state.constraint_orientation[j].q[_Q3_] >>
    robot_state.constraint_orientation[j].ad[_X_] >>
    robot_state.constraint_orientation[j].ad[_Y_] >>
    robot_state.constraint_orientation[j].ad[_Z_] >>
    robot_state.constraint_orientation[j].add[_X_] >>
    robot_state.constraint_orientation[j].add[_Y_] >>
    robot_state.constraint_orientation[j].add[_Z_];
  }

  //base state
  file >>
  robot_state.base_position.x[_X_] >>
  robot_state.base_position.x[_Y_]  >>
  robot_state.base_position.x[_Z_]  >>
  robot_state.base_position.xd[_X_] >>
  robot_state.base_position.xd[_Y_] >>
  robot_state.base_position.xd[_Z_] >>
  robot_state.base_position.xdd[_X_] >>
  robot_state.base_position.xdd[_Y_] >>
  robot_state.base_position.xdd[_Z_];

  file >>
  robot_state.base_orientation.q[_Q0_] >>
  robot_state.base_orientation.q[_Q1_] >>
  robot_state.base_orientation.q[_Q2_] >>
  robot_state.base_orientation.q[_Q3_] >>
  robot_state.base_orientation.ad[_X_] >>
  robot_state.base_orientation.ad[_Y_] >>
  robot_state.base_orientation.ad[_Z_] >>
  robot_state.base_orientation.add[_X_] >>
  robot_state.base_orientation.add[_Y_] >>
  robot_state.base_orientation.add[_Z_];

  for(int j=1; j<=N_DOFS; ++j)
  {
    file >> robot_state.joints[j].th;
    file >> robot_state.joints[j].thd;
    file >> robot_state.joints[j].thdd;
    file >> robot_state.joints[j].u;
  }

  for(int j=1; j<=N_ENDEFFS; ++j)
  {
    file >> robot_state.is_constrained[j].m;
    file >> robot_state.is_constrained[j].mcm[_X_] >>
    robot_state.is_constrained[j].mcm[_Y_] >>
    robot_state.is_constrained[j].mcm[_Z_];
    file >> robot_state.is_constrained[j].a[_X_] >>
    robot_state.is_constrained[j].a[_Y_] >>
    robot_state.is_constrained[j].a[_Z_];
    file >> robot_state.is_constrained[j].cf[_X_] >>
    robot_state.is_constrained[j].cf[_Y_] >>
    robot_state.is_constrained[j].cf[_Z_];
    file >> robot_state.is_constrained[j].ct[_X_] >>
    robot_state.is_constrained[j].ct[_Y_] >>
    robot_state.is_constrained[j].ct[_Z_];
    file >> robot_state.is_constrained[j].x[_X_] >>
    robot_state.is_constrained[j].x[_Y_] >>
    robot_state.is_constrained[j].x[_Z_];
    file >>
    robot_state.is_constrained[j].c[1] >>
    robot_state.is_constrained[j].c[2] >>
    robot_state.is_constrained[j].c[3] >>
    robot_state.is_constrained[j].c[4] >>
    robot_state.is_constrained[j].c[5] >>
    robot_state.is_constrained[j].c[6];
  }

  file.close();
}

void create_robot_state(SL_Cstate& basec, SL_quat& baseo, SL_Cstate* cart_state, SL_quat* cart_orient, SL_Jstate* jstate, SL_endeff* endeff, RobotState& robot_state)
{
  robot_state.constraint_position.resize(N_ENDEFFS+1);
  robot_state.constraint_orientation.resize(N_ENDEFFS+1);
  robot_state.is_constrained.resize(N_ENDEFFS+1);
  robot_state.joints.resize(N_DOFS+1);

  robot_state.base_position = base_state;
  robot_state.base_orientation = base_orient;

  for(int i=1; i<=N_DOFS; ++i)
    robot_state.joints[i] = joint_state[i];


  for(int i=0; i<=N_ENDEFFS; ++i)
  {
    robot_state.is_constrained[i] = endeff[i];
    robot_state.constraint_position[i] = cart_state[i];
    robot_state.constraint_orientation[i] = cart_orient[i];
  }
}

void save_robot_state(const char* file_name, const RobotState& robot_state)
{
  std::vector<inverse_kinematics::RobotState> r_state;
  r_state.push_back(robot_state);
  save_robot_state(file_name, r_state);
}

void save_robot_state(const char* file_name, const std::vector<inverse_kinematics::RobotState>& robot_state)
{

  std::ofstream file;
  file.open(file_name);

  for(int i=0; i<(int)robot_state.size(); ++i)
  {

    for(int j=0; j<=N_ENDEFFS; ++j)
    {

      file << robot_state[i].constraint_position[j].x[_X_] << " "
          << robot_state[i].constraint_position[j].x[_Y_] << " "
          << robot_state[i].constraint_position[j].x[_Z_] << " "
          << robot_state[i].constraint_position[j].xd[_X_] << " "
          << robot_state[i].constraint_position[j].xd[_Y_] << " "
          << robot_state[i].constraint_position[j].xd[_Z_] << " "
          << robot_state[i].constraint_position[j].xdd[_X_] << " "
          << robot_state[i].constraint_position[j].xdd[_Y_] << " "
          << robot_state[i].constraint_position[j].xdd[_Z_] << " ";



      file << robot_state[i].constraint_orientation[j].q[_Q0_] << " "
          << robot_state[i].constraint_orientation[j].q[_Q1_] << " "
          << robot_state[i].constraint_orientation[j].q[_Q2_] << " "
          << robot_state[i].constraint_orientation[j].q[_Q3_] << " "
          << robot_state[i].constraint_orientation[j].ad[_X_] << " "
          << robot_state[i].constraint_orientation[j].ad[_Y_] << " "
          << robot_state[i].constraint_orientation[j].ad[_Z_] << " "
          << robot_state[i].constraint_orientation[j].add[_X_] << " "
          << robot_state[i].constraint_orientation[j].add[_Y_] << " "
          << robot_state[i].constraint_orientation[j].add[_Z_] << " ";
    }

    //base state
    file << robot_state[i].base_position.x[_X_] << " "
        << robot_state[i].base_position.x[_Y_]  << " "
        << robot_state[i].base_position.x[_Z_]  << " "
        << robot_state[i].base_position.xd[_X_] << " "
        << robot_state[i].base_position.xd[_Y_] << " "
        << robot_state[i].base_position.xd[_Z_] << " "
        << robot_state[i].base_position.xdd[_X_] << " "
        << robot_state[i].base_position.xdd[_Y_] << " "
        << robot_state[i].base_position.xdd[_Z_] << " ";

    file << robot_state[i].base_orientation.q[_Q0_] << " "
        << robot_state[i].base_orientation.q[_Q1_] << " "
        << robot_state[i].base_orientation.q[_Q2_] << " "
        << robot_state[i].base_orientation.q[_Q3_] << " "
        << robot_state[i].base_orientation.ad[_X_] << " "
        << robot_state[i].base_orientation.ad[_Y_] << " "
        << robot_state[i].base_orientation.ad[_Z_] << " "
        << robot_state[i].base_orientation.add[_X_] << " "
        << robot_state[i].base_orientation.add[_Y_] << " "
        << robot_state[i].base_orientation.add[_Z_] << " ";


    //joints
    for(int j=1; j<=N_DOFS; ++j)
    {
      file << robot_state[i].joints[j].th << " ";
      file << robot_state[i].joints[j].thd << " ";
      file << robot_state[i].joints[j].thdd << " ";
      file << robot_state[i].joints[j].u << " ";
    }

    for(int j=1; j<=N_ENDEFFS; ++j)
    {
      file << robot_state[i].is_constrained[j].m << " ";
      file <<
          robot_state[i].is_constrained[j].mcm[_X_] << " " <<
          robot_state[i].is_constrained[j].mcm[_Y_] << " " <<
          robot_state[i].is_constrained[j].mcm[_Z_] << " ";
      file <<
          robot_state[i].is_constrained[j].a[_X_] << " " <<
          robot_state[i].is_constrained[j].a[_Y_] << " " <<
          robot_state[i].is_constrained[j].a[_Z_] << " ";
      file <<
          robot_state[i].is_constrained[j].cf[_X_] << " " <<
          robot_state[i].is_constrained[j].cf[_Y_] << " " <<
          robot_state[i].is_constrained[j].cf[_Z_] << " ";
      file <<
          robot_state[i].is_constrained[j].ct[_X_] << " " <<
          robot_state[i].is_constrained[j].ct[_Y_] << " " <<
          robot_state[i].is_constrained[j].ct[_Z_] << " ";
      file <<
          robot_state[i].is_constrained[j].x[_X_] << " " <<
          robot_state[i].is_constrained[j].x[_Y_] << " " <<
          robot_state[i].is_constrained[j].x[_Z_] << " ";
      file <<
          robot_state[i].is_constrained[j].c[1] << " " <<
          robot_state[i].is_constrained[j].c[2] << " " <<
          robot_state[i].is_constrained[j].c[3] << " " <<
          robot_state[i].is_constrained[j].c[4] << " " <<
          robot_state[i].is_constrained[j].c[5] << " " <<
          robot_state[i].is_constrained[j].c[6] << " ";
    }

    file << std::endl;
  }
  printf("done saving\n");
  file.close();
}

} /* namespace inverse_kinematics */
