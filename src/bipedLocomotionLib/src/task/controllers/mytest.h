/*********************************************************************
 Autonomous Motion Department
 Max-Planck Intelligent Systems
 Prof. Stefan Schaal

 *********************************************************************
 \remarks      I modify from Alexander Herzog's example code

 \file         mytest.h

 \author       Zhibin Li
 \date         Dec 1, 2015

 *********************************************************************/

#pragma once

#include <string>
#include <Eigen/Eigen>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include "DynamicsConstraint.h"
#include "CartesianPositionCtrl.h"
#include "CartesianForceCtrl.h"
#include "MomentumRateCtrl.h"
#include "JointSpaceCtrl.h"
#include "HierarchInverseDynamics.h"
#include "KinematicsEigen.h"
#include "MomentumComputation.h"
#include "FloatingBaseKinematics.h"
#include "FootContactHandlerHermes.h"

namespace hierarchical_inverse_dynamics_example
{
    class mytest {
     public:
      mytest();
      ~mytest();

      int run();

     private:
      // we will use a config file to tune parameters conveniently
      std::string config_file_;

      // this is for PD control
      Eigen::Vector3d cog_des_, cog_p_gains_, cog_d_gains_;
    //  Eigen::Matrix<double, 6, 1> cog_ref_;
    //  Eigen::Matrix<double, N_DOFS+6,1> default_posture_;
    //  Eigen::Matrix<double, N_DOFS+6,1> posture_p_gains_, posture_d_gains_;

    //  // weights
    //  Eigen::Matrix<double, 6, 1> foot_constr_weight_;
    //  Eigen::Matrix<double, 6, 1> frc_reg_weight_;
    //  Eigen::Matrix<double, 6, 1> cog_ctrl_weight_;
    //  Eigen::Matrix<double, N_DOFS+6, 1> joint_ctrl_weight_;

      // parameters for the push simulation
      double push_force, push_duration;

      // some info about task
      double task_start_time_;
      Eigen::Matrix<double, N_DOFS,1> init_joint_state_uff_, init_joint_state_th_, init_joint_state_thd_, init_joint_state_thdd_;
    };

}  // Namespace
