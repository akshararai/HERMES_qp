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
//#include "ArmReflex.hpp"

/* I try to add headers by myself one by one but doesnt work out*/
//#include "SL_user.h"
//#include "SL_collect_data.h"
//#include "SL_integrate.h"

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

#include "ArmReflex.hpp"    // I must put my class's header here to get it compiled without conflicting with SL stuffs

using namespace std;
using namespace Eigen;
using namespace floating_base_utilities;
using namespace momentum_balance_control;

namespace wholebody_demo
{
    class mytest
    {
    public:
        mytest();
        ~mytest();

        ArmReflex Arm;

        int run();

    private:
        // we will use a config file to tune parameters conveniently
        string config_file_;

        // We have upper and lower torque limit constraints
        static const int Max_Ineq_Rows = 2*N_DOFS;
        // We will do momentum control (6 rows) and constrain
        // the feet not to move (6 each)
        static const int Max_Eq_Rows = 6+2*6+N_DOFS+6+2*6;

        // this is to set endeffectors as un-/constrained
        SL_endeff endeff_constraints_[N_ENDEFFS+1];

        // our helper classes to construct various quanitties with eigen
        KinematicsEigen kinematics_;
        FloatingBaseKinematics endeff_kinematics_;
        MomentumComputation momentum_helper_;
        FootContactHandlerHermes contact_helper_;

        // the hierarchical inverse dynamics solver and task composers
        HierarchInverseDynamics<Max_Ineq_Rows, Max_Eq_Rows> hinvdyn_solver_;

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
        double sampleT;
        double push_force, push_duration, push_time;

        // some info about task
        double task_start_time;
        double real_time;
        int Num_loop;
        bool isFall;
        double stability_margin;

        double cog_kp; //=500.0;
        double cog_kd; //=50.0;

        double reflex_mag;
        double reflex_time;

        double m_yaw;
        double m_roll;
        double m_pitch;
        double m_dpitch;

        double torso_des;
        double torso_pitch_w;        double torso_pitch_th;

        double torso_pitch_kp; //=500.0;
        double torso_pitch_kd;


        void getEuler();
        void attitudeControl();
        /* vectors */
        Eigen::Matrix<double, N_DOFS,1> init_joint_state_uff_, init_joint_state_th_, init_joint_state_thd_, init_joint_state_thdd_;
    };

}  // Namespace
