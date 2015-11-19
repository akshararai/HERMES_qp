/*!=============================================================================
  ==============================================================================

  \file    cog_force_utils.cpp

  \author  righetti
  \date    Mar 8, 2013

  ==============================================================================
  \remarks


  ============================================================================*/


#include <Eigen/Eigen>

// SL system headers
#include "SL_system_headers.h"

// SL includes
#include "SL.h"
#include "SL_user.h"
#include "SL_tasks.h"
#include "SL_task_servo.h"
#include "SL_kinematics.h"
#include "SL_dynamics.h"
#include "SL_collect_data.h"
#include "SL_shared_memory.h"
#include "SL_man.h"


#include <iostream>

static void where_cop();

extern "C"{
void add_cog_force_utils();
}


void add_cog_force_utils()
{
  addToMan("where_cop", "Shows where is the CoP given foot sensors", where_cop);
}

static void where_cop()
{
  //left leg forces
  Eigen::Vector3d left_leg_force(misc_sensor[L_CFx], misc_sensor[L_CFy], misc_sensor[L_CFz]);
  Eigen::Vector3d left_leg_moment(misc_sensor[L_CTa], misc_sensor[L_CTb], misc_sensor[L_CTg]);

  Eigen::Quaterniond left_leg_orientation(cart_orient[LEFT_FOOT].q[_QW_],cart_orient[LEFT_FOOT].q[_QX_],
                                          cart_orient[LEFT_FOOT].q[_QY_],cart_orient[LEFT_FOOT].q[_QZ_]);

  //take into account the distance from sensor to endeffector
  Eigen::Vector3d left_sens_pos(FTA_Z_OFF, FTA_X_OFF, -FTA_Y_OFF);
  left_leg_moment += left_sens_pos.cross(left_leg_force);

  //right leg forces
  Eigen::Vector3d right_leg_force(misc_sensor[R_CFx], misc_sensor[R_CFy], misc_sensor[R_CFz]);
  Eigen::Vector3d right_leg_moment(misc_sensor[R_CTa], misc_sensor[R_CTb], misc_sensor[R_CTg]);

  Eigen::Quaterniond right_leg_orientation(cart_orient[RIGHT_FOOT].q[_QW_],cart_orient[RIGHT_FOOT].q[_QX_],
                                           cart_orient[RIGHT_FOOT].q[_QY_],cart_orient[RIGHT_FOOT].q[_QZ_]);

  //take into account the distance from sensor to endeffector
  Eigen::Vector3d right_sens_pos(FTA_Z_OFF, FTA_X_OFF, FTA_Y_OFF);
  right_leg_moment += right_sens_pos.cross(right_leg_force);


  //local COPs using the local coordinate frame
  double dist_to_floor = 0.0;
  Eigen::Vector3d left_leg_local_cop(0.0,
                                     -left_leg_moment(2)/left_leg_force(0),
                                     left_leg_moment(1)/left_leg_force(0));
  Eigen::Vector3d right_leg_local_cop(0.0,
                                      -right_leg_moment(2)/right_leg_force(0),
                                      right_leg_moment(1)/right_leg_force(0));

  printf("left leg local cop foot frame:  \t x:%f \t y: %f \t z: %f\n", left_leg_local_cop(0), left_leg_local_cop(1), left_leg_local_cop(2));
  printf("right leg local cop foot frame: \t x:%f \t y: %f \t z: %f\n\n", right_leg_local_cop(0), right_leg_local_cop(1), right_leg_local_cop(2));


  Eigen::Vector3d left_leg_position(cart_state[LEFT_FOOT].x[_X_],
                                    cart_state[LEFT_FOOT].x[_Y_],
                                    cart_state[LEFT_FOOT].x[_Z_]);
  Eigen::Vector3d right_leg_position(cart_state[RIGHT_FOOT].x[_X_],
                                     cart_state[RIGHT_FOOT].x[_Y_],
                                     cart_state[RIGHT_FOOT].x[_Z_]);


  //put forces in the left foot frame
  right_leg_force = left_leg_orientation.toRotationMatrix().transpose() * right_leg_orientation.toRotationMatrix()*right_leg_force;
  right_leg_moment = left_leg_orientation.toRotationMatrix().transpose() * right_leg_orientation.toRotationMatrix()*right_leg_moment;


  //moment at left foot from right foot
  Eigen::Vector3d total_moment_left = (left_leg_orientation.toRotationMatrix().transpose()*(right_leg_position-left_leg_position)).cross(right_leg_force) + left_leg_moment + right_leg_moment;
  Eigen::Vector3d total_force_left = left_leg_force + right_leg_force;
  Eigen::Vector3d total_cop = Eigen::Vector3d(0.0,
                                              -total_moment_left(2)/total_force_left(0),
                                              total_moment_left(1)/total_force_left(0));

  printf("total cop in left foot frame: \t\t x:%f \t y: %f \t z: %f\n\n", total_cop(0), total_cop(1), total_cop(2));

  //display cop and cog relative to each other
  Eigen::Vector3d com(cog.x[_X_],cog.x[_Y_], cog.x[_Z_]);
  com =  left_leg_orientation.toRotationMatrix().transpose() * (com - left_leg_position);

  printf("total com in left foot frame:  \t\t x:%f \t y: %f \t z: %f\n\n", com(0), com(1), com(2));
  printf("com dist to cop                \t\t x:%f \t y: %f \t z: %f\n\n", com(0)-total_cop(0), com(1) - total_cop(1), com(2)-total_cop(2));
}
