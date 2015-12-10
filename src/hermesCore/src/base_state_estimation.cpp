/*
 * base_state_estimation.cpp
 *
 *  Created on: Mar 7, 2013
 *      Author: herzog
 */

#include <string.h>
#include "base_state_estimation.h"

#include "SL.h"
#include <SL_user.h>
#include <SL_common.h>
#include "SL_task_servo.h"
#include <utility_macros.h>

BaseStateEstimation::BaseStateEstimation()
{
	I_rot_B_ = my_matrix(1,3,1,3);
	I_linpos_B_ = my_vector(1,3);
	O_rot_I_= my_matrix(1,3,1,3);
	O_angrate_I_ = my_vector(1,3);
	O_linacc_I_ = my_vector(1,3);
	O_angacc_I_ = my_vector(1,3);
	O_linvel_I_ = my_vector(1,3);
	O_linpos_I_ = my_vector(1,3);
	O_rot_B_= my_matrix(1,3,1,3);
	O_linacc_B_ = my_vector(1,3);
	O_linvel_B_ = my_vector(1,3);
	O_linpos_B_ = my_vector(1,3);
	prev_O_angrate_I_ = my_vector(1,3);
//	gravity_ = my_vector(1,3);

	for(int i=1; i<=3; i++)
	{
	  prev_O_angrate_I_[i] =  I_linpos_B_[i] = O_linvel_I_[i] = O_linpos_I_[i] =
			  O_linacc_B_[i] = O_linvel_B_[i] =  O_linpos_B_[i] = 0.0;
	}
	leakage_factor_ = 0.0;
	update_rate_ = 1000;
}
BaseStateEstimation::~BaseStateEstimation()
{
	 my_free_matrix(I_rot_B_,1,3,1,3);
	 my_free_vector(I_linpos_B_,1,3);
	my_free_matrix(O_rot_I_,1,3,1,3);
	 my_free_vector(O_angrate_I_,1,3);
	 my_free_vector(O_linacc_I_,1,3);
	 my_free_vector(O_angacc_I_,1,3);
	 my_free_vector(O_linvel_I_,1,3);
	 my_free_vector(O_linpos_I_,1,3);
	my_free_matrix(O_rot_B_,1,3,1,3);
	 my_free_vector(O_linacc_B_,1,3);
	 my_free_vector(O_linvel_B_,1,3);
	 my_free_vector(O_linpos_B_,1,3);
	 my_free_vector(prev_O_angrate_I_,1,3);
//	 my_free_vector(gravity_,1,3);
}

void BaseStateEstimation::initialize(const double* imu_quaternion,
	      const double* imu_angrate,
	      const double* imu_linacc, int update_rate)
{
	update_rate_ = update_rate;
	imu_quaternion_ = imu_quaternion;
	imu_angrate_ = imu_angrate;
	imu_linacc_ = imu_linacc;

  double row_first_mat[10];
  read_parameter_pool_double_array("bs_est_config.cf", "IMU_BASE_ROT",
		  N_CART*N_CART, row_first_mat);
  for(int i=0;i<9;i++)
	  I_rot_B_[i/3+1][i%3+1] = row_first_mat[i+1];

  read_parameter_pool_double_array("bs_est_config.cf", "IMU_BASE_TRANS",
		  N_CART, I_linpos_B_);
//  read_parameter_pool_double_array("bs_est_config.cf", "MISC_SENSOR_GRAV",
//		  N_CART, gravity_);
  read_parameter_pool_double("bs_est_config.cf", "LEAKAGE_FACTOR",
		  &leakage_factor_);
}

void BaseStateEstimation::update(SL_quat& base_orientation, SL_Cstate& base_position)
{
	SL_quat orient;
	memcpy(&(orient.q[1]), imu_quaternion_, 4*sizeof(double));
	MY_MATRIX(quat_to_rot_helper, 1,3,1,3);
	quatToRotMat(&orient, quat_to_rot_helper);
	for(int r=1; r<=3; r++)
	  for(int c=1; c<=3; c++)
	      O_rot_I_[r][c] = quat_to_rot_helper[c][r];

	memcpy(&(O_angrate_I_[1]), imu_angrate_, 3*sizeof(double));
//	memcpy(&(O_linvel_I_[1]), imu_linvel_, 3*sizeof(double));
	memcpy(&(O_linacc_I_[1]), imu_linacc_, 3*sizeof(double));

	// compensate for gravity
//	for(int i=1; i<=3; i++)
//	  O_linacc_I_[i] -= gravity_[i];

	// do numerical integration
	for(int i=1; i<=3; i++)
	{
		//integrate
		O_angacc_I_[i] = (double)update_rate_*(O_angrate_I_[i] - prev_O_angrate_I_[i]);
//		O_linpos_I_[i] += (1.0/(double)update_rate_)*O_linvel_I_[i];

		//apply leakage term
//		O_linvel_I_[i] *= leakage_factor_;
//		O_linvel_I_[i] += (1.0/(double)update_rate_)*O_linacc_I_[i];
	}

	// do coordinate transformation
	MY_MATRIX(S_angrate, 1,3,1,3);
	vectorToSkewMatrix(O_angrate_I_, S_angrate);
	MY_MATRIX(S_angacc, 1,3,1,3);
	vectorToSkewMatrix(O_angacc_I_, S_angacc);
	MY_MATRIX(S_helper, 1,3,1,3);
	mat_mult(S_angrate, S_angrate, S_helper);
	mat_add(S_angacc, S_helper, S_helper);

	mat_mult(O_rot_I_, I_rot_B_, O_rot_B_);

//	mat_vec_mult(O_rot_I_, I_linpos_B_, O_linpos_B_);
//	vec_add(O_linpos_I_, O_linpos_B_, O_linpos_B_);
//
//	mat_vec_mult(O_rot_I_, I_linpos_B_, O_linvel_B_);
//	mat_vec_mult(S_angrate, O_linvel_B_, O_linvel_B_);
//	vec_add(O_linvel_I_, O_linvel_B_, O_linvel_B_);

	mat_vec_mult(O_rot_I_, I_linpos_B_, O_linacc_B_);
	mat_vec_mult(S_helper, O_linacc_B_, O_linacc_B_);
	vec_add(O_linacc_I_, O_linacc_B_, O_linacc_B_);

	// integrate base state
	for(int i=1; i<=3; i++)
	{
		//integrate
		O_linpos_B_[i] += (1.0/(double)update_rate_)*O_linvel_B_[i];

		//apply leakage term
		O_linvel_B_[i] *= leakage_factor_;
		O_linvel_B_[i] += (1.0/(double)update_rate_)*O_linacc_B_[i];
	}

	// write data back
	linkQuat(O_rot_B_, &O_orient_B_);
	memcpy(&(O_orient_B_.ad[1]), &(O_angrate_I_[1]), 3*sizeof(double));
	memcpy(&(O_orient_B_.add[1]), &(O_angacc_I_[1]), 3*sizeof(double));
	memcpy(&(O_trans_B_.x[1]), &(O_linpos_B_[1]),3*sizeof(double));
	memcpy(&(O_trans_B_.xd[1]), &(O_linvel_B_[1]),3*sizeof(double));
	memcpy(&(O_trans_B_.xdd[1]), &(O_linacc_B_[1]),3*sizeof(double));

//	linkQuat(O_rot_I_, &O_orient_B_);
//	memcpy(&(O_orient_B_.ad[1]), &(O_angrate_I_[1]), 3*sizeof(double));
//	memcpy(&(O_orient_B_.add[1]), &(O_angacc_I_[1]), 3*sizeof(double));
//	memcpy(&(O_trans_B_.x[1]), &(O_linpos_I_[1]),3*sizeof(double));
//	memcpy(&(O_trans_B_.xd[1]), &(O_linvel_I_[1]),3*sizeof(double));
//	memcpy(&(O_trans_B_.xdd[1]), &(O_linacc_I_[1]),3*sizeof(double));

//	quatDerivatives(&O_orient_B_);
	base_orientation = O_orient_B_;
	base_position = O_trans_B_;

	// update buffers
	memcpy(&(prev_O_angrate_I_[1]), &(O_angrate_I_[1]), 3*sizeof(double));
}


void BaseStateEstimation::vectorToSkewMatrix(Vector w, Matrix S)
{
	S[1][1] = S[2][2] = S[3][3] = 0.0;

	S[1][2] = -w[3];
	S[1][3] = w[2];
	S[2][3] = -w[1];

	S[2][1] = -S[1][2];
	S[3][1] = -S[1][3];
	S[3][2] = -S[2][3];
}

void BaseStateEstimation::transformImuStateToBase()
{
	  MY_MATRIX(S_angrate, 1,3,1,3);
	  vectorToSkewMatrix(O_angrate_I_, S_angrate);
	  MY_MATRIX(S_angacc, 1,3,1,3);
	  vectorToSkewMatrix(O_angacc_I_, S_angacc);
	  MY_MATRIX(S_helper, 1,3,1,3);
	  mat_mult(S_angrate, S_angrate, S_helper);
	  mat_add(S_angacc, S_helper, S_helper);

	  mat_mult(O_rot_I_, I_rot_B_, O_rot_B_);

	  mat_vec_mult(O_rot_I_, I_linpos_B_, O_linpos_B_);
	  vec_add(O_linpos_I_, O_linpos_B_, O_linpos_B_);

	  mat_vec_mult(O_rot_I_, I_linpos_B_, O_linvel_B_);
	  mat_vec_mult(S_angrate, O_linvel_B_, O_linvel_B_);
	  vec_add(O_linvel_I_, O_linvel_B_, O_linvel_B_);

	  mat_vec_mult(O_rot_I_, I_linpos_B_, O_linacc_B_);
	  mat_vec_mult(S_helper, O_linacc_B_, O_linacc_B_);
	  vec_add(O_linacc_I_, O_linacc_B_, O_linacc_B_);
}

/////////////// C interface //////////
BaseStateEstimation estimation;
extern "C"
{
void initialize_base_state_estimation(const double* imu_quaternion,
const double* imu_angrate, const double* imu_linacc, int update_rate);
void update_base_state_estimation(SL_quat* base_orientation,
      SL_Cstate* base_position);
} // extern "C"

void initialize_base_state_estimation(const double* imu_quaternion,
      const double* imu_angrate,
      const double* imu_linacc, int update_rate)
{
	estimation.initialize(imu_quaternion, imu_angrate, imu_linacc, update_rate);
}
void update_base_state_estimation(SL_quat* base_orientation, SL_Cstate* base_position)
{
	estimation.update(*base_orientation, *base_position);
}

