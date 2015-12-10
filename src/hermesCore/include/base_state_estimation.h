/*
 * base_state_estimation.h
 *
 *  Created on: Mar 7, 2013
 *      Author: herzog
 */

#ifndef BASE_STATE_ESTIMATION_H_
#define BASE_STATE_ESTIMATION_H_

#include <SL.h>
#include <SL_user.h>
#include <utility.h>

class BaseStateEstimation
{
public:
	BaseStateEstimation();
	virtual ~BaseStateEstimation();

	void initialize(const double* imu_quaternion,
	      const double* imu_angrate,
	      const double* imu_linacc, int update_rate);
	void update(SL_quat& base_orientation, SL_Cstate& base_position);
private:
	// sensor data access
	const double* imu_quaternion_;
	const double* imu_angrate_;
	const double* imu_linacc_;

	// output
	SL_quat O_orient_B_;
	SL_Cstate O_trans_B_;

	// constant
	Matrix I_rot_B_;
	Vector I_linpos_B_;
//	Vector gravity_;
	int update_rate_;
	double leakage_factor_;

	// read from imu
	Matrix O_rot_I_;
	Vector O_angrate_I_;
	Vector O_linacc_I_;

	// numerically derived/integrated
	Vector O_angacc_I_;
	Vector O_linpos_I_;
	Vector O_linvel_I_;

	// transformed
	Matrix O_rot_B_;
	Vector O_linacc_B_;
	Vector O_linvel_B_;
	Vector O_linpos_B_;
	// O_angrate_B_ == O_angrate_I_;
	// O_angacc_B_ == O_angacc_I_;

	// buffering
	Vector prev_O_angrate_I_;


	void vectorToSkewMatrix(Vector w, Matrix S);
	void transformImuStateToBase();

};


#endif /* BASE_STATE_ESTIMATION_H_ */
