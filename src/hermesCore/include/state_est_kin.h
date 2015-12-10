/*
 * state_est.h
 *
 *  Created on: Dec 3, 2015
 *      Author: perry
 */

#ifndef STATE_EST_KIN_H_
#define STATE_EST_KIN_H_

#include <Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>

#include "SL.h"

class StateEstimatorKinematic
{
public:

	StateEstimatorKinematic();

	int initialize();
	int run_state_est_lin_task();
	void getPelv(SL_quat *base_quat, SL_Cstate *base );
	void initKF(const Eigen::Matrix<double,3,1> &root_pos, const Eigen::Matrix<double,3,1> &root_vel);

protected:

	void makeProcessJacobian ();
	void makeObservationJacobian ();
	void dare(const Eigen::Matrix<double,6,6> &A, const Eigen::Matrix<double,6,6> &B, Eigen::Matrix<double,6,6> &P,int zDim = 6);
	void computeKss(const Eigen::Matrix<double,6,6> &A, const Eigen::Matrix<double,6,6> &C, int zDim = 6);
	void setKss();
	void computeAnkleRelated();
	void set_hack_footpos();
	void makeInputs(const Eigen::Quaterniond &q, const Eigen::Matrix<double,3,1> &w, const Eigen::Matrix<double,3,1> &a, const Eigen::Matrix<double,50,1> &joints, const Eigen::Matrix<double,50,1> &jointsd);
	void setContactState (int cs, int toe_flag);
	void makeMeasurement(double left_fz, double right_fz);
	void predictX();
	void updateZ(int zDim = 6);
	void updateX(int zDim = 6);
	void computeInnovation(const Eigen::Matrix<double, 6, 1> & z);
	void filterOneTimeStep_ss();
	void integrate_angular_velocity(Eigen::Vector3d xyz);

	Eigen::Matrix<double,3,1> _gravity;
	const Eigen::Matrix<double,3,1> _imu_lin_offset;
	const Eigen::Quaterniond _imu_quat_offset;

	static const int X0 = 0;
	static const int V0 = 3;
	// Filter Params
	int zDim;
	Eigen::Matrix<double,6,1> _y;
	Eigen::Matrix<double,6,1> _x;
	Eigen::Matrix<double,6,1> _z;
	Eigen::Matrix<double,6,1> _innov;

	Eigen::Matrix<double,6,6> _P;
	Eigen::Matrix<double,6,6> _Q;
	Eigen::Matrix<double,6,6> _R;
	Eigen::Matrix<double,6,6> _K;
	Eigen::Matrix<double,6,6> _S;

	Eigen::Matrix<double,6,6> _A;
	Eigen::Matrix<double,6,6> _C;
	Eigen::Matrix<double,6,1> _dx;
	// Input
	Eigen::Quaternion<double> _q;
	Eigen::Matrix<double,3,1> _a; // Acceleration as input from IMU ekf or raw data
	Eigen::Matrix<double,3,1> _w; // Angular velocity as input IMU ekf or raw data
	Eigen::Matrix<double,3,1> _vimu;
	Eigen::Matrix<double,50,1> _joints;
	Eigen::Matrix<double,50,1> _jointsd;
	//Flag
	int _toe_flag; // Toe off flag from controller, reset reference point.
	int _previous_toe_flag; // Toe off flag from controller, reset reference point.
	int _contact_state;
	int _contact_state_full;
	int _previous_contact_state; // Previous contact state
	// int _contact_timer;
	bool buffer_first_time;
	bool contact_change_flag;
	// ComputeFootstuff
	Eigen::Matrix<double,3,1> foot_pos[2]; // Contact point of foot
	Eigen::Matrix<double,3,1> foot_vel[2]; // Velocity of contact point
	Eigen::Matrix<double,3,1> foot_toe_pos[2]; // Position of toe
	Eigen::Matrix<double,3,1> foot_toe_vel[2]; // Velocity of toe
	Eigen::Matrix<double,12,1> foot_registered; // registered foot position/velocity, used as measurement
	// Weight distribution between left and right leg
	double _beta;

};

#endif /* STATE_EST_KIN_H_ */
