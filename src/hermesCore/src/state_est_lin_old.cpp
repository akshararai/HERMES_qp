/*
 * state_est_lin.cpp
 *
 *  Created on: Nov 4, 2015
 *      Author: perry
 */

#include <iostream>
#include <Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>

// system headers
#include "SL_system_headers.h"

/* SL includes */
#include "SL.h"
#include "SL_user.h"
#include "SL_tasks.h"
#include "SL_task_servo.h"
#include "SL_kinematics.h"
#include "SL_dynamics.h"
#include "SL_collect_data.h"
#include "SL_shared_memory.h"
#include "SL_man.h"

extern "C"
{
int init_state_est_lin_task();
int run_state_est_lin_task();
static int change_state_est_lin_task();
void add_state_est_lin_task();
void getPelv(SL_quat *base_quat, SL_Cstate *base );

}

Eigen::Matrix<double,3,1> _gravity(0,0,-9.81);
Eigen::Matrix<double,3,1> _imu_lin_offset(-0.1, 0.11, -0.09);
Eigen::Quaterniond _imu_quat_offset(0.38268, 0, 0, -0.92388);

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
bool buffer_first_time = true;
bool contact_change_flag = true;
// ComputeFootstuff
Eigen::Matrix<double,3,1> foot_pos[2]; // Contact point of foot
Eigen::Matrix<double,3,1> foot_vel[2]; // Velocity of contact point
Eigen::Matrix<double,3,1> foot_toe_pos[2]; // Position of toe
Eigen::Matrix<double,3,1> foot_toe_vel[2]; // Velocity of toe
Eigen::Matrix<double,12,1> foot_registered; // registered foot position/velocity, used as measurement
// Weight distribution between left and right leg
double _beta;

const static double timestep = 1/250.0;

/*-----------------------------------------------------------------------------------------*/
void makeProcessJacobian ()
{
  _A.setIdentity();
  _A.block(0,3,3,3).setIdentity();
  _A.block(0,3,3,3) *= timestep;
}

void makeObservationJacobian ()
{
  _C.setIdentity();
}

void dare(const Eigen::Matrix<double,6,6> &A, const Eigen::Matrix<double,6,6> &B, Eigen::Matrix<double,6,6> &P,int zDim = 6)
{
  Eigen::Matrix<double,6,6> Ainv = A.inverse();
  Eigen::Matrix<double,6,6> ABRB;
  if (zDim == 6)
  {
    ABRB = Ainv * B * _R.llt().solve(B.transpose());
  }
  else{
    ABRB = Ainv * B.topLeftCorner(6,zDim) * _R.topLeftCorner(zDim,zDim).llt().solve(B.topLeftCorner(6,zDim).transpose());
  }
  Eigen::Matrix<double,2*6,2*6> Z;
  Z.block(0,0,6,6) = Ainv;
  Z.block(0,6,6,6) = ABRB;
  Z.block(6,0,6,6) = _Q * Ainv;
  Z.block(6,6,6,6) = A.transpose() + _Q * ABRB;

  Eigen::ComplexEigenSolver <Eigen::Matrix<double,2*6,2*6> > ces;
  ces.compute(Z);

  Eigen::Matrix<std::complex<double>,2*6,1> eigVal = ces.eigenvalues();
  Eigen::Matrix<std::complex<double>,2*6,2*6> eigVec = ces.eigenvectors();

  Eigen::Matrix<std::complex<double>,2*6,6> unstableEigVec;

  int ctr = 0;
  for (int i = 0; i < 2*6; i++) {
    if (eigVal(i).real()*eigVal(i).real() + eigVal(i).imag()*eigVal(i).imag() > 1) {
      unstableEigVec.col(ctr) = eigVec.col(i);
      ctr++;
      if (ctr > 6)
        break;
    }
  }

  Eigen::Matrix<std::complex<double>,6,6> U21inv = unstableEigVec.block(0,0,6,6).inverse();
  Eigen::Matrix<std::complex<double>,6,6> PP = unstableEigVec.block(6,0,6,6) * U21inv;

  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      P(i,j) = PP(i,j).real();
    }
  }
}

void computeKss(const Eigen::Matrix<double,6,6> &A, const Eigen::Matrix<double,6,6> &C, int zDim = 6)
{

  Eigen::Matrix<double,6,6> B = C.transpose();
  // Eigen::Matrix<double,6,6> P;
  dare(A.transpose(), B, _P, zDim); // A^T is used here
  _K.setZero();
  Eigen::Matrix<double,6,6> PB;
  Eigen::Matrix<double,6,6> BtPB_R;
  if (zDim == 6){
    PB = _P * B;
    BtPB_R = B.transpose() * PB + _R;
    _K = PB* BtPB_R.inverse()
;  }
  else
  {
    PB.topLeftCorner(6,zDim) = _P * B.topLeftCorner(6,zDim);
    BtPB_R.topLeftCorner(zDim,zDim) = B.topLeftCorner(6,zDim).transpose() * PB.topLeftCorner(6,zDim) + _R.topLeftCorner(zDim,zDim);
    _K.topLeftCorner(6,zDim) = PB.topLeftCorner(6,zDim)* BtPB_R.topLeftCorner(zDim,zDim).inverse();
  }
}

void setKss()
{
  makeProcessJacobian();
  makeObservationJacobian();
  //setQ(DSc_Q);
  //setR(DSc_R);
  computeKss(_A,_C, 6);
}

void computeAnkleRelated()
{
	  for(int i=3; i<=4; ++i){
	    for(int j=1; j<=6; ++j)
	      endeff[i].c[j] = 1;
	  }

//  rs->makeSDFastState(Pose(_x.block(X0,0,3,1), _q), _x.block(V0,0,3,1), _w, _joints, _jointsd, sdFastState);
//  set_state(rs->model, sdFastState.data());

  foot_pos[0][0] = cart_state[4].x[1];
  foot_pos[0][1] = cart_state[4].x[2];
  foot_pos[0][2] = cart_state[4].x[3];

  Eigen::Vector3d l_foot_temp(cart_state[4].x[1],cart_state[4].x[2],cart_state[4].x[3]);
  Eigen::Vector3d l_foot_rot_vel = _w.cross(l_foot_temp);

  foot_pos[1][0] = cart_state[3].x[1];
  foot_pos[1][1] = cart_state[3].x[2];
  foot_pos[1][2] = cart_state[3].x[3];


  foot_vel[0][0] = cart_state[4].xd[1];
  foot_vel[0][1] = cart_state[4].xd[2];
  foot_vel[0][2] = cart_state[4].xd[3];

  foot_vel[1][0] = cart_state[3].xd[1];
  foot_vel[1][1] = cart_state[3].xd[2];
  foot_vel[1][2] = cart_state[3].xd[3];

}

void set_hack_footpos()
{
  if (buffer_first_time)
  {
    buffer_first_time = false;
    _beta = 0.5; // Initialize beta here
  }
  if (contact_change_flag )
  {
    foot_registered.block(0,0,3,1) = foot_pos[0];
    foot_registered.block(3,0,3,1) = foot_vel[0];
    foot_registered.block(6,0,3,1) = foot_pos[1];
    foot_registered.block(9,0,3,1) = foot_vel[1];

    contact_change_flag = false;
  }
}

int initializing_q = 0;
Eigen::Vector3d init_gravity_vector;

void initKF(const Eigen::Matrix<double,3,1> &root_pos, const Eigen::Matrix<double,3,1> &root_vel){

	buffer_first_time = true;
	contact_change_flag = true;
	_toe_flag = -1;

	Eigen::Matrix<double,6,6> P0;
	P0.setIdentity();
	P0 *= 0.01;
	_P = P0;

	_x.block(X0,0,3,1) = root_pos;
	_x.block(V0,0,3,1) = root_vel;
	_vimu = _x.block(V0,0,3,1); // This is not quite accurate, but only one time step, so does not matter
	setKss();

	// The following steps are additional step for next step measurement
	computeAnkleRelated(); // Compute ankle position and velocity
	set_hack_footpos();
	// Display in terminal
	std::cout << "===================================\n";
	std::cout << "Kinematic Filter QR" <<std::endl;
	std::cout << "Q = " << std::endl;
	std::cout <<_Q << std::endl;
	std::cout << "R = " << std::endl;
	std::cout <<_R << std::endl;
	std::cout << "P = " << std::endl;
	std::cout <<_P << std::endl;
	std::cout << "===================================\n";

	std::cout << "Initializing Q" <<std::endl;
	std::cout << "Please don't move the robot" <<std::endl;
	initializing_q = 250;
	init_gravity_vector = Eigen::Vector3d(0,0,9.81);

}

void makeInputs(const Eigen::Quaterniond &q, const Eigen::Matrix<double,3,1> &w, const Eigen::Matrix<double,3,1> &a, const Eigen::Matrix<double,50,1> &joints, const Eigen::Matrix<double,50,1> &jointsd)
{
  _q = q;
  _w = w;
  _a = a;
  _joints = joints;
  _jointsd = jointsd;
}


void setContactState (int cs, int toe_flag)
{
  //setContactState(cs);
  _previous_toe_flag = _toe_flag; // store previous toe flag;
  _toe_flag = toe_flag;
}

template<typename T> T low_pass_filter(T &hist, T raw, T alpha)
{
  hist = hist * alpha + raw * (1-alpha);
  return hist;
}

void makeMeasurement(double left_fz, double right_fz)
{
  static double fz_hist[2] = {750, 750};
  left_fz = low_pass_filter(fz_hist[0], left_fz, 0.99);
  right_fz = low_pass_filter(fz_hist[1], right_fz, 0.99);

  //_beta = left_fz / (left_fz + right_fz);

  if (!(_previous_contact_state == _contact_state_full))
  {
    _previous_contact_state = _contact_state_full;
    contact_change_flag = true;
//    set_hack_footpos();
  }

  if (sqrt(left_fz*left_fz+right_fz*right_fz) < 50) // Total z force small
  {
    contact_change_flag = true; // Just update foot position until the robot drops from the air
//    set_hack_footpos();
  }
  /*
  if (_toe_flag > _previous_toe_flag)// toe off
  {
    contact_change_flag = true;
    set_hack_footpos();
  }
  */
  _y = _beta*foot_registered.block(0,0,6,1) + (1.0 - _beta)*foot_registered.block(6,0,6,1);
  // Set measured velocity to zero
  _y.block(3,0,3,1).setZero();
}


void predictX()
{
  // IMU acceleration in world frame without gravity (a = R^{-1}*a_imu + g)
  Eigen::Matrix<double,3,1> acc;
  acc = _q*_imu_quat_offset.conjugate()*_a + _gravity;

  // IMU velocity in world frame
  _vimu += timestep*acc;
  // Position
  _x.block(X0,0,3,1) += timestep*_x.block(V0,0,3,1);// + 0.5*timestep*timestep*acc;
  // Velocity
  // V = R(-q)*(Vimu - \omega x imuoffset)
  _x.block(V0,0,3,1) = _vimu; //- _q*_w.cross(_imu_lin_offset);

	static int printcounter = 0;
	printcounter++;
	if (printcounter > 100){
		printcounter = 0;
//		std::cout<<"acc : " <<_a[0] <<" "<<_a[1]<<" "<<_a[2];
//		std::cout<<"acc : " <<acc[0] <<" "<<acc[1]<<" "<<acc[2];
//		std::cout<<std::endl;
	}
}

void updateZ(int zDim = 6)
{
  _z.setZero();
  _z.block(0,0,3,1) = _beta*foot_pos[0] + (1.0 - _beta)*foot_pos[1];
  _z.block(3,0,3,1) = _beta*foot_vel[0] + (1.0 - _beta)*foot_vel[1];
}

void updateX(int zDim = 6)
{
  _dx= _K* _innov;
  _x += _dx;
  _vimu = _x.block(V0,0,3,1) + _q*_w.cross(_imu_lin_offset);
}

void computeInnovation(const Eigen::Matrix<double, 6, 1> & z)
{
  _innov =  z-_z ;
}

void filterOneTimeStep_ss()
{
  predictX();
  computeAnkleRelated(); // Compute ankle position and velocity
  updateZ(6);
  computeInnovation(_y);
  updateX(6);
  // The following steps are additional step for next step measurement
  computeAnkleRelated(); // Compute ankle position and velocity
}



/*---------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------*/
/*---------------------------                              ------------------------------------*/
/*---------------------------           SL STUFF           ------------------------------------*/
/*---------------------------                              ------------------------------------*/
/*---------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------*/

bool first_time = false;

int init_state_est_lin_task(){
	_P.setIdentity();
	_Q.setIdentity();
	_R.setIdentity();

	Eigen::Quaterniond imu_orientation;
	imu_orientation.w() = misc_sensor[B_Q0_IMU];
	imu_orientation.x() = misc_sensor[B_Q1_IMU];
	imu_orientation.y() = misc_sensor[B_Q2_IMU];
	imu_orientation.z() = misc_sensor[B_Q3_IMU];

	Eigen::Matrix<double,3,1> imu_angular_velocity;
	imu_angular_velocity[0] = misc_sensor[B_AD_G_IMU];
	imu_angular_velocity[1] = misc_sensor[B_AD_B_IMU];
	imu_angular_velocity[2] = misc_sensor[B_AD_A_IMU];

	Eigen::Matrix<double,3,1> imu_linear_acceleration;
	imu_linear_acceleration[0] = misc_sensor[B_XACC_IMU];
	imu_linear_acceleration[1] = misc_sensor[B_YACC_IMU];
	imu_linear_acceleration[2] = misc_sensor[B_ZACC_IMU];



	_x[0] = base_state.x[1];
	_x[1] = base_state.x[2];
	_x[2] = base_state.x[3];

	_x[3] = base_state.xd[1];
	_x[4] = base_state.xd[2];
	_x[5] = base_state.xd[3];

	first_time = true;


	return TRUE;
}

void integrate_angular_velocity(Eigen::Vector3d rpy){
	Eigen::Quaterniond m;
	m = Eigen::AngleAxisd(rpy[0]*timestep, Eigen::Vector3d::UnitX())*
	    Eigen::AngleAxisd(rpy[1]*timestep, Eigen::Vector3d::UnitY())*
	    Eigen::AngleAxisd(rpy[2]*timestep, Eigen::Vector3d::UnitZ());

	_q = _q*m;
}


/*---------------------------------------------------------------------------------------------*/
int currentForce = 0;
int run_state_est_lin_task(){

	if (first_time){

		//initKF(Eigen::Vector3d(base_state.x[1],base_state.x[2],base_state.x[3]), Eigen::Vector3d(base_state.xd[1],base_state.xd[2],base_state.xd[3]));
		initKF(Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,0));

		first_time = false;

		buffer_first_time = true;
		contact_change_flag = true;
		computeAnkleRelated();
		set_hack_footpos();
		std::cout<<foot_pos[0][0] <<" "<<foot_pos[0][1]<<" "<<foot_pos[0][2] <<"\t" <<foot_pos[1][0] <<" "<<foot_pos[1][1]<<" "<<foot_pos[1][2] <<std::endl;
		std::cout<<foot_registered[0] <<" "<<foot_registered[1]<<" "<<foot_registered[2] <<"\t" <<foot_registered[3] <<" "<<foot_registered[4]<<" "<<foot_registered[5] <<std::endl;
//		std::cout<<std::endl<<std::endl;


	}

	if (initializing_q > 0){
		initializing_q--;

		Eigen::Matrix<double,3,1> current_imu_linear_acceleration;
		current_imu_linear_acceleration[0] = misc_sensor[B_XACC_IMU];
		current_imu_linear_acceleration[1] = misc_sensor[B_YACC_IMU];
		current_imu_linear_acceleration[2] = misc_sensor[B_ZACC_IMU];

		init_gravity_vector = 0.995 * init_gravity_vector + 0.005 *( _imu_quat_offset.conjugate()*current_imu_linear_acceleration);

		if(initializing_q == 0){
			_q.setFromTwoVectors(init_gravity_vector,Eigen::Vector3d(0, 0, 1));

			std::cout << "Done Initializing Q" <<std::endl;
			std::cout << "Final vector = " <<std::endl;
			std::cout << _imu_quat_offset*init_gravity_vector;
			std::cout<<std::endl<<std::endl;

			_gravity = -(_q.matrix()*(init_gravity_vector));
			std::cout << "Final gravity = " <<std::endl;
			std::cout << _gravity;
			std::cout<<std::endl<<std::endl;
		}
		return TRUE;
	}

	static int printcounter = 1000;
	printcounter++;
	if (printcounter > 1000){
		printcounter = 0;

		/*std::cout<<"innov === "<<_innov[0] <<"\t"<<_innov[1]<<"\t"<<_innov[2] <<"\t" <<_innov[3] <<"\t"<<_innov[4]<<"\t"<<_innov[5];
		std::cout<<std::endl;
		std::cout<<"z === "<<_z[0] <<"\t"<<_z[1]<<"\t"<<_z[2] <<"\t" <<_z[3] <<"\t"<<_z[4]<<"\t"<<_z[5];
		std::cout<<std::endl;
		std::cout<<"y === "<<_y[0] <<" "<<_y[1]<<" "<<_y[2] <<"\t" <<_y[3] <<" "<<_y[4]<<" "<<_y[5];
		std::cout<<std::endl;
//		std::cout<<"err: "<<base_state.x[1] - _x[0] <<" "<<base_state.x[2] - _x[1]<<" "<<base_state.x[3] - _x[2] <<
//				"\t" <<base_state.xd[1] - _x[3] <<" "<<base_state.xd[2] - _x[4]<<" "<<base_state.xd[3] - _x[5];
		std::cout<<std::endl;
		std::cout<<std::endl;*/
	}

	Eigen::Matrix<double,3,1> imu_angular_velocity;
	imu_angular_velocity[0] = misc_sensor[B_AD_A_IMU];
	imu_angular_velocity[1] = misc_sensor[B_AD_B_IMU];
	imu_angular_velocity[2] = misc_sensor[B_AD_G_IMU];

	Eigen::Matrix<double,3,1> imu_linear_acceleration;
	imu_linear_acceleration[0] = misc_sensor[B_XACC_IMU];
	imu_linear_acceleration[1] = misc_sensor[B_YACC_IMU];
	imu_linear_acceleration[2] = misc_sensor[B_ZACC_IMU];

	Eigen::Matrix<double, 50, 1> joints;
	for(int x =0; x < 50; x++) joints[x] = joint_state[x+1].th;

	Eigen::Matrix<double, 50, 1> jointsd;
	for(int x =0; x < 50; x++) jointsd[x] = joint_state[x+1].thd;

	imu_angular_velocity = _imu_quat_offset.normalized().conjugate().toRotationMatrix()*imu_angular_velocity;

	integrate_angular_velocity(imu_angular_velocity);

	makeInputs(_q, imu_angular_velocity, imu_linear_acceleration, joints, jointsd);
	setContactState(2, -1);

	makeMeasurement(misc_sensor[L_CFx], misc_sensor[R_CFx]);
	filterOneTimeStep_ss();





		  if (currentForce != 0){
			  uext_sim[R_HAA].f[_Y_] = currentForce;
			  sendUextSim();
		  }

//	static int reset_counter = 0;
//	reset_counter++;
//	if (reset_counter == 10000){
//		reset_counter = 0;
//		_x[0] = 0;
//		_x[1] = 0;
//		_x[2] = 0;
//		_x[3] = 0;
//		_x[4] = 0;
//		_x[5] = 0;
//		_vimu[0] = 0;
//		_vimu[1] = 0;
//		_vimu[2] = 0;
//	}

	return TRUE;
}

/*---------------------------------------------------------------------------------------------*/

static int change_state_est_lin_task(){
	  int    ivar;
	  get_int("ints cause forces",currentForce,&currentForce);



	return TRUE;
}

/*---------------------------------------------------------------------------------------------*/

void add_state_est_lin_task(){
	addTask("state_est_lin", init_state_est_lin_task, run_state_est_lin_task, change_state_est_lin_task);
}
/*---------------------------------------------------------------------------------------------*/

void getPelv(SL_quat *base_quat, SL_Cstate *base ){
	base->x[1] = _x[0];
	base->x[2] = _x[1];
	base->x[3] = _x[2];
	base->xd[1] = _x[3];
	base->xd[2] = _x[4];
	base->xd[3] = _x[5];

  

	Eigen::Quaterniond base_quat_eigen = _q;
	base_quat->q[1] = _q.w();
	base_quat->q[2] = _q.x();
	base_quat->q[3] = _q.y();
	base_quat->q[4] = _q.z();
  

}

