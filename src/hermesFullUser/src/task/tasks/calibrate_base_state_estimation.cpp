#include <iostream>
#include <unistd.h>

// SL system headers
#include "SL_system_headers.h"

// SL includes
#include "SL.h"
#include "SL_user.h"
#include "SL_common.h"
#include "SL_tasks.h"
#include "SL_task_servo.h"
#include "SL_kinematics.h"
#include "SL_dynamics.h"
#include "SL_collect_data.h"
#include "SL_shared_memory.h"
#include "SL_man.h"


/* local variables */

extern "C"{

void add_calibrate_base_state_estimation_task();
void calibrate_base_state_estimation();

//static int init_calibrate_base_state_estimation_task();
//static int run_calibrate_base_state_estimation_task();
//static int change_calibrate_base_state_estimation_task();
}

//static void show_torques();

void add_calibrate_base_state_estimation_task()
{
//  addTask("Calibrate Base State Estimation", init_calibrate_base_state_estimation_task,
//		  run_calibrate_base_state_estimation_task, change_calibrate_base_state_estimation_task);

  addToMan("cal_bs_est", "calibrates the base state estimation and returns parameters",
		  calibrate_base_state_estimation);
}

void calibrate_base_state_estimation()
{
	int num_meas;
	int meas_freq;
	read_parameter_pool_int("bs_est_config.cf", "NUM_MEAS", &num_meas);
	read_parameter_pool_int("bs_est_config.cf", "SAMPLE_FREQ", &meas_freq);
	int sleep_delay = (int)(1000000/(double)meas_freq);

	double sensor_gravity[4];
	for(int j=1; j<=3; j++)
		sensor_gravity[j] = 0.0;

	for(int i=0; i<num_meas; i++)
	{
		for(int j=1; j<=3; j++)
			sensor_gravity[j] += misc_sensor[B_XACC_IMU+j-1];
		usleep(sleep_delay);//TODO: make this rt safe
	}
	for(int j=1; j<=3; j++)
		sensor_gravity[j] /= num_meas;

	std::cout << "filtered misc_sensor gravity (x,y,z): " << sensor_gravity[1] << ", " << sensor_gravity[2] <<
			", " << sensor_gravity[3] << std::endl;
 }



