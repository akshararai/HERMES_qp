/*
 * joint_calibration_task.cpp
 *
 *  Created on: Jan 29, 2014
 *      Author: righetti
 */

#include <vector>

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
#include "SL_common.h"


/* local variables */
static double start_time;
static double sine_freq = 0.1;

static double max_torque[N_DOFS+1];
static std::vector<int> joints_to_calibrate;
static int j_index = 0;

extern "C"{

void add_joint_calibration_task();

static int init_joint_calibration_task();
static int run_joint_calibration_task();
static int change_joint_calibration_task();
}


void add_joint_calibration_task()
{
  addTask("Joint calibration task task", init_joint_calibration_task,
          run_joint_calibration_task, change_joint_calibration_task);
}


static int init_joint_calibration_task()
{
  int ans;
  start_time = 0.0;
  j_index = 0;
  joints_to_calibrate.clear();

  for(int i=1; i<=N_DOFS; ++i)
  {
    if(read_parameter_pool_double("JointCalibrationTask.cf",joint_names[i],&(max_torque[i])))
      joints_to_calibrate.push_back(i);
  }

  if(!read_parameter_pool_double("JointCalibrationTask.cf","sine_freq",&sine_freq))
  {
    printf("Warning sine freq not defined. Using default\n");
  }
  printf("Sine Freq set to %f Hz\n", sine_freq);

  // ready to go
  ans = 999;
  while (ans == 999) {
    if (!get_int("Enter 1 to start or anthing else to abort ...",ans,&ans))
      return FALSE;
  }

  // only go when user really types the right thing
  if (ans != 1)
    return FALSE;

  scd();

  return TRUE;
}


static int run_joint_calibration_task()
{
  if(j_index < (int)joints_to_calibrate.size())
  {
    int j = joints_to_calibrate[j_index];
    joint_des_state[j].th = joint_state[j].th;
    joint_des_state[j].thd = 0.0; //damping is good
    joint_des_state[j].uff = max_torque[j] * sin(2*M_PI*start_time*sine_freq);

    //we stop after 110% of a sine
    if(start_time > 1.1*(1.0/sine_freq))
    {
      start_time = 0.0;
      j_index++;
    }
  }
  else//we are done
  {
    freeze();
  }

  start_time += 1/double(task_servo_rate);

  return TRUE;
}

static int change_joint_calibration_task()
{
  return TRUE;
}





