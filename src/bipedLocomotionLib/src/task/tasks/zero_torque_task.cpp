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



extern "C"{

void add_zero_torque_task();

static int init_zero_torque_task();
static int run_zero_torque_task();
static int change_zero_torque_task();
}


void add_zero_torque_task()
{
  addTask("Zero torque task", init_zero_torque_task,
          run_zero_torque_task, change_zero_torque_task);
}


static int init_zero_torque_task()
{
  printf("WARNING!!\n\n This will send 0 torque commands to all the joints, be careful!!\n");

  // ready to go
  int ans = 999;
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


static int run_zero_torque_task()
{
  for(int i=1; i<=N_DOFS; i++)
  {
    joint_des_state[i].th = joint_state[i].th;
    joint_des_state[i].thd = joint_state[i].thd;
    joint_des_state[i].uff = 0.0;
  }
  return TRUE;
}

static int change_zero_torque_task()
{
  return TRUE;
}


