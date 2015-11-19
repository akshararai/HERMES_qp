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


/* local variables */
static double start_time = 0.0;
static double freq = 0.1;
static double amp = 1.0;
static double off = 0.0;
static int jid = 0;
static int step_tests = 0;

extern "C"{

void add_torque_sine_task();

static int init_torque_sine_task();
static int run_torque_sine_task();
static int change_torque_sine_task();
}


void add_torque_sine_task()
{
  addTask("Torque sine task", init_torque_sine_task,
          run_torque_sine_task, change_torque_sine_task);
}


static int init_torque_sine_task()
{
  int j, i;
  int ans;
  static int firsttime = TRUE;

  if (firsttime){
    firsttime = FALSE;
    //    freq = 0.1; // frequency
    //    amp  = 0.5; // amplitude
  }

  get_int("Do step tests?", step_tests, &step_tests);
  if(step_tests)
  {
    get_int("Torque step which joint?",jid,&jid);
    get_double("Step amplitude [Nm]", amp,&amp);
    get_double("Step offset [Nm]", off, &off);
    get_double("Step frequency [Hz]",freq,&freq);


    if (jid < 1 || jid > n_dofs || amp > 10.0 || amp < -10.0 || freq < 0.1 || freq > 1.0)
      return FALSE;

    printf("running a %f Nm step test with offset %f for joint %s\n", amp, off, freq, joint_names[jid]);
  }
  else
  {

    get_int("Torque sine which joint?",jid,&jid);
    get_double("Sine amplitude [Nm]",amp,&amp);
    get_double("Sine frequency [Hz]",freq,&freq);
    get_double("Sine offset [Nm]", off, &off);

    start_time = 0.0;

    if (jid < 1 || jid > n_dofs || amp > 10.0 || amp < -10.0 || freq < 0.1 || freq > 40.0)
      return FALSE;

    printf("running a %f Nm sine offset at %f at %f Hz for joint %s\n", amp, off, freq, joint_names[jid]);
  }

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


static int run_torque_sine_task()
{
  static double desired_torque = off;
  if(step_tests)
  {
    static double running_time = 0.0;
    static double sign = 1.0;
    if(running_time >1/freq)
    {
      desired_torque  = off + sign*amp;
      sign *=-1.0;
      running_time = 0.0;
    }
    running_time += 1/double(task_servo_rate);
  }
  else
  {
    //transition for 2 seconds
    double mult = 0.0;
    if(start_time < 2.0)
      mult = start_time/2.0;
    else
      mult = 1.0;
    desired_torque = mult*(amp * sin(start_time * freq * 2 * M_PI) + off);
  }

  joint_des_state[jid].th = joint_state[jid].th;
  joint_des_state[jid].thd = joint_state[jid].thd;
  joint_des_state[jid].uff = desired_torque;


  start_time += 1/double(task_servo_rate);

  return TRUE;
}

static int change_torque_sine_task()
{
  return TRUE;
}


