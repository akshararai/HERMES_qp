/*!=============================================================================
  ==============================================================================

  \file    fb_sine.cpp

  \author  righetti
  \date    Dec 19, 2012

  ==============================================================================
  \remarks


  ============================================================================*/


#include <eigen3/Eigen/Eigen>
#include "OptimalInverseDynamicsEigen.h"

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
#include "utility_macros.h"

//local variables
static int use_computed_torques = FALSE;
static floating_base_utilities::OptimalInverseDynamicsEigen fb_id;


static Eigen::Matrix<double, 6, 1> fbs_momentum;

#define MAX_SINE 4
static double       *off;
static double      **amp;
static double      **phase;
static double      **freq;
static int          *n_sine;
static SL_DJstate   target[N_DOFS+1];
static double        task_time;
static double        speed=1.0;
static double        trans_mult;
static double        trans_period;

extern "C"{

void add_fb_sine_task();

static int init_fb_sine_task();
static int run_fb_sine_task();
static int change_fb_sine_task();
}


static int read_sine_script(void);

void add_fb_sine_task()
{
  static int firsttime = TRUE;

  if (firsttime) {
    firsttime = FALSE;
    off = my_vector(1,n_dofs);
    amp = my_matrix(1,n_dofs,1,MAX_SINE);
    phase = my_matrix(1,n_dofs,1,MAX_SINE);
    freq = my_matrix(1,n_dofs,1,MAX_SINE);
    n_sine = my_ivector(1,n_dofs);
  }

  addTask("FB sine task", init_fb_sine_task,
          run_fb_sine_task, change_fb_sine_task);
}


static int init_fb_sine_task()
{
  int j, i;
  char string[100];
  double max_range=0;
  int ans;

  /* check whether any other task is running */
  if (strcmp(current_task_name,NO_TASK) != 0) {
    printf("New task can only be run if no other task is running!\n");
    return FALSE;
  }

  /* allow or speed adjustment */
  get_double("Frequency Multiplier",speed,&speed);

  /* read the script for this task */
  if (!read_sine_script())
    return FALSE;

  /* transient multiplier: ramps up the amplitude in one period to
     avoid discontinuous motor commands */
  trans_mult = 0.0;

  /* what is the longest period? this is what we use to ramp up trans_mult to one */
  trans_period = 0.0;
  for (i=1; i<=n_dofs; ++i)
    for (j=1; j<=n_sine[i]; ++j)
      if (freq[i][j] > 0)
        if (1./freq[i][j] > trans_period)
          trans_period = 1./freq[i][j];

  /* go to a save posture */
  bzero((char *)&(target[1]),n_dofs*sizeof(target[1]));
  for (i=1; i<=n_dofs; ++i) {
    target[i].th  = off[i];
    for (j=1; j<=n_sine[i]; ++j) {
      target[i].th  += trans_mult*amp[i][j]*sin(phase[i][j]);
    }
  }

  if (!go_target_wait(target))
    return FALSE;


  fb_id.initialize();
  ans = 0;
  if (!get_int("Use computed torques?",ans,&ans))
    return FALSE;
  if(ans == 1)
    use_computed_torques = TRUE;

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


static int run_fb_sine_task()
{
  int j, i;

  task_time += 1./(double)task_servo_rate;

  trans_mult = task_time/trans_period;
  if (trans_mult > 1.0)
    trans_mult = 1.0;

  for (i=1; i<=n_dofs; ++i) {
    joint_des_state[i].th  = off[i];
    joint_des_state[i].thd = 0.0;
    joint_des_state[i].thdd = 0.0;
    joint_des_state[i].uff = 0.0;
    for (j=1; j<=n_sine[i]; ++j) {
      joint_des_state[i].th +=
          trans_mult*amp[i][j] * sin(2.*PI*speed*freq[i][j]*task_time+phase[i][j]);
      joint_des_state[i].thd += trans_mult*amp[i][j] * 2.*PI*speed*freq[i][j] *
          cos(2.*PI*speed*freq[i][j]*task_time+phase[i][j]);
      joint_des_state[i].thdd += -trans_mult*amp[i][j] * sqr(2.*PI*speed*freq[i][j]) *
          sin(2.*PI*speed*freq[i][j]*task_time+phase[i][j]);
    }
  }

  //copy the current joint states
  SL_DJstate state[n_dofs+1];

  SL_Cstate base_pos = base_state;
  SL_quat base_ori = base_orient;

  base_pos.xdd[_X_] = 0.0;
  base_pos.xdd[_Y_] = 0.0;
  base_pos.xdd[_Z_] = 0.0;
  base_ori.add[_A_] = 0.0;
  base_ori.add[_B_] = 0.0;
  base_ori.add[_G_] = 0.0;


  for(int i=1; i<=n_dofs; ++i)
  {
    state[i].thdd = joint_des_state[i].thdd;
    state[i].uex = 0;
    state[i].uff = 0;

    if(use_computed_torques)
    {
      base_ori.q[_QW_] = 1.0;
      base_ori.q[_QX_] = 0.0;
      base_ori.q[_QY_] = 0.0;
      base_ori.q[_QZ_] = 0.0;
      base_pos.xd[_X_] = 0.0;
      base_pos.xd[_Y_] = 0.0;
      base_pos.xd[_Z_] = 0.0;
      base_ori.ad[_A_] = 0.0;
      base_ori.ad[_B_] = 0.0;
      base_ori.ad[_G_] = 0.0;
      state[i].th = joint_des_state[i].th;
      state[i].thd = joint_des_state[i].thd;
    }
    else
    {
      state[i].th = joint_state[i].th;
      state[i].thd = joint_state[i].thd;
    }
  }
  for(int i=1; i<=6; ++i)
  {
    endeff[RIGHT_FOOT].c[i] = 1;
    endeff[LEFT_FOOT].c[i] = 1;
  }

  fb_id.computeInverseDynamics(state, endeff, base_pos, base_ori);
  for(int i=L_HFE; i<=B_TFE; ++i)
  {
    joint_des_state[i].uff = state[i].uff;
  }

  return TRUE;
}

static int change_fb_sine_task()
{
  return TRUE;
}

static int
read_sine_script(void)
{
  int j,i,rc;
  static char string[100];
  static char fname[100] = "default.sine";
  FILE *fp;
  int found = FALSE;
  double total_amp;
  double ratio;

  /* clear the current parameters */
  for (i=1; i<=n_dofs; ++i) {
    off[i] = joint_default_state[i].th;
    n_sine[i] = 0;
    for (j=1; j<=MAX_SINE; ++j) {
      amp[i][j] = phase[i][j] = freq[i][j] = 0.0;
    }
  }

  /* open the script, and parse the parameters */

  while (TRUE) {
    if (!get_string("Name of the Sine Script File\0",fname,fname))
      return FALSE;

    /* try to read this file */
    sprintf(string,"%s%s",PREFS,fname);
    fp = fopen_strip(string);
    if (fp != NULL)
      break;
  }

  for (i=1; i<= n_dofs; ++i) {
    if (find_keyword(fp, &(joint_names[i][0]))) {
      found = TRUE;
      total_amp = 0.0;
      rc=fscanf(fp,"%d %lf",&n_sine[i],&off[i]);
      /* check for out of range */
      if (off[i] > joint_range[i][MAX_THETA]) {
        off[i] = joint_range[i][MAX_THETA];
        printf("Reduced offset of joint %s to %f\n",joint_names[i],off[i]);
      }
      if (off[i] < joint_range[i][MIN_THETA]) {
        off[i] = joint_range[i][MIN_THETA];
        printf("Reduced offset of joint %s to %f\n",joint_names[i],off[i]);
      }
      for (j=1; j<=n_sine[i]; ++j) {
        rc=fscanf(fp,"%lf %lf %lf",&(amp[i][j]),&(phase[i][j]),&(freq[i][j]));
        total_amp += amp[i][j];
      }
      /* check for out of range */
      if (total_amp+off[i] > joint_range[i][MAX_THETA]) {
        ratio = total_amp/(joint_range[i][MAX_THETA]-off[i]+1.e-10);
        for (j=1; j<=n_sine[i]; ++j)
          amp[i][j] /= ratio;
        printf("Reduced amplitude of joint %s to %f\n",joint_names[i],
               amp[i][j]);
      }
      if (-total_amp+off[i] < joint_range[i][MIN_THETA]) {
        ratio = total_amp/(off[i]-joint_range[i][MIN_THETA]+1.e-10);
        for (j=1; j<=n_sine[i]; ++j)
          amp[i][j] /= ratio;
        printf("Reduced amplitude of joint %s to %f\n",joint_names[i],
               amp[i][j]);
      }
    }
  }

  fclose(fp);
  remove_temp_file();


  return found;

}


