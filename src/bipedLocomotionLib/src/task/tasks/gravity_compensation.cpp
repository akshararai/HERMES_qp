/*!=============================================================================
  ==============================================================================

  \file    gravity_compensation.cpp

  \author  righetti
  \date    Dec 19, 2012

  ==============================================================================
  \remarks


  ============================================================================*/



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
static int fb_compensation = FALSE;
static int use_computed_torques = FALSE;
static floating_base_utilities::OptimalInverseDynamicsEigen fb_id;
static Eigen::Matrix<double, N_DOFS, 1> initial_uff, initial_th, initial_thd;
static double task_time_start;
static double transition_dur = 2.; //[s]

extern "C"{

void add_gravity_compensation_task();

static int init_gravity_compensation_task();
static int run_gravity_compensation_task();
static int change_gravity_compensation_task();
}


void add_gravity_compensation_task()
{
  addTask("Gravity Compensation", init_gravity_compensation_task,
          run_gravity_compensation_task, change_gravity_compensation_task);
}


static int init_gravity_compensation_task()
{
  int j, i;
  int ans=0;
  static int firsttime = TRUE;

  //to make sure it doesn't crash
  stopcd();

  if (firsttime){
    firsttime = FALSE;
  }

  if (!get_int("Do gravity compensation in floating space?",ans,&ans))
    return FALSE;

  if(ans == 1)
  {
    fb_id.initialize();
    fb_compensation = TRUE;
    ans = 0;
    if (!get_int("Use computed torques?",ans,&ans))
      return FALSE;
    if(ans == 1)
      use_computed_torques = TRUE;
  }
  else
    fb_compensation = FALSE;

  // ready to go
  ans = 999;
  while (ans == 999) {
    if (!get_int("Enter 1 to start or anthing else to abort ...",ans,&ans))
      return FALSE;
  }

  // only go when user really types the right thing
  if (ans != 1)
    return FALSE;


  for(int i=1;i<=N_DOFS;++i){
	initial_uff[i-1] = joint_des_state[i].uff;
	initial_th[i-1] = joint_des_state[i].th;
	initial_thd[i-1] = joint_des_state[i].thd;
  }
  task_time_start = task_servo_time;

  scd();

  return TRUE;
}


static int run_gravity_compensation_task()
{
  //copy the current joint states
  SL_Jstate state[n_dofs+1];
  for(int i=1; i<=n_dofs; ++i)
    state[i] = joint_state[i];

  SL_Cstate base_pos = base_state;
  SL_quat base_ori = base_orient;

  base_pos.xdd[_X_] = 0.0;
  base_pos.xdd[_Y_] = 0.0;
  base_pos.xdd[_Z_] = 0.0;
  base_ori.add[_A_] = 0.0;
  base_ori.add[_B_] = 0.0;
  base_ori.add[_G_] = 0.0;

  //  base_pos.xd[_X_] = 0.0;
  //  base_pos.xd[_Y_] = 0.0;
  //  base_pos.xd[_Z_] = 0.0;
  //  base_ori.ad[_A_] = 0.0;
  //  base_ori.ad[_B_] = 0.0;
  //  base_ori.ad[_G_] = 0.0;

  SL_DJstate des_state[n_dofs+1];


  //FLOATING BASE CASE

  double transition = (1./transition_dur)*std::min(transition_dur, task_servo_time - task_time_start);
  if(fb_compensation)
  {
    for(int i=1; i<=n_dofs; ++i)
    {
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
        des_state[i].th = joint_des_state[i].th;
      }
      else
        des_state[i].th = joint_state[i].th;
      des_state[i].thd = 0.0;
      des_state[i].thdd = 0.0;
      des_state[i].uff = 0.0;
      des_state[i].uex = 0.0;
    }
    for(int i=1; i<=6; ++i)
    {
      endeff[RIGHT_FOOT].c[i] = 1;
      endeff[LEFT_FOOT].c[i] = 1;
    }

    fb_id.computeInverseDynamics(des_state, endeff, base_pos, base_ori);
    for(int i=L_HFE; i<=R_AAA; ++i)
    {
	  joint_des_state[i].thd = /*transition*0. + */ (1.-transition)*initial_thd[i-1];
      joint_des_state[i].uff = transition*des_state[i].uff + (1.-transition)*initial_uff[i-1];
    }
  }
  else//NO FLOATING BASE
  {
    for(int i=1; i<=n_dofs; ++i)
    {
      des_state[i].th = state[i].th;
      des_state[i].thd = 0.0;
      des_state[i].thdd = 0.0;
      des_state[i].uff = 0.0;
      des_state[i].uex = 0.0;
    }

    //compute inverse dynamics
    double fbase[6+1];
    SL_InvDynNEBase(NULL, des_state, endeff, &base_pos, &base_ori, fbase);


    for(int i=L_HFE; i<=R_AAA; ++i)
    {
      if(i==L_HFE || i==L_KFE || i==R_HFE || i==R_KFE || i==L_AFE || i==L_AAA || i==R_AFE || i==R_AAA || i==L_HAA || i==R_HAA)
      {
        joint_des_state[i].th = transition*joint_state[i].th + (1.-transition)*initial_th[i-1];
        joint_des_state[i].thd = /*transition*0. + */ (1.-transition)*initial_thd[i-1];
        joint_des_state[i].uff = transition*des_state[i].uff + (1.-transition)*initial_uff[i-1];
      }
    }
  }

  return TRUE;
}

static int change_gravity_compensation_task()
{
  return TRUE;
}



