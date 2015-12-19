/*============================================================================
==============================================================================
                      
                              sample_task.c
 
==============================================================================
Remarks:

      sekeleton to create the sample task

============================================================================*/

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

/* defines */

/* local variables */
static double start_time = 0.0;


/* global functions */

/* local functions */
static int  init_zero_task(void);
static int  run_zero_task(void);
static int  change_zero_task(void);

static SL_DJstate *target;
/*****************************************************************************
******************************************************************************
Function Name	: add_sample_task
Date		: Feb 1999
Remarks:

adds the task to the task menu

******************************************************************************
Paramters:  (i/o = input/output)

none

*****************************************************************************/
void
add_zero_task( void )

{
  int i, j;
  static int firsttime = TRUE;
  
  if (firsttime) {
    firsttime = FALSE;
    
    target     = (SL_DJstate *)my_calloc(n_dofs+1,sizeof(SL_DJstate), MY_STOP);
  }
  addTask("Zero position Task", init_zero_task, 
	  run_zero_task, change_zero_task);

}    

/*****************************************************************************
******************************************************************************
  Function Name	: init_sample_task
  Date		: Dec. 1997

  Remarks:

  initialization for task

******************************************************************************
  Paramters:  (i/o = input/output)

       none

 *****************************************************************************/
static int 
init_zero_task(void)
{
  int j, i;
  int ans;

  printf("Reached %d \n", N_DOFS);
  target[L_SFE].th  = -0.35;
  target[L_SAA].th = -0.2;
  target[L_HR].th = 0.0;
  target[L_EB].th = 0.2;
  target[L_WR].th = 0.0;
  target[L_WFE].th = 0.1;
  target[L_WAA].th = 0.0;

  target[R_SFE].th  = -0.05;
  target[R_SAA].th = -0.2;
  target[R_HR].th = 0.0;
  target[R_EB].th = 0.1;
  target[R_WR].th = 0.0;
  target[R_WFE].th = 0.0;
  target[R_WAA].th = 0.0;

  target[L_HFE].th = 0.125;
  target[L_HAA].th = -0.15;
  target[L_HFR].th = 0.0;
  target[L_KFE].th = 0.35;
  target[L_AR].th = 0.1;
  target[L_AFE].th = 0.45;
  target[L_AAA].th = 0.1;


  target[R_HFE].th = 0.175;
  target[R_HAA].th = -0.1;
  target[R_HFR].th = 0.0;
  target[R_KFE].th = 0.4;
  target[R_AR].th = 0.05;
  target[R_AFE].th = 0.35;
  target[R_AAA].th = 0.15;

  target[B_TR].th = -0.3;
  target[B_TAA].th = 0.0;
  target[B_TFE].th = -0.17;

  printf("Reached 2 \n");

  target[B_HN].th = 0.100;
  target[B_HT].th = -0.050;//  thd= 0.000  thdd=    0.000  ff=  0.00
 target[B_HR].th= 0.300;//  thd= 0.000  thdd=    0.000  ff=  0.00
 target[R_EP].th = 0.000;//  thd= 0.000  thdd=    0.000  ff=  0.00
 target[R_ET].th= 0.000;//  thd= 0.000  thdd=    0.000  ff=  0.00
 target[L_EP].th= 0.000;//  thd= 0.000  thdd=    0.000  ff=  0.00
 target[L_ET].th=-0.000;//  thd= 0.000  thdd=    0.000  ff=  0.00
target[L_THR].th= 0.000;//  thd= 0.000  thdd=    0.000  ff=  0.00
target[L_THF].th= 0.200;//  thd= 0.000  thdd=    0.000  ff=  0.00
 target[L_IF].th= 0.400;//  thd= 0.000  thdd=    0.000  ff=  0.00
 target[L_MF].th= 0.400;//  thd= 0.000  thdd=    0.000  ff=  0.00
 target[L_RF].th= 0.400;//  thd= 0.000  thdd=    0.000  ff=  0.00
 target[L_LF].th= 0.400;//  thd= 0.000  thdd=    0.000  ff=  0.00
 target[R_THR].th= 0.000;//  thd= 0.000  thdd=    0.000  ff=  0.00
 target[R_THF].th= 0.200;//  thd= 0.000  thdd=    0.000  ff=  0.00
 target[R_IF].th= 0.400;//  thd= 0.000  thdd=    0.000  ff=  0.00
 target[R_MF].th= 0.400;//  thd= 0.000  thdd=    0.000  ff=  0.00
 target[R_RF].th= 0.400;//  thd= 0.000  thdd=    0.000  ff=  0.00
 target[R_LF].th= 0.400;//  thd= 0.000  thdd=    0.000  ff=  0.00

  printf("Reached 2 \n");


  // ready to go
  ans = 999;
  while (ans == 999) {
    if (!get_int("Enter 1 to start or anthing else to abort ...",ans,&ans))
      return FALSE;
  }
  
  // only go when user really types the right thing
  if (ans != 1) 
    return FALSE;

  return TRUE;
}

/*****************************************************************************
******************************************************************************
  Function Name	: run_sample_task
  Date		: Dec. 1997

  Remarks:

  run the task from the task servo: REAL TIME requirements!

******************************************************************************
  Paramters:  (i/o = input/output)

  none

 *****************************************************************************/
static int 
run_zero_task(void)
{
  int i;
  for(i=1; i<=N_DOFS; i++)  {
    joint_des_state[i].th = target[i].th;
  }
  joint_des_state[R_AAA].uff = 20.0;
  joint_des_state[R_SAA].uff = 10.0;

  SL_InvDyn(joint_state,joint_des_state,endeff,&base_state,&base_orient);
  return TRUE;
}

/*****************************************************************************
******************************************************************************
  Function Name	: change_sample_task
  Date		: Dec. 1997

  Remarks:

  changes the task parameters

******************************************************************************
  Paramters:  (i/o = input/output)

  none

 *****************************************************************************/
static int 
change_zero_task(void)
{
  int    ivar;
  double dvar;

  get_int("This is how to enter an integer variable",ivar,&ivar);
  get_double("This is how to enter a double variable",dvar,&dvar);

  return TRUE;

}

