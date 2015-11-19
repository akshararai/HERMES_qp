/*============================================================================
==============================================================================
                      
                              ranger_task.c
 
==============================================================================
Remarks:

      sekeleton to create the ranger task

============================================================================*/

// system headers
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
#include "SL_objects.h"

// defines 

// local variables
static double start_time = 0.0;
static double freq;
static double amp;
static SL_DJstate  target[N_DOFS+1];
static int         use_invdyn     = TRUE;
static SL_Cstate   ctarget[N_ENDEFFS+1];
static int         stat_s[N_ENDEFFS*6+1];

static double trans[N_CART+1];
static double rot[N_CART+1];
static char   name[100];


// global functions 

// local functions 
static int  init_ranger_task(void);
static int  run_ranger_task(void);
static int  change_ranger_task(void);
static void create_contact_objects(void);


/*****************************************************************************
******************************************************************************
Function Name	: add_ranger_task
Date		: Feb 1999
Remarks:

adds the task to the task menu

******************************************************************************
Paramters:  (i/o = input/output)

none

*****************************************************************************/
void
add_ranger_task( void )

{
  int i, j;
  
  addTask("Ranger Task", init_ranger_task, 
	  run_ranger_task, change_ranger_task);

}    

/*****************************************************************************
******************************************************************************
  Function Name	: init_ranger_task
  Date		: Dec. 1997

  Remarks:

  initialization for task

******************************************************************************
  Paramters:  (i/o = input/output)

       none

 *****************************************************************************/
static int 
init_ranger_task(void)
{
  int j, i;
  int ans;
  static int firsttime = TRUE;
  
  if (firsttime){
    firsttime = FALSE;
    create_contact_objects();
  }

  // prepare going to the default posture
  bzero((char *)&(target[1]),N_DOFS*sizeof(target[1]));
  for (i=1; i<=N_DOFS; i++)
    target[i] = joint_default_state[i];

  // go to the target using inverse dynamics (ID)
  if (!go_target_wait_ID(target)) 
    return FALSE;

  // draw the ranger
  float b[N_CART] = {0.,1.0,-0.95};
  sendUserGraphics("rangerScenario",b,N_CART*sizeof(float));

  // ready to go
  ans = 999;
  while (ans == 999) {
    if (!get_int("Enter 1 to start or anthing else to abort ...",ans,&ans))
      return FALSE;
  }
  
  // only go when user really types the right thing
  if (ans != 1) 
    return FALSE;

  start_time = task_servo_time;
  printf("start time = %.3f, task_servo_time = %.3f\n", 
	 start_time, task_servo_time);

  return TRUE;
}

/*****************************************************************************
******************************************************************************
  Function Name	: run_ranger_task
  Date		: Dec. 1997

  Remarks:

  run the task from the task servo: REAL TIME requirements!

******************************************************************************
  Paramters:  (i/o = input/output)

  none

 *****************************************************************************/
static int 
run_ranger_task(void)
{
  int i;

  double task_time;

  // NOTE: all array indices start with 1 in SL

  task_time = task_servo_time - start_time;

  // the following variables need to be assigned
  for (i=1; i<=N_DOFS; ++i) {
    joint_des_state[i].th   = joint_default_state[i].th;
    joint_des_state[i].thd  = 0.0;
    joint_des_state[i].thdd = 0.0;
    joint_des_state[i].uff  = 0.0;
  }

  //SL_InvDyn(joint_state,joint_des_state,endeff,&base_state,&base_orient);


  return TRUE;
}

/*****************************************************************************
******************************************************************************
  Function Name	: change_ranger_task
  Date		: Dec. 1997

  Remarks:

  changes the task parameters

******************************************************************************
  Paramters:  (i/o = input/output)

  none

 *****************************************************************************/
static int 
change_ranger_task(void)
{
  int i;
  char string[20];

  get_string("name",name,name);

  for (i=1; i<=N_CART; ++i) {
    sprintf(string,"%d.pos",i);
    get_double(string,trans[i],&trans[i]);
  }

  for (i=1; i<=N_CART; ++i) {
    sprintf(string,"%d.rot",i);
    get_double(string,rot[i],&rot[i]);
  }

  changeObjPosByName(name,trans,rot);



  return TRUE;

}

/*****************************************************************************
******************************************************************************
  Function Name	: create_contact_objecs
  Date		: May 2013

  Remarks:

  creates the necessary objects for contacts

******************************************************************************
  Paramters:  (i/o = input/output)

  none

 *****************************************************************************/
static void
create_contact_objects(void)

{
    int    n = 0;
    int    type;
    int    contact;
    double rgb[3+1];
    double scale[N_CART+1];
    double cspecs[10];
    double ospecs[10];

    // add all the objects we need for contact checking -- just the bare minimum
    // add all cyclinders
    type = 4;
    contact = 1;
    ospecs[0] = 1;
    ospecs[1] = 10;
    cspecs[0] = 6;
    cspecs[1] = 40000;
    cspecs[2] = 200;
    cspecs[3] = 40000;
    cspecs[4] = 200;
    cspecs[5] = 0.8;
    cspecs[6] = 0.8;
    rgb[1] = 1.0;
    rgb[2] = 0.5;
    rgb[3] = 0.5;
    scale[_X_] = 0.05;
    scale[_Y_] = 0.05;

    // now add specific cyclinders
    // horizontal top bar
    sprintf(name,"cyl-%d",++n);
    trans[_X_]=0.0;
    trans[_Y_]=0.39;
    trans[_Z_]=0.97;
    rot[_A_] = 0.0;
    rot[_B_] = 1.61;
    rot[_G_] = 0.0;
    scale[_Z_] = 0.7;
    addObject(name, type, contact, rgb, trans, rot, scale, cspecs, ospecs);

    // almost vertical front bar
    sprintf(name,"cyl-%d",++n);
    trans[_X_]=0.53;
    trans[_Y_]=0.39;
    trans[_Z_]=0.5;
    rot[_A_] = 0.0;
    rot[_B_] = -.45;
    rot[_G_] = 0.0;
    scale[_Z_] = 1.05;
    addObject(name, type, contact, rgb, trans, rot, scale, cspecs, ospecs);

    // almost vertical back top bar
    sprintf(name,"cyl-%d",++n);
    trans[_X_]=-0.38;
    trans[_Y_]=0.39;
    trans[_Z_]=0.87;
    rot[_A_] = 0.0;
    rot[_B_] = 0.39;
    rot[_G_] = 0.0;
    scale[_Z_] = 0.3;
    addObject(name, type, contact, rgb, trans, rot, scale, cspecs, ospecs);

    // almost vertical back lower bar
    sprintf(name,"cyl-%d",++n);
    trans[_X_]=-0.375;
    trans[_Y_]=0.39;
    trans[_Z_]=0.32;
    rot[_A_] = 0.0;
    rot[_B_] = -.14;
    rot[_G_] = 0.0;
    scale[_Z_] = 0.90;
    addObject(name, type, contact, rgb, trans, rot, scale, cspecs, ospecs);

    // top part of handle for entering 
    sprintf(name,"cyl-%d",++n);
    trans[_X_]=-0.23;
    trans[_Y_]=0.39;
    trans[_Z_]=0.225;
    rot[_A_] = 0.0;
    rot[_B_] = 1.8;
    rot[_G_] = 0.0;
    scale[_X_] = 0.03;
    scale[_Y_] = 0.03;
    scale[_Z_] = 0.250;
    addObject(name, type, contact, rgb, trans, rot, scale, cspecs, ospecs);

    // lower part of handle for entering 
    sprintf(name,"cyl-%d",++n);
    trans[_X_]=-0.2;
    trans[_Y_]=0.39;
    trans[_Z_]=0.03;
    rot[_A_] = 0.0;
    rot[_B_] = 1.75;
    rot[_G_] = 0.0;
    scale[_Z_] = 0.250;
    addObject(name, type, contact, rgb, trans, rot, scale, cspecs, ospecs);

    // vertical part of handle for entering 
    sprintf(name,"cyl-%d",++n);
    trans[_X_]=-0.1;
    trans[_Y_]=0.39;
    trans[_Z_]=0.1;
    rot[_A_] = 0.0;
    rot[_B_] = -.15;
    rot[_G_] = 0.0;
    scale[_Z_] = 0.20;
    addObject(name, type, contact, rgb, trans, rot, scale, cspecs, ospecs);


    // cubic parts
    type = 1;
    contact = 3;
    ospecs[0] = 1;
    ospecs[1] = 10;
    cspecs[0] = 8;
    cspecs[1] = 7000;
    cspecs[2] = 20;
    cspecs[3] = 7000;
    cspecs[4] = 20;
    cspecs[5] = 0.8;
    cspecs[6] = 0.8;
    cspecs[7] = 0.5;
    cspecs[8] = 0.5;
    rgb[1] = 1.0;
    rgb[2] = 0.5;
    rgb[3] = 0.5;
    rot[_A_] = 0.0;
    rot[_B_] = 0.0;
    rot[_G_] = 0.0;

    // the floor of the ranger
    sprintf(name,"cube-%d",++n);
    trans[_X_]=0.2;
    trans[_Y_]=1.0;
    trans[_Z_]=-.6;
    scale[_X_] = 1.0;
    scale[_Y_] = 1.0;
    scale[_Z_] = 0.1;
    addObject(name, type, contact, rgb, trans, rot, scale, cspecs, ospecs);

    // the bench of the ranger
    sprintf(name,"cube-%d",++n);
    trans[_X_]=-0.05;
    trans[_Y_]=1.0;
    trans[_Z_]=-.35;
    scale[_X_] = 0.5;
    scale[_Y_] = 1.0;
    scale[_Z_] = 0.5;
    addObject(name, type, contact, rgb, trans, rot, scale, cspecs, ospecs);

    // the front plate
    sprintf(name,"cube-%d",++n);
    trans[_X_]=0.9;
    trans[_Y_]=1.0;
    trans[_Z_]=-.25;
    scale[_X_] = 0.5;
    scale[_Y_] = 1.0;
    scale[_Z_] = 0.8;
    addObject(name, type, contact, rgb, trans, rot, scale, cspecs, ospecs);

    // the back-rest of the ranger
    sprintf(name,"cube-%d",++n);
    trans[_X_]=-0.3;
    trans[_Y_]=1.0;
    trans[_Z_]=0.18;
    scale[_X_] = 0.1;
    scale[_Y_] = 1.0;
    scale[_Z_] = 0.4;
    rot[_B_] = -.18;
    addObject(name, type, contact, rgb, trans, rot, scale, cspecs, ospecs);

}
