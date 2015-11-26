/*============================================================================
==============================================================================

                              initUserTasks.c

==============================================================================
Remarks:

         Functions needed to initialize and link user tasks for the
         simulation

============================================================================*/

// system headers
#include "SL_system_headers.h"

// local headers
#include "SL.h"
#include "SL_user.h"
#include "SL_integrate.h"
#include "SL_common.h"
#include "SL_dynamics.h"

// global variables

// local variables
static int user_tasks_initialized = FALSE;

// external functions
extern void toggleSimulatedBaseState(void);


/*****************************************************************************
 ******************************************************************************
Function Name	: initUserTasks
Date		: June 1999

Remarks:

      initialize tasks that are not permanently linked in the simulation
      This replaces the <ltask facility in vxworks -- just that we cannot
      do on-line linking in C.

 ******************************************************************************
Paramters:  (i/o = input/output)

  none   

 *****************************************************************************/
void
initUserTasks(void)
{
  //extern void add_calibrate_base_state_estimation_task();
  //add_calibrate_base_state_estimation_task();
  //freezeBase(TRUE);


  // Alex added here new task (I think it is not like this, I can add new files in biped_locomotion_lib_tasks)
  //extern void add_wholebody_task();
  //add_wholebody_task();

  // use the true base state from the simulation servo
  if(!real_robot_flag)
    toggleSimulatedBaseState();

#ifdef BIPED_LOCOMOTION_LIB
  extern void add_biped_locomotion_lib_tasks();
  add_biped_locomotion_lib_tasks();
#endif

#ifdef USE_ATI_SENSOR
  extern void add_ati_sensor();
  add_ati_sensor();
#endif
}
