/*
 * walking_task.cpp
 *
 *  Created on: Apr 28, 2013
 *      Author: righetti
 */

#include <Eigen/Eigen>

#include "walking_controller.h"

#include "SL.h"
#include "SL_user.h"
#include "SL_tasks.h"
#include "SL_task_servo.h"
#include "SL_collect_data.h"
#include "SL_objects.h"
#include "utility_macros.h"


extern "C"
{
/* global functions */
void add_walking_task(void);

/* local functions */
static int init_walking_task(void);
static int run_walking_task(void);
static int change_walking_task(void);
}

WalkingController walking_ctrl;

void add_walking_task(void)
{
  addTask("Walking Task", init_walking_task, run_walking_task, change_walking_task);
}

int change_walking_task()
{
  return TRUE;
}

int init_walking_task()
{
  if(walking_ctrl.initialize())
  {
    // ready to go
    int ans = 999;
    while (ans == 999) {
      if (!get_int("Enter 1 to start or anthing else to abort ...",ans,&ans))
        return FALSE;
    }
    scd();
    return TRUE;
  }
  else
    return FALSE;
}

int run_walking_task()
{
  return int(walking_ctrl.run());
}
