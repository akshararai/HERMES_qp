/*
 * balance_task.cpp
 *
 *  Created on: Aug 28, 2013
 *      Author: righetti
 */

#include <Eigen/Eigen>

#include "balance_controller.h"

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
void add_balance_task(void);

/* local functions */
static int init_balance_task(void);
static int run_balance_task(void);
static int change_balance_task(void);
}

BalanceController balance_ctrl;

void add_balance_task(void)
{
  addTask("balance Task", init_balance_task, run_balance_task, change_balance_task);
}

int change_balance_task()
{
  return TRUE;
}

int init_balance_task()
{
  if(balance_ctrl.initialize())
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

int run_balance_task()
{
  return int(balance_ctrl.run());
}
