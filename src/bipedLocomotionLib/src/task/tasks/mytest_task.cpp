/*********************************************************************
 Autonomous Motion Department
 Max-Planck Intelligent Systems
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         HInvDynExample_task.cpp

 \author       Alexander Herzog
 \date         Aug 18, 2014

 *********************************************************************/

#include "mytest.h"

#include "SL_tasks.h"

boost::shared_ptr<hierarchical_inverse_dynamics_example::
    mytest> mytest_control;

extern "C"
{
/* global functions */
void add_mytest_task(void);

/* local functions */
static int init_mytest_task(void);
static int run_mytest_task(void);
static int change_mytest_task(void);
}

void add_mytest_task(void){
  addTask("My Test", init_mytest_task, run_mytest_task, change_mytest_task);
}

int change_mytest_task(){
  return TRUE;
}

int init_mytest_task(){
  mytest_control.reset(new hierarchical_inverse_dynamics_example::mytest());
  return TRUE;
}

int run_mytest_task(){
  return mytest_control->run();
}



