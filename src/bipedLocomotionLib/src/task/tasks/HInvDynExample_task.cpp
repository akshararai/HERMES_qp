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

#include "HInvDynExample.h"

#include "SL_tasks.h"

boost::shared_ptr<hierarchical_inverse_dynamics_example::
    HInvDynExample> hinvdyn_ctrl_;

extern "C"
{
/* global functions */
void add_hinvdyn_example_task(void);

/* local functions */
static int init_hinvdyn_example_task(void);
static int run_hinvdyn_example_task(void);
static int change_hinvdyn_example_task(void);
}

void add_hinvdyn_example_task(void){
  addTask("HInvDyn Example", init_hinvdyn_example_task, run_hinvdyn_example_task, change_hinvdyn_example_task);
}

int change_hinvdyn_example_task(){
  return TRUE;
}

int init_hinvdyn_example_task(){
  hinvdyn_ctrl_.reset(new hierarchical_inverse_dynamics_example::HInvDynExample());
  return TRUE;
}

int run_hinvdyn_example_task(){
  return hinvdyn_ctrl_->run();
}



