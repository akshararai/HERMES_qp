#ifndef WHOLEBODY_TASK_H
#define WHOLEBODY_TASK_H

//#include "mytest.h"

#include <Eigen/Eigen>
#include <iostream>
//#include "SL.h"
using namespace std;

// some info about task


/* local variables, newly added */
double task_start_time;
double real_time = 0.0;
double transition_time = 2.0;


// config file

string config_file_;

double push_force_=100, push_duration_=0.5;

//Eigen::Matrix<double, n_dofs,1> init_joint_state_uff, init_joint_state_th, init_joint_state_thd, init_joint_state_thdd;


#endif // WHOLEBODY_TASK_H
