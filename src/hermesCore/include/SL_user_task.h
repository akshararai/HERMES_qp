/*!
 * @file
 * some definitions useful for the task servo
 *
 * @authors Ludovic Righetti
*/

#ifndef SL_USER_TASK_H
#define SL_USER_TASK_H

#include "SL_user.h"

// external functions
#ifdef __cplusplus
extern "C"
{
SL_Jstate raw_joint_state[N_DOFS+1];
double raw_misc_sensor[N_MISC_SENSORS+1];
}
#endif


#endif // SL_USER_TASK_H

