/*
 * display_kinematics.h
 *
 *  Created on: Feb 28, 2012
 *      Author: righetti
 */

#ifndef DISPLAY_KINEMATICS_H_
#define DISPLAY_KINEMATICS_H_

#include "SL.h"
#include "SL_user.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct kinematicState
{
  SL_Cstate base_state;
  SL_quat base_orient;
  SL_Jstate joints[N_DOFS+1];
  int enabled;
} kinematicState;

void display_kinematics(void *buf);

#ifdef __cplusplus
}
#endif

#endif /* DISPLAY_KINEMATICS_H_ */
