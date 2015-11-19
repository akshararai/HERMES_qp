/*
 * display_kinematics.c
 *
 *  Created on: Feb 28, 2012
 *      Author: righetti
 */

#include "GL/glut.h"
#include "GL/glu.h"

#include "display_kinematics.h"
#include "SL_openGL.h"

void display_kinematics(void *buf)
{
  kinematicState* user_data = (kinematicState *)buf;
  userGraphics_base_orient = user_data->base_orient;
  userGraphics_base_state = user_data->base_state;

  int i;
  for (i=1; i<=N_DOFS; ++i)
  {
    userGraphics_joint_state[i] = user_data->joints[i];
  }

  //printf("%f\n", userGraphics_base_state.x[_Z_]);

  setUserGraphicsUpdateMode(user_data->enabled);
}


