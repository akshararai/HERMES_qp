/*============================================================================
==============================================================================
                      
                              initUserGraphics.c
 
==============================================================================
Remarks:

         Functions needed for user graphics
         simulation

============================================================================*/

#include "SL.h"
#include "SL_user.h"
#include "SL_man.h"
#include "string.h"

// openGL includes
#include "GL/glut.h"
#include "SL_openGL.h"
#include "SL_userGraphics.h"
#include "SL_objects.h"

#ifdef BIPED_LOCOMOTION_LIB
#include "display_kinematics.h"
#include "drawUtils.h"
#include "capture_opengl.h"
#endif
// global variables

// local variables
static int n_bytes_endeffectors;
static double opacity = 0.5;

// local functions
static void display_endeffector_target(void *cbuf);
static void display_ranger_scenario(void *cbuf);


/*****************************************************************************
******************************************************************************
Function Name	: initUserGraphics
Date		: June 1999
   
Remarks:

      allows adding new graphics functions to openGL interface

******************************************************************************
Paramters:  (i/o = input/output)

  none   

*****************************************************************************/
int
initUserGraphics(void)

{


  n_bytes_endeffectors= N_ENDEFFS*N_CART*sizeof(float);
  addToUserGraphics("endeffectorTarget","Display targets for the hands and feet",
		    display_endeffector_target,n_bytes_endeffectors);

  addToUserGraphics("rangerScenario","Display Polaris ranger scenario",
		    display_ranger_scenario,N_CART*sizeof(float));
#ifdef BIPED_LOCOMOTION_LIB
  addToUserGraphics("display_kinematics", "display arbitrary kin", display_kinematics, sizeof(struct kinematicState));
  addToUserGraphics("draw_ball", "draw a ball", drawBall, sizeof(struct BallsInfo));
  addToUserGraphics("capture_frame", "captures a frame", captureFrame, 300*sizeof(char));
#endif
  return TRUE;

}

/*****************************************************************************
 *****************************************************************************
 Function Name	: display_endeffector_target
 Date		: Aug. 99
 
 Remarks:

 displays a ball for simulations

*****************************************************************************
 Paramters:  (i/o = input/output)

     none

****************************************************************************/
static void
display_endeffector_target(void *cbuf)
{
  int i,j;
  static int firsttime = TRUE;
  GLfloat  col[4+1][4] ={{0.0,0.0,0.0,0.0},
			 {(float)1.0,(float)1.0,(float)0.0,(float)1.0},
			 {(float)0.3,(float)0.3,(float)1.0,(float)1.0},
			 {(float)0.8,(float)0.3,(float)1.0,(float)1.0},
			 {(float)0.0,(float)1.0,(float)0.2,(float)1.0}};
  SL_Cstate endeffector_target[N_ENDEFFS+1];
  float buf[n_bytes_endeffectors/sizeof(float)+1];
  int   count = 0;
  double sd = 0.01;

  if (firsttime) {
    firsttime = FALSE;
  }

  // copy to float buffer
  memcpy(&(buf[1]),cbuf,n_bytes_endeffectors);

  // parse foot targets and nominal
  for (i=1; i<=N_ENDEFFS; ++i) {
    for (j=1; j<=N_CART; ++j) {
      endeffector_target[i].x[j] = buf[++count];
    }
  }

  // the foot targets of the feet
  for (i=1; i<=N_ENDEFFS; ++i) {
    glPushMatrix();
    glTranslated((GLdouble)endeffector_target[i].x[_X_],
		 (GLdouble)endeffector_target[i].x[_Y_],
		 (GLdouble)endeffector_target[i].x[_Z_]);
      
    glColor4fv(col[i]);
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, col[i]);
    glutSolidSphere(sd,8,8);
    glPopMatrix();
  }

}

/*****************************************************************************
 *****************************************************************************
 Function Name	: display_ranger_scencario
 Date		: May 2013
 
 Remarks:

 displays the scenario of the ranger for DRC

*****************************************************************************
 Paramters:  (i/o = input/output)

     none

****************************************************************************/
static void
display_ranger_scenario(void *cbuf)
{
  int i,j;
  static int firsttime = TRUE;
  GLfloat  gray[4]={(float)0.8,(float)0.8,(float)0.8,(float)opacity};
  GLfloat  green[4]={(float)0.1,(float)0.5,(float)0.5,(float)opacity};
  GLfloat  red[4]={(float)1.0,(float)0.25,(float)0.25,(float)opacity};
  GLfloat  blue[4]={(float)0.1,(float)0.1,(float)1.0,(float)opacity};
  GLfloat  black[4]={(float)0.,(float)0.,(float)0.0,(float)opacity};
  GLfloat  white[4]={(float)1.,(float)1.,(float)1.0,(float)opacity};
  GLfloat  yellow[4]={(float)1.0,(float)0.8,(float)0.7,(float)opacity};
  double     position[N_CART+1];
  float      buf[N_CART+1];
  static int polaris;


  if (firsttime) {
    char  fname[300];
    sprintf(fname,"3D/polaris_ev.obj");
    polaris  = displayListFromObjFileFlag(fname,1.0,FALSE);

    firsttime = FALSE;

  }

  // copy to float buffer
  memcpy(&(buf[1]),cbuf,N_CART*sizeof(float));
  for (i=1; i<=N_CART; ++i) 
    position[i] = buf[i];

  glPushMatrix();
  glEnable(GL_BLEND);
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  // translate to position
  glTranslated((GLdouble)position[_X_],(GLdouble)position[_Y_],(GLdouble)position[_Z_]);


  // draw ranger
  glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, blue);
  glColor4fv(blue);
  glCallList(polaris);

  glEnable(GL_LIGHTING);   
  glDisable(GL_BLEND);
  glPopMatrix();

}


