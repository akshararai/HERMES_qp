/*!=============================================================================
  ==============================================================================

  \file    SL_user_openGL.c

  \author  Stefan Schaal
  \date    Nov. 2007

  ==============================================================================
  \remarks
  
  This program contains all graphics user functions for a particular simulation.
  
  ============================================================================*/
  
// SL general includes of system headers
#include "SL_system_headers.h"

// openGL headers
#include "GL/freeglut_std.h"
#include "GL/freeglut_ext.h"
#include "GL/glu.h"
#include <X11/Xlib.h>

// user specific headers
#include "utility.h"
#include "utility_macros.h"

#include "SL.h"
#include "SL_openGL.h"
#include "SL_user.h"
#include "SL_common.h"
#include "SL_unix_common.h"
#include "SL_objects.h"
#include "SL_openGL_servo.h"
#include "SL_collect_data.h"
#include "mdefs.h"
#include "SL_kinematics.h"
#include "SL_shared_memory.h"
#include "SL_userGraphics.h"
#include "SL_man.h"

// global variables 
int print_Hmat = FALSE;

// local variables
static  OpenGLWPtr w_left_eye_fovea=NULL;
static  OpenGLWPtr w_right_eye_fovea=NULL;
static  OpenGLWPtr w_left_eye_wide=NULL;
static  OpenGLWPtr w_right_eye_wide=NULL;
static  double     fscale = 0.01;
static  double     opacity = 0.2; // between 0 and 1 the level of opacity
static  int        comsDisplay = FALSE;

  
// local functions
static int createWindows(void);
static void myDrawGLElement(int num, double length, int flag);
static void drawCenterOfPressure(void);
static void drawCOG(void);
static void drawIMU(void);
static void drawFootSensor(void);
static void updateEyes(void);

static void drawCOMs(SL_Jstate *state, SL_endeff *eff, SL_Cstate *basec, SL_quat *baseo);

static void displayPhantom(void *b);

static void drawRobot(SL_Jstate *state, SL_Cstate *basec, SL_quat *baseo);
static void toggleCOMsDisplay(void);



// global functions
void display(void);
void idle(void);

// external functions


/*!*****************************************************************************
 *******************************************************************************
 \note  init_user_openGL
 \date  July 1998
 
 \remarks 
 
 initializes everything needed for the graphics for this simulation
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     argc : number of elements in argv
 \param[in]     argv : array of argc character strings
 

 ******************************************************************************/
int
init_user_openGL(int argc, char** argv)

{
  
  int i,j,n;
  int rc;
  int ans;

  // we need the kinematics initialized
  init_kinematics();

  // read objects
  readObjects(config_files[OBJECTS]);

  // assign contact force mappings
#include "LEKin_contact.h"

  // create simulation windows
  if (!createWindows())
    return FALSE;

  // user graphics
  addToUserGraphics("phantom","Display a phantom robot",
		    displayPhantom,(N_CART+N_QUAT+n_dofs)*sizeof(float));

  addToMan("comsDisplay","toggle drawing COMs per link",toggleCOMsDisplay);


  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
 \note  createWindows
 \date  July 1998
 
 \remarks 
 
 initializes graphic windows
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 none
 

 ******************************************************************************/
static int
createWindows(void)

{

  int i;
  OpenGLWPtr w;
  int width_eye=640/3;
  int height_eye=480/3;
  int height_main = 400;
  int width_main = 400;
  int hspace = 10;
  int vspace = 60;
  
  Display *disp;
  int  screen_num;
  int  display_width;
  int  display_height;

  char string[100];
  char xstring[100];

  double eye[N_CART+1];
  
  // connect to X server using the DISPLAY environment variable
  if ( (disp=XOpenDisplay(NULL)) == NULL ) {
    printf("Cannot connect to X servo %s\n",XDisplayName(NULL));
    exit(-1);
  }
  
  // get screen size from display structure macro 
  screen_num = DefaultScreen(disp);
  display_width = DisplayWidth(disp, screen_num);
  display_height = DisplayHeight(disp, screen_num);
  
  /* get a window structure, initialized with default values */
  w=getOpenGLWindow();
  if (w==NULL)
    return FALSE;

  w->display = display;
  w->idle    = idle;
  w->width   = width_main;
  w->height  = height_main;

  // check for user parameters
  if (read_parameter_pool_string(config_files[PARAMETERPOOL], 
				 "main_window_geometry", string))
    parseWindowSpecs(string, display_width,display_height,xstring, 
		     &(w->x), 
		     &(w->y), 
		     &(w->width),
		     &(w->height));

  if (read_parameter_pool_double_array(config_files[PARAMETERPOOL], 
				       "main_window_camera_pos", N_CART, eye))
    for (i=1; i<=N_CART; ++i)
      w->eye[i] = eye[i];
  
  // finally create the window
  for (i=1; i<=N_CART; ++i)
    w->eye0[i] = w->eye[i];

  if (!createWindow(w))
    return FALSE;

  /* get additional windows for the eyes */
  w=getOpenGLWindow();
  if (w==NULL)
    return FALSE;

  w->x      += width_main+hspace;
  w->fovea   = FOVEA_ANGLE;
  w->width   = width_eye;
  w->height  = height_eye;
  w->display = display;
  w->idle    = idle;
  sprintf(w->name,"LeftEyeFovea");
  if (!createWindow(w))
    return FALSE;

  toggleHideWindow(w);
  w_left_eye_fovea = w;

  w=getOpenGLWindow();
  if (w==NULL)
    return FALSE;

  w->x      += width_main+hspace+width_eye+hspace;
  w->fovea   = FOVEA_ANGLE;
  w->width   = width_eye;
  w->height  = height_eye;
  w->display = display;
  w->idle    = idle;
  sprintf(w->name,"RightEyeFovea");
  if (!createWindow(w))
    return FALSE;

  toggleHideWindow(w);
  w_right_eye_fovea = w;

  w=getOpenGLWindow();
  if (w==NULL)
    return FALSE;

  w->x      += width_main+hspace;
  w->y      += height_eye+vspace;
  w->fovea   = WIDE_ANGLE;
  w->width   = width_eye;
  w->height  = height_eye;
  w->display = display;
  w->idle    = idle;
  sprintf(w->name,"LeftEyeWide");
  if (!createWindow(w))
    return FALSE;
  
  toggleHideWindow(w);
  w_left_eye_wide = w;

  w=getOpenGLWindow();
  if (w==NULL)
    return FALSE;

  w->x      += width_main+hspace+width_eye+hspace;
  w->y      += height_eye+vspace;
  w->fovea   = WIDE_ANGLE;
  w->width   = width_eye;
  w->height  = height_eye;
  w->display = display;
  w->idle    = idle;
  sprintf(w->name,"RightEyeWide");
  if (!createWindow(w))
    return FALSE;

  toggleHideWindow(w);
  w_right_eye_wide = w;

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  display
\date  August 7, 1992
   
\remarks 

        this function updates the OpenGL graphics

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output


 ******************************************************************************/
void 
display(void)

{
  int i;
  static SL_Jstate  *state = joint_sim_state;
  static SL_endeff  *eff   = endeff;
  static SL_Cstate  *basec = &base_state;
  static SL_quat    *baseo = &base_orient;
  GLfloat  objscolor[4]={(float)0.2,(float)0.2,(float)0.2,(float)1.0};
  static int    firsttime = TRUE;

  if (firsttime) {
    firsttime = FALSE;
    double w;
    if (read_parameter_pool_double(config_files[PARAMETERPOOL],"contact_force_scale",&w))
      fscale = w;
  }

  drawRobot(state,basec,baseo);

  // the standard display functions for openGL
#include "SL_user_display_core.h"

  drawCenterOfPressure();
  drawCOG();
  drawIMU();
  drawFootSensor();

  if (comsDisplay)
    drawCOMs(state, eff, basec, baseo);

}

/*!*****************************************************************************
 *******************************************************************************
\note  myDrawGLElement
\date  August 7, 1992
   
\remarks 

        draws a GL element of a particular length in z direction

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     num   : ID number of element (just for info)
 \param[in]     length: length of element
 \param[in]     flag  : draw joint element (yes, not)


 ******************************************************************************/
static void  
myDrawGLElement(int num, double length, int flag)

{
		
		
  static int firsttime = TRUE;
  double width=0.03;
  double head_width = 0.3;
  double hand_width = 0.07;
  double foot_width = 0.1;
  double torso_width = 0.3;
  double belly_width = 0.25;
  double thumb_width = 0.02;
  double shoulder_width  = 0.4;

  static int pelvis,b_tr,b_taa,b_tfe;
  static int l_haa, l_hfe, l_hfr, l_kfe, l_ar, l_afe, l_aaa;
  static int r_haa, r_hfe, r_hfr, r_kfe, r_ar, r_afe, r_aaa;
  static int b_hn, b_ht, b_hr;
  static int head,left_eye,right_eye;
  static int r_sfe, r_saa, r_hr, r_eb, r_wr, r_wfe, r_waa;
  static int l_sfe, l_saa, l_hr, l_eb, l_wr, l_wfe, l_waa;


  GLfloat  gray[4]={(float)0.8,(float)0.8,(float)0.8,(float)opacity};
  GLfloat  green[4]={(float)0.1,(float)0.5,(float)0.5,(float)opacity};
  GLfloat  red[4]={(float)1.0,(float)0.25,(float)0.25,(float)opacity};
  GLfloat  blue[4]={(float)0.1,(float)0.1,(float)1.0,(float)opacity};
  GLfloat  black[4]={(float)0.,(float)0.,(float)0.0,(float)opacity};
  GLfloat  white[4]={(float)1.,(float)1.,(float)1.0,(float)opacity};
  GLfloat  yellow[4]={(float)1.0,(float)0.8,(float)0.7,(float)opacity};

  int      isphere = 10;

  if (solid)
    glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
  else
    glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );

  if (firsttime) {
    firsttime = FALSE;

    char *lab_root;
    char  fname[300];

    lab_root = getenv("LAB_ROOT");

    sprintf(fname,"%s/hermesCore/3D/Pelvis.obj",lab_root);
    pelvis         = displayListFromObjFileFlag(fname,0.0256,TRUE);

    sprintf(fname,"%s/hermesCore/3D/B_TR.obj",lab_root);
    b_tr           = displayListFromObjFileFlag(fname,0.0256,TRUE);

    sprintf(fname,"%s/hermesCore/3D/B_TAA.obj",lab_root);
    b_taa          = displayListFromObjFileFlag(fname,0.0256,TRUE);

    sprintf(fname,"%s/hermesCore/3D/B_TFE.obj",lab_root);
    b_tfe          = displayListFromObjFileFlag(fname,0.0256,TRUE);

    sprintf(fname,"%s/hermesCore/3D/L_HFE.obj",lab_root);
    l_hfe          = displayListFromObjFileFlag(fname,0.0256,TRUE);

    sprintf(fname,"%s/hermesCore/3D/L_HAA.obj",lab_root);
    l_haa          = displayListFromObjFileFlag(fname,0.0256,TRUE);

    sprintf(fname,"%s/hermesCore/3D/L_HFR.obj",lab_root);
    l_hfr          = displayListFromObjFileFlag(fname,0.0256,TRUE);

    sprintf(fname,"%s/hermesCore/3D/L_KFE.obj",lab_root);
    l_kfe          = displayListFromObjFileFlag(fname,0.0256,TRUE);

    sprintf(fname,"%s/hermesCore/3D/L_AR.obj",lab_root);
    l_ar          = displayListFromObjFileFlag(fname,0.0256,TRUE);

    sprintf(fname,"%s/hermesCore/3D/L_AFE.obj",lab_root);
    l_afe          = displayListFromObjFileFlag(fname,0.0256,TRUE);

    sprintf(fname,"%s/hermesCore/3D/L_AAA.obj",lab_root);
    l_aaa          = displayListFromObjFileFlag(fname,0.0256,FALSE);

    sprintf(fname,"%s/hermesCore/3D/L_HFE.obj",lab_root);
    r_hfe          = displayListFromObjFileFlag(fname,0.0256,TRUE);

    sprintf(fname,"%s/hermesCore/3D/L_HAA.obj",lab_root);
    r_haa          = displayListFromObjFileFlag(fname,0.0256,TRUE);

    sprintf(fname,"%s/hermesCore/3D/L_HFR.obj",lab_root);
    r_hfr          = displayListFromObjFileFlag(fname,0.0256,TRUE);

    sprintf(fname,"%s/hermesCore/3D/L_KFE.obj",lab_root);
    r_kfe          = displayListFromObjFileFlag(fname,0.0256,TRUE);

    sprintf(fname,"%s/hermesCore/3D/L_AR.obj",lab_root);
    r_ar          = displayListFromObjFileFlag(fname,0.0256,TRUE);

    sprintf(fname,"%s/hermesCore/3D/L_AFE.obj",lab_root);
    r_afe          = displayListFromObjFileFlag(fname,0.0256,TRUE);

    sprintf(fname,"%s/hermesCore/3D/R_AAA.obj",lab_root);
    r_aaa          = displayListFromObjFileFlag(fname,0.0256,FALSE);

    sprintf(fname,"%s/hermesCore/3D/B_HN.obj",lab_root);
    b_hn           = displayListFromObjFileFlag(fname,0.0256,TRUE);

    sprintf(fname,"%s/hermesCore/3D/B_HT.obj",lab_root);
    b_ht           = displayListFromObjFileFlag(fname,0.0256,TRUE);

    sprintf(fname,"%s/hermesCore/3D/B_HR.obj",lab_root);
    b_hr           = displayListFromObjFileFlag(fname,0.0256,TRUE);

    sprintf(fname,"%s/hermesCore/3D/humanoidHeadLeftEye.obj",lab_root);
    left_eye         = displayListFromObjFile(fname,0.0256);

    sprintf(fname,"%s/hermesCore/3D/humanoidHeadRightEye.obj",lab_root);
    right_eye         = displayListFromObjFile(fname,0.0256);

    sprintf(fname,"%s/hermesCore/3D/humanoidHead.obj",lab_root);
    head         = displayListFromObjFile(fname,0.0256);

    sprintf(fname,"%s/hermesCore/3D/R_SFE.obj",lab_root);
    r_sfe        = displayListFromObjFileFlag(fname,0.0256,TRUE);

    sprintf(fname,"%s/hermesCore/3D/R_SAA.obj",lab_root);
    r_saa        = displayListFromObjFileFlag(fname,0.0256,TRUE);

    sprintf(fname,"%s/hermesCore/3D/R_HR.obj",lab_root);
    r_hr         = displayListFromObjFileFlag(fname,0.0256,TRUE);

    sprintf(fname,"%s/hermesCore/3D/R_EB.obj",lab_root);
    r_eb         = displayListFromObjFileFlag(fname,0.0256,TRUE);

    sprintf(fname,"%s/hermesCore/3D/R_WR.obj",lab_root);
    r_wr         = displayListFromObjFileFlag(fname,0.0256,TRUE);

    sprintf(fname,"%s/hermesCore/3D/R_WFE.obj",lab_root);
    r_wfe        = displayListFromObjFileFlag(fname,0.0256,TRUE);

    sprintf(fname,"%s/hermesCore/3D/R_WAA.obj",lab_root);
    l_waa        = displayListFromObjFileFlag(fname,0.0256,TRUE);

    sprintf(fname,"%s/hermesCore/3D/L_SFE.obj",lab_root);
    l_sfe        = displayListFromObjFileFlag(fname,0.0256,TRUE);

    sprintf(fname,"%s/hermesCore/3D/L_SAA.obj",lab_root);
    l_saa        = displayListFromObjFileFlag(fname,0.0256,TRUE);

    sprintf(fname,"%s/hermesCore/3D/L_HR.obj",lab_root);
    l_hr         = displayListFromObjFileFlag(fname,0.0256,TRUE);

    sprintf(fname,"%s/hermesCore/3D/L_EB.obj",lab_root);
    l_eb         = displayListFromObjFileFlag(fname,0.0256,TRUE);

    sprintf(fname,"%s/hermesCore/3D/L_WR.obj",lab_root);
    l_wr         = displayListFromObjFileFlag(fname,0.0256,TRUE);

    sprintf(fname,"%s/hermesCore/3D/L_WFE.obj",lab_root);
    l_wfe        = displayListFromObjFileFlag(fname,0.0256,TRUE);

    sprintf(fname,"%s/hermesCore/3D/L_WAA.obj",lab_root);
    r_waa        = displayListFromObjFileFlag(fname,0.0256,TRUE);

  }
	
  if (flag==1 && num != 35 && num != 37) {
    glTranslated(0.0,0.0,length); 

    if (num==999) {/* the base coordinate system */
      glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, blue);
      glColor4fv(blue);
    } else if (num > 38) {
      glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, gray);
      glColor4fv(gray);
    } else {
      glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, green);
      glColor4fv(green);
    }

    if (num == B_HN || num == B_HT) {
      if (solid)
	glutSolidSphere(0.25*width,isphere,isphere);
      else
	glutWireSphere(0.25*width,isphere,isphere);
    } else {
      if (solid)
	glutSolidSphere(0.25*width,isphere,isphere);
      else
	glutWireSphere(0.25*width,isphere,isphere);
    }
    glTranslated(0.0,0.0,-length);
  }
	

  switch (num) {

  case BASE:
    break;

  case L_SAA:   /* L_SFE */
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, gray);
    glColor4fv(gray);
    glRotated((GLdouble)135.,(GLdouble)0.,(GLdouble)1.,(GLdouble)0.);
    glRotated((GLdouble)180.,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
    glTranslated(-0.002,-0.005,0.290);
    glCallList(l_sfe);
    break;

  case L_HR:  /* L_SAA */
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, gray);
    glColor4fv(gray);
    glRotated((GLdouble)90.,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
    glRotated((GLdouble)-90.,(GLdouble)0.,(GLdouble)1.,(GLdouble)0.);
    glRotated((GLdouble)180.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
    glTranslated(-0.2,-0.005,0.490);
    glCallList(l_saa);
    break;

  case L_EB: /* L_HR */
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, gray);
    glColor4fv(gray);
    //glRotated((GLdouble)90.,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
    //glRotated((GLdouble)90.,(GLdouble)0.,(GLdouble)1.,(GLdouble)0.);
    glTranslated(-0.222,-0.005,0.495);
    glCallList(l_hr);
    break;

  case L_WR: /* L_EB */
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, gray);
    glColor4fv(gray);
    glRotated((GLdouble)90.,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
    glRotated((GLdouble)-90.,(GLdouble)0.,(GLdouble)1.,(GLdouble)0.);
    glTranslated(-0.223,-0.005,0.240);
    glCallList(l_eb);
    break;

  case L_WFE: /* L_WR */
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, gray);
    glColor4fv(gray);
    glRotated((GLdouble)180.,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
    //glRotated((GLdouble)-90.,(GLdouble)0.,(GLdouble)1.,(GLdouble)0.);
    glTranslated(-0.223,-0.000,0.2150);
    glCallList(l_wr);
    break;


  case L_WAA: /* L_WFE */
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, gray);
    glColor4fv(gray);
    glRotated((GLdouble)90.,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
    glRotated((GLdouble)-90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
    glTranslated(-0.22,-0.000,-0.017);
    glCallList(l_wfe);
    break;

  case 108:  /* L_WAA */
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, gray);
    glColor4fv(gray);
    glRotated((GLdouble)-90.,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
    //glRotated((GLdouble)90.,(GLdouble)0.,(GLdouble)1.,(GLdouble)0.);
    glTranslated(0.220,-0.020,-0.017);
    glCallList(l_waa);
    break;


  case R_SAA:   /* R_SFE */
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, gray);
    glColor4fv(gray);
    glRotated((GLdouble)135.,(GLdouble)0.,(GLdouble)1.,(GLdouble)0.);
    glTranslated(0.0,-0.005,0.290);
    glCallList(r_sfe);
    break;

  case R_HR:  /* R_SAA */
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, gray);
    glColor4fv(gray);
    glRotated((GLdouble)90.,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
    glRotated((GLdouble)90.,(GLdouble)0.,(GLdouble)1.,(GLdouble)0.);
    glTranslated(0.2,-0.005,0.490);
    glCallList(r_saa);
    break;

  case R_EB: /* R_HR */
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, gray);
    glColor4fv(gray);
    //glRotated((GLdouble)90.,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
    //glRotated((GLdouble)90.,(GLdouble)0.,(GLdouble)1.,(GLdouble)0.);
    glTranslated(0.222,-0.005,0.490);
    glCallList(r_hr);
    break;

  case R_WR: /* R_EB */
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, gray);
    glColor4fv(gray);
    glRotated((GLdouble)90.,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
    glRotated((GLdouble)-90.,(GLdouble)0.,(GLdouble)1.,(GLdouble)0.);
    glTranslated(0.223,-0.005,0.2350);
    glCallList(r_eb);
    break;

  case R_WFE: /* R_WR */
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, gray);
    glColor4fv(gray);
    glRotated((GLdouble)0.,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
    //glRotated((GLdouble)-90.,(GLdouble)0.,(GLdouble)1.,(GLdouble)0.);
    glTranslated(0.223,-0.000,0.2150);
    glCallList(r_wr);
    break;


  case R_WAA: /* R_WFE */
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, gray);
    glColor4fv(gray);
    glRotated((GLdouble)-90.,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
    glRotated((GLdouble)90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
    glTranslated(0.22,-0.000,-0.017);
    glCallList(r_wfe);
    break;

  case 115:  /* R_WAA */
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, gray);
    glColor4fv(gray);
    glRotated((GLdouble)90.,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
    //glRotated((GLdouble)90.,(GLdouble)0.,(GLdouble)1.,(GLdouble)0.);
    glTranslated(-0.220,-0.020,-0.017);
    glCallList(r_waa);
    break;

    break;


  case B_TR: /* pelvis */

    glPushMatrix();
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, gray);
    glColor4fv(gray);
    glRotated((GLdouble)180.,(GLdouble)0.,(GLdouble)1.,(GLdouble)0.);
    //glTranslated(0.0,0.0,0.1);
    glCallList(pelvis);
    glPopMatrix();

    // draw IMU
    glTranslated(-IMU_X_OFFSET,IMU_Y_OFFSET,0.0);
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, red);
    glColor4fv(red);
    glScaled(0.02,0.01,0.04);
    if (solid)
      glutSolidCube(1.0);
    else
      glutWireCube(1.0);


    break;

  case B_TAA: /* B_TR */

    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, gray);
    glColor4fv(gray);
    //glRotated((GLdouble)180.,(GLdouble)0.,(GLdouble)1.,(GLdouble)0.);
    glTranslated(0.0,0.0,0.1725);
    glCallList(b_tr);
    break;

  case B_TFE: /* B_TAA */

    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, gray);
    glColor4fv(gray);
    glRotated((GLdouble)90.,(GLdouble)0.,(GLdouble)1.,(GLdouble)0.);
    glRotated((GLdouble)90.,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
    glTranslated(0.0,0.0,0.1725);
    glCallList(b_taa);
    break;

  case B_HN: /* B_TFE */

    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, gray);
    glColor4fv(gray);
    glRotated((GLdouble)180.,(GLdouble)0.,(GLdouble)1.,(GLdouble)0.);
    glRotated((GLdouble)180.,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
    glTranslated(-0.004,-0.005,0.1725); // strange lateral offset ....
    glCallList(b_tfe);

    break;

  case L_HFE: /* L_HAA */

    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, gray);
    glColor4fv(gray);
    glRotated((GLdouble)90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
    glRotated((GLdouble)90.,(GLdouble)0.,(GLdouble)1.,(GLdouble)0.);
    glTranslated(0.09,0.0,0.0);
    glCallList(l_haa);
    break;

  case L_HFR: /* L_HFE */

    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, gray);
    glColor4fv(gray);
    glRotated((GLdouble)90,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
    //glRotated((GLdouble)90.,(ppGLdouble)0.,(GLdouble)1.,(GLdouble)0.);
    glTranslated(0.09,0.0,0.0);
    glCallList(l_hfe);
    break;

  case L_KFE: /* L_HFR */

    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, gray);
    glColor4fv(gray);
    glRotated((GLdouble)-90.,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
    glRotated((GLdouble)-8.5,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
    glTranslated(0.09,-0.06,-0.01);
    glCallList(l_hfr);
    break;

  case L_AR: /* L_KFE */

    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, gray);
    glColor4fv(gray);
    glRotated((GLdouble)90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
    glRotated((GLdouble)-90.,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
    glTranslated(0.09,0.0,-0.404);
    glCallList(l_kfe);
    break;

  case L_AFE: /* L_AR */

    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, gray);
    glColor4fv(gray);
    glRotated((GLdouble)-90.,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
    //glRotated((GLdouble)-90.,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
    glTranslated(0.09,0.0,-0.404);
    glCallList(l_ar);
    break;

  case L_AAA: /* L_AFE */

    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, gray);
    glColor4fv(gray);
    glRotated((GLdouble)90.,(GLdouble)0.,(GLdouble)1.,(GLdouble)0.);
    glRotated((GLdouble)180.,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
    glTranslated(0.09,0.0,-0.784);
    glCallList(l_afe);
    break;

  case 128: /* L_AAA */
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, gray);
    glColor4fv(gray);
    glRotated((GLdouble)-90.,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
    //glRotated((GLdouble)180.,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
    glScaled(MAG,MAG,1.0);
    glTranslated(0.08,0.0,-0.78);
    glCallList(l_aaa);
    break;

  case R_HFE: /* R_HAA */

    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, gray);
    glColor4fv(gray);
    glRotated((GLdouble)-90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
    glRotated((GLdouble)90.,(GLdouble)0.,(GLdouble)1.,(GLdouble)0.);
    glTranslated(0.095,0.0,0.0);
    glCallList(r_haa);
    break;

  case R_HFR: /* R_HFE */

    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, gray);
    glColor4fv(gray);
    glRotated((GLdouble)90,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
    //glRotated((GLdouble)90.,(ppGLdouble)0.,(GLdouble)1.,(GLdouble)0.);
    glTranslated(0.09,0.0,0.0);
    glCallList(r_hfe);
    break;

  case R_KFE: /* R_HFR */

    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, gray);
    glColor4fv(gray);
    glRotated((GLdouble)90.,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
    glRotated((GLdouble)-8.5,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
    glTranslated(0.09,-0.06,-0.01);
    glCallList(r_hfr);
    break;

  case R_AR: /* R_KFE */

    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, gray);
    glColor4fv(gray);
    glRotated((GLdouble)90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
    glRotated((GLdouble)-90.,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
    glTranslated(0.09,0.0,-0.404);
    glCallList(r_kfe);
    break;

  case R_AFE: /* R_AR */

    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, gray);
    glColor4fv(gray);
    glRotated((GLdouble)-90.,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
    //glRotated((GLdouble)-90.,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
    glTranslated(0.09,0.0,-0.404);
    glCallList(r_ar);
    break;

  case R_AAA: /* R_AFE */

    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, gray);
    glColor4fv(gray);
    glRotated((GLdouble)90.,(GLdouble)0.,(GLdouble)1.,(GLdouble)0.);
    glRotated((GLdouble)180.,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
    glTranslated(0.09,0.0,-0.784);
    glCallList(r_afe);
    break;

  case 135: /* R_AAA */
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, gray);
    glColor4fv(gray);
    glRotated((GLdouble)90.,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
    //glRotated((GLdouble)180.,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
    glScaled(MAG,MAG,1.0);
    glTranslated(-.08,0.0,-0.78);
    glCallList(r_aaa);
    break;

  case B_HT: /* B_HN */

    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, gray);
    glColor4fv(gray);
    glRotated((GLdouble)-90.,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
    glRotated((GLdouble)180.,(GLdouble)0.,(GLdouble)1.,(GLdouble)0.);
    glTranslated(-0.004,-0.006,0.56);
    glCallList(b_hn);
    break;

  case B_HR: /* B_HT */

    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, gray);
    glColor4fv(gray);
    //glRotated((GLdouble)-90.,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
    glRotated((GLdouble)-90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
    glTranslated(-0.004,-0.006,0.56);
    glCallList(b_ht);
    break;


  case 138: /* the head */
    
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, white);
    glColor4fv(white);
    glRotated((GLdouble)90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
    glTranslated(-0.035,-0.04+.12,-0.065);
    glCallList(head);
    break;
    
  case 35: /* dummies */
  case 37: /* dummies */
    break;
   
	
  case 137: /* the right eye */
	
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, gray);
    glColor4fv(gray);
    glRotated((GLdouble)90.,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
    glRotated((GLdouble)180.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
    glTranslated(-0.072,-0.0545,0.00);
    glCallList(right_eye);

    glTranslated(0.072,0.0545,-.022);

    glPushMatrix();
    glScaled(0.02,0.02,0.001);
    glColor4fv(black);
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, black);
    if (solid)
      glutSolidCylinder(1.0,1.0,isphere*2,1);
    else
      glutWireCylinder(1.0,1.0,isphere*2,1);
    glPopMatrix();

    glTranslated(0.0,0.0,-0.001);
    glPushMatrix();
    glScaled(0.007,0.007,0.001);
    glColor4fv(white);
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, white);
    if (solid)
      glutSolidCylinder(1.0,1.0,isphere*2,1);
    else
      glutWireCylinder(1.0,1.0,isphere*2,1);
    glPopMatrix();

    glTranslated(0.0,-0.017,0.0);
    glPushMatrix();
    glScaled(0.006,0.006,0.01);
    glColor4fv(gray);
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, gray);
    if (solid)
      glutSolidCylinder(1.0,1.0,isphere*2,1);
    else
      glutWireCylinder(1.0,1.0,isphere*2,1);
    glPopMatrix();

    break;

  case 139: /* the left eye */
	
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, gray);
    glColor4fv(gray);
    glRotated((GLdouble)90.,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
    glRotated((GLdouble)180.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
    glTranslated(0.00,-0.0545,0.0);
    glCallList(left_eye);

    glTranslated(0.0,0.0545,-0.022);

    glPushMatrix();
    glScaled(0.02,0.02,0.001);
    glColor4fv(black);
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, black);
    if (solid)
      glutSolidCylinder(1.0,1.0,isphere*2,1);
    else
      glutWireCylinder(1.0,1.0,isphere*2,1);
    glPopMatrix();

    glTranslated(0.0,0.0,-0.001);
    glPushMatrix();
    glScaled(0.007,0.007,0.001);
    glColor4fv(white);
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, white);
    if (solid)
      glutSolidCylinder(1.0,1.0,isphere*2,1);
    else
      glutWireCylinder(1.0,1.0,isphere*2,1);
    glPopMatrix();

    glTranslated(0.0,-0.017,0.0);
    glPushMatrix();
    glScaled(0.006,0.006,0.01);
    glColor4fv(gray);
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, gray);
    if (solid)
      glutSolidCylinder(1.0,1.0,isphere*2,1);
    else
      glutWireCylinder(1.0,1.0,isphere*2,1);
    glPopMatrix();

    break;

 case 129: /* dummies of feet */
 case 130:
 case 131:
 case 132:
 case 133:
 case 134:
 case 122:
 case 123:
 case 124:
 case 125:
  case 126:
 case 127:
	
   /*
    glScaled(width/2.,width/2.,length);
    glTranslated(0.0,0.0,0.5);
    glColor4fv(gray);
    if (solid)
      glutSolidCube(1.0);
    else
      glutWireCube(1.0);
   */			
    break;

  case 152:
  case 155:
  case 158:
  case 161:
  case 164:
  case 166:
  case 169:
  case 172:
  case 175:
  case 178:
    glPushMatrix();
    glTranslated(0.0,0.0,length);
    if (solid)
      glutSolidSphere(0.25*width,isphere,isphere);
    else
      glutWireSphere(0.25*width,isphere,isphere);
    glPopMatrix();

  default:
	
    glScaled(width/4,width/4,length);
    glColor4fv(gray);

    if (solid)
      glutSolidCylinder(1.0,1.0,isphere,1);
    else
      glutWireCylinder(1.0,1.0,isphere,1);

  }
}

/*!*****************************************************************************
 *******************************************************************************
\note  idle
\date  June 1999
   
\remarks 

      The function called by openGL whenever processing time available.

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output


 ******************************************************************************/
void
idle(void)

{
  int i;
  struct timeval t;
  static double last_draw_time = 0.0;
  double current_time;


  //-------------------------------------------------------------------------
  // the main functions are the most handy in an include file

#include "SL_user_idle_core.h"

  // update the eye view positions
  updateEyes();


  //-------------------------------------------------------------------------
  // refresh display

  gettimeofday(&t,NULL);
  current_time = (double) t.tv_sec + ((double)t.tv_usec)/1.e6;

  if (current_time-last_draw_time >= 1./window_update_rate) {
    glutPostRedisplayAll();
    last_draw_time = current_time; 
  }

}

/*!*****************************************************************************
 *******************************************************************************
\note  drawCenterOfPressure
\date  Feb. 2004
   
\remarks 

      draws the position of the center of pressure

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 ******************************************************************************/
static void
drawCenterOfPressure(void)

{
  int i,j,n;
  GLfloat   color_point[4]={(float)1.0,(float)0.0,(float)0.0,(float)1.0};
  double    radius = 0.04;
  double    xcop[N_CART+1];
  double    x[N_CART+1];
  double    sum;

  /* if there are no objects, exit */
  if (objs==NULL)
    return;

  for (j=1; j<=N_CART; ++j)
    xcop[j]=0;
  sum = 1.e-10;

  for (i=1; i<=n_contacts; ++i) { /* loop over all foot contact points */

    // check whether there is an active contact
    if (!contacts[i].active || !contacts[i].status)
      continue;

    // check whether the foot is involved
    if ( (contacts[i].id_start >= R_OUT_TOE && contacts[i].id_start <= R_IN_HEEL) ||
	 (contacts[i].id_start >= L_OUT_TOE && contacts[i].id_start <= L_IN_HEEL) ) {

      // where is this contact point
      computeContactPoint(&(contacts[i]),link_pos_sim,Alink_sim,x);

      // accumulate the cop position as a weighted sum, where the weight is the
      // z component of the force at each contact point
      for (j=1; j<=N_CART; ++j)
	xcop[j] += x[j] * contacts[i].f[_Z_];
      
      sum += contacts[i].f[_Z_];

    }

  }

  for (j=1; j<=N_CART; ++j)
    xcop[j] /= sum;
  
  
  // draw a blob at the cop
  glPushMatrix();
  glTranslated((GLdouble)xcop[_X_],(GLdouble)xcop[_Y_],(GLdouble)xcop[_Z_]);
  glColor4fv(color_point);
  if (solid)
    glutSolidSphere(radius,10,10);
  else
    glutWireSphere(radius,10,10);
  glPopMatrix();

}


/*!*****************************************************************************
 *******************************************************************************
\note  drawCOG
\date  Feb. 2004
   
\remarks 

      draws the position of the center of gravity

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 ******************************************************************************/
static void
drawCOG(void)

{
  int i,j,n;
  GLfloat   color_point[4]={(float)0.5,(float)0.5,(float)0.25,(float)opacity};
  double    radius = 0.04;
  double    sum;
  
  // draw a blob at the cog
  glPushMatrix();
  glTranslated((GLdouble)cog.x[_X_],(GLdouble)cog.x[_Y_],(GLdouble)cog.x[_Z_]);
  glColor4fv(color_point);
  if (solid)
    glutSolidSphere(radius,10,10);
  else
    glutWireSphere(radius,10,10);
  
  /* draw vertical line from COG downwards */
  glDisable(GL_LIGHTING); /*to have constant colors */
  glColor4fv(color_point);
  glBegin(GL_LINES);
  glVertex3d(0.0,0.0,0.0);
  glVertex3d(0.0,0.0,-2.0);
  glEnd();
  glEnable(GL_LIGHTING);
  glPopMatrix();
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  drawCOMs
\date  April 2013
   
\remarks 

      draws the position of the center of gravity of every link

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 ******************************************************************************/
static 
void drawCOMs(SL_Jstate *state, SL_endeff *eff, SL_Cstate *basec,
	      SL_quat *baseo) 
{
  int i, j, n;
  GLfloat color_point[4] = { (float) 1.0, (float) 1.0, (float) 0.5,
			     (float) opacity };
  double radius = 0.01;
  double sum;
  
  MY_MATRIX(Xmcog,0,N_DOFS,1,3);
  MY_MATRIX(Xaxis,0,N_DOFS,1,3);
  MY_MATRIX(Xorigin,0,N_DOFS,1,3);
  MY_MATRIX(Xlink,0,N_LINKS,1,3);
  MY_MATRIX_ARRAY(Ahmat,1,4,1,4,N_LINKS);
  MY_MATRIX_ARRAY(Ahmatdof,1,4,1,4,N_DOFS);
  MY_MATRIX(Ec,1,(N_CART*2),1,(N_CART*2));
  MY_MATRIX(Jac,1,(N_ENDEFFS*2*N_CART),1,N_DOFS);
  
  mat_eye(Ec);
  
  // compute the link information for this state
  linkInformation(state, basec, baseo, eff, Xmcog, Xaxis, Xorigin, Xlink,
		  Ahmat,Ahmatdof);
  
  for (i = 0; i <= N_DOFS; i++) {
  
  // draw a blob at each center of mass of each link
    glPushMatrix();
    glTranslated((GLdouble) (Xmcog[i][_X_] / links[i].m),
		 (GLdouble) (Xmcog[i][_Y_] / links[i].m),
		 (GLdouble) (Xmcog[i][_Z_] / links[i].m));
    glColor4fv(color_point);
    glutSolidSphere(radius, 10, 10);
    
    glPopMatrix();
  }
}

/*!*****************************************************************************
 *******************************************************************************
\note  updateEyes
\date  March 2003
   
\remarks 

      sets the view variables for the eyes

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
static void
updateEyes(void)
{
  int i;
  double length;
  double dist=4;

  if (w_left_eye_fovea != NULL) {

    length = 0;
    for (i=1; i<=N_CART; ++i)
      length += sqr(link_pos[L_EYE][i]-link_pos[L_EYE_AXIS][i]);
    length = sqrt(length);

    for (i=1; i<=N_CART; ++i) {
      w_left_eye_fovea->eye[i] = link_pos[L_EYE_AXIS][i];
      w_left_eye_fovea->up[i]  = link_pos[B_HEAD][i] - link_pos[B_CERVICAL2][i];
      w_left_eye_fovea->center[i] = (link_pos[L_EYE][i]-link_pos[L_EYE_AXIS][i])*dist/length +
	link_pos[L_EYE_AXIS][i];
    }

  }
    
  if (w_left_eye_wide != NULL) {

    length = 0;
    for (i=1; i<=N_CART; ++i)
      length += sqr(link_pos[L_EYE][i]-link_pos[L_EYE_AXIS][i]);
    length = sqrt(length);

    for (i=1; i<=N_CART; ++i) {
      w_left_eye_wide->eye[i] = link_pos[L_EYE_AXIS][i];
      w_left_eye_wide->up[i]  = link_pos[B_HEAD][i] - link_pos[B_CERVICAL2][i];
      w_left_eye_wide->center[i] = (link_pos[L_EYE][i]-link_pos[L_EYE_AXIS][i])*dist/length +
	link_pos[L_EYE_AXIS][i];
    }

  }
    
    
  if (w_right_eye_fovea != NULL) {

    length = 0;
    for (i=1; i<=N_CART; ++i)
      length += sqr(link_pos[R_EYE][i]-link_pos[R_EYE_AXIS][i]);
    length = sqrt(length);

    for (i=1; i<=N_CART; ++i) {
      w_right_eye_fovea->eye[i] = link_pos[R_EYE_AXIS][i];
      w_right_eye_fovea->up[i]  = link_pos[B_HEAD][i] - link_pos[B_CERVICAL2][i];
      w_right_eye_fovea->center[i]=(link_pos[R_EYE][i]-link_pos[R_EYE_AXIS][i])*dist/length +
	link_pos[R_EYE_AXIS][i];
    }

  }
    
    
  if (w_right_eye_wide != NULL) {

    length = 0;
    for (i=1; i<=N_CART; ++i)
      length += sqr(link_pos[R_EYE][i]-link_pos[R_EYE_AXIS][i]);
    length = sqrt(length);

    for (i=1; i<=N_CART; ++i) {
      w_right_eye_wide->eye[i] = link_pos[R_EYE_AXIS][i];
      w_right_eye_wide->up[i]  = link_pos[B_HEAD][i] - link_pos[B_CERVICAL2][i];
      w_right_eye_wide->center[i]=(link_pos[R_EYE][i]-link_pos[R_EYE_AXIS][i])*dist/length +
	link_pos[R_EYE_AXIS][i];
    }

  }
    
}

 

/*!*****************************************************************************
 *******************************************************************************
\note  drawIMU
\date  April 2013
   
\remarks 

 draws the trans. acceleration , and rot. velocity vector of IMU

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 ******************************************************************************/
static void
drawIMU(void)

{
  int i,j,n;
  GLfloat   color_point1[4]={(float)1.0,(float)1.0,(float)0.5,(float)opacity};
  GLfloat   color_point2[4]={(float)0.0,(float)1.0,(float)0.5,(float)opacity};
  double    arrow_width = 0.01;
  double    s[N_CART+1];
  double    e[N_CART+1];
  double    acc_scale = 0.02;
  double    vel_scale = 1;


  glPushMatrix();

  // translate to base
  glTranslated(base_state.x[_X_],base_state.x[_Y_],base_state.x[_Z_]);   

  // rotate into BASE coordinate system
  glRotated((GLdouble)2.*acos(base_orient.q[_Q0_])/PI*180.,
	    (GLdouble)base_orient.q[_Q1_],
	    (GLdouble)base_orient.q[_Q2_],
	    (GLdouble)base_orient.q[_Q3_]);

  // translate to IMU
  glTranslated(IMU_X_OFFSET,-IMU_Y_OFFSET,0.0);     

  // rotate into IMU coordinate system
  glRotated((GLdouble)-90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
  glRotated((GLdouble) 90.,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);

  // the start and end point of the acceleration vector
  s[_X_] =  0.0;
  s[_Y_] =  0.0;
  s[_Z_] =  0.0;

  e[_X_] = s[_X_] + misc_sim_sensor[B_XACC_IMU]*acc_scale;
  e[_Y_] = s[_Y_] + misc_sim_sensor[B_YACC_IMU]*acc_scale;
  e[_Z_] = s[_Z_] + misc_sim_sensor[B_ZACC_IMU]*acc_scale;


  glColor4fv(color_point1);
  drawArrow(s,e,arrow_width);

  // the start and end point of the angular velocity vector
  s[_X_] =  0.0;
  s[_Y_] =  0.0;
  s[_Z_] =  0.0;

  e[_X_] = s[_X_] + misc_sim_sensor[B_AD_A_IMU]*vel_scale;
  e[_Y_] = s[_Y_] + misc_sim_sensor[B_AD_B_IMU]*vel_scale;
  e[_Z_] = s[_Z_] + misc_sim_sensor[B_AD_G_IMU]*vel_scale;

  glColor4fv(color_point2);
  drawArrow(s,e,arrow_width);

  glPopMatrix();

}

/*!*****************************************************************************
 *******************************************************************************
\note  drawFootSensor
\date  April 2013
   
\remarks 

 draws force/torque and acceleration at the foot sensor

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 ******************************************************************************/
static void
drawFootSensor(void)

{
  int i,j,n;
  GLfloat   color_point1[4]={(float)1.0,(float)1.0,(float)0.5,(float)opacity};
  GLfloat   color_point2[4]={(float)0.0,(float)1.0,(float)0.5,(float)opacity};
  GLfloat   color_point3[4]={(float)0.0,(float)1.0,(float)1.0,(float)opacity};
  GLfloat   color_point4[4]={(float)0.5,(float)0.5,(float)1.0,(float)opacity};
  double    arrow_width = 0.01;
  double    s[N_CART+1];
  double    e[N_CART+1];
  double    acc_scale = 0.02;
  double    vel_scale = 1;
  double    force_scale  = fscale*0.1;
  double    torque_scale = fscale*2.0;


  glPushMatrix();

  // translate to L_FOOT
  glTranslated(link_pos_sim[L_FOOT][_X_],link_pos_sim[L_FOOT][_Y_],link_pos_sim[L_FOOT][_Z_]);

  // rotate into L_FOOT coordinates
  linkQuat(Alink_sim[L_FOOT],&(cart_orient[LEFT_FOOT]));
  glRotated((GLdouble)2.*acos(cart_orient[LEFT_FOOT].q[_Q0_])/PI*180.,
	    (GLdouble)cart_orient[LEFT_FOOT].q[_Q1_],
	    (GLdouble)cart_orient[LEFT_FOOT].q[_Q2_],
	    (GLdouble)cart_orient[LEFT_FOOT].q[_Q3_]);


  // the start and end point of the acceleration vector
  s[_X_] =  0.0;
  s[_Y_] =  0.0;
  s[_Z_] =  0.0;

  e[_X_] = s[_X_] + misc_sim_sensor[L_FOOT_XACC]*acc_scale;
  e[_Y_] = s[_Y_] + misc_sim_sensor[L_FOOT_YACC]*acc_scale;
  e[_Z_] = s[_Z_] + misc_sim_sensor[L_FOOT_ZACC]*acc_scale;

  glColor4fv(color_point1);
  drawArrow(s,e,arrow_width);

  // the start and end point of the force vector
  s[_X_] =  -ZTOE+FTA_Z_OFF;
  s[_Y_] =  FTA_X_OFF;
  s[_Z_] =  -FTA_Y_OFF;

  e[_X_] = s[_X_] + misc_sim_sensor[L_CFx]*force_scale;
  e[_Y_] = s[_Y_] + misc_sim_sensor[L_CFy]*force_scale;
  e[_Z_] = s[_Z_] + misc_sim_sensor[L_CFz]*force_scale;

  glColor4fv(color_point3);
  drawArrow(s,e,arrow_width);

  // the end point of the torque vector
  e[_X_] = s[_X_] + misc_sim_sensor[L_CTa]*torque_scale;
  e[_Y_] = s[_Y_] + misc_sim_sensor[L_CTb]*torque_scale;
  e[_Z_] = s[_Z_] + misc_sim_sensor[L_CTg]*torque_scale;

  glColor4fv(color_point4);
  drawArrow(s,e,arrow_width);


  glPopMatrix();


  glPushMatrix();

  // translate to R_FOOT
  glTranslated(link_pos_sim[R_FOOT][_X_],link_pos_sim[R_FOOT][_Y_],link_pos_sim[R_FOOT][_Z_]);

  // rotate into L_FOOT coordinates
  linkQuat(Alink_sim[R_FOOT],&(cart_orient[RIGHT_FOOT]));
  glRotated((GLdouble)2.*acos(cart_orient[RIGHT_FOOT].q[_Q0_])/PI*180.,
	    (GLdouble)cart_orient[RIGHT_FOOT].q[_Q1_],
	    (GLdouble)cart_orient[RIGHT_FOOT].q[_Q2_],
	    (GLdouble)cart_orient[RIGHT_FOOT].q[_Q3_]);


  // the start and end point of the acceleration vector
  s[_X_] =  0.0;
  s[_Y_] =  0.0;
  s[_Z_] =  0.0;

  e[_X_] = s[_X_] + misc_sim_sensor[R_FOOT_XACC]*acc_scale;
  e[_Y_] = s[_Y_] + misc_sim_sensor[R_FOOT_YACC]*acc_scale;
  e[_Z_] = s[_Z_] + misc_sim_sensor[R_FOOT_ZACC]*acc_scale;


  glColor4fv(color_point1);
  drawArrow(s,e,arrow_width);

  // the start and end point of the force vector
  s[_X_] =  -ZTOE+FTA_Z_OFF;
  s[_Y_] =  FTA_X_OFF;
  s[_Z_] =  FTA_Y_OFF;

  e[_X_] = s[_X_] + misc_sim_sensor[R_CFx]*force_scale;
  e[_Y_] = s[_Y_] + misc_sim_sensor[R_CFy]*force_scale;
  e[_Z_] = s[_Z_] + misc_sim_sensor[R_CFz]*force_scale;

  glColor4fv(color_point3);
  drawArrow(s,e,arrow_width);

  // the end point of the torque vector
  e[_X_] = s[_X_] + misc_sim_sensor[R_CTa]*torque_scale;
  e[_Y_] = s[_Y_] + misc_sim_sensor[R_CTb]*torque_scale;
  e[_Z_] = s[_Z_] + misc_sim_sensor[R_CTg]*torque_scale;

  glColor4fv(color_point4);
  drawArrow(s,e,arrow_width);




  glPopMatrix();

}

/*!*****************************************************************************
 *******************************************************************************
 \note  displayPhantom
 \date  April 2013
 
 \remarks 

 creates a translucent phantom copy of the robot at the given position

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]      b      : the general array of bytes

 ******************************************************************************/
static void
displayPhantom(void *b)
{
  int        i,n;
  float      data[N_CART+N_QUAT+n_dofs];
  SL_Cstate  basec;
  SL_quat    baseo;
  SL_Jstate  state[n_dofs+1];
  SL_endeff  *eff=endeff;

  // assign the data to apprpriate variables
  n = 0;
  for (i=1; i<=N_CART; ++i)
    basec.x[i] = (double) ((float *)b)[n++];

  for (i=1; i<=N_QUAT; ++i)
    baseo.q[i] = (double) ((float *)b)[n++];
  
  for (i=1; i<=n_dofs; ++i)
    state[i].th = (double) ((float *)b)[n++];

  /* here is the drawing rountines */
  glPushMatrix();
  glEnable(GL_BLEND);
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  //glDisable(GL_LIGHTING); 
  drawRobot(state,&basec,&baseo);
  glEnable(GL_LIGHTING);   
  glDisable(GL_BLEND);
  glPopMatrix();

}

/*!*****************************************************************************
 *******************************************************************************
\note  drawRobot
\date  April 2013
   
\remarks 

 this function draws the robot

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]   state : pointer to joint state
 \param[in]   basec : pointer to base state
 \param[in]   baseo : pointer to base orientation


 ******************************************************************************/
static void 
drawRobot(SL_Jstate *state, SL_Cstate *basec, SL_quat *baseo)

{
  int i;
  static SL_endeff  *eff   = endeff;

#include "OpenGL.h"


}

/*!*****************************************************************************
*******************************************************************************
\note  toggleCOMsDisplay
\date  April 2013
   
\remarks 

toggle drawing the COMs of all links

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
static void
toggleCOMsDisplay(void)
{

  if (!comsDisplay){
    printf("Draw COMS is ON\n");
    comsDisplay = TRUE;
  }
  else  {
    printf("Draw DOMS is  OFF\n");
    comsDisplay = FALSE;
  }

}
