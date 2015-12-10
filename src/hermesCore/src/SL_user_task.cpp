/*!=============================================================================
  ==============================================================================

  \file    SL_user_task.c

  \author  Stefan Schaal
  \date    Nov. 2007

  ==============================================================================
  \remarks
  
  Allows running user specific computation in the main task servo loop
  
  ============================================================================*/
  
// SL general includes of system headers
#include "SL_system_headers.h"

// user specific headers
#include "SL.h"
#include "SL_user.h"
#include "SL_common.h"
#include "SL_task_servo.h"
#include "utility.h"
#include "utility_macros.h"
#include "mdefs.h"
#include "SL_shared_memory.h"
#include "SL_man.h"
#include "SL_user_task.h"
#include "raw_data_access.h"
#include "SL_collect_data.h"

// global variables

// local variables
static int use_simulated_base_state = FALSE;
static RawDataAccess raw_data;

// local functions
static int read_simulated_base(void);

extern "C"
{
void initialize_base_state_estimation(const double* imu_quaternion,
                                      const double* imu_angrate, const double* imu_linacc, int update_rate);
void update_base_state_estimation(SL_quat* base_orientation, SL_Cstate* base_position);

void differentiate_cog(SL_Cstate* cog);

int init_state_est_lin_task();
int run_state_est_lin_task();
void getPelv(SL_quat *base_quat, SL_Cstate *base );

void toggleSimulatedBaseState(void);
void worldToBaseAcc(double *xw, SL_Cstate *cbase, SL_quat *obase, double *xl);
void baseToWorldAcc(double *xl, SL_Cstate *cbase, SL_quat *obase, double *xw);

void initialize_fall_detector();
void test_fall(SL_Cstate * cog_);
}

/*!*****************************************************************************
 *******************************************************************************
 \note  init_user_task
 \date  Nov. 2007
 
 \remarks 
 
 initializes user task functionality
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 none 

 ******************************************************************************/
int
init_user_task(void)

{
  
  int i,j;


  addToMan("toggleSimBase","toggles the use of the simulated base state on/off",
	   toggleSimulatedBaseState);

  initialize_base_state_estimation(&(misc_sensor[B_Q0_IMU]), &(misc_sensor[B_AD_A_IMU]),
   		&(misc_sensor[B_XACC_IMU]), task_servo_rate);

  init_state_est_lin_task();

  if(!raw_data.init())
      return FALSE;

  //data collect for raw joint states and misc sensors
  char string[100];
  for (i=1; i<=n_dofs; ++i) {
      sprintf(string,"%s_raw_th",joint_names[i]);
      addVarToCollect((char *)&(raw_joint_state[i].th),string,"rad", DOUBLE,FALSE);
      sprintf(string,"%s_raw_thd",joint_names[i]);
      addVarToCollect((char *)&(raw_joint_state[i].thd),string,"rad/s", DOUBLE,FALSE);
      sprintf(string,"%s_raw_thdd",joint_names[i]);
      addVarToCollect((char *)&(raw_joint_state[i].thdd),string,"rad/s^2", DOUBLE,FALSE);
//      sprintf(string,"%s_raw_u",joint_names[i]);
//      addVarToCollect((char *)&(raw_joint_state[i].u),string,"Nm", DOUBLE,FALSE);
//      sprintf(string,"%s_raw_ufb",joint_names[i]);
//      addVarToCollect((char *)&(raw_joint_state[i].ufb),string,"Nm", DOUBLE,FALSE);
      sprintf(string,"%s_raw_load",joint_names[i]);
      addVarToCollect((char *)&(raw_joint_state[i].load),string,"Nm", DOUBLE,FALSE);
  }

  for (i=1; i<=n_misc_sensors; ++i) {
    sprintf(string,"%s_raw",misc_sensor_names[i]);
    addVarToCollect((char *)&(raw_misc_sensor[i]),string,"-",DOUBLE,FALSE);
  }

  updateDataCollectScript();

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  run_user_task
 \date  Nov. 2007
 
 \remarks 
 
 this function is clocked out of the task servo
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 none 

 ******************************************************************************/
int
run_user_task(void)

{
  
  int i,j;
  MY_MATRIX(M,1,N_CART,1,N_CART);
  MY_VECTOR(v,1,N_CART);

  // compute the contact forces in world coordinates at the feet from the
  // force sensors

  /* rotation matrix from world to L_AAA coordinates:
     we can borrow this matrix from the toes, which have the same
     rotation, but just a different offset vector, which is not
     needed here */

#ifdef HAS_LOWER_BODY
  // transform forces
  v[_X_] = misc_sensor[L_CFx];
  v[_Y_] = misc_sensor[L_CFy];
  v[_Z_] = misc_sensor[L_CFz];
  mat_vec_mult_size(Alink[L_IN_HEEL],N_CART,N_CART,v,N_CART,endeff[LEFT_FOOT].cf);

  // transform torques
  v[_A_] = misc_sensor[L_CTa];
  v[_B_] = misc_sensor[L_CTb];
  v[_G_] = misc_sensor[L_CTg];
  mat_vec_mult_size(Alink[L_IN_HEEL],N_CART,N_CART,v,N_CART,endeff[LEFT_FOOT].ct);

  /* rotation matrix from world to R_AAA coordinates :
     we can borrow this matrix from the toes, which have the same
     rotation, but just a different offset vector, which is not
     needed here */

  // transform forces
  // transform forces
  v[_X_] = misc_sensor[R_CFx];
  v[_Y_] = misc_sensor[R_CFy];
  v[_Z_] = misc_sensor[R_CFz];
  mat_vec_mult_size(Alink[R_IN_HEEL],N_CART,N_CART,v,N_CART,endeff[RIGHT_FOOT].cf);

  // transform torques
  v[_A_] = misc_sensor[R_CTa];
  v[_B_] = misc_sensor[R_CTb];
  v[_G_] = misc_sensor[R_CTg];
  mat_vec_mult_size(Alink[R_IN_HEEL],N_CART,N_CART,v,N_CART,endeff[RIGHT_FOOT].ct);
#endif
  
differentiate_cog(&cog);  

  // use the simulated base state if required
  if (use_simulated_base_state)
    read_simulated_base();
  else
  {
	
	  // do base state estimation
//	  update_base_state_estimation(&base_orient, &base_state);
	  run_state_est_lin_task();
	  getPelv(&base_orient, &base_state);

	  // base state
	  if (semTake(sm_base_state_sem,ns2ticks(NO_WAIT)) == ERROR)
	  {
	    //printf("sm_base_state_sem take error\n");
	    return TRUE;
	  }

	  cSL_Cstate((&base_state)-1, sm_base_state_data, 1, DOUBLE2FLOAT);

	  sm_base_state->state[1] = sm_base_state_data[1];

	  semGive(sm_base_state_sem);


	  // base orient
	  if (semTake(sm_base_orient_sem,ns2ticks(NO_WAIT)) == ERROR)
	  {
	    //printf("sm_base_orien_sem take error\n");
	    return TRUE;

	  }
	  cSL_quat(&base_orient-1, sm_base_orient_data, 1, DOUBLE2FLOAT);

	  sm_base_orient->orient[1] = sm_base_orient_data[1];

	  semGive(sm_base_orient_sem);
  }

  //get raw data from motor servo
  raw_data.get_from_sm(raw_joint_state, raw_misc_sensor);

  return TRUE;

}


/*!*****************************************************************************
 *******************************************************************************
\note  useSimulatedBaseState
\date  Aug. 2008
   
\remarks 

        sets a flag to read the simulated base state and put it into the
        base_state and base_orient variables
	

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
void
useSimulatedBaseState(int flag)
{
  use_simulated_base_state = flag;
}

/*!*****************************************************************************
 *******************************************************************************
\note  toggleSimulatedBaseState
\date  Aug. 2008
   
\remarks 

        toggles the use_simulated_base_state flag
	

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
void
toggleSimulatedBaseState(void)
{
  if (use_simulated_base_state) {
    use_simulated_base_state = FALSE;
    printf("***** Simulated Base State is NOT used anymore *****\n");
  } else {
    use_simulated_base_state = TRUE;
    printf("***** Simulated Base State is used now *****\n");
  }
}

/*!*****************************************************************************
 *******************************************************************************
\note  read_simulated_base
\date  Nov. 2007
   
\remarks 

        read the simulated base from shared memory, which can be used in
        simulatin for debugging purposes.
	

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     none

 ******************************************************************************/
#define TIME_OUT_NS 1000000000
static int 
read_simulated_base(void)
{

  // base state
  if (semTake(sm_base_state_sem,ns2ticks(TIME_OUT_NS)) == ERROR)
    return FALSE;

  sm_base_state_data[1] = sm_base_state->state[1];
  
  cSL_Cstate(&base_state-1, sm_base_state_data, 1, FLOAT2DOUBLE);

  semGive(sm_base_state_sem);


  // base orient
  if (semTake(sm_base_orient_sem,ns2ticks(TIME_OUT_NS)) == ERROR)
    return FALSE;

  sm_base_orient_data[1] = sm_base_orient->orient[1];
  
  cSL_quat(&base_orient-1, sm_base_orient_data, 1, FLOAT2DOUBLE);

  semGive(sm_base_orient_sem);


  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
\note  worldToBaseAcc
\date  April 2006
   
\remarks 

        converts a 3D acceleration in world coordinates to base coordinates

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output


 \param[in]     xw     : the vector in world coordinates
 \param[in]     cbase  : the base position
 \param[in]     obase  : the base orientation
 \param[out]    xl     : the vector in local coordinates


 ******************************************************************************/
void
worldToBaseAcc(double *xw, SL_Cstate *cbase, SL_quat *obase, double *xl)
{

  int           i,j;
  static int    firsttime=TRUE;
  double        temp[N_CART+1];

  MY_MATRIX(R,1,N_CART,1,N_CART);

  for (i=1; i<=N_CART; ++i)
    temp[i] = xw[i];

  quatToRotMat(obase,R);
  mat_vec_mult_size(R,N_CART,N_CART,temp,N_CART,xl);

}

/*!*****************************************************************************
 *******************************************************************************
\note  baseToWorldAcc
\date  April 2006
   
\remarks 

        converts a 3D acceleration in base coordinates to world coordinates

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output


 \param[in]     xl     : the vector in local coordinates
 \param[in]     cbase  : the base position
 \param[in]     obase  : the base orientation
 \param[out]    xw     : the vector in world coordinates


 ******************************************************************************/
void
baseToWorldAcc(double *xl, SL_Cstate *cbase, SL_quat *obase, double *xw)
{

  int           i,j;
  static int    firsttime=TRUE;
  double tmp[N_CART+1];

  MY_MATRIX(R,1,N_CART,1,N_CART);
  
  quatToRotMatInv(obase,R);
  mat_vec_mult_size(R,N_CART,N_CART,xl,N_CART,tmp);

  for (i=1; i<=N_CART; ++i)
    xw[i] = tmp[i];

}

/*!*****************************************************************************
 *******************************************************************************
\note  drawPhantomFromCurrentState
\date  April 2013
   
\remarks 

 draws the phantom robot using the current state information

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 none

 ******************************************************************************/
void
drawPhantomFromCurrentState(void)
{
  int   n,i;
  float b[N_CART+N_QUAT+N_DOFS];

  n = 0;
  for (i=1; i<=N_CART; ++i)
    b[n++] = base_state.x[i];

  for (i=1; i<=N_QUAT; ++i)
    b[n++] = base_orient.q[i];

  for (i=1; i<=N_DOFS; ++i)
    b[n++] = joint_state[i].th;

  sendUserGraphics("phantom",b, (N_CART+N_QUAT+N_DOFS)*sizeof(float));
}

/*!*****************************************************************************
 *******************************************************************************
\note  drawPhantomFromDesiredState
\date  April 2013
   
\remarks 

 draws the phantom robot using the desired state information

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 none

 ******************************************************************************/
void
drawPhantomFromDesiredState(void)
{
  int   n,i;
  float b[N_CART+N_QUAT+N_DOFS];

  n = 0;
  for (i=1; i<=N_CART; ++i)
    b[n++] = base_state.x[i];

  for (i=1; i<=N_QUAT; ++i)
    b[n++] = base_orient.q[i];

  for (i=1; i<=N_DOFS; ++i)
    b[n++] = joint_des_state[i].th;

  sendUserGraphics("phantom",b, (N_CART+N_QUAT+N_DOFS)*sizeof(float));
}
