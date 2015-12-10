/*!=============================================================================
  ==============================================================================

  \file    SL_user_simulation.c

  \author  Stefan Schaal
  \date    Nov. 2007

  ==============================================================================
  \remarks
  
  This program contains all user functions for the numerical simluation of
  a robot
  
  ============================================================================*/
  
// SL general includes of system headers
#include "SL_system_headers.h"

/* user specific headers */
#include "SL.h"
#include "SL_integrate.h"
#include "SL_user.h"
#include "SL_common.h"
#include "SL_objects.h"
#include "SL_simulation_servo.h"
#include "utility.h"
#include "mdefs.h"
#include "SL_dynamics.h"
#include "SL_kinematics.h"

/* global variables */

/* local variables */
  
/* local functions */
static void
compute_force_sensors(void);
static void
compute_inertial(void);

/* external functions */


/*!*****************************************************************************
 *******************************************************************************
 \note  init_user_simulation
 \date  July 1998
 
 \remarks 
 
 initializes everything needed for the numerical simulation.
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 none 

 ******************************************************************************/
int
init_user_simulation(void)

{
  
  int i,j,n;

  // initalize objects in the environment
  readObjects(config_files[OBJECTS]);

  // assign contact force mappings
#include "LEKin_contact.h"
#ifdef HAS_LOWER_BODY
  contacts[R_FOOT].active = FALSE;
  contacts[L_FOOT].active = FALSE;
#endif
#ifdef HAS_UPPER_BODY
  contacts[B_HEAD].active = FALSE;
#endif
  // change numerical integration
  n_integration = 8;
  integrate_method = INTEGRATE_EULER;

  // change the freeze_base_pos[] array if needed

  // zero the state of the robot
  reset();
  

  return TRUE;
}


/*!*****************************************************************************
 *******************************************************************************
\note  run_user_simulation
\date  Nov. 7, 2007
   
\remarks 

      computes additional simulation values, e.g., misc sensors

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output


 ******************************************************************************/
int
run_user_simulation(void)

{
  int i;


  // compute vestibular sensors based on joint_sim_state
  compute_inertial();
  
#ifdef HAS_LOWER_BODY
  // compute force sensor information at feet
  compute_force_sensors();
#endif

  return TRUE;
}


/*!*****************************************************************************
 *******************************************************************************
\note  compute_inertial
\date  Oct 2005
   
\remarks 

      This functions computes simulated inertial signals, i.e., body 
      quaternion, angular velocity, and linear acceleration, and 
      linear acceleration at the feet

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     All results can be computed based on global variables. Results are
     assigned to the misc_sensor structures.


 ******************************************************************************/
static void
compute_inertial(void)

{
  int i,j;
  static int firsttime = TRUE;
  static Matrix R;
  static Matrix RT;
  static Vector ad;
  static Vector xdd;
  static Vector r;
  static Vector r0;
  static Vector t1;
  static Vector t2;
  static Vector t3;
  static Vector lv;
  static Vector rv;
  static Vector lv_last;
  static Vector rv_last;
  static Vector lacc;
  static Vector racc;


  if (firsttime) {
    R   = my_matrix(1,N_CART,1,N_CART);
    RT  = my_matrix(1,N_CART,1,N_CART);
    ad  = my_vector(1,N_CART);
    xdd = my_vector(1,N_CART);
    r   = my_vector(1,N_CART);
    r0  = my_vector(1,N_CART);
    t1  = my_vector(1,N_CART);
    t2  = my_vector(1,N_CART);
    t3  = my_vector(1,N_CART);
    lv  = my_vector(1,N_CART);
    rv  = my_vector(1,N_CART);
    lacc= my_vector(1,N_CART);
    racc= my_vector(1,N_CART);
    lv_last  = my_vector(1,N_CART);
    rv_last  = my_vector(1,N_CART);
    firsttime = FALSE;
  }

  // copy quaternion from base coordinates (perfect data)
  misc_sim_sensor[B_Q0_IMU] = base_orient.q[_Q0_];
  misc_sim_sensor[B_Q1_IMU] = base_orient.q[_Q1_];
  misc_sim_sensor[B_Q2_IMU] = base_orient.q[_Q2_];
  misc_sim_sensor[B_Q3_IMU] = base_orient.q[_Q3_];


  // the angular vel. and translatory acc are first computed in world coordinates
  // and then transformed to base coordinates.

  // need the base coordinate rotatin matrix, which transforms world to base
  quatToRotMat(&base_orient,R);
  mat_trans(R,RT);

  // offset is IMU_X_OFFSET and -IMU_Y_OFFSET
  r[_X_] = IMU_X_OFFSET;
  r[_Y_] = -IMU_Y_OFFSET;
  r[_Z_] = 0.0;

  // get offset vector in global coordinates
  mat_vec_mult(RT,r,r0);

  // w x r0
  vec_mult_outer_size(base_orient.ad,r0,N_CART,t1);

  // w x (w x r0) = w x t1 (which is one term of xdd)
  vec_mult_outer_size(base_orient.ad,t1,N_CART,t2);

  // wd x r0
  vec_mult_outer_size(base_orient.add,r0,N_CART,t1);  

  // add to xdd
  vec_add(t1,t2,t1);

  // and add the base acceleration
  vec_add_size(t1,base_state.xdd,N_CART,t1);

  // add gravity to t1
  t1[_Z_] -= gravity;

  // rotate all into base coordinates
  mat_vec_mult_size(R,N_CART,N_CART,base_orient.ad,N_CART,t3);
  mat_vec_mult_size(R,N_CART,N_CART,t1,N_CART,t2);

  // and rotate this information into IMU coordinates
  // IMU_X = -Z_BASE; IMU_Y = -X_BASE; IMU_Z = Y_BASE

  t1[_A_] = -PI/2.;
  t1[_B_] = 0.0;
  t1[_G_] = PI/2.;
  eulerToRotMat(t1,R);
  mat_vec_mult(R,t2,xdd);
  mat_vec_mult(R,t3,ad);

  // the final result
  misc_sim_sensor[B_AD_A_IMU] = ad[_A_];
  misc_sim_sensor[B_AD_B_IMU] = ad[_B_];
  misc_sim_sensor[B_AD_G_IMU] = ad[_G_];

  misc_sim_sensor[B_XACC_IMU] = xdd[_X_];
  misc_sim_sensor[B_YACC_IMU] = xdd[_Y_];
  misc_sim_sensor[B_ZACC_IMU] = xdd[_Z_];


  // now get the foot accelerations in WORLD coordinate, computed at the L_FOOT position,
  // which is not exactly the same as the load cell offset, but very close.
  computeLinkVelocity(L_FOOT, link_pos_sim, joint_origin_pos_sim, 
		      joint_axis_pos_sim, joint_sim_state, lv);

  computeLinkVelocity(R_FOOT, link_pos_sim, joint_origin_pos_sim, 
		      joint_axis_pos_sim, joint_sim_state, rv);

  for (i=1; i<=N_CART; ++i) {
    lacc[i] = (lv[i]-lv_last[i])*(double)simulation_servo_rate;
    lv_last[i] = lv[i];
    racc[i] = (rv[i]-rv_last[i])*(double)simulation_servo_rate;
    rv_last[i] = rv[i];
  }

  // transform to local foot coordinates after adding gravity
  lacc[_Z_] -= gravity;
  racc[_Z_] -= gravity;

  // rotation matrix from world to L_AAA coordinates 
  mat_trans_size(Alink_sim[L_FOOT],N_CART,N_CART,R);
  mat_vec_mult(R,lacc,t1);

  // rotation matrix from world to R_AAA coordinates 
  mat_trans_size(Alink_sim[R_FOOT],N_CART,N_CART,R);
  mat_vec_mult(R,racc,t2);

#ifdef HAS_LOWER_BODY
  // the final result
  misc_sim_sensor[L_FOOT_XACC] = t1[_X_];
  misc_sim_sensor[L_FOOT_YACC] = t1[_Y_];
  misc_sim_sensor[L_FOOT_ZACC] = t1[_Z_];

  misc_sim_sensor[R_FOOT_XACC] = t2[_X_];
  misc_sim_sensor[R_FOOT_YACC] = t2[_Y_];
  misc_sim_sensor[R_FOOT_ZACC] = t2[_Z_];
#endif
}

#ifdef HAS_LOWER_BODY
/*!*****************************************************************************
 *******************************************************************************
\note  compute_force_sensors
\date  March 2003
   
\remarks 

      This function computes simulated force sensor information for both
      feet. The results are force and torque values for in link coordinates
      of the last ankle joint. This informatin can be extracted from the
      contact force computation, which has world coordinates. Thus, only
      a simple coordinate transformatino from world to local link coordinates
      is needed. A small correction is need to transform the force/torque
      information to the exact position of the load cell center.

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

     All results can be computed based on global variables. Results are
     assigned to the misc_sensor structures.


 ******************************************************************************/
static void
compute_force_sensors(void)

{
  int i,j;
  static int firsttime = TRUE;
  static Matrix M;
  static Vector v;
  static Vector vv;
  static Vector r;

  if (firsttime) {
    firsttime = FALSE;
    M = my_matrix(1,N_CART,1,N_CART);
    v = my_vector(1,N_CART);
    vv= my_vector(1,N_CART);
    r = my_vector(1,N_CART);
  }

  // rotation matrix from world to L_AAA coordinates 
  mat_trans_size(Adof_sim[L_AAA],N_CART,N_CART,M);

  // transform forces
  for (i=1; i<=N_CART; ++i)
    v[i] = ucontact[L_AAA].f[i] - uext_sim[L_AAA].f[i];
  mat_vec_mult(M,v,vv);

  misc_sim_sensor[L_CFx] = vv[_X_];
  misc_sim_sensor[L_CFy] = vv[_Y_];
  misc_sim_sensor[L_CFz] = vv[_Z_];

  // transform torques
  for (i=1; i<=N_CART; ++i)
    v[i] = ucontact[L_AAA].t[i] - uext_sim[L_AAA].t[i];
  mat_vec_mult(M,v,v);

  // correct for load cell offset (need to use local coordinates)
  r[_X_] = FTA_Z_OFF;
  r[_Y_] = FTA_X_OFF;
  r[_Z_] = -FTA_Y_OFF;
  vec_mult_outer(vv,r,vv);

  misc_sim_sensor[L_CTa] = v[_A_] + vv[_A_];
  misc_sim_sensor[L_CTb] = v[_B_] + vv[_B_];
  misc_sim_sensor[L_CTg] = v[_G_] + vv[_G_];

  // rotation matrix from world to R_AAA coordinates
  mat_trans_size(Adof_sim[R_AAA],N_CART,N_CART,M);

  // transform forces
  for (i=1; i<=N_CART; ++i)
    v[i] = ucontact[R_AAA].f[i] - uext_sim[R_AAA].f[i];
  mat_vec_mult(M,v,vv);

  misc_sim_sensor[R_CFx] = vv[_X_];
  misc_sim_sensor[R_CFy] = vv[_Y_];
  misc_sim_sensor[R_CFz] = vv[_Z_];

  // transform torques
  for (i=1; i<=N_CART; ++i)
    v[i] = ucontact[R_AAA].t[i] - uext_sim[R_AAA].t[i];
  mat_vec_mult(M,v,v);

  // correct for load cell offset (need to use local coordinates)
  r[_X_] = FTA_Z_OFF;
  r[_Y_] = FTA_X_OFF;
  r[_Z_] = FTA_Y_OFF;
  vec_mult_outer(vv,r,vv);

  misc_sim_sensor[R_CTa] = v[_A_] + vv[_X_];
  misc_sim_sensor[R_CTb] = v[_B_] + vv[_Y_];
  misc_sim_sensor[R_CTg] = v[_G_] + vv[_Z_];

}
#endif
