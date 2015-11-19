/*
 * lwpr_sfun.c: Based on matlabroot/simulink/src/sfuntmpl_doc.c
 * Stefan Schaal, February 2003
 */

#define S_FUNCTION_NAME  lwpr_sfun
#define S_FUNCTION_LEVEL 2

enum S_FUNCTION_PARMS {
  N_STATES = 0,
  N_COMMANDS,
  N_TARGETS,
  N_PREDICTIONS,
  T_SAMPLE,
  LWPR_SCRIPT,
  LWPR_ID,
  ALPHA_TRACK,
  PART_MAT,
  NEW_LWPR,
  N_S_FUNCTION_PARMS
};

enum S_FUNCTION_INPUT_PORT {
  STATE_PREDICT_PORT=0,
  COMMAND_PREDICT_PORT,
  STATE_LEARN_PORT,
  COMMAND_LEARN_PORT,
  TARGET_LEARN_PORT,
  TRACKING_ERROR_PORT,
  N_S_FUNCTION_INPUT_PORTS
};

enum S_FUNCTION_OUTPUT_PORTS {
  FUNCTION_PREDICT_PORT=0,
  TARGET_PREDICT_PORT,
  TARGET_PORT,
  N_S_FUNCTION_OUTPUT_PORTS
};

#include "simstruc.h"
#include "utility.h"
#include "lwpr.h"

/* local variable for learning */
static int models_read[MAX_LWPRS+1];
static int rfID_predict[MAX_LWPRS+1];
static int rfID_learn[MAX_LWPRS+1];

/* string for matlab error messages -- needs to be static according to MATLAB docs */
static char errormsg[100];

/*====================*
 * S-function methods *
 *====================*/

/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *    The sizes information is used by Simulink to determine the S-function
 *    block's characteristics (number of inputs, outputs, states, etc.).
 */
static void mdlInitializeSizes(SimStruct *S)
{

  const real_T *n_states_ptr     = mxGetData(ssGetSFcnParam(S, N_STATES));
  const real_T *n_commands_ptr   = mxGetData(ssGetSFcnParam(S, N_COMMANDS));
  const real_T *n_targets_ptr    = mxGetData(ssGetSFcnParam(S, N_TARGETS));
  const real_T *n_predictions_ptr= mxGetData(ssGetSFcnParam(S, N_PREDICTIONS));
  int   n_states      = (int) n_states_ptr[0];
  int   n_commands    = (int) n_commands_ptr[0];
  int   n_targets     = (int) n_targets_ptr[0];
  int   n_predictions = (int) n_predictions_ptr[0];

  int i;

  ssSetNumSFcnParams(S, N_S_FUNCTION_PARMS); /* Number of exp. parameters */
  if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
    ssSetErrorStatus(S,"Incorrect number of s-function parameters");
    return;
  }
  ssSetSFcnParamTunable(S, 0, 0); /* all parameters are constant */
  
  /* this function has the predicted targets as internal states */
  ssSetNumContStates(S, 0);
  ssSetNumDiscStates(S, n_targets*2);
  
  /* set input ports: due to time alignment issues, it is often useful
     to provide different state and command vectors for learning than
     for lookup
     port 0: state vector for lookup
     port 1: command vector for lookup
     port 2: state vector for learning
     port 3: command vector for learning
     port 4: target vector for learning
     port 5: tracking error
  */
  if (!ssSetNumInputPorts(S, N_S_FUNCTION_INPUT_PORTS)) 
    return;
  
  ssSetInputPortWidth(S, STATE_PREDICT_PORT, n_states);
  ssSetInputPortWidth(S, COMMAND_PREDICT_PORT, n_commands);
  ssSetInputPortWidth(S, STATE_LEARN_PORT, n_states);
  ssSetInputPortWidth(S, COMMAND_LEARN_PORT, n_commands);
  ssSetInputPortWidth(S, TARGET_LEARN_PORT, n_targets);
  ssSetInputPortWidth(S, TRACKING_ERROR_PORT, n_targets);
  
  for (i=0; i<N_S_FUNCTION_INPUT_PORTS; ++i) {
    ssSetInputPortRequiredContiguous(S, i, true);
    ssSetInputPortDirectFeedThrough(S, i, false);
  }
  ssSetInputPortDirectFeedThrough(S, STATE_PREDICT_PORT, true);
  ssSetInputPortDirectFeedThrough(S, COMMAND_PREDICT_PORT, true);
  
  /* set output ports:
     port 1: predicted terms according to the partition matrix part_mat
     port 2: target and predicted values
  */
  
  if (!ssSetNumOutputPorts(S, N_S_FUNCTION_OUTPUT_PORTS)) 
    return;

  ssSetOutputPortWidth(S, FUNCTION_PREDICT_PORT, n_predictions);
  ssSetOutputPortWidth(S, TARGET_PREDICT_PORT, n_targets);
  ssSetOutputPortWidth(S, TARGET_PORT, n_targets);
  
  ssSetNumSampleTimes(S, 1);
  ssSetNumRWork(S, 0);
  ssSetNumIWork(S, 0);
  ssSetNumPWork(S, 0);
  ssSetNumModes(S, 0);
  ssSetNumNonsampledZCs(S, 0);
  
  ssSetOptions(S, 0);
}



/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    This function is used to specify the sample time(s) for your
 *    S-function. You must register the same number of sample times as
 *    specified in ssSetNumSampleTimes.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
  const real_T *t_sample_ptr = mxGetData(ssGetSFcnParam(S, T_SAMPLE));

  ssSetSampleTime(S, 0, t_sample_ptr[0]);
  ssSetOffsetTime(S, 0, 0.0);
  
}

#define MDL_START
/* Function: mdlStart =======================================================
 * Abstract:
 *    This function is called once at start of model execution. If you
 *    have states that should be initialized once, this is the place
 *    to do it.
 */
static void mdlStart(SimStruct *S)
{
  const real_T  *n_states_ptr     = mxGetData(ssGetSFcnParam(S, N_STATES));
  const real_T  *n_commands_ptr   = mxGetData(ssGetSFcnParam(S, N_COMMANDS));
  const real_T  *n_targets_ptr    = mxGetData(ssGetSFcnParam(S, N_TARGETS));
  const real_T  *n_predictions_ptr= mxGetData(ssGetSFcnParam(S, N_PREDICTIONS));
  const real_T  *lwpr_ID_ptr      = mxGetData(ssGetSFcnParam(S, LWPR_ID));
  const int16_T *lwpr_script_ptr  = mxGetData(ssGetSFcnParam(S, LWPR_SCRIPT));
  const int_T    n_lwpr_script    = mxGetNumberOfElements(ssGetSFcnParam(S, LWPR_SCRIPT));
  const real_T  *new_lwpr_ptr     = mxGetData(ssGetSFcnParam(S, NEW_LWPR));
  real_T        *disc_states      = ssGetRealDiscStates(S);
  int   n_states      = (int) n_states_ptr[0];
  int   n_commands    = (int) n_commands_ptr[0];
  int   n_targets     = (int) n_targets_ptr[0];
  int   n_predictions = (int) n_predictions_ptr[0];
  int   lwpr_ID       = (int) lwpr_ID_ptr[0];
  int   new_lwpr      = (int) new_lwpr_ptr[0];
  char  lwpr_script[100];

  int i;
  static int firsttime = FALSE;
  int n_in;
  int n_out;
  int linear_gx;

  /* convert the character string for lwpr_script */
  for (i=0; i<n_lwpr_script; ++i)
    lwpr_script[i] = (char) lwpr_script_ptr[i];
  lwpr_script[n_lwpr_script]='\0';
  
  if (firsttime) { /* need to make sure that multiple LWPR blocks do not interfer */
    for (i=1; i<=MAX_LWPRS; ++i)
      models_read[i] = FALSE;
    firsttime = TRUE;
  }

  /* read or initialize the LWPR models */
  if (models_read[lwpr_ID]) {
    deleteLWPR(lwpr_ID);
    models_read[lwpr_ID] = FALSE;
  }

  if (!readLWPRScript(lwpr_script,new_lwpr,lwpr_ID)) {
    sprintf(errormsg,"Error when reading script file >%s<\n",lwpr_script);
    ssSetErrorStatus(S,errormsg);
    return;
  }

  models_read[lwpr_ID] = TRUE;

  /* check whether the LWPR model specification coincide with the number of
     states and commands of the s-function */
  n_in      = lwprs[lwpr_ID].n_in_reg;
  n_out     = lwprs[lwpr_ID].n_out;
  linear_gx = lwprs[lwpr_ID].linear_gx;

  if (!linear_gx && n_in != n_states+n_commands) {
    sprintf(errormsg,
	    "Number of inputs (%d) in >%s< does not match n_states+n_commands (%d+%d) from LWPR block\n",
	    n_in,lwpr_script,n_states,n_commands);
    ssSetErrorStatus(S,errormsg);
    return;
  } else if (linear_gx && n_in != n_states+n_commands+n_states*n_commands) {
    sprintf(errormsg,
	    "Number of inputs (%d) in >%s< does not match n_states+n_commands+n_states*n_commands (%d+%d+%d) from LWPR block\n",
	    n_in,lwpr_script,n_states,n_commands,n_states*n_commands);
    ssSetErrorStatus(S,errormsg);
    return;
  }

  if (n_out != n_targets) {
    sprintf(errormsg,
	    "Number of outputs (%d) in >%s< does not match n_targets (%d) from LWPR block\n",
	    n_out,lwpr_script,n_targets);
    ssSetErrorStatus(S,errormsg);
    return;
  }

}


/* Function: mdlOutputs =======================================================
 * Abstract:
 *    In this function, you compute the outputs of your S-function
 *    block. Generally outputs are placed in the output vector, ssGetY(S).
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
  const real_T  *n_states_ptr     = mxGetData(ssGetSFcnParam(S, N_STATES));
  const real_T  *n_commands_ptr   = mxGetData(ssGetSFcnParam(S, N_COMMANDS));
  const real_T  *n_targets_ptr    = mxGetData(ssGetSFcnParam(S, N_TARGETS));
  const real_T  *n_predictions_ptr= mxGetData(ssGetSFcnParam(S, N_PREDICTIONS));
  const real_T  *lwpr_ID_ptr      = mxGetData(ssGetSFcnParam(S, LWPR_ID));
  const real_T  *part_mat         = mxGetData(ssGetSFcnParam(S, PART_MAT));
  const int_T    n_part_mat       = mxGetNumberOfElements(ssGetSFcnParam(S, PART_MAT));
  real_T        *disc_states      = ssGetRealDiscStates(S);
  int   n_states      = (int) n_states_ptr[0];
  int   n_commands    = (int) n_commands_ptr[0];
  int   n_targets     = (int) n_targets_ptr[0];
  int   n_predictions = (int) n_predictions_ptr[0];
  int   lwpr_ID       = (int) lwpr_ID_ptr[0];

  const real_T *x  = (const real_T*) ssGetInputPortSignal(S,STATE_PREDICT_PORT);
  const real_T *u  = (const real_T*) ssGetInputPortSignal(S,COMMAND_PREDICT_PORT);
  real_T       *y  = ssGetOutputPortSignal(S,FUNCTION_PREDICT_PORT);
  real_T       *tp = ssGetOutputPortSignal(S,TARGET_PREDICT_PORT);
  real_T       *t  = ssGetOutputPortSignal(S,TARGET_PORT);

  /* local variables */
  static double cutoff = 0.001;
  static Matrix out=NULL;
  static iMatrix ypart=NULL;
  static int n_in_allocated=0;
  static int n_out_allocated=0;
  static int n_predictions_allocated=0;

  int n_in;
  int n_out;
  int flag=FALSE;
  int delta_predictions=0;
  int i,j,count;
  int linear_gx;

  /* given that we share the in, out, and ypart variables across all LWPR
     models in one simulink simulation, we need to ensure that they are
     sufficiently dimensioned */
  n_in      = lwprs[lwpr_ID].n_in_reg;
  n_out     = lwprs[lwpr_ID].n_out;
  linear_gx = lwprs[lwpr_ID].linear_gx;

  if (n_out > n_out_allocated || n_predictions > n_predictions_allocated) {
    if (out==NULL)
      out = my_matrix(1,n_out,1,n_predictions);
    else
      mat_add_shape(&out, n_out-n_out_allocated,n_predictions-n_predictions_allocated);
    delta_predictions = n_predictions-n_predictions_allocated;
    n_out_allocated = n_out;
    n_predictions_allocated = n_predictions;
  }

  if (delta_predictions!=0 || n_in > n_in_allocated) {
    if (ypart==NULL)
      ypart = my_imatrix(1,n_predictions,1,n_in+1);
    else
      imat_add_shape(&ypart, delta_predictions,n_in-n_in_allocated);
    n_in_allocated = n_in;
  }

  if (n_part_mat != n_predictions*(n_in+1)) {
    sprintf(errormsg,
	    "Number of elements in partion matrix (=%d) does not match n_precitions*(n_in+1) (=%d)\n",
	    n_part_mat,n_predictions*(n_in+1));
    ssSetErrorStatus(S,errormsg);
    return;
  }

  /* assign the partition matrix */
  count=0;
  for (i=1; i<=n_in+1; ++i)
    for (j=1; j<=n_predictions; ++j)
      ypart[j][i] = part_mat[count++];

  predictLWPROutputPartCompControl(lwpr_ID,(double *)(x-1),(double *)(u-1),cutoff,TRUE,
				   n_predictions,ypart,out,&(rfID_predict[lwpr_ID]));

  count = 0;
  for (i=1; i<=n_out; ++i)
    for (j=1; j<=n_predictions; ++j)
      y[count++] = out[i][j];

  /* the predicted outputs and training outputs are kept as internal state, such
     that they can serve as output in mdlOutput. Keeping as the the training
     data as internal state is to avoid a time mal-alignment between prediction
     and training output */

  for (i=1; i<=n_out; ++i) {
    tp[i-1] = disc_states[i-1];
    t[i-1]  = disc_states[n_out+i-1];
  }

}



#define MDL_UPDATE
/* Function: mdlUpdate ======================================================
 * Abstract:
 *    This function is called once for every major integration time step.
 *    Discrete states are typically updated here, but this function is useful
 *    for performing any tasks that should only take place once per
 *    integration step.
 */
static void mdlUpdate(SimStruct *S, int_T tid)
{
  const real_T  *n_states_ptr     = mxGetData(ssGetSFcnParam(S, N_STATES));
  const real_T  *n_commands_ptr   = mxGetData(ssGetSFcnParam(S, N_COMMANDS));
  const real_T  *n_targets_ptr    = mxGetData(ssGetSFcnParam(S, N_TARGETS));
  const real_T  *n_predictions_ptr= mxGetData(ssGetSFcnParam(S, N_PREDICTIONS));
  const real_T  *lwpr_ID_ptr      = mxGetData(ssGetSFcnParam(S, LWPR_ID));
  const real_T  *alpha_track_ptr  = mxGetData(ssGetSFcnParam(S, ALPHA_TRACK));
  real_T        *disc_states      = ssGetRealDiscStates(S);
  int    n_states      = (int) n_states_ptr[0];
  int    n_commands    = (int) n_commands_ptr[0];
  int    n_targets     = (int) n_targets_ptr[0];
  int    n_predictions = (int) n_predictions_ptr[0];
  int    lwpr_ID       = (int) lwpr_ID_ptr[0];
  double alpha_track   = (double) alpha_track_ptr[0];
  int    i;

  const real_T *x  = (const real_T*) ssGetInputPortSignal(S,STATE_LEARN_PORT);
  const real_T *u  = (const real_T*) ssGetInputPortSignal(S,COMMAND_LEARN_PORT);
  const real_T *y  = (const real_T*) ssGetInputPortSignal(S,TARGET_LEARN_PORT);
  const real_T *e  = (const real_T*) ssGetInputPortSignal(S,TRACKING_ERROR_PORT);

  addDataToLWPRCompControl(lwpr_ID, (double *)(x-1), (double *)(u-1), 
			   (double *)(y-1), FALSE, &(rfID_learn[lwpr_ID]),
			   (double *)(disc_states-1),(double *)(e-1), alpha_track);

  for (i=1; i<=n_targets; ++i)
    disc_states[n_targets+i-1]=y[i-1];

}


/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S)
{

  const real_T  *lwpr_ID_ptr      = mxGetData(ssGetSFcnParam(S, LWPR_ID));
  int   lwpr_ID       = (int) lwpr_ID_ptr[0];

  if (models_read[lwpr_ID]) {
    writeLWPR(lwpr_ID);
  }

}


/*======================================================*
 * See sfuntmpl_doc.c for the optional S-function methods *
 *======================================================*/

/*=============================*
 * Required S-function trailer *
 *=============================*/

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
