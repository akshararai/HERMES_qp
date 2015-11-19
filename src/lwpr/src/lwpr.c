/*!=============================================================================
  ==============================================================================

  \file    lwpr.c

  \author  Stefan Schaal
  \date    March 1999

  ==============================================================================
  \remarks
  
  This implements locally weighted projection regression

  ============================================================================*/

// general UNIX headers

#include "stdio.h"
#include "math.h"
#include "string.h"

// my utilities library 
#include "utility.h"

// the lwpr header file 
#include "lwpr.h"

//! sphering of local model distribution TRUE/FALSE
#define SPHERING FALSE

//! a macro for changing the forgetting factor as function of the weight
#define LAMBDA(l,w) (1.+w*(l-1.))
//! #define LAMBDA(l,w) (l)

// all the LWPR models which can be handled by this program
LWPR  lwprs[MAX_LWPRS+1];

// this flag turns on a large amount of print-outs
static int  DEBUG_FLAG = FALSE;

// needed for weight normalization
static double  sum_activation=0;

// local functions
static int 
addRF(int ID, int n_proj, int n_proj_2nd, int n_nn);
static int 
initRF(int      ID, 
       int      rfID,
       Vector   c,
       Matrix   M);
static double
myExp(double value, int lookup);
static double
calculateWeight(int ID, int rfID, Vector x);
static double
calculateMahal(int ID, int rfID, Vector x);
static double
calculateWeightp(int ID, int rfID, Vector x);
static double
calculateMahalp(int ID, int rfID, Vector x);
static void
updateMeanVar(int ID, int rfID, Vector x_w, Vector x, Vector xmz);
static void
updateRegMeanVar(int ID, int rfID, Vector y, Vector ymz);
static int
checkAddDim(int ID, int rfID);
static int
addDimension(int ID, int rfID);
static void
calculateRegression(int ID, int rfID, Vector x_mz, Vector y_mz);
static void
updateAdmVars(int ID, int rfID);
static void
updateDistanceMetric(int ID, int rfID, double *x, double *x_mz);
static void
createFakeTarget(int ID,int i, Vector xmz, Vector err,int flag, Vector yfake);
static double
checkNeighborhood(int ID, int rfID1, int rfID2);
static int
deleteNNFromRF(int ID, int rfID, int del_rfID);
static int
addNNToRF(int ID, int rfID, int add_rfID, double w);
static int
swapNNID(int ID, int rfID, int old_rfID, int new_rfID);
static void 
calculateDWDM(int ID, int rfID, Vector x_w, double penalty, Matrix dwDM, Matrix dpdM,
	      Matrix dppdMdM, Matrix dwwdMdM, double *max_M);
static void
calculateRegression2nd(int ID, int rfID, Vector x_mz, Vector y_mz);
static void
updateRegMeanVar2nd(int ID, int rfID, Vector y, Vector ymz);
static void
createFakeTarget2nd(int ID,int rfID, Vector xmz, Vector err,int flag, 
		    Vector yfake);
static void
updateMeanVar2nd(int ID, int rfID, Vector x, Vector xmz);
static void
updateAdmVars2nd(int ID, int rfID);
static int
addDimension2nd(int ID, int rfID);
static int
checkAddDim2nd(int ID, int rfID);


/*!*****************************************************************************
 *******************************************************************************
 \note  initLWPR
 \date  November, 1994
 
 \remarks 
 
 Initiates a lwpr model with all mandatory values and default
 values otherwise. The default values can be changed by dedicated functions.
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     n_in_w          : number of input dimensions for weights
 \param[in]     n_in_reg        : number of input dimensions for regression
 \param[in]     n_in_reg_2nd    : number of input dimensions for 2nd regression 
                      (0=no 2nd model)
 \param[in]     n_out           : number of output dimensions for regression
 \param[in]     n_out_2nd       : number of output dimensions for 2nd regression
 \param[in]     diag_only       : TRUE/FALSE to update only the diagonal distance metric
 \param[in]     meta            : TRUE/FALSE to allow the use of a meta learning parameter
 \param[in]     meta_rate       : the meta learning rate
 \param[in]     penalty         : a smoothness bias, usually a pretty small number (1.e-4)
 \param[in]     init_alpha      : the initial learning rates
 \param[in]     norm_in_w       : the normalization of the inputs for weights
 \param[in]     norm_in_reg     : the normalization of the inputs for regression
 \param[in]     norm_in_reg_2nd : the normalization of the inputs for 2nd regression
 \param[in]     norm_out        : the normalization of the outputs for regression
 \param[in]     norm_out_2nd    : the normalization of the outputs for 2nd regression
 \param[in]     name            : a name for the model
 \param[in]     ID              : the id to be assigned to this LWPR model
 \param[in]     ini             : an LWPR structure to initialize the scalar values
                      (pass NULL for default initialization)
 
 ******************************************************************************/
int 
initLWPR(int      n_in_w,
	 int      n_in_reg,
	 int      n_in_reg_2nd,
	 int      n_out, 
	 int      n_out_2nd, 
	 int      diag_only, 
	 int      meta, 
	 double   meta_rate, 
	 double	  penalty, 
	 double   init_alpha, 
	 Vector   norm_in_w,
	 Vector   norm_in_reg,
	 Vector   norm_in_reg_2nd,
	 Vector   norm_out,
	 Vector   norm_out_2nd,
	 char    *name, 
	 int      ID,
	 LWPR    *ini)
     
{
  int     i,j;
  int     rc = TRUE;
  double  aux;
  double *temp;

  if (ID > MAX_LWPRS || ID < 0) {
    printf("LWPR ID=%d must be in the range of %d until %d --  change ID\n",
	   ID,0,MAX_LWPRS);
    return FALSE;
  }
  
  if (lwprs[ID].lwpr_used) {
    printf("LWPR ID=%d is already used -- change ID\n",ID);
    return FALSE;
  }
  
  if (ini != NULL) {

    lwprs[ID] = *ini;

  } else {

    /* otherwise just assign the few variables to the appropriate structures */
    lwprs[ID].n_in_reg            = n_in_reg;
    lwprs[ID].n_in_reg_2nd        = n_in_reg_2nd;
    lwprs[ID].n_in_w              = n_in_w;
    lwprs[ID].n_out               = n_out;
    lwprs[ID].n_out_2nd           = n_out_2nd;
    lwprs[ID].n_data              = 0;
    lwprs[ID].n_data_2nd          = 0;
    lwprs[ID].w_gen               = DEF_W_GEN;
    lwprs[ID].w_prune             = DEF_W_PRUNE;
    lwprs[ID].factor_prune        = DEF_FACTOR_PRUNE;
    lwprs[ID].n_rf                = 0;
    lwprs[ID].n_rf_allocated      = 0;
    lwprs[ID].lwpr_used           = TRUE;
    lwprs[ID].use_offsets         = TRUE;
    lwprs[ID].use_offsets_2nd     = TRUE;
    lwprs[ID].final_lambda        = DEF_FINAL_LAMBDA;
    lwprs[ID].tau_lambda          = DEF_TAU_LAMBDA;
    lwprs[ID].init_S2             = DEF_INIT_S2;
    lwprs[ID].diag_only           = diag_only;
    lwprs[ID].penalty             = penalty;
    lwprs[ID].allow_meta_learning = meta;
    lwprs[ID].meta_learning_rate  = meta_rate;
    lwprs[ID].n_pruned_overlap    = 0;
    lwprs[ID].n_pruned_error      = 0;
    lwprs[ID].add_threshold       = DEF_ADD_THRESHOLD;
    lwprs[ID].max_rfs             = MAX_RFS;
    lwprs[ID].d_noise             = DEF_D_NOISE;
    lwprs[ID].c_noise             = DEF_C_NOISE;
    lwprs[ID].allow_d_update      = TRUE;
    lwprs[ID].allow_addDim        = TRUE;
    lwprs[ID].allow_add_rfs       = TRUE;
    lwprs[ID].write_rfs           = TRUE;
    lwprs[ID].mean_n_proj         = 0.0;
    lwprs[ID].mean_n_proj_2nd     = 0.0;
    lwprs[ID].contrib_n_rf        = 0.0;
    lwprs[ID].use_nn              = FALSE;
    lwprs[ID].max_nn_comp         = 1000000000;
    lwprs[ID].kernel_function     = GAUSSIAN;
    strcpy(lwprs[ID].lwpr_name,name);

    // check whether second regression model should be initialized 
    if (n_in_reg_2nd == 0)

      lwprs[ID].use_reg_2nd = FALSE;

    else {

      lwprs[ID].use_reg_2nd = TRUE;
      if (n_in_reg_2nd > DEF_DIM) {
	lwprs[ID].init_n_proj_2nd = DEF_DIM + 1;
      } else {
	lwprs[ID].init_n_proj_2nd = n_in_reg_2nd;
      }

    }

    if (n_in_reg > DEF_DIM) {
      lwprs[ID].init_n_proj = DEF_DIM + 1;
    } else {
      lwprs[ID].init_n_proj = n_in_reg;
    }
    
    lwprs[ID].init_alpha       = init_alpha;
 
  }

  // the matrices in this structure
  lwprs[ID].init_D           = my_matrix(1,n_in_w,1,n_in_w);
  lwprs[ID].init_M           = my_matrix(1,n_in_w,1,n_in_w);
  lwprs[ID].mean_x_w         = my_vector(1,n_in_w);
  lwprs[ID].var_x_w          = my_vector(1,n_in_w);
  lwprs[ID].mean_x_reg       = my_vector(1,n_in_reg);
  lwprs[ID].var_x_reg        = my_vector(1,n_in_reg);
  lwprs[ID].norm_in_w        = my_vector(1,n_in_w);
  lwprs[ID].norm_in_reg      = my_vector(1,n_in_reg);
  lwprs[ID].norm_out         = my_vector(1,n_out);

  if (lwprs[ID].use_reg_2nd) {
    lwprs[ID].norm_in_reg_2nd = my_vector(1,n_in_reg_2nd);
    lwprs[ID].norm_out_2nd    = my_vector(1,n_out_2nd);
  }
  
  /* some initialization */
  vec_zero(lwprs[ID].mean_x_reg);
  vec_zero(lwprs[ID].var_x_reg);
  vec_zero(lwprs[ID].mean_x_w);
  vec_zero(lwprs[ID].var_x_w);

  /* the normalization factors */
  vec_equal(norm_in_w,lwprs[ID].norm_in_w);
  vec_equal(norm_in_reg,lwprs[ID].norm_in_reg);
  vec_equal(norm_out,lwprs[ID].norm_out);

  if (lwprs[ID].use_reg_2nd) {
    vec_equal(norm_in_reg_2nd,lwprs[ID].norm_in_reg_2nd);
    vec_equal(norm_out_2nd,lwprs[ID].norm_out_2nd);
  }

  /* the distance metric */
  for (i=1; i<=n_in_w; ++i) {
    for (j=i; j<=n_in_w; ++j) {
      if (i==j) {
	lwprs[ID].init_D[i][j] = DEF_DIAG_D;
      } else {
	lwprs[ID].init_D[i][j] = DEF_OFFDIAG_D;
	if (diag_only) 
	  lwprs[ID].init_D[i][j] = 0.0;
      }
      lwprs[ID].init_D[j][i] = lwprs[ID].init_D[i][j];
      lwprs[ID].init_M[j][i] = lwprs[ID].init_D[i][j];
      lwprs[ID].init_M[i][j] = lwprs[ID].init_D[i][j];
    }
  }

  /* get the init_M by cholesky decomposition */
  temp = my_vector(1,n_in_w);

  if (!my_choldc(lwprs[ID].init_M,n_in_w,temp)) {

    printf("ERROR: Matrix is not positive definite\n");
    return FALSE;

  } else {

    /* since choldc returns the M matrix as lower triangular matrix
       in init_M, and the diagonal elements in temp, I have
       to fix this nonsense */

    for (i=1; i<=n_in_w; ++i) {
      for (j=1; j<=n_in_w; ++j) {
	lwprs[ID].init_M[i][j] = lwprs[ID].init_M[j][i];
	if (i==j) {
	  lwprs[ID].init_M[i][j] = temp[i];
	} else if (i>j) {
	  lwprs[ID].init_M[i][j] = 0.0;
	}
      }
    }

  }

  my_free_vector(temp,1,n_in_w);

  /* init the exp table by doing one lookup */
  aux = myExp(-.01,USE_EXP_LOOKUP);

  /* allocate working memory for this LWPR */
  lwprs[ID].x                = my_vector(1,n_in_reg);
  lwprs[ID].x_w              = my_vector(1,n_in_w);
  lwprs[ID].y                = my_vector(1,n_out);
  lwprs[ID].xp               = my_vector(1,n_in_reg);
  lwprs[ID].xp_w             = my_vector(1,n_in_w);
  lwprs[ID].x_mz             = my_vector(1,n_in_reg);
  lwprs[ID].x_mzp            = my_vector(1,n_in_reg);
  lwprs[ID].y_mz             = my_vector(1,n_out);
  lwprs[ID].delta            = my_vector(1,n_in_w);
  lwprs[ID].delta2           = my_vector(1,n_in_w);
  lwprs[ID].xres             = my_vector(1,n_in_reg);
  lwprs[ID].yres             = my_vector(1,n_out);
  lwprs[ID].ypred            = my_vector(1,n_out);
  lwprs[ID].cr               = my_vector(1,n_in_reg);
  lwprs[ID].x_temp           = my_vector(1,n_in_reg);
  lwprs[ID].x_temp_w         = my_vector(1,n_in_w);
  lwprs[ID].x_tempp          = my_vector(1,n_in_reg);
  lwprs[ID].D_temp           = my_matrix(1,n_in_w,1,n_in_w);
  lwprs[ID].error_cv         = my_matrix(1,n_in_reg,1,n_out);
  lwprs[ID].error            = my_matrix(1,n_in_reg,1,n_out);
  lwprs[ID].dwdM             = my_matrix(1,n_in_w,1,n_in_w);
  lwprs[ID].dpdM             = my_matrix(1,n_in_w,1,n_in_w);
  lwprs[ID].dwwdMdM          = my_matrix(1,n_in_w,1,n_in_w);
  lwprs[ID].dppdMdM          = my_matrix(1,n_in_w,1,n_in_w);
  lwprs[ID].sum_y_predicted  = my_vector(1,n_out);
  lwprs[ID].derivatives_ok   = my_ivector(1,n_in_reg);
  lwprs[ID].y_mean_predicted = my_vector(1,n_out);
  lwprs[ID].sum_y_predicted2 = my_vector(1,n_out);
  lwprs[ID].sum_y_conf       = my_vector(1,n_out);
  lwprs[ID].y_predicted      = my_vector(1,n_out);
  lwprs[ID].y_predicted2     = my_vector(1,n_out);
  lwprs[ID].y_conf           = my_vector(1,n_out);

  if (lwprs[ID].use_reg_2nd) {
    lwprs[ID].sum_y_predicted_2nd  = my_vector(1,n_out_2nd);
    lwprs[ID].sum_y_conf_2nd       = my_vector(1,n_out_2nd);
    lwprs[ID].x_2nd                = my_vector(1,n_in_reg_2nd);
    lwprs[ID].y_2nd                = my_vector(1,n_out_2nd);
    lwprs[ID].sum_y_predicted2_2nd = my_vector(1,n_out_2nd);
    lwprs[ID].y_mz_2nd             = my_vector(1,n_out_2nd);
    lwprs[ID].x_mz_2nd             = my_vector(1,n_in_reg_2nd);
    lwprs[ID].x_mzp_2nd            = my_vector(1,n_in_reg_2nd);
    lwprs[ID].ypred_2nd            = my_vector(1,n_out_2nd);
    lwprs[ID].x_temp_2nd           = my_vector(1,n_in_reg_2nd);
    lwprs[ID].x_tempp_2nd          = my_vector(1,n_in_reg_2nd);
    lwprs[ID].xres_2nd             = my_vector(1,n_in_reg_2nd);
    lwprs[ID].error_cv_2nd         = my_matrix(1,n_in_reg_2nd,1,n_out_2nd);
    lwprs[ID].yres_2nd             = my_vector(1,n_out_2nd);
    lwprs[ID].cr_2nd               = my_vector(1,n_in_reg_2nd);
    lwprs[ID].error_2nd            = my_matrix(1,n_in_reg_2nd,1,n_out_2nd);
    lwprs[ID].xp_2nd               = my_vector(1,n_in_reg_2nd);
    lwprs[ID].y_conf_2nd           = my_vector(1,n_out_2nd);
    lwprs[ID].y_predicted_2nd      = my_vector(1,n_out_2nd);
    lwprs[ID].y_predicted2_2nd     = my_vector(1,n_out_2nd);
  }

  /* zero some matrices */

  return rc;
  
}

/*!*****************************************************************************
 *******************************************************************************
 \note  deleteLWPR
 \date  November, 1994
 
 \remarks 
 
 erases a LWPR from memory
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ID        : ID of lwpr

 ******************************************************************************/
int
deleteLWPR(int ID)

{

  int i;
  int n_in_w,n_in_reg,n_out,n_out_2nd,n_in_reg_2nd;

  if (!lwprs[ID].lwpr_used) {
    printf("No #LWPR=%d existent!\n",ID);
    return FALSE;
  }

  n_in_reg      = lwprs[ID].n_in_reg;
  n_in_reg_2nd  = lwprs[ID].n_in_reg_2nd;
  n_in_w        = lwprs[ID].n_in_w;
  n_out         = lwprs[ID].n_out;
  n_out_2nd     = lwprs[ID].n_out_2nd;

  /* delete all rfs */
  while (lwprs[ID].n_rf > 0)
    deleteRF(ID,1);

  /* free LWPR memory */
  if (lwprs[ID].n_rf_allocated > 0)
    free(&lwprs[ID].rfs[1]);
  my_free_matrix(lwprs[ID].init_D,1,n_in_w,1,n_in_w);
  my_free_matrix(lwprs[ID].init_M,1,n_in_w,1,n_in_w);
  my_free_vector(lwprs[ID].mean_x_reg,1,n_in_reg);
  my_free_vector(lwprs[ID].var_x_reg,1,n_in_reg);
  my_free_vector(lwprs[ID].mean_x_w,1,n_in_w);
  my_free_vector(lwprs[ID].var_x_w,1,n_in_w);
  my_free_vector(lwprs[ID].norm_in_w,1,n_in_w);
  my_free_vector(lwprs[ID].norm_in_reg,1,n_in_reg);
  my_free_vector(lwprs[ID].norm_out,1,n_out);

  my_free_vector(lwprs[ID].x,1,n_in_reg);
  my_free_vector(lwprs[ID].x_w,1,n_in_w);
  my_free_vector(lwprs[ID].xp,1,n_in_reg);
  my_free_vector(lwprs[ID].xp_w,1,n_in_w);
  my_free_vector(lwprs[ID].x_mz,1,n_in_reg);
  my_free_vector(lwprs[ID].x_mzp,1,n_in_reg);
  my_free_vector(lwprs[ID].y_mz,1,n_out);
  my_free_vector(lwprs[ID].delta,1,n_in_w);
  my_free_vector(lwprs[ID].delta2,1,n_in_w);
  my_free_vector(lwprs[ID].xres,1,n_in_reg);
  my_free_vector(lwprs[ID].yres,1,n_out);
  my_free_vector(lwprs[ID].ypred,1,n_out);
  my_free_vector(lwprs[ID].cr,1,n_in_reg);
  my_free_vector(lwprs[ID].x_temp,1,n_in_reg);
  my_free_vector(lwprs[ID].x_temp_w,1,n_in_w);
  my_free_vector(lwprs[ID].x_tempp,1,n_in_reg);
  my_free_matrix(lwprs[ID].D_temp,1,n_in_w,1,n_in_w);
  my_free_matrix(lwprs[ID].error_cv,1,n_in_reg,1,n_out);
  my_free_matrix(lwprs[ID].error,1,n_in_reg,1,n_out);
  my_free_matrix(lwprs[ID].dwdM,1,n_in_w,1,n_in_w);
  my_free_matrix(lwprs[ID].dpdM,1,n_in_w,1,n_in_w);
  my_free_matrix(lwprs[ID].dwwdMdM,1,n_in_w,1,n_in_w);
  my_free_matrix(lwprs[ID].dppdMdM,1,n_in_w,1,n_in_w);
  my_free_vector(lwprs[ID].sum_y_predicted,1,n_out);
  my_free_ivector(lwprs[ID].derivatives_ok,1,n_in_reg);
  my_free_vector(lwprs[ID].y_mean_predicted,1,n_out);
  my_free_vector(lwprs[ID].sum_y_predicted2,1,n_out);
  my_free_vector(lwprs[ID].sum_y_conf,1,n_out);
  my_free_vector(lwprs[ID].y_predicted,1,n_out);
  my_free_vector(lwprs[ID].y_predicted2,1,n_out);
  my_free_vector(lwprs[ID].y_conf,1,n_out);

  if (lwprs[ID].use_reg_2nd) {
     my_free_vector(lwprs[ID].norm_in_reg_2nd,1,n_in_reg_2nd);
     my_free_vector(lwprs[ID].norm_out_2nd,1,n_out_2nd);

     my_free_vector(lwprs[ID].sum_y_predicted_2nd,1,n_out_2nd);
     my_free_vector(lwprs[ID].sum_y_conf_2nd,1,n_out_2nd);
     my_free_vector(lwprs[ID].x_2nd,1,n_in_reg_2nd);
     my_free_vector(lwprs[ID].y_2nd,1,n_out_2nd);
     my_free_vector(lwprs[ID].sum_y_predicted2_2nd,1,n_out_2nd);
     my_free_vector(lwprs[ID].y_mz_2nd,1,n_out_2nd);
     my_free_vector(lwprs[ID].x_mz_2nd,1,n_in_reg_2nd);
     my_free_vector(lwprs[ID].x_mzp_2nd,1,n_in_reg_2nd);
     my_free_vector(lwprs[ID].ypred_2nd,1,n_out_2nd);
     my_free_vector(lwprs[ID].x_temp_2nd,1,n_in_reg_2nd);
     my_free_vector(lwprs[ID].x_tempp_2nd,1,n_in_reg_2nd);
     my_free_vector(lwprs[ID].xres_2nd,1,n_in_reg_2nd);
     my_free_matrix(lwprs[ID].error_cv_2nd,1,n_in_reg_2nd,1,n_out_2nd);
     my_free_vector(lwprs[ID].yres_2nd,1,n_out_2nd);
     my_free_vector(lwprs[ID].cr_2nd,1,n_in_reg_2nd);
     my_free_matrix(lwprs[ID].error_2nd,1,n_in_reg_2nd,1,n_out_2nd);
     my_free_vector(lwprs[ID].xp_2nd,1,n_in_reg_2nd);
     my_free_vector(lwprs[ID].y_conf_2nd,1,n_out_2nd);
     my_free_vector(lwprs[ID].y_predicted_2nd,1,n_out_2nd);
     my_free_vector(lwprs[ID].y_predicted2_2nd,1,n_out_2nd);

  }

  lwprs[ID].lwpr_used = FALSE;

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  addRF 
 \date  January, 1994
 
 \remarks 
 
 adds a receptive field to the given LWPR and returns the receptive field ID.
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ID         : the ID of the LWPR
 \param[in]     n_proj     : the number of projections of the regression model
 \param[in]     n_proj_2nd : the number of projections of the 2nd regression model
 \param[in]     n_nn       : the number of nearest neighbors to be allocated
 
 ******************************************************************************/
#define RF_CHUNK 50  /*!< I always add chunks to the RF structure */
static int 
addRF(int ID, int n_proj, int n_proj_2nd, int n_nn)
     
{

  int      i,j,k;
  size_t   number,length;
  RF      *nptr;
  RF      *optr;
  int      rfID;
  int      n_in_reg,n_in_w,n_out;
  
  n_in_reg      = lwprs[ID].n_in_reg;
  n_in_w        = lwprs[ID].n_in_w;
  n_out         = lwprs[ID].n_out;

  if (lwprs[ID].n_rf_allocated <= lwprs[ID].n_rf) {
  
    // this means the vector for the rf must be increased
    length = sizeof(RF);
    number = lwprs[ID].n_rf_allocated + RF_CHUNK;
    if (lwprs[ID].n_rf_allocated == 0) {
      nptr   = (RF *) calloc(number,length);
    } else {
      optr   = lwprs[ID].rfs + 1;
      nptr   = (RF *) realloc(optr,length*number);
    }
    if (nptr == NULL) {
      printf("Memory Alloc. Error in >add_rf< -- increase available memory\n");
      getchar();
      return FALSE;
    }
    
    lwprs[ID].n_rf_allocated += RF_CHUNK;
    lwprs[ID].rfs = nptr-1;
  
  }
  
  /* allocate the memory for the rf */
  rfID = lwprs[ID].n_rf + 1;
  
  /* note that some matrices are allocated as symmetric matrices to save 
     memory */
  lwprs[ID].rfs[rfID].B  = my_matrix(1,n_proj,1,n_out);

  if (lwprs[ID].diag_only) {
    lwprs[ID].rfs[rfID].D = my_matrix_diag(1,n_in_w,1,n_in_w);
    lwprs[ID].rfs[rfID].M = my_matrix_diag(1,n_in_w,1,n_in_w);
    lwprs[ID].rfs[rfID].alpha = my_matrix_diag(1,n_in_w,1,n_in_w);
  } else {
    lwprs[ID].rfs[rfID].D = my_matrix_symm(1,n_in_w,1,n_in_w);
    lwprs[ID].rfs[rfID].M = my_matrix_symm(1,n_in_w,1,n_in_w);
    lwprs[ID].rfs[rfID].alpha = my_matrix_symm(1,n_in_w,1,n_in_w);
  }

  if (lwprs[ID].allow_meta_learning) {
    if (lwprs[ID].diag_only) {
      lwprs[ID].rfs[rfID].h = my_matrix_diag(1,n_in_w,1,n_in_w);
      lwprs[ID].rfs[rfID].b = my_matrix_diag(1,n_in_w,1,n_in_w);
    } else {
      lwprs[ID].rfs[rfID].h = my_matrix_symm(1,n_in_w,1,n_in_w);
      lwprs[ID].rfs[rfID].b = my_matrix_symm(1,n_in_w,1,n_in_w);
    }
  }
  
  lwprs[ID].rfs[rfID].c   = my_vector(1,n_in_w);
  lwprs[ID].rfs[rfID].s   = my_vector(1,n_proj);

  lwprs[ID].rfs[rfID].SXresYres = my_matrix(1,n_proj,1,n_in_reg);
  lwprs[ID].rfs[rfID].ss2 = my_vector(1,n_proj);
  lwprs[ID].rfs[rfID].dof = my_vector(1,n_proj);
  lwprs[ID].rfs[rfID].SSYres = my_matrix(1,n_proj,1,n_out);
  lwprs[ID].rfs[rfID].SSXres = my_matrix(1,n_proj,1,n_in_reg);

  /* the matrix of projection vectors  */
  lwprs[ID].rfs[rfID].W     = my_matrix(1,n_proj,1,n_in_reg);
  lwprs[ID].rfs[rfID].Wnorm = my_vector(1,n_proj);
  lwprs[ID].rfs[rfID].U     = my_matrix(1,n_proj,1,n_in_reg);
  lwprs[ID].rfs[rfID].H     = my_matrix(1,n_proj,1,n_out);
  lwprs[ID].rfs[rfID].r     = my_vector(1,n_proj);
  lwprs[ID].rfs[rfID].T     = my_matrix(1,n_proj,1,n_in_reg);
  lwprs[ID].rfs[rfID].A     = my_matrix(1,n_proj,1,n_in_reg);

  /* mean and variance info */
  lwprs[ID].rfs[rfID].mean_x_w   = my_vector(1,n_in_w);
  lwprs[ID].rfs[rfID].var_x_w    = my_vector(1,n_in_w);
  lwprs[ID].rfs[rfID].mean_x_reg = my_vector(1,n_in_reg);
  lwprs[ID].rfs[rfID].var_x_reg  = my_vector(1,n_in_reg);
  lwprs[ID].rfs[rfID].mean_y     = my_vector(1,n_out);
  lwprs[ID].rfs[rfID].var_y      = my_vector(1,n_out);

  /* the sum of weights for every regression dimension */
  lwprs[ID].rfs[rfID].sum_weights   = my_vector(1,n_proj);
  lwprs[ID].rfs[rfID].sum_error     = my_vector(1,n_proj);
  lwprs[ID].rfs[rfID].sum_error_cvi = my_vector(1,n_proj);
  lwprs[ID].rfs[rfID].n_data        = my_vector(1,n_proj);
  lwprs[ID].rfs[rfID].lambda        = my_vector(1,n_proj);
  
  /* the nearest neigbor structure */
  lwprs[ID].rfs[rfID].n_nn_allocated = n_nn;
  lwprs[ID].rfs[rfID].nn  = my_ivector(1,lwprs[ID].rfs[rfID].n_nn_allocated);
  lwprs[ID].rfs[rfID].nnw = my_vector(1,lwprs[ID].rfs[rfID].n_nn_allocated);

  /* assign what is known now */
  lwprs[ID].rfs[rfID].n_proj     = n_proj;

  if (lwprs[ID].use_reg_2nd) {
    int n_in_reg_2nd;
    int n_out_2nd;

    n_in_reg_2nd = lwprs[ID].n_in_reg_2nd;
    n_out_2nd    = lwprs[ID].n_out_2nd;
    
    lwprs[ID].rfs[rfID].B_2nd  = my_matrix(1,n_proj_2nd,1,n_out_2nd);

    lwprs[ID].rfs[rfID].s_2nd   = my_vector(1,n_proj_2nd);
    
    lwprs[ID].rfs[rfID].SXresYres_2nd = my_matrix(1,n_proj_2nd,1,n_in_reg_2nd);
    lwprs[ID].rfs[rfID].ss2_2nd       = my_vector(1,n_proj_2nd);
    lwprs[ID].rfs[rfID].SSYres_2nd    = my_matrix(1,n_proj_2nd,1,n_out_2nd);
    lwprs[ID].rfs[rfID].SSXres_2nd    = my_matrix(1,n_proj_2nd,1,n_in_reg_2nd);
    lwprs[ID].rfs[rfID].dof_2nd       = my_vector(1,n_proj_2nd);
    
    /* the matrix of projection vectors  */
    lwprs[ID].rfs[rfID].W_2nd     = my_matrix(1,n_proj_2nd,1,n_in_reg_2nd);
    lwprs[ID].rfs[rfID].Wnorm_2nd = my_vector(1,n_proj_2nd);
    lwprs[ID].rfs[rfID].U_2nd     = my_matrix(1,n_proj_2nd,1,n_in_reg_2nd);
    
    /* mean and variance info */
    lwprs[ID].rfs[rfID].mean_x_reg_2nd = my_vector(1,n_in_reg_2nd);
    lwprs[ID].rfs[rfID].var_x_reg_2nd  = my_vector(1,n_in_reg_2nd);
    lwprs[ID].rfs[rfID].mean_y_2nd     = my_vector(1,n_out_2nd);
    lwprs[ID].rfs[rfID].var_y_2nd      = my_vector(1,n_out_2nd);
    
    /* the sum of weights for every regression dimension */
    lwprs[ID].rfs[rfID].sum_weights_2nd   = my_vector(1,n_proj_2nd);
    lwprs[ID].rfs[rfID].sum_error_2nd     = my_vector(1,n_proj_2nd);
    lwprs[ID].rfs[rfID].sum_error_cvi_2nd = my_vector(1,n_proj_2nd);
    lwprs[ID].rfs[rfID].n_data_2nd        = my_vector(1,n_proj_2nd);
    lwprs[ID].rfs[rfID].lambda_2nd        = my_vector(1,n_proj_2nd);
    
    /* assign what is known now */
    lwprs[ID].rfs[rfID].n_proj_2nd     = n_proj_2nd;

  }

  ++lwprs[ID].n_rf;

  return rfID;
		
}
/*!*****************************************************************************
 *******************************************************************************
 \note  deleteRF
 \date  January, 1994
 
 \remarks 
 
 deletes a receptive field from the given LWPR and receptive field ID.
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ID     : the ID of the LWPR
 \param[in]     rfID   : the ID of the rf
 
 ******************************************************************************/
int 
deleteRF(int ID, int rfID)
     
{

  int i,j,k;
  int n_in_w,n_in_reg,n_proj,n_out;

  
  n_in_w     = lwprs[ID].n_in_w;
  n_in_reg   = lwprs[ID].n_in_reg;
  n_out      = lwprs[ID].n_out;
  n_proj     = lwprs[ID].rfs[rfID].n_proj;

  /* erase this rfID from all nearest neighbor structures */
  for (i=1; i<=lwprs[ID].n_rf; ++i) {
    deleteNNFromRF(ID,i,rfID);
  }

  /* deallocate the memory for the rf */
  my_free_matrix(lwprs[ID].rfs[rfID].B,1,n_proj,1,n_out);
  my_free_matrix(lwprs[ID].rfs[rfID].D,1,n_in_w,1,n_in_w);
  my_free_matrix(lwprs[ID].rfs[rfID].M,1,n_in_w,1,n_in_w);
  my_free_matrix(lwprs[ID].rfs[rfID].alpha,1,n_in_w,1,n_in_w);
  if (lwprs[ID].allow_meta_learning) {
    my_free_matrix(lwprs[ID].rfs[rfID].h,1,n_in_w,1,n_in_w);
    my_free_matrix(lwprs[ID].rfs[rfID].b,1,n_in_w,1,n_in_w);
  }
  
  my_free_vector(lwprs[ID].rfs[rfID].c,1,n_in_w);
  my_free_vector(lwprs[ID].rfs[rfID].s,1,n_proj);
  
  my_free_matrix(lwprs[ID].rfs[rfID].SXresYres,1,n_proj,1,n_in_reg);
  my_free_vector(lwprs[ID].rfs[rfID].ss2,1,n_proj);
  my_free_vector(lwprs[ID].rfs[rfID].dof,1,n_proj);
  my_free_matrix(lwprs[ID].rfs[rfID].SSYres,1,n_proj,1,n_out);
  my_free_matrix(lwprs[ID].rfs[rfID].SSXres,1,n_proj,1,n_in_reg);

  /* the matrix of projection vectors  */
  my_free_matrix(lwprs[ID].rfs[rfID].W,1,n_proj,1,n_in_reg);
  my_free_vector(lwprs[ID].rfs[rfID].Wnorm,1,n_proj);
  my_free_matrix(lwprs[ID].rfs[rfID].U,1,n_proj,1,n_in_reg);
  my_free_matrix(lwprs[ID].rfs[rfID].H,1,n_proj,1,n_out);
  my_free_vector(lwprs[ID].rfs[rfID].r,1,n_proj);
  my_free_matrix(lwprs[ID].rfs[rfID].T,1,n_proj,1,n_in_reg);
  my_free_matrix(lwprs[ID].rfs[rfID].A,1,n_proj,1,n_in_reg);
  my_free_vector(lwprs[ID].rfs[rfID].lambda,1,n_proj);

  /* mean and variance info */
  my_free_vector(lwprs[ID].rfs[rfID].mean_x_w,1,n_in_w);
  my_free_vector(lwprs[ID].rfs[rfID].var_x_w,1,n_in_w);
  my_free_vector(lwprs[ID].rfs[rfID].mean_x_reg,1,n_in_reg);
  my_free_vector(lwprs[ID].rfs[rfID].var_x_reg,1,n_in_reg);
  my_free_vector(lwprs[ID].rfs[rfID].mean_y,1,n_out);
  my_free_vector(lwprs[ID].rfs[rfID].var_y,1,n_out);
  my_free_vector(lwprs[ID].rfs[rfID].sum_weights,1,n_proj);
  my_free_vector(lwprs[ID].rfs[rfID].sum_error,1,n_proj);
  my_free_vector(lwprs[ID].rfs[rfID].sum_error_cvi,1,n_proj);
  my_free_vector(lwprs[ID].rfs[rfID].n_data,1,n_proj);

  /* the nearest neighbors */
  my_free_ivector(lwprs[ID].rfs[rfID].nn,1,lwprs[ID].rfs[rfID].n_nn_allocated);
  my_free_vector(lwprs[ID].rfs[rfID].nnw,1,lwprs[ID].rfs[rfID].n_nn_allocated);

  /* variable for the 2nd regression model */
  if (lwprs[ID].use_reg_2nd) {
    int n_proj_2nd;
    int n_in_reg_2nd;
    int n_out_2nd;

    n_in_reg_2nd = lwprs[ID].n_in_reg_2nd;
    n_proj_2nd    = lwprs[ID].rfs[rfID].n_proj_2nd;
    n_out_2nd    = lwprs[ID].n_out_2nd;
    
    my_free_matrix(lwprs[ID].rfs[rfID].B_2nd,1,n_proj_2nd,1,n_out_2nd);

    my_free_vector(lwprs[ID].rfs[rfID].s_2nd,1,n_proj_2nd);
    
    my_free_matrix(lwprs[ID].rfs[rfID].SXresYres_2nd,1,n_proj_2nd,1,n_in_reg_2nd);
    my_free_vector(lwprs[ID].rfs[rfID].ss2_2nd,1,n_proj_2nd);
    my_free_matrix(lwprs[ID].rfs[rfID].SSYres_2nd,1,n_proj_2nd,1,n_out_2nd);
    my_free_matrix(lwprs[ID].rfs[rfID].SSXres_2nd,1,n_proj_2nd,1,n_in_reg_2nd);
    my_free_vector(lwprs[ID].rfs[rfID].dof_2nd,1,n_proj_2nd);
    
    /* the matrix of projection vectors  */
    my_free_matrix(lwprs[ID].rfs[rfID].W_2nd,1,n_proj_2nd,1,n_in_reg_2nd);
    my_free_vector(lwprs[ID].rfs[rfID].Wnorm_2nd,1,n_proj_2nd);
    my_free_matrix(lwprs[ID].rfs[rfID].U_2nd,1,n_proj_2nd,1,n_in_reg_2nd);
    
    /* mean and variance info */
    my_free_vector(lwprs[ID].rfs[rfID].mean_x_reg_2nd,1,n_in_reg_2nd);
    my_free_vector(lwprs[ID].rfs[rfID].var_x_reg_2nd,1,n_in_reg_2nd);
    my_free_vector(lwprs[ID].rfs[rfID].mean_y_2nd,1,n_out_2nd);
    my_free_vector(lwprs[ID].rfs[rfID].var_y_2nd,1,n_out_2nd);
    
    /* the sum of weights for every regression dimension */
    my_free_vector(lwprs[ID].rfs[rfID].sum_weights_2nd,1,n_proj_2nd);
    my_free_vector(lwprs[ID].rfs[rfID].sum_error_2nd,1,n_proj_2nd);
    my_free_vector(lwprs[ID].rfs[rfID].sum_error_cvi_2nd,1,n_proj_2nd);
    my_free_vector(lwprs[ID].rfs[rfID].n_data_2nd,1,n_proj_2nd);
    my_free_vector(lwprs[ID].rfs[rfID].lambda_2nd,1,n_proj_2nd);
    
  }



  /* copy the last rf to the to be deleted one thus that the last rf in the
     row can be freed */

  if (rfID != lwprs[ID].n_rf) {

    /* swap in the last RF in the array */
    lwprs[ID].rfs[rfID] = lwprs[ID].rfs[lwprs[ID].n_rf];

    /* fix the first NN elements to reflect the change of rfID */
    lwprs[ID].rfs[rfID].nn[1] = rfID;

    /* make sure the check counter is not too high */
    if (lwprs[ID].rfs[rfID].next_nn_check >= rfID)
      lwprs[ID].rfs[rfID].next_nn_check = rfID - 1;
  }

  /* fix the new ID of the moved RF */
  for (i=1; i<=lwprs[ID].n_rf-1; ++i) {
    swapNNID(ID,i,lwprs[ID].n_rf,rfID);
  }

  --lwprs[ID].n_rf;  /* this must come last to avoid screwing up the swap */


  return TRUE;

}


/*!*****************************************************************************
 *******************************************************************************
 \note  initRF
 \date  January, 1994
 
 \remarks 
 
 Initializes a receptive field with a given ID number.
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ID         : the ID of the LWPR
 \param[in]     rfID       : the ID of the receptive field
 \param[in]     c          : the center of the receptive field
 \param[in]     M          : the decomposed distance metric for the receptive field
 
 ******************************************************************************/
static int 
initRF(int      ID, 
       int      rfID, 
       Vector   c, 
       Matrix   M)
     
{
  int      i,j,k;
  double   aux;
  int      n_proj, n_in_w, n_in_reg, n_out;
  LWPR     *s;

  s         = &lwprs[ID];
  n_in_reg  = s->n_in_reg;
  n_in_w    = s->n_in_w;
  n_out     = s->n_out;
  n_proj    = s->rfs[rfID].n_proj;

  mat_zero(s->rfs[rfID].B);

  /* note that some matrices are allocated as symmetric matrices to save 
     memory */
  vec_equal_scalar(s->init_S2,s->rfs[rfID].ss2);
  vec_zero(s->rfs[rfID].dof);
  vec_equal_scalar(TINY_NUMBER,s->rfs[rfID].sum_weights);
  vec_zero(s->rfs[rfID].sum_error);
  vec_zero(s->rfs[rfID].sum_error_cvi);
  vec_zero(s->rfs[rfID].n_data);

  mat_zero(s->rfs[rfID].SXresYres);
  mat_zero(s->rfs[rfID].SSXres);
  mat_zero(s->rfs[rfID].SSYres);

  for (i=1; i<=n_in_w; ++i) {
    for (j=i; j<=n_in_w; ++j) {
      if (!s->diag_only || i==j) {
	s->rfs[rfID].M[i][j] = uniform(M[i][j], M[i][j]*s->d_noise);
      }
    }
  }

  getLWPRDistMetric(ID,rfID,s->rfs[rfID].D);

  mat_equal_scalar(s->init_alpha,s->rfs[rfID].alpha);
  
  if (s->allow_meta_learning) {
    
    mat_zero(s->rfs[rfID].h);
    mat_equal( s->rfs[rfID].alpha,s->rfs[rfID].b);
    mat_add_scalar(s->rfs[rfID].b,TINY_NUMBER,s->rfs[rfID].b);
    mat_equal_apply_math(s->rfs[rfID].b,&log,s->rfs[rfID].b);

  }

  for (i=1; i<=n_in_w; ++i) {
    s->rfs[rfID].c[i] = c[i] + uniform(0.0,0.5774)*s->c_noise;
  }
  
  if (DEBUG_FLAG) print_vec("Center",s->rfs[rfID].c);

  s->rfs[rfID].sum_D2              = 0.0;
  s->rfs[rfID].sum_error_cv        = 0.0;
  s->rfs[rfID].trustworthy         = FALSE;
  s->rfs[rfID].n_updates           = 0;
  s->rfs[rfID].w                   = 0;
  s->rfs[rfID].next_nn_check       = rfID-1;
  vec_equal_scalar(s->init_lambda,s->rfs[rfID].lambda);

  vec_zero(s->rfs[rfID].var_x_reg);
  vec_zero(s->rfs[rfID].mean_x_reg);
  vec_zero(s->rfs[rfID].var_x_w);
  vec_zero(s->rfs[rfID].mean_x_w);
  vec_zero(s->rfs[rfID].var_y);
  vec_zero(s->rfs[rfID].mean_y);

  mat_eye(s->rfs[rfID].W);
  mat_eye(s->rfs[rfID].U);

  /* the second regression model if needed */
  if (lwprs[ID].use_reg_2nd) {
    int n_proj_2nd;
    int n_in_reg_2nd;
    int n_out_2nd;

    n_in_reg_2nd = s->n_in_reg_2nd;
    n_out_2nd    = s->n_out_2nd;
    n_proj_2nd    = s->rfs[rfID].n_proj_2nd;
  
    mat_zero(s->rfs[rfID].B_2nd);
    
    vec_equal_scalar(s->init_S2,s->rfs[rfID].ss2_2nd);
    vec_zero(s->rfs[rfID].dof_2nd);
    vec_equal_scalar(TINY_NUMBER,s->rfs[rfID].sum_weights_2nd);
    vec_zero(s->rfs[rfID].sum_error_2nd);
    vec_zero(s->rfs[rfID].sum_error_cvi_2nd);
    vec_zero(s->rfs[rfID].n_data_2nd);
    
    mat_zero(s->rfs[rfID].SXresYres_2nd);
    mat_zero(s->rfs[rfID].SSXres_2nd);
    mat_zero(s->rfs[rfID].SSYres_2nd);
    
    vec_equal_scalar(s->init_lambda,s->rfs[rfID].lambda_2nd);
    
    vec_zero(s->rfs[rfID].var_x_reg_2nd);
    vec_zero(s->rfs[rfID].mean_x_reg_2nd);
    vec_zero(s->rfs[rfID].var_y_2nd);
    vec_zero(s->rfs[rfID].mean_y_2nd);
    
    mat_eye(s->rfs[rfID].W_2nd);
    mat_eye(s->rfs[rfID].U_2nd);
    
  }
  
  /* the nearest neigbor structure */
  lwprs[ID].rfs[rfID].n_nn   = 1;
  lwprs[ID].rfs[rfID].nn[1]  = rfID;
  lwprs[ID].rfs[rfID].nnw[1] = 1.0;

  return TRUE;

}


/*!*****************************************************************************
 *******************************************************************************
 \note  addDataToLWPRCompControl
 \date  March 2003
 
 \remarks 
 
 Customized function for adding data for a model used in composite control.
 All it does is to do an input vector expansion for linear g(x) terms, as
 specified in the LWPR structure.
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ID     : the ID of the LWPR
 \param[in]     x      : the input state vector
 \param[in]     u      : the input control vector
 \param[in]     y      : the output vector
 \param[in]     flag   : FALSE if y is an output, TRUE if y is an error
 \param[in,out] rfID   : neighborhood rfID for input, max activated rfID for output
 \param[out]    y_pred : the prediction for the current training point after updating
 \param[in]     tracking_error : the tracking error per output
 \param[in]     tracking_error_gain: the gain for tracking error updates
 
 ******************************************************************************/
int 
addDataToLWPRCompControl(int ID, Vector x, Vector u, Vector y, int flag, int *rfID,
			 Vector y_pred, Vector tracking_error, double tracking_error_gain)
{
  int rc;
  int i,j,r;
  int n_in_expected;
  double aux;
  static Vector x_aug = NULL;
  LWPR   *s;
  
  if (!checkLWPRExists(ID)) 
    return FALSE;

  s = &lwprs[ID];
  
  if (s->linear_gx) 
    n_in_expected = s->n_states + 
      s->n_controls*s->n_states + s->n_controls;
  else
    n_in_expected = s->n_states + s->n_controls;
  
  if (n_in_expected != s->n_in_reg) {
    printf("addDataToLWPRCompControl: #inputs in LWPR (%d) does not match #inputs computed (%d)\n",s->n_in_reg,n_in_expected);
    return FALSE;
  }
  
  if (x_aug == NULL)
    x_aug = my_vector(1,n_in_expected);
  else if (x_aug[NR] < n_in_expected)
    vec_add_shape(&x_aug,n_in_expected-x_aug[NR]);
  
  /* if a linear g(x) is to be learned, the input vector has to be augmented */
  if (s->linear_gx) {
    r=0;
    for (i=1; i<=s->n_states; ++i)      
      x_aug[++r] = x[i];
    
    for (j=1; j<=s->n_controls; ++j)
      for (i=1; i<=s->n_states; ++i)
	x_aug[++r] = x[i]*u[j];
    
    for (j=1; j<=s->n_controls; ++j)
      x_aug[++r] = u[j];
  } else 
    vec_equal_size(x,s->n_in_reg,x_aug);

  /* the regression update */
  rc = addDataToLWPR(ID, x, x_aug, y, flag, rfID);

  /* finalize the prediction */
  aux = 0;
  for (i=1; i<=lwprs[ID].n_out; ++i) {
    y_pred[i] = lwprs[ID].sum_y_predicted[i];
    aux += y_pred[i];
  }

  if (aux == 0.0)
    rc = FALSE;

  /* the tracking error update */
  for (r=1; r<=s->n_rf; ++r) {

    /* should we update this RF: only those RFs that have a nonzero weights
       need to be considered */
    if (s->rfs[r].w < ACTIVATION_THRESHOLD) {
      continue;
    } 

    /* update in the subspace with the normalized weight */
    for (i=1; i<=s->rfs[r].n_proj; ++i) {
      aux = s->rfs[r].s[i]/(s->rfs[r].ss2[i]+1.e-10)*s->rfs[r].w/sum_activation;
      for (j=1; j<=s->n_out; ++j)
	s->rfs[r].B[i][j] -= aux * tracking_error[j] * tracking_error_gain / s->norm_out[j];
    }
    /* update the constant regression term */
    aux = 1./(s->rfs[r].sum_weights[1]+1.e-10)*s->rfs[r].w/sum_activation;
    for (j=1; j<=s->n_out; ++j)
      s->rfs[r].mean_y[j] -= aux * tracking_error[j] * tracking_error_gain / s->norm_out[j];

  }
  
  return rc;

}
/*!*****************************************************************************
 *******************************************************************************
 \note  addDataToLWPR
 \date  March 1999
 
 \remarks 
 
 Adds a data point to the model
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ID     : the ID of the LWPR
 \param[in]     x_w    : the input vector for the weights
 \param[in]     x      : the input vector for the regression
 \param[in]     y      : the output vector for the regression
 \param[in]     flag   : FALSE if y is an output, TRUE if y is an error
 \param[in,out] rfID   : neighborhood rfID for input, max activated rfID for output
 
 ******************************************************************************/
int 
addDataToLWPRPredictConf(int ID, Vector x_w, Vector x, Vector y, int flag, int *rfID, 
			 Vector y_pred, Vector conf)

{
  int i,rc;
  double aux=0;

  rc = addDataToLWPR(ID,x_w, x,y,flag,rfID);

  for (i=1; i<=lwprs[ID].n_out; ++i) {
    y_pred[i] = lwprs[ID].sum_y_predicted[i];
    conf[i]   = lwprs[ID].sum_y_conf[i];
    aux += y_pred[i];
  }

  if (aux != 0.0)
    return rc;
  else
    return FALSE;

}

int 
addDataToLWPRPredict(int ID, Vector x_w, Vector x, Vector y, int flag, int *rfID, 
		     Vector y_pred)

{
  int i,rc;
  double aux=0;

  rc = addDataToLWPR(ID,x_w, x,y,flag,rfID);

  for (i=1; i<=lwprs[ID].n_out; ++i) {
    y_pred[i] = lwprs[ID].sum_y_predicted[i];
    aux += y_pred[i];
  }

  if (aux != 0.0)
    return rc;
  else
    return FALSE;

}

int 
addDataToLWPR(int ID, Vector x_w, Vector x, Vector y, int flag, int *rfID)
     
{
  double w_misc = 1.0;
  return addWeightedDataToLWPR(ID, x_w, x, y, w_misc, flag, rfID);
}

int 
addWeightedDataToLWPR(int ID, Vector x_w, Vector x, Vector y, 
		      double w_misc, int flag, int *rfID)
     
{
  int      i,j,k,r;
  double   max_activation=0;
  double   sec_activation=0;
  int      index_max;
  int      index_sec;
  int      index_prune;
  int      index_non_prune;
  double   activation;
  int      rfIDaux;
  double   aux,aux1;
  LWPR     *s;
  char     stmp[100];
  double   dofs;

  static int count_updates = -1;

  s = &lwprs[ID];

  /**********************************************/
  /* normalize the inputs and outputs           */
  /**********************************************/

  for (i=1; i<=s->n_in_reg; ++i) {
    s->x[i] = x[i]/s->norm_in_reg[i];
  }

  for (i=1; i<=s->n_in_w; ++i) {
    s->x_w[i] = x_w[i]/s->norm_in_w[i];
  }

  for (i=1; i<=s->n_out; ++i) {
    s->y[i] = y[i]/s->norm_out[i];
  }

  /**********************************************/
  /* update the mean and variance of the inputs */
  /**********************************************/

  for (i=1; i<=s->n_in_reg; ++i) {
    s->mean_x_reg[i] = (((double)s->n_data) * s->mean_x_reg[i] + s->x[i]) /
      (((double)s->n_data) + 1.);
    s->var_x_reg[i] = (((double)s->n_data) * s->var_x_reg[i] + 
		       sqr(s->x[i]-s->mean_x_reg[i]))/(((double)s->n_data) + 1.);
  }
  for (i=1; i<=s->n_in_w; ++i) {
    s->mean_x_w[i] = (((double)s->n_data) * s->mean_x_w[i] + s->x_w[i]) /
      (((double)s->n_data) + 1.);
    s->var_x_w[i] = (((double)s->n_data) * s->var_x_w[i] + 
		     sqr(s->x_w[i]-s->mean_x_w[i]))/(((double)s->n_data) + 1.);
  }
  s->n_data += 1;

  /*****************************************************************/
  /* Run through all receptive fields and update their             */
  /* parameters appropriately                                      */
  /*****************************************************************/
     
  sum_activation  = 0;
  max_activation  = 0;
  sec_activation  = 0;
  index_max       = 0;
  index_sec       = 0;
  s->n_no_trust   = 0;
  s->mean_n_proj  = 0.0;
  s->contrib_n_rf = 0.0;
  
  for (i=1; i<=s->n_out; ++i) {
    s->sum_y_predicted[i]  =0.0;
    s->sum_y_predicted2[i] =0.0;
    s->sum_y_conf[i]       =0.0;
  }


  for (r=1; r<=s->n_rf; ++r) {

    /* zero the weight to recognize which RF was updated */
    s->rfs[i].w = 0;

    /* fudge in the nearest neighbor add */
    if (s->use_nn && *rfID > 0 && *rfID <= s->n_rf) {
      if ( (r > s->max_nn_comp && max_activation > s->w_gen) || 
	   r > s->rfs[*rfID].n_nn)
	break;
      i = s->rfs[*rfID].nn[r];
    } else {
      i = r;
    }

    if (!s->rfs[i].trustworthy)   
      ++s->n_no_trust;

    s->mean_n_proj  += (double) s->rfs[i].n_proj;

    /* get the activation weight */
    activation = s->rfs[i].w = calculateWeight(ID,i,s->x_w)*w_misc;
 
    /* should we update this RF ? */
    if (activation < ACTIVATION_THRESHOLD) {
      s->rfs[i].w = 0;
      continue;
    } 


    /* yes, we are going to update now */
    ++s->rfs[i].n_updates;

   /* update the mean and variance, return the mean subtracted input for
      regression */
    updateMeanVar(ID,i,s->x_w,s->x,s->x_mz);

    if (DEBUG_FLAG) {
      printf("w=%f\n",activation);
      print_vec("x_mz",s->x_mz);
    }

    /* create a fake training point if error-based learning is on */
    createFakeTarget(ID,i,s->x_mz,s->y,flag,s->y_mz);
    
    /* update the mean and variance of the outputs, return the mean 
       subtracted output */
    updateRegMeanVar(ID,i,s->y_mz,s->y_mz);

    if (DEBUG_FLAG) {
      print_vec("y_mz",s->y_mz);
      getchar();
    }

    /* calculate regression update */
    calculateRegression(ID,i,s->x_mz,s->y_mz);
    
    /* update adminstrative variables */
    updateAdmVars(ID,i);

    /* update the distance metric */
    updateDistanceMetric(ID,i,s->x_w,s->x_mz);

    /* accumulate the sum of predictions */
    if (s->rfs[i].trustworthy) {
      sum_activation += activation;
      for (j=1; j<=s->n_out; ++j) {
	s->sum_y_predicted[j]  += s->ypred[j]*activation;
	s->sum_y_predicted2[j] += sqr(s->ypred[j])*activation;
	dofs = s->rfs[i].sum_weights[1];
	aux  = 1;
	for (k=1; k<=s->rfs[i].n_proj; ++k) {
	  dofs -= s->rfs[i].dof[k];
	  aux  += sqr(s->rfs[i].s[k])/s->rfs[i].ss2[k]*activation;
	}
	if (dofs < TINY_NUMBER)
	  dofs = TINY_NUMBER;
	s->sum_y_conf[j] += activation*s->rfs[i].sum_error[s->rfs[i].n_proj]/dofs*aux;
      }
    } else {
      /* indicate that this RF did not contribute to predictions */
      s->rfs[i].w = 0;
    }

    /* check if another dimension has to be added */
    if (s->allow_addDim) {
      if (checkAddDim(ID,i)){
	addDimension(ID,i);
      }
    }

    /* I have to keep track of the largest and the second largest activation */
    
    if (activation > sec_activation) {
      sec_activation = activation;
      index_sec      = i;
      if (sec_activation > max_activation) {
        sec_activation = max_activation;
        index_sec      = index_max;
        max_activation = activation;
        index_max      = i;
      }
    }

 
  }

  /* finish some statistics */

  --r;
  if (r > 0) {
    s->mean_n_proj  /= (double) r;
  }
  s->contrib_n_rf = r;

  /* this is the current prediction for the newly added data point,
     AFTER the RFs have been updated */

  if (sum_activation > 0) {
    for (i=1; i<=s->n_out; ++i) {
      /* first computations without taking the output normalization into account */
      s->sum_y_predicted[i] = 
	s->sum_y_predicted[i]/sum_activation;
      aux = fabs(s->sum_y_predicted2[i]/sum_activation - 
	     sqr(s->sum_y_predicted[i]))/sum_activation;
      s->sum_y_conf[i] = sqrt(s->sum_y_conf[i]/sqr(sum_activation)+aux);

      /* now add the correction for output normalization */
      s->sum_y_predicted[i] *= s->norm_out[i];
      s->sum_y_conf[i] *= s->norm_out[i];

    }
  }

  *rfID = index_max;

  /*****************************************************************/
  /* check whether a new rf must be created */
  /*****************************************************************/

  
  if (max_activation < s->w_gen && s->n_rf < s->max_rfs && s->allow_add_rfs) {
  
    /* this is a simple speed up: if a neighboring RF is activated
       sufficiently and if it is trustworthy, I copy some of its
       parameters */

    if (max_activation > s->w_gen*.1 && index_max > 0 &&
	s->rfs[index_max].trustworthy) {
      
      rfIDaux = 
	addRF(ID,s->rfs[index_max].n_proj,s->rfs[index_max].n_proj_2nd,NN_BLOCK);
      
      if (rfIDaux != FALSE) {
	initRF(ID, rfIDaux, s->x_w, s->rfs[index_max].M);
      }
      
    } else {
      
      /* default initizialization */
      
      rfIDaux = 
	addRF(ID,s->init_n_proj,s->init_n_proj_2nd,NN_BLOCK);
      
      if (rfIDaux != FALSE) {
 	initRF(ID, rfIDaux, s->x_w, s->init_M);
      }
      
    }
    
    *rfID = rfIDaux;

  }
  
  /*****************************************************************/
  /* update the nearest neighbor relations                         */
  /*****************************************************************/

  if (index_max > 0 && lwprs[ID].use_nn) {
    checkNeighborhood(ID,index_max,
		      lwprs[ID].rfs[index_max].next_nn_check);
    if (--lwprs[ID].rfs[index_max].next_nn_check < 1)
      lwprs[ID].rfs[index_max].next_nn_check = index_max-1;
  }

  /*****************************************************************/
  /* check whether a rf must be pruned because of overlap          */
  /*****************************************************************/

  if (max_activation > s->w_prune && sec_activation > s->w_prune 
      && index_max != index_sec && index_max > 0 && index_sec > 0) {
  
    /* pruning the RF with the larger error */
    index_prune     = index_sec;
    index_non_prune = index_max;

    aux  = s->rfs[index_max].sum_error_cv;
    aux1 = s->rfs[index_sec].sum_error_cv;

    if (aux > aux1) {
      index_prune = index_max;
      index_non_prune = index_sec;
    }

    sprintf(stmp,"overlap prune due to %d",index_non_prune);
    if (s->write_rfs) writeRFAscii(ID,index_prune,TRUE,stmp);
    sprintf(stmp,"this RF caused overlap prune of %d",index_prune);
    if (s->write_rfs) writeRFAscii(ID,index_non_prune,TRUE,stmp);
    deleteRF(ID,index_prune);
    ++s->n_pruned_overlap;
    /* printf("Pruned ID#%d  RF#%d\n",ID,index_prune); */

    *rfID = index_non_prune;
  
  }

  /*****************************************************************/
  /* does any rf have an excessive error? I check this in some
     kind of arbitary interval */
  /*****************************************************************/

  if (count_updates < 0 || count_updates++ >(10*s->n_rf) || 
      count_updates > 1000) {

    count_updates = 0;

    /* get the statistics calculated */
#ifdef FIXXXXXXX
    calculateLWPRStatistics(ID);

    /* find those RF which have an excessive error */

    checkLargeError(ID);

    /* find those RF which have almost no data and have not
       been update for ages */

    checkLostRFs(ID);
#endif
  }

  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  calculateWeight
\date  Nov 3, 1992

\remarks 

       calculates the activation of a given rf for a given input vector


 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     ID    : lwpr ID
 \param[in]     rfID  : the rfID
 \param[in]     x     : the input vector for weights

 ******************************************************************************/
static double
calculateWeight(int ID, int rfID, Vector x)

{

  int    i,j,k;
  double mahal;
  double aux;


  /* get the mahalanobis distance */

  mahal = calculateMahal(ID, rfID, x);

  switch (lwprs[ID].kernel_function) {

  case GAUSSIAN:
  case VONMISES:
  
    return (myExp(-0.5*mahal,USE_EXP_LOOKUP));

  case BISQUARE:

    if (mahal > 1.0) {
      return (0.0);
    } else {
      aux = 1.- mahal;
      return (aux * aux);
    }

  }

  return FALSE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  calculateMahal
\date  Nov 3, 1992

\remarks 

       calculates the mahalanobis distance


 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     ID    : lwpr ID
 \param[in]     rfID  : the rfID
 \param[in]     x     : the input vector for weights

 ******************************************************************************/
static double
calculateMahal(int ID, int rfID, Vector x)

{

  int    i,j,k;
  RF    *rfptr;
  int    diag_only;
  double mahal;
  double aux;
  int    iaux;
  Vector delta;

  /* hand variables */

  rfptr     = &lwprs[ID].rfs[rfID];
  diag_only = lwprs[ID].diag_only;
  delta     = lwprs[ID].delta;

  /* the distance from the center */
  vec_sub(x,rfptr->c,delta);

  switch (lwprs[ID].kernel_function) {
  case VONMISES:
    for (i=1; i<=lwprs[ID].n_in_w; ++i) {
      delta[i] = sqrt(1.-cos(delta[i]));
    }
    mahal = mat_mahal(rfptr->D,delta);
    break;

  case GAUSSIAN:
  default:
    /* the mahalanobis distance */
    mahal = mat_mahal(rfptr->D,delta);
    break;
  }

  return mahal;

}

/*!*****************************************************************************
 *******************************************************************************
\note  updateMeanVar
\date  Sept, 1996

\remarks 

       update the weighted mean and variance of a RF and return
       a centered input vector for regression
            
	
       NOTE: the weight w must be assigned in the RF structure already

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     ID    : lwpr ID
 \param[in]     rfID  : the rfID
 \param[in]     x_w   : the input vector for weights
 \param[in]     x     : the input vector for regression
 \param[out]    xmz   : x-mean_x

 ******************************************************************************/
static void
updateMeanVar(int ID, int rfID, Vector x_w, Vector x, Vector xmz)

{

  int    i,j,k;
  RF    *rfptr;
  int    n_in_w;
  int    n_in_reg;
  double w, lambda;

  /* handy variables */

  rfptr     = &lwprs[ID].rfs[rfID];
  n_in_w    = lwprs[ID].n_in_w;
  n_in_reg  = lwprs[ID].n_in_reg;
  w         = rfptr->w;
  lambda    = LAMBDA(rfptr->lambda[1],w);
  
  for (i=1; i<=n_in_w; ++i) {

    rfptr->mean_x_w[i] = 
      (rfptr->sum_weights[1] * rfptr->mean_x_w[i] * lambda + x_w[i]*w) /
      (rfptr->sum_weights[1] *lambda + w);

    rfptr->var_x_w[i] = 
      (rfptr->sum_weights[1] * rfptr->var_x_w[i] * lambda  + 
		   sqr(x_w[i]-rfptr->mean_x_w[i])*w)/
      (rfptr->sum_weights[1]  * lambda + w);

  }

  for (i=1; i<=n_in_reg; ++i) {

    if (lwprs[ID].use_offsets) {
      rfptr->mean_x_reg[i] = 
	(rfptr->sum_weights[1] * rfptr->mean_x_reg[i] * lambda + x[i]*w) /
	(rfptr->sum_weights[1] *lambda + w);
    }
      
    rfptr->var_x_reg[i] = 
      (rfptr->sum_weights[1] * rfptr->var_x_reg[i] * lambda  + 
		   sqr(x[i]-rfptr->mean_x_reg[i])*w)/
      (rfptr->sum_weights[1]  * lambda + w);

    xmz[i] = (x[i] - rfptr->mean_x_reg[i]);

  }

}

/*!*****************************************************************************
 *******************************************************************************
\note  updateRegMeanVar
\date  Sept, 1996

\remarks 

       update the weighted mean and variance of a RF for the 
       dependent variables
	
       NOTE: the weight w must be assigned in the RF structure already

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     ID    : lwpr ID
 \param[in]     rfID  : the rfID
 \param[in]     y     : the output vector
 \param[out]    ymz   : y-mean_y

 ******************************************************************************/
static void
updateRegMeanVar(int ID, int rfID, Vector y, Vector ymz)

{

  int    i,j,k;
  RF    *rfptr;
  int    n_out;
  double w;
  double lambda;

  /* handy variables */

  rfptr     = &lwprs[ID].rfs[rfID];
  n_out     = lwprs[ID].n_out;
  w         = rfptr->w;
  lambda    = LAMBDA(rfptr->lambda[1],w);
  
  for (i=1; i<=n_out; ++i) {

    if (lwprs[ID].use_offsets) {
      rfptr->mean_y[i] = 
	(rfptr->sum_weights[1] * rfptr->mean_y[i] * lambda + y[i]*w) /
	(rfptr->sum_weights[1] * lambda + w);
    }

    rfptr->var_y[i] = 
      (rfptr->sum_weights[1] * rfptr->var_y[i] * lambda  + 
		   sqr(y[i]-rfptr->mean_y[i])*w)/
      (rfptr->sum_weights[1]  * lambda + w);

    ymz[i] = (y[i] - rfptr->mean_y[i]);

  }


}

/*!*****************************************************************************
 *******************************************************************************
\note  updateAdmVars
\date  April 1999

\remarks 

       updates several adminstrative variables for a receptive field.
       This function should be called last since various other update
       function require the non-updated adminstrative variables.

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     ID        : lwpr ID
 \param[in]     rfID      : the rfID

 ******************************************************************************/
static void
updateAdmVars(int ID, int rfID)

{

  int    i;
  RF    *rfptr;

  /* handy variables */
  rfptr     = &lwprs[ID].rfs[rfID];

  /* several trace values */
  for (i=1; i<=rfptr->n_proj; ++i) {

    rfptr->sum_weights[i] = LAMBDA(rfptr->lambda[i],rfptr->w)*rfptr->sum_weights[i] + rfptr->w;
    rfptr->n_data[i]      = rfptr->lambda[i] * rfptr->n_data[i] + 1;

    /* and finally update the forgetting factor lambda itself */
    if (lwprs[ID].final_lambda - rfptr->lambda[i] > TINY_NUMBER) {
      rfptr->lambda[i] = lwprs[ID].tau_lambda * rfptr->lambda[i] + 
	lwprs[ID].final_lambda*(1.-lwprs[ID].tau_lambda);
      
    }
  }


}
/*!*****************************************************************************
 *******************************************************************************
\note  myExp
\date  Nov 3, 1992

\remarks 

       calculates the value of exp from the lookup table
       NOTE: only negative values are tablulated, tailored for
       distance metric calculations


 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     value   : the argument to exp
 \param[in]     lookup  : TRUE=use lookup table, FALSE=use normal exp()

 ******************************************************************************/
static double
myExp(double value, int lookup)

{

  static double             *exp_array;
  static int                 firsttime = TRUE;
  int    i;

  if (firsttime) {
    firsttime = FALSE;
    exp_array = (double *) my_vector(0, 10000);
    for (i = 0; i <= 10000; ++i) {
      exp_array[i] = exp((double) (-i) / 1000.);
    }
  }

  if ( !lookup ) return (exp(value));

  if (fabs(value) > 10.0) {
    return 0.000001;
  } else {
    i = (int) fabs(value * 1000.0);
    return (exp_array[i]);
  }

}

/*!*****************************************************************************
 *******************************************************************************
 \note  calculateRegression
 \date  April 1999
 
 \remarks 
 
 recursively calculates the regressions in all directions
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ID       : the ID of the LWPR
 \param[in]     rfID     : the receptive field ID
 \param[in]     x_mz : the input vector (zero mean)
 \param[in]     y_mz     : the output vector (mean zero)
 
 ******************************************************************************/
static void
calculateRegression(int ID, int rfID, Vector x_mz, Vector y_mz)

{
  RF       *rfptr;
  int       n_in_reg, n_out;
  int       i, j, k;
  int       n_proj;
  double    aux;
  double    sum_val;
  double   *xres;
  double   *yres;
  double    s;
  double   *ypred;
  double    w;
  double    y_partial;
  double  **error_cv;
  double  **error;
  double   *xproj;
  Vector    cr;

  rfptr    = &lwprs[ID].rfs[rfID];
  n_in_reg =  lwprs[ID].n_in_reg;
  n_out    =  lwprs[ID].n_out;
  n_proj   =  rfptr->n_proj;
  xres     =  lwprs[ID].xres;
  yres     =  lwprs[ID].yres;
  w        =  rfptr->w;
  ypred    =  lwprs[ID].ypred;
  error_cv =  lwprs[ID].error_cv;
  error    =  lwprs[ID].error;
  xproj    =  rfptr->s;
  cr       =  lwprs[ID].cr;

  /* prepare residual variables */

  for (i=1; i<=n_in_reg; ++i)
#if SPHERING
    xres[i] = x_mz[i]/(sqrt(rfptr->var_x_reg[i])+TINY_NUMBER);
#else
    xres[i] = x_mz[i];
#endif
  for (i=1; i<=n_out; ++i) {
    yres[i]  = y_mz[i];
    ypred[i] = rfptr->mean_y[i];
  }

  /* loop over all regression outputs, find the projections, and update
     the regression parameters */
  for (i=1; i<=n_proj; ++i) {

    /* project the input onto the projection vector *before* updating the
       projections: this is like "synchronous" update for PLS while 
       classical PLS is asynchonous -- but synchronous updates keep
       cross validation for PLS cleaner */
    s=0.0;
    for (j=1; j<=n_in_reg; ++j)
      s += rfptr->W[i][j]*xres[j];

    /* update the projection direction; NOTE: this is not the proper
       version for multivariate outputs, but rather a quick and dirty
       version. Multivariate outputs are not recommended in any of the 
       LWPR methods, and in particular not in PLS */

    sum_val = 0.0;
    for (j=1; j<=n_in_reg; ++j) {
      cr[j] = 0.0;
      for (k=1; k<=n_out; ++k) {
	cr[j] += xres[j]*yres[k];
      }
      /* NOTE: for nonstationary learning problems, it turns out to be
	 useful if the projections change a bit more slowly than the
	 regression -- thus the weird formula below for lambda */
      rfptr->SXresYres[i][j] = 
	LAMBDA((1.-(1.-rfptr->lambda[i])/10.),w) * rfptr->SXresYres[i][j] + w * cr[j];
      sum_val += sqr(rfptr->SXresYres[i][j]);
    }
    sum_val = sqrt(sum_val)+TINY_NUMBER;
    rfptr->Wnorm[i] = sum_val;

    /* normalize this direction */
    for (j=1; j<=n_in_reg; ++j)
      rfptr->W[i][j] = rfptr->SXresYres[i][j]/sum_val;

    /* update sufficient statistics for regression based on this projection */
    rfptr->ss2[i] = LAMBDA(rfptr->lambda[i],w) * rfptr->ss2[i] + sqr(s) * w;
    for (j=1; j<=n_out; ++j) {
      rfptr->SSYres[i][j] = LAMBDA(rfptr->lambda[i],w) * rfptr->SSYres[i][j] +  
	w * yres[j] * s;
    }

    /* update the regression parameters for this dimension and predict
       the outputs */
    aux = 0;
    for (j=1; j<=n_out; ++j) {

      /* the cross validation error at this regression stage */
      y_partial      = rfptr->B[i][j]*s;
      error_cv[i][j] = yres[j] - y_partial;
      ypred[j]      += y_partial;
      yres[j]       -= y_partial;
      aux           += sqr(error_cv[i][j]);

      /* update the local model */
      rfptr->B[i][j] = rfptr->SSYres[i][j]/rfptr->ss2[i];


    }
    aux /= (double)n_out;  /* the mean squared cv error accross all outputs */

    /* update the sum_error term: this is a sum of CV terms */
    rfptr->sum_error_cvi[i]   = rfptr->sum_error_cvi[i] * LAMBDA(rfptr->lambda[i],w) + w * aux;

    /* the total CV error of the RF needs to be accumulated separately as
       individual projections have different life times */
    if (i==n_proj)
      rfptr->sum_error_cv   = rfptr->sum_error_cv * LAMBDA(rfptr->lambda[1],w) + w * aux;

    /* the regression parameters on the inputs variables and
       prepare the variables for the next iteration */
    for (j=1; j<=n_in_reg; ++j) {
      rfptr->SSXres[i][j] = LAMBDA(rfptr->lambda[i],w) * rfptr->SSXres[i][j] +  
	w * xres[j] * s;
      xres[j] -= rfptr->U[i][j]*s;
      rfptr->U[i][j] = rfptr->SSXres[i][j]/rfptr->ss2[i];
    }
    
  }

  /* finally calulate the predicted regression output *after* updating */
  for (i=1; i<=n_out; ++i) {
    yres[i]  = y_mz[i];
    ypred[i] = rfptr->mean_y[i];
  }
  for (i=1; i<=n_in_reg; ++i)
#if SPHERING
    xres[i] = x_mz[i]/(sqrt(rfptr->var_x_reg[i])+TINY_NUMBER);
#else
    xres[i] = x_mz[i];
#endif

  for (i=1; i<=n_proj; ++i) {

    s=0.0;
    for (j=1; j<=n_in_reg; ++j)
      s += rfptr->W[i][j]*xres[j];

    xproj[i] = s;

    /* update the local degree of freedoms */
    rfptr->dof[i] = LAMBDA(rfptr->lambda[i],w) * rfptr->dof[i] + 
      sqr(w)*sqr(s)/rfptr->ss2[i];

    /* compute the predicted output */
    aux = 0;
    for (j=1; j<=n_out; ++j) {
      /* the non-CV error at this regression stage */
      y_partial   = rfptr->B[i][j]*s;
      error[i][j] = yres[j] - y_partial;
      ypred[j]   += y_partial;
      yres[j]    -= y_partial;
      aux        += sqr(error[i][j]);
    }
    aux /= (double)n_out;  /* the mean squared error accross all outputs */

    /* update the sum_error term: this is a sum of NON-CV terms */
    rfptr->sum_error[i]   = rfptr->sum_error[i] * LAMBDA(rfptr->lambda[i],w) + w * aux;

    /* update the residual input for the next iteration */
    for (j=1; j<=n_in_reg; ++j) {
      xres[j] -= rfptr->U[i][j]*s;
    }

  }

  if (rfptr->n_data[1] > 2*n_in_reg)
    rfptr->trustworthy = TRUE; 


}

/*!*****************************************************************************
 *******************************************************************************
 \note  predictLWPROutput
 \date  January, 1994
 
 \remarks 
 
 generates a prediction from a given input
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ID     : the ID of the LWPR
 \param[in]     x_w    : the input vector for weights
 \param[in]     x      : the input vector for regression
 \param[in]     cutoff : a number between [0,1] which indicates at which threshold
             a receptive field is not used for averaging anymore
 \param[in]     blend  : blend predictions or use max only 
 \param[out]    y      : the output vector
 \param[in,out] rfID   : the rf for the neighborhood for input, returns the max activated rf 

 returns the max activation contributing to this output
 
 ******************************************************************************/
double
predictLWPROutputConf(int ID, double *x_w, double *x, double cutoff, 
		  int blend, double *y, int *rfID, double *conf)
{
  int i;
  double rc;

  rc = predictLWPROutput(ID, x_w, x, cutoff, blend, y, rfID);
  for (i=1; i<=lwprs[ID].n_out; ++i)
    conf[i] = lwprs[ID].y_conf[i];
  
  return rc;

}

double
predictLWPROutput(int ID, double *x_w, double *x, double cutoff, 
		  int blend, double *y, int *rfID)
     
{
  
  int     i,j,k,n,r;
  double  max_activation = 0;
  double  sum_w = 0;
  double  w=0.;
  int     n_in_w;
  int     n_in_reg;
  int     n_proj;
  int     n_rf;
  int     n_out;
  int     index_max= -999;
  RF     *rfptr;
  int     count_rf=0;
  LWPR    *s;
  double  x_proj;
  Vector  xres;
  double  aux;
  double  dofs;

  if (!checkLWPRExists(ID)) 
    return FALSE;

  s         = &lwprs[ID];
  n_in_w    = s->n_in_w;
  n_in_reg  = s->n_in_reg;
  n_out     = s->n_out;
  n_rf      = s->n_rf;
  xres      = s->x_tempp;

  /* normalize the query point */
  for (j=1; j<=n_in_reg; ++j) {
    s->xp[j] = x[j]/s->norm_in_reg[j];
  }

  for (j=1; j<=n_in_w; ++j) {
    s->xp_w[j] = x_w[j]/s->norm_in_w[j];
  }

  /* reset the y[j] here since we sum over them */
  for (j=1; j<=n_out; ++j) {
    y[j] = 0.0;
    s->y_predicted[j]  = 0.0;
    s->y_predicted2[j] = 0.0;
    s->y_conf[j] = 0.0;
  }

  /* if there are no receptive fields */
  if (n_rf == 0)
    return max_activation;
  
  /* scan the receptive fields */

  for (r=1; r<=n_rf; ++r) {

    /* fudge in the nearest neighbor lookup */
    if (s->use_nn && *rfID > 0 && *rfID <= n_rf) {
      if ( (r > s->max_nn_comp && max_activation > cutoff) || 
	   r > s->rfs[*rfID].n_nn)
	break;
      i = s->rfs[*rfID].nn[r];
    } else {
      i = r;
    }
  
    rfptr     = &s->rfs[i];
    n_proj    = rfptr->n_proj;

    if (!rfptr->trustworthy) continue;
    
    /* the mean subtracted input  */
    for (j=1; j<=n_in_reg; ++j) {
      s->x_mzp[j] = s->xp[j] - rfptr->mean_x_reg[j];
    }
    
    /* generate the weight for this RF */
    rfptr->ww = w = calculateWeightp(ID,i,s->xp_w);

    if (w > max_activation) {
      max_activation = w;
      index_max = i;
    }

    if (w > cutoff && blend) {

      for (j=1; j<=n_in_reg; ++j) {
#if SPHERING
	xres[j] = s->x_mzp[j]/(sqrt(rfptr->var_x_reg[j])+TINY_NUMBER);
#else
	xres[j] = s->x_mzp[j];
#endif
      }

      /* add the constant part */
      for (j=1; j<=n_out; ++j) {
	aux = rfptr->mean_y[j];
	s->y_predicted[j] = aux; 
	y[j] += aux * w;
      }


      for (k=1; k<=n_proj; ++k) {

	/* project the input onto the current projection */
	x_proj = vec_mult_inner_size(xres,rfptr->W[k],n_in_reg);
	
	/* do single variate prediction */
	for (j=1; j<=n_out; ++j) {
	  aux   = x_proj * rfptr->B[k][j];
	  y[j] += aux * w;
	  s->y_predicted[j] += aux;
	}
	
	/* adjust the input data */
	for (j=1; j<=n_in_reg; ++j) {
	  xres[j] -= x_proj * rfptr->U[k][j];
	}

      }
      
      /* prepare summary statistics and confidence intervals */
      sum_w += w;
      for (j=1; j<=n_out; ++j) {
	s->y_predicted2[j] += sqr(s->y_predicted[j]) * w;
	dofs = rfptr->sum_weights[1];
	aux  = 1;
	for (k=1; k<=n_proj; ++k) {
	  dofs -= rfptr->dof[k];
	  aux  += sqr(rfptr->s[k])/rfptr->ss2[k] * w;
	}
	if (dofs < TINY_NUMBER)
	  dofs = TINY_NUMBER;
	s->y_conf[j] += w * rfptr->sum_error[n_proj]/dofs*aux;
      }
      ++count_rf;

    }

  }


  /* finish up */

  if (blend) {

    if (sum_w < TINY_NUMBER) 
      sum_w = TINY_NUMBER;

    for (i=1; i<=n_out; ++i) {
      y[i] = y[i]/sum_w;
      aux = fabs(s->y_predicted2[i]/sum_w - sqr(y[i]))/sum_w;
      s->y_conf[i] = sqrt(s->y_conf[i]/sqr(sum_w)+aux);

      /* add output normalization */
      y[i] *= s->norm_out[i];
      s->y_conf[i] *= s->norm_out[i];
    }
    
  } else {
    
    /* predict y from the max activated rf */
    rfptr = &s->rfs[index_max];
    n_proj = rfptr->n_proj;

    for (j=1; j<=n_in_reg; ++j) {
      s->x_mzp[j] = s->xp[j] - rfptr->mean_x_reg[j];
#if SPHERING
      xres[j] = s->x_mzp[j]/(sqrt(rfptr->var_x_reg[j])+TINY_NUMBER);
#else
      xres[j] = s->x_mzp[j];
#endif
    }

    for (k=1; k<=n_proj; ++k) {
      
      /* project the input onto the current projection */
      x_proj = vec_mult_inner_size(xres,rfptr->W[k],n_in_reg);
      
      /* do single variate prediction */
      for (j=1; j<=n_out; ++j) {
	y[j] += x_proj * rfptr->B[k][j];
      }
      
      /* adjust the input data */
      for (j=1; j<=n_in_reg; ++j) {
	xres[j] -= x_proj * rfptr->U[k][j];
      }
    
    }
    
    /* add the constant part and compute confidence interval */
    for (j=1; j<=n_out; ++j) {
      y[j] = (y[j] + rfptr->mean_y[j])*s->norm_out[j]; 

      dofs = rfptr->sum_weights[1];
      aux  = 1;
      for (k=1; k<=n_proj; ++k) {
	dofs -= rfptr->dof[k];
	aux  += sqr(rfptr->s[k])/rfptr->ss2[k] * w;
      }
      if (dofs < TINY_NUMBER) 
	dofs = TINY_NUMBER;
      s->y_conf[j] = rfptr->sum_error[n_proj]/dofs*aux/w;
    }

    if (!s->rfs[index_max].trustworthy) max_activation = 0;
    
  }

  *rfID = index_max;

  return max_activation;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  predictLWPROutputPart
 \date  January, 1994
 
 \remarks 
 
 generates a prediction from a given input, but does not generate the total
 output. Instead, the output for each output dimensions is partitioned such
 into y1,  y2, y3 ..... Paritioning is decided based on an index
 matrix that creates paritions according to the information in each of its
 rows, e.g.:

 row1: 1 1 1 0 0 0 1 means that there are coefficients 1-3 and 7 contribute
                     to the results.

 Note that each row is d+1 elements long, indices starting at 1, and the d+1-th
 element denoting the constant part of the regression

 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ID     : the ID of the LWPR
 \param[in]     x_w    : the input vector for weights
 \param[in]     x      : the input vector for regression
 \param[in]     cutoff : a number between [0,1] which indicates at which threshold
             a receptive field is not used for averaging anymore
 \param[in]     blend  : blend predictions or use max only 
 \param[in]     n_part : number of partitions (=number of rows in partition matrix)
 \param[in]     partmat: partition matrix
 \param[out]    y      : the output **matrix** (one column for every partition)
 \param[in,out] rfID   : the rf for the neighborhood for input, returns the max activated rf 

 returns the max activation contributing to this output
 
 ******************************************************************************/
double
predictLWPROutputPart(int ID, double *x_w, double *x, double cutoff, int blend, int n_part,
		      int **partmat, double **y, int *rfID)
     
{
  int     i,j,k,n,r,m;
  double  max_activation = 0;
  double  sum_w = 0;
  double  w=0.;
  int     n_in_w;
  int     n_in_reg;
  int     n_proj;
  int     n_rf;
  int     n_out;
  int     index_max= -999;
  RF     *rfptr=NULL;
  int     count_rf=0;
  LWPR    *s;
  double  x_proj;
  Vector  xres;
  Vector  y_mean_p;

  if (!checkLWPRExists(ID)) 
    return FALSE;

  s         = &lwprs[ID];
  n_in_w    = s->n_in_w;
  n_in_reg  = s->n_in_reg;
  n_out     = s->n_out;
  n_rf      = s->n_rf;
  xres      = s->x_tempp;
  y_mean_p  = s->y_mean_predicted;

  /* normalize the query point */
  for (j=1; j<=n_in_reg; ++j) {
    s->xp[j] = x[j]/s->norm_in_reg[j];
  }

  for (j=1; j<=n_in_w; ++j) {
    s->xp_w[j] = x_w[j]/s->norm_in_w[j];
  }

  /* reset the y[j] here since we sum over them */
  for (j=1; j<=n_out; ++j) {
    y_mean_p[j] = 0;
    for (m=1; m<=n_part; ++m) 
      y[j][m] = 0.0;
  }

  /* if there are no receptive fields */
  if (n_rf == 0)
    return max_activation;
  
  /* scan the receptive fields */
  for (r=1; r<=n_rf; ++r) {

    /* fudge in the nearest neighbor lookup */
    if (s->use_nn && *rfID > 0 && *rfID <= n_rf) {
      if ( (r > s->max_nn_comp && max_activation > cutoff) || 
	   r > s->rfs[*rfID].n_nn)
	break;
      i = s->rfs[*rfID].nn[r];
    } else {
      i = r;
    }
  
    rfptr     = &s->rfs[i];
    n_proj    = rfptr->n_proj;

    if (!rfptr->trustworthy) 
      continue;
    
    /* the mean subtracted input  */
    for (j=1; j<=n_in_reg; ++j) {
      /* NOTE: in order to obtain the true slope component associated with
	 each input, the rfptr->mean_x MUST NOT be subtracted from the
	 input. However, we need to make a correction on the constant 
	 part of the regression, by subtracting the prediction for the
	 mean -- this is handled in an extra loop below */
      s->x_mzp[j] = s->xp[j];
    }
    
    
    /* generate the weight for this RF */
    rfptr->ww = w = calculateWeightp(ID,i,s->xp_w);

    if (w > max_activation) {
      max_activation = w;
      index_max = i;
    }

    if (w > cutoff && blend) {

      /* add the constant part */
      for (j=1; j<=n_out; ++j) {
	for (m=1; m<=n_part; ++m) {
	  if (partmat[m][n_in_reg+1] == 1)
	    y[j][m] += rfptr->mean_y[j] * w; 
	}
      }


      for (m=1; m<=n_part+1; ++m) {

	for (j=1; j<=n_in_reg; ++j) {
#if SPHERING
	  if (m==n_part+1)
	    xres[j] = rfptr->mean_x_reg[j]/(sqrt(rfptr->var_x_reg[j])+TINY_NUMBER);
	  else
	    xres[j] = s->x_mzp[j]/(sqrt(rfptr->var_x_reg[j])+TINY_NUMBER)*partmat[m][j];
#else
	  if (m==n_part+1)
	    xres[j] = rfptr->mean_x_reg[j];
	  else
	    xres[j] = s->x_mzp[j]*partmat[m][j];
#endif
	}
	
	for (k=1; k<=n_proj; ++k) {
	  
	  /* project the input onto the current projection */
	  x_proj = vec_mult_inner_size(xres,rfptr->W[k],n_in_reg);
	  
	  /* do single variate prediction */
	  for (j=1; j<=n_out; ++j) {
	    if (m==n_part+1)
	      y_mean_p[j] += x_proj * rfptr->B[k][j] * w;
	    else
	      y[j][m] += x_proj * rfptr->B[k][j] * w;
	  }
	  
	  /* adjust the input data */
	  for (j=1; j<=n_in_reg; ++j) {
	    xres[j] -= x_proj * rfptr->U[k][j];
	  }
	  
	}

      } /* end  for (m=1; m<=n_part+1; ++m) */
      
      /* fix the constant part according to mean of RF*/
      for (j=1; j<=n_out; ++j) {
	for (m=1; m<=n_part; ++m) {
	  if (partmat[m][n_in_reg+1] == 1)
	    y[j][m] -= y_mean_p[j];
	}
	y_mean_p[j] = 0;
      }

      sum_w += w;
      ++count_rf;

    }

  }


  /* finish up */

  if (blend) {

    if (sum_w < TINY_NUMBER) 
      sum_w = TINY_NUMBER;


    for (i=1; i<=n_out; ++i) 
      for (m=1; m<=n_part; ++m)
	y[i][m] = y[i][m]/sum_w*s->norm_out[i];
    
  } else {
    
    /* predict y from the max activated rf */
    rfptr = &s->rfs[index_max];
    n_proj = rfptr->n_proj;
    for (j=1; j<=n_in_reg; ++j) {
      /* again, do not subtract the mean */
      s->x_mzp[j] = s->xp[j];
    }

    for (m=1; m<=n_part+1; ++m) {
      for (j=1; j<=n_in_reg; ++j) {
#if SPHERING
	if (m==n_part+1)
	  xres[j] = rfptr->mean_x_reg[j]/(sqrt(rfptr->var_x_reg[j])+TINY_NUMBER);
	else
	  xres[j] = s->x_mzp[j]/(sqrt(rfptr->var_x_reg[j])+TINY_NUMBER)*partmat[m][j];
#else
	if (m==n_part+1)
	  xres[j] = rfptr->mean_x_reg[j];
	else
	  xres[j] = s->x_mzp[j]*partmat[m][j];
#endif
      }
      
      for (k=1; k<=n_proj; ++k) {
	
	/* project the input onto the current projection */
	x_proj = vec_mult_inner_size(xres,rfptr->W[k],n_in_reg);
	
	/* do single variate prediction */
	for (j=1; j<=n_out; ++j) {
	  y[j][m] += x_proj * rfptr->B[k][j];
	  if (m==n_part+1)
	    y_mean_p[j] += x_proj * rfptr->B[k][j];
	  else
	    y[j][m] += x_proj * rfptr->B[k][j];
	}
	
	/* adjust the input data */
	for (j=1; j<=n_in_reg; ++j) {
	  xres[j] -= x_proj * rfptr->U[k][j];
	}
	
      }
    }
    
    /* add the constant part */
    for (j=1; j<=n_out; ++j) {
      for (m=1; m<=n_part; ++m) {
	if (partmat[m][n_in_reg+1] == 1)
	  y[j][m] = (y[j][m] + rfptr->mean_y[j] - y_mean_p[j])*s->norm_out[j]; 
      }
    }

    if (!s->rfs[index_max].trustworthy) max_activation = 0;
    
  }

  *rfID = index_max;

  return max_activation;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  predictLWPROutputPartCompControl
 \date  March 2003
 
 \remarks 
 
 A special wrapper for using predictLWPROutputPart for composite control 
 that splits the input vector into x and u components. The partition matrix,
 however, needs to be provided correctly for the entire (potentially
 augmented) input vector consisting of [x x*u' u 1].
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ID     : the ID of the LWPR
 \param[in]     x      : the input state vector
 \param[in]     u      : the input controlvector
 \param[in]     cutoff : a number between [0,1] which indicates at which threshold
             a receptive field is not used for averaging anymore
 \param[in]     blend  : blend predictions or use max only 
 \param[in]     n_part : number of partitions (=number of rows in partition matrix)
 \param[in]     partmat: partition matrix
 \param[out]    y      : the output **matrix** (one column for every partition)
 \param[in,out] rfID   : the rf for the neighborhood for input, returns the max activated rf 

 returns the max activation contributing to this output
 
 ******************************************************************************/
double
predictLWPROutputPartCompControl(int ID, double *x, double *u, double cutoff, 
				 int blend, int n_part, int **partmat, double **y, 
				 int *rfID)
{
  int i,j,r;
  int n_in_expected;
  static Vector x_aug = NULL;
  
  if (!checkLWPRExists(ID)) 
    return FALSE;
  
  if (lwprs[ID].linear_gx) 
    n_in_expected = lwprs[ID].n_states + 
      lwprs[ID].n_controls*lwprs[ID].n_states + lwprs[ID].n_controls;
  else
    n_in_expected = lwprs[ID].n_states + lwprs[ID].n_controls;
  
  if (n_in_expected != lwprs[ID].n_in_reg) {
    printf("predictLWPROutputCompControl: #inputs in LWPR (%d) does not match #inputs computed (%d)\n",lwprs[ID].n_in_reg,n_in_expected);
    return FALSE;
  }
  
  if (x_aug == NULL)
    x_aug = my_vector(1,n_in_expected);
  else if (x_aug[NR] < n_in_expected)
    vec_add_shape(&x_aug,n_in_expected-x_aug[NR]);
  
  /* if a linear g(x) is to be learned, the input vector has to be augmented */
  if (lwprs[ID].linear_gx) {
    r=0;
    for (i=1; i<=lwprs[ID].n_states; ++i)      
      x_aug[++r] = x[i];
    
    for (j=1; j<=lwprs[ID].n_controls; ++j)
      for (i=1; i<=lwprs[ID].n_states; ++i)
	x_aug[++r] = x[i]*u[j];
    
    for (j=1; j<=lwprs[ID].n_controls; ++j)
      x_aug[++r] = u[j];
  } else 
    vec_equal_size(x,lwprs[ID].n_in_reg,x_aug);
  
  return predictLWPROutputPart(ID, x, x_aug, cutoff, blend, n_part,partmat,y,rfID);
}

/*!*****************************************************************************
 *******************************************************************************
 \note  predictLWPROutputRF
 \date  January, 1994
 
 \remarks 
 
 generates a prediction from a given input for a particular RF
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ID     : the ID of the LWPR
 \param[in]     x_w    : the input vector
 \param[in]     x      : the input vector
 \param[in]     rfID   : the targeted
 \param[out]    y      : the output vector

 returns the activation contributing to this output
 
 ******************************************************************************/
double
predictLWPROutputRF(int ID, double *x_w, double *x, int rfID, double *y)
     
{

  int     i,j,k,n;
  double  max_activation = 0;
  double  sum_w = 0;
  double  w=0.;
  int     n_in_w;
  int     n_in_reg;
  int     n_proj;
  int     n_rf;
  int     n_out;
  int     index_max= -999;
  RF     *rfptr;
  int     count_rf=0;
  LWPR    *s;
  double  x_proj;

  if (!checkLWPRExists(ID)) 
    return FALSE;

  s         = &lwprs[ID];
  n_in_w    = s->n_in_w;
  n_in_reg  = s->n_in_reg;
  n_out     = s->n_out;
  n_rf      = s->n_rf;

  rfptr  = &s->rfs[rfID];
  n_proj = rfptr->n_proj;
  
  /* normalize the query point */
  for (j=1; j<=n_in_reg; ++j) {
    s->xp[j] = x[j]/s->norm_in_reg[j];
  }

  for (j=1; j<=n_in_w; ++j) {
    s->xp_w[j] = x_w[j]/s->norm_in_w[j];
  }

  /* the mean subtracted input  */
  for (j=1; j<=n_in_reg; ++j) {
    s->x_mzp[j] = s->xp[j] - rfptr->mean_x_reg[j];
  }
    
  w = calculateWeightp(ID,rfID,s->xp_w);

  /* reset the y[j] here since we sum over them */

  /* add the constant part */
  for (j=1; j<=n_out; ++j) {
    y[j] = rfptr->mean_y[j]; 
  }
  
  for (j=1; j<=n_in_reg; ++j) {
#if SPHERING
    s->xp[j] = s->x_mzp[j]/(sqrt(rfptr->var_x_reg[j])+TINY_NUMBER);
#else
    s->xp[j] = s->x_mzp[j];
#endif
  }

  for (k=1; k<=n_proj; ++k) {
    
    /* project the input onto the current projection */
    x_proj = vec_mult_inner_size(s->xp,rfptr->W[k],n_in_reg);
    
    /* do single variate prediction */
    for (j=1; j<=n_out; ++j) {
      y[j] += x_proj * rfptr->B[k][j];
    }
    
    /* adjust the input data */
    for (j=1; j<=n_in_reg; ++j) {
      s->xp[j] -= x_proj * rfptr->U[k][j];
    }
    
  }
    
  /* adjust the normalization */
  for (j=1; j<=n_out; ++j) {
    y[j] *= s->norm_out[j];
  }

  if (!s->rfs[index_max].trustworthy) 
    max_activation = 0;
  else
    max_activation = w;

  return max_activation;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  createFakeTarget
 \date  June 1999
 
 \remarks 
 
 creates a fake target form an error based learning input
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ID     : the ID of the LWPR
 \param[in]     rfID   : the receptive field ID
 \param[in]     xmz    : mean zero input
 \param[in]     err    : error
 \param[in]     flag   : TRUE if this is an error
 \param[out]    yfake  : faked output

 this function assumes that various quantities have been pre-computed
 
 ******************************************************************************/
static void
createFakeTarget(int ID,int rfID, Vector xmz, Vector err,int flag, 
		 Vector yfake)
     
{

  int     j,k,n;
  int     n_in_reg;
  int     n_proj;
  int     n_out;
  RF     *rfptr;
  LWPR    *s;
  double  x_proj;
  double  w;

  s         = &lwprs[ID];
  n_in_reg  = s->n_in_reg;
  n_out     = s->n_out;

  rfptr  = &s->rfs[rfID];
  n_proj = rfptr->n_proj;
  
  w = rfptr->w;

  /* the constant part */

  if (!flag) {
    for (j=1; j<=n_out; ++j) 
      yfake[j] = err[j];
    return;
  }

  for (j=1; j<=n_out; ++j) 
    yfake[j] = rfptr->mean_y[j]+err[j];
  
  for (j=1; j<=n_in_reg; ++j) {
#if SPHERING
    s->x_temp[j] = xmz[j]/(sqrt(rfptr->var_x_reg[j])+TINY_NUMBER);
#else
    s->x_temp[j] = xmz[j];
#endif
  }

  for (k=1; k<=n_proj; ++k) {
    
    /* project the input onto the current projection */
    x_proj = vec_mult_inner_size(s->x_temp,rfptr->W[k],n_in_reg);
    
    /* do single variate prediction */
    for (j=1; j<=n_out; ++j) {
      yfake[j] += x_proj * rfptr->B[k][j];
    }
    
    /* adjust the input data */
    for (j=1; j<=n_in_reg; ++j) {
      s->x_temp[j] -= x_proj * rfptr->U[k][j];
    }
    
  }

  return;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  printLWPRStatistics
 \date  June, 1995
 
 \remarks 
 
 prints various information about the LWPR
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ID     : the ID of the LWPR
 
 ******************************************************************************/
void
printLWPRStatistics(int ID) 

{

  int   n_in_reg;
  int   n_in_w;
  int   n_out;
  int   diag_only;
  int   n_rf;
  LWPR  *s;
  int   i,j,k;
  char  name[100];
  int   num;
  FILE *out;

  /* some handy variables */
  
  n_in_reg  = lwprs[ID].n_in_reg;
  n_in_w    = lwprs[ID].n_in_w;
  n_out     = lwprs[ID].n_out;
  diag_only = lwprs[ID].diag_only;
  n_rf      = lwprs[ID].n_rf;
  s         = &lwprs[ID];
  out       = stdout;
  
  fprintf(out,"Data of LWPR >%s<:\n\n",s->lwpr_name);

  fprintf(out,"       n_in_w            = %d\n",lwprs[ID].n_in_w);
  fprintf(out,"       n_in_reg          = %d\n",lwprs[ID].n_in_reg);
  fprintf(out,"       n_out             = %d\n",lwprs[ID].n_out);
  fprintf(out,"       init_n_proj       = %d\n",s->init_n_proj);
  fprintf(out,"       diag_only         = %d\n",s->diag_only);
  fprintf(out,"       #n_data           = %ld\n",lwprs[ID].n_data);
  fprintf(out,"       #rfs              = %ld\n",lwprs[ID].n_rf);
  fprintf(out,"       max_rfs           = %ld\n",lwprs[ID].max_rfs);
  fprintf(out,"       use_nn            = %d\n",lwprs[ID].use_nn);
  fprintf(out,"       max_nn_comp       = %d\n",lwprs[ID].max_nn_comp);
  fprintf(out,"       #no_trust         = %ld\n",lwprs[ID].n_no_trust);
  fprintf(out,"       #pruned error     = %ld\n",lwprs[ID].n_pruned_error);
  fprintf(out,"       #pruned overlap   = %ld\n",lwprs[ID].n_pruned_overlap);
  fprintf(out,"       w_gen             = %f\n",s->w_gen);
  fprintf(out,"       w_prune           = %f\n",s->w_prune);
  fprintf(out,"       factor_prune      = %f\n",s->factor_prune);
  fprintf(out,"       final_lambda      = %g\n",s->final_lambda);
  fprintf(out,"       init_lambda       = %g\n",s->init_lambda);
  fprintf(out,"       tau_lambda        = %g\n",s->tau_lambda);
  fprintf(out,"       penalty           = %g\n",s->penalty);
  fprintf(out,"       init_S2           = %g\n",s->init_S2);
  fprintf(out,"       meta_learn_rate   = %f\n",s->meta_learning_rate);
  fprintf(out,"       d_noise           = %f\n",lwprs[ID].d_noise);
  fprintf(out,"       c_noise           = %f\n",lwprs[ID].c_noise);
  fprintf(out,"       mean_error        = %f\n",lwprs[ID].mean_error);
  fprintf(out,"       std_error         = %f\n",lwprs[ID].std_error);
  fprintf(out,"       mean_n_proj       = %f\n",lwprs[ID].mean_n_proj);
  fprintf(out,"       allow meta learn. = %d\n",lwprs[ID].allow_meta_learning);
  fprintf(out,"       allow D update    = %d\n",lwprs[ID].allow_d_update);
  fprintf(out,"       allow add dim     = %d\n",lwprs[ID].allow_addDim);
  fprintf(out,"       allow add rfs     = %d\n",lwprs[ID].allow_add_rfs);
  fprintf(out,"       add threshold     = %f\n",lwprs[ID].add_threshold);
  fprintf(out,"       composite control = %d\n",lwprs[ID].composite_control);
  fprintf(out,"       n_states          = %d\n",lwprs[ID].n_states);
  fprintf(out,"       n_controls        = %d\n",lwprs[ID].n_controls);
  fprintf(out,"       linear g(x)       = %d\n",lwprs[ID].linear_gx);
  fprintf(out,"       kernel_function   = %d\n",lwprs[ID].kernel_function);

  if (lwprs[ID].use_reg_2nd) {
    fprintf(out,"       n_in_reg_2nd      = %d\n",lwprs[ID].n_in_reg_2nd);
    fprintf(out,"       n_out_2nd         = %d\n",lwprs[ID].n_out_2nd);
    fprintf(out,"       #n_data_2nd       = %ld\n",lwprs[ID].n_data_2nd);
    fprintf(out,"       mean_n_proj_2nd   = %f\n",lwprs[ID].mean_n_proj_2nd);
    fprintf(out,"       init_n_proj_2nd   = %d\n",s->init_n_proj_2nd);
  }

  fprintf(out,"\n");

  fprint_vec(out,"Normalization Factors norm_in_w",s->norm_in_w);
  fprint_vec(out,"Normalization Factors norm_in_reg",s->norm_in_reg);
  fprint_vec(out,"Normalization Factors norm_out",s->norm_out);
  if (lwprs[ID].use_reg_2nd) {
    fprint_vec(out,"Normalization Factors norm_in_reg_2nd",s->norm_in_reg_2nd);
    fprint_vec(out,"Normalization Factors norm_out_2nd",s->norm_out_2nd);
  }
  fprint_vec(out,"Mean Inputs mean_x_w",s->mean_x_w);
  fprint_vec(out,"Var Inputs var_x_w",s->var_x_w);
  fprint_vec(out,"Mean Inputs mean_x_reg",s->mean_x_reg);
  fprint_vec(out,"Var Inputs var_x_reg",s->var_x_reg);
  
}

/*!*****************************************************************************
 *******************************************************************************
 \note  printRFStatistics
 \date  January, 1995
 
 \remarks 
 
 writes the major RF stats to the screen
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ID     : the ID of the LWPR
 \param[in]     rfID   : the ID of the RF 

 ******************************************************************************/
int
printRFStatistics(int ID, int rfID)
     
{

  FILE *out;
  int   n_in_reg;
  int   n_in_w;
  int   n_proj;
  int   n_out;
  int   diag_only;
  int   n_rf;
  LWPR  *s;
  int   i,j,k;
  char  name[100];
  int   num;

  out = stdout;

  /* some handy variables */
  
  n_in_w    = lwprs[ID].n_in_w;
  n_in_reg  = lwprs[ID].n_in_reg;
  n_out     = lwprs[ID].n_out;
  n_proj    = lwprs[ID].rfs[rfID].n_proj;
  diag_only = lwprs[ID].diag_only;
  n_rf      = lwprs[ID].n_rf;
  s         = &lwprs[ID];

  sprintf(name,"%s.%d.rf",s->lwpr_name,rfID);
  out = fopen(name,"w");
  if (out == NULL) return FALSE;
  
  fprintf(out,"Data of RF #%d of LWPR >%s<:\n\n",rfID,s->lwpr_name);

  fprintf(out,"n_proj              = %d\n",n_proj);
  fprintf(out,"n_nn                = %d\n",s->rfs[rfID].n_nn);
  fprintf(out,"cost                = %f\n",0.0);
  fprintf(out,"trustworthy         = %d\n",lwprs[ID].rfs[rfID].trustworthy);
  fprintf(out,"sum_D2              = %f\n",lwprs[ID].rfs[rfID].sum_D2);
  fprintf(out,"n_updates           = %ld\n",lwprs[ID].rfs[rfID].n_updates);
  for (i=1; i<=n_proj; ++i) {
    fprintf(out,"%d.sum_weights      = %f\n",
	    i,lwprs[ID].rfs[rfID].sum_weights[i]);
    fprintf(out,"%d.sum_error        = %f\n",
	    i,lwprs[ID].rfs[rfID].sum_error[i]);
    fprintf(out,"%d.sum_error_cvi    = %f\n",
	    i,lwprs[ID].rfs[rfID].sum_error_cvi[i]);
    fprintf(out,"%d.n_data           = %f\n",
	    i,lwprs[ID].rfs[rfID].n_data[i]);
  }

  if (lwprs[ID].use_reg_2nd) {

    fprintf(out,"n_proj_2nd              = %d\n",lwprs[ID].rfs[rfID].n_proj_2nd);
    for (i=1; i<=lwprs[ID].rfs[rfID].n_proj_2nd; ++i) {
      fprintf(out,"%d.sum_weights_2nd      = %f\n",
	      i,lwprs[ID].rfs[rfID].sum_weights_2nd[i]);
      fprintf(out,"%d.sum_error_2nd        = %f\n",
	      i,lwprs[ID].rfs[rfID].sum_error_2nd[i]);
      fprintf(out,"%d.sum_error_cvi_2nd    = %f\n",
	      i,lwprs[ID].rfs[rfID].sum_error_cvi_2nd[i]);
      fprintf(out,"%d.n_data_2nd           = %f\n",
	      i,lwprs[ID].rfs[rfID].n_data_2nd[i]);
    }

  }

  fflush(out);

  return TRUE;
  

} 

/*!*****************************************************************************
 *******************************************************************************
 \note  getLWPRCenterInfo
 \date  January, 1995

 \remarks 

 returns the location of a RF center

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     ID     : the ID of the LWPR
 \param[in]     rfID   : the ID of the RF
 \param[out]    c      : the center

 ******************************************************************************/
int
getLWPRCenterInfo(int ID,int rfID,double *c)

{
  int i;

  if (rfID > lwprs[ID].n_rf || rfID < 1) return FALSE;

  for (i=1; i<=lwprs[ID].n_in_w; ++i) {
    c[i] = lwprs[ID].rfs[rfID].c[i]*lwprs[ID].norm_in_w[i];
  }

  return TRUE;
}
/*!*****************************************************************************
 *******************************************************************************
 \note  getLWPRCenterInfoJac
 \date  January, 1995

 \remarks 

 returns the location and prediction at a RF center and the locally
 linear model

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     ID     : the ID of the LWPR
 \param[in]     rfID   : the ID of the RF
 \param[out]    c      : the center
 \param[out]    jac    : linear model at center

 ******************************************************************************/
int
getLWPRCenterInfoJac(int ID,int rfID,double *c, double **jac)

{
  int i,j,k;

  printf("this needs to be fixed ....\n");

  if (rfID > lwprs[ID].n_rf || rfID < 1) return FALSE;

  for (i=1; i<=lwprs[ID].n_in_w; ++i) {
    c[i] = lwprs[ID].rfs[rfID].c[i]*lwprs[ID].norm_in_w[i];
  }

  for (j=1; j<=lwprs[ID].n_out; ++j) {
    for (k=1; k<=lwprs[ID].rfs[rfID].n_proj; ++k) {
      jac[j][k] = lwprs[ID].rfs[rfID].B[k][j] / lwprs[ID].norm_in_reg[j];
    }
  }

  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
 \note  several simple functions to change varibles of an lwpr
 \date  January, 1994
 
 \remarks 
 
 none
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ID     : the ID of the LWPR
 \param[in]     val    : the value to be changed
 
 ******************************************************************************/
void
changeLWPRPenalty(int ID, double val)
{
  if (!checkLWPRExists(ID)) return;
  lwprs[ID].penalty = val;
}
void
changeLWPRInitLambda(int ID, double init_val)
{
  if (!checkLWPRExists(ID)) return;
  lwprs[ID].init_lambda  = init_val;
}
void
changeLWPRFinalLambda(int ID, double final_val)
{
  if (!checkLWPRExists(ID)) return;
  lwprs[ID].final_lambda = final_val;
}
void
changeLWPRTauLambda(int ID, double tau_val)
{
  if (!checkLWPRExists(ID)) return;
  lwprs[ID].tau_lambda = tau_val;
}
void
changeLWPRInitS2(int ID, double init_S2)
{
  if (!checkLWPRExists(ID)) return;
  lwprs[ID].init_S2 = init_S2;
}
void
changeLWPRMetaLearningRate(int ID, double val)
{
  if (!checkLWPRExists(ID)) return;
  lwprs[ID].meta_learning_rate = val;
}
void
changeLWPRInitD(int ID, double **init_D)
{

  int     i,j;
  double *temp;
  int     n_in;

  if (!checkLWPRExists(ID)) return;

  /* the distance metric */

  n_in = lwprs[ID].n_in_w;
  
  for (i=1; i<=n_in; ++i) {
    for (j=i; j<=n_in; ++j) {
      lwprs[ID].init_D[i][j]     = init_D[i][j];
      if (i==j && lwprs[ID].init_D[i][j] == 0) {
	lwprs[ID].init_D[i][j] = TINY_NUMBER;
      }
      if (lwprs[ID].diag_only && i != j) lwprs[ID].init_D[i][j] = 0.0;
      lwprs[ID].init_D[j][i]     = lwprs[ID].init_D[i][j];
      lwprs[ID].init_M[j][i]     = lwprs[ID].init_D[i][j];
      lwprs[ID].init_M[i][j]     = lwprs[ID].init_D[i][j];
    }
  }

  /* get the init_M by cholesky decomposition */

  temp = my_vector(1,n_in);

  if (!my_choldc(lwprs[ID].init_M,n_in,temp)) {

    printf("ERROR: Matrix is not positive definite\n");
    exit(-50);

  } else {

    /* since choldc returns the M matrix as lower triangular matrix
       in init_M, and the diagonal elements in temp, I have
       to fix this nonsense */

    for (i=1; i<=n_in; ++i) {
      for (j=1; j<=n_in; ++j) {
	lwprs[ID].init_M[i][j] = lwprs[ID].init_M[j][i];
	if (i==j) {
	  if (lwprs[ID].init_D[i][j] == 0)
	    lwprs[ID].init_M[i][j] = 0.0;
	  else	    
	    lwprs[ID].init_M[i][j] = temp[i];
	} else if (i>j) {
	  lwprs[ID].init_M[i][j] = 0.0;
	}
      }
    }

  }

  my_free_vector(temp,1,n_in);

}
void
changeLWPRInitAlpha(int ID, double init_alpha)
{
  int     i,j;

  if (!checkLWPRExists(ID)) return;
  lwprs[ID].init_alpha = init_alpha;
}
void
changeLWPRInitNProj(int ID, int init_n_proj)
{
  int     i,j;

  if (!checkLWPRExists(ID)) return;
  if (init_n_proj <= lwprs[ID].n_in_reg && init_n_proj >= 2)
    lwprs[ID].init_n_proj = init_n_proj;
  else if (init_n_proj == 1) { /* special learning rule for only one dimension */
    if (lwprs[ID].n_rf == 0) {
      lwprs[ID].allow_addDim = FALSE;
      lwprs[ID].init_n_proj = init_n_proj;
    } else if (init_n_proj > 1) {
      printf("ERROR: n_proj=1 cannot be used to change a trained LWPR!\n");
    }
  }

}
void
changeLWPRInitNProj2nd(int ID, int init_n_proj_2nd)
{
  int     i,j;

  if (!checkLWPRExists(ID)) return;
  if (init_n_proj_2nd <= lwprs[ID].n_in_reg_2nd && init_n_proj_2nd >= 2)
    lwprs[ID].init_n_proj_2nd = init_n_proj_2nd;
}
void
changeLWPRAddThreshold(int ID, double add_threshold)
{
  int     i,j;

  if (!checkLWPRExists(ID)) return;
  lwprs[ID].add_threshold = add_threshold;
}
void
changeLWPRAllowAddDim(int ID, int allow_addDim)
{
  if (!checkLWPRExists(ID)) return;
  if (lwprs[ID].init_n_proj > 1)
    lwprs[ID].allow_addDim = allow_addDim;
}
void
changeLWPRAllowAddRFS(int ID, int allow_add_rfs)
{
  if (!checkLWPRExists(ID)) return;
  lwprs[ID].allow_add_rfs = allow_add_rfs;
}
void
changeLWPRUseNN(int ID, int use_nn)
{
  if (!checkLWPRExists(ID)) return;
  if (use_nn > 1)
    use_nn = 1;
  if (use_nn < 0)
    use_nn = 0;
  lwprs[ID].use_nn = use_nn;
}
void
changeLWPRMaxNNComp(int ID, int max_nn_comp)
{
  if (!checkLWPRExists(ID)) return;
  if (max_nn_comp < 0)
    max_nn_comp = 0;
  lwprs[ID].max_nn_comp = max_nn_comp;
}
void
changeLWPRAllowDUpdate(int ID, int allow_d_update)
{
  if (!checkLWPRExists(ID)) return;
  lwprs[ID].allow_d_update        = allow_d_update;
}
void
changeLWPRWriteRFs(int ID, int write_rfs)
{
  if (!checkLWPRExists(ID)) return;
  lwprs[ID].write_rfs             = write_rfs;
}
void
changeLWPRMaxRFs(int ID, long max_rfs)
{
  if (!checkLWPRExists(ID)) return;
  lwprs[ID].max_rfs               = max_rfs;
}
void
changeLWPRWgen(int ID, double w_gen)
{
  if (!checkLWPRExists(ID)) return;
  lwprs[ID].w_gen          = w_gen;
}
void
changeLWPRWprune(int ID, double w_prune)
{
  if (!checkLWPRExists(ID)) return;
  lwprs[ID].w_prune        = w_prune;
}
void
changeLWPRFactorPrune(int ID, double factor_prune)
{
  if (!checkLWPRExists(ID)) return;
  lwprs[ID].factor_prune   = factor_prune;
}
void
changeLWPRDNoise(int ID, double val)
{
  if (!checkLWPRExists(ID)) return;
  lwprs[ID].d_noise        = val;
}
void
changeLWPRCNoise(int ID, double val)
{
  if (!checkLWPRExists(ID)) return;
  lwprs[ID].c_noise        = val;
}
int
checkLWPRExists(int ID)
{

  if (ID > MAX_LWPRS || ID < 0) {
    printf("LWPR ID=%d must be in the range of %d until %d --  change ID\n",
	   ID,0,MAX_LWPRS);
    return FALSE;
  }
  
  if (!lwprs[ID].lwpr_used) {
    printf("LWPR ID=%d is not inited yet\n",ID);
    return FALSE;
  }
  return TRUE;
}
void
changeLWPRCompositeControl(int ID, int composite_control, 
			   int n_states, int n_controls, int linear_gx)
{
  if (!checkLWPRExists(ID)) return;
  lwprs[ID].composite_control  = composite_control;
  lwprs[ID].n_states           = n_states;
  lwprs[ID].n_controls         = n_controls;
  lwprs[ID].linear_gx          = linear_gx;
}
void
changeLWPRUseOffset(int ID, int val)
{
  if (!checkLWPRExists(ID)) return;
  lwprs[ID].use_offsets        = val;
}
void
changeLWPRUseOffset2nd(int ID, int val)
{
  if (!checkLWPRExists(ID)) return;
  lwprs[ID].use_offsets_2nd        = val;
}
void
changeLWPRKernelFunction(int ID, int val)
{
  if (!checkLWPRExists(ID)) return;
  if (lwprs[ID].n_rf == 0)
    lwprs[ID].kernel_function = val;
  else 
    printf("Cannot change kernel function of an existing LWPR model\n");
}
void
changeLWPRName(int ID, char * new_name)
{
  if (!checkLWPRExists(ID)) return;
  /* overwrite the lwpr name */
  strcpy(lwprs[ID].lwpr_name,new_name);
}

/*!*****************************************************************************
 *******************************************************************************
 \note  writeLWPR
 \date  January, 1994
 
 \remarks 
 
 dumps the lwpr with the given ID to a binary file
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ID     : the ID of the LWPR
 
 ******************************************************************************/
int
writeLWPR(int ID)
     
{

  FILE *out;
  int   diag_only;
  int   meta;
  int   n_rf;
  LWPR  *s,st;
  int   i,j,k;
  char  name[100];
  int   num;
  RF   *r,rt;

  if (!lwprs[ID].lwpr_used) {
    printf("No #LWPR=%d existent!\n",ID);
    return FALSE;
  }

  /* some handy variables */
  
  diag_only = lwprs[ID].diag_only;
  n_rf      = lwprs[ID].n_rf;
  s         = &(lwprs[ID]);
  meta      = lwprs[ID].allow_meta_learning;

  sprintf(name,"%s.lwpr",s->lwpr_name);
  out = fopen(name,"w");
  if (out == NULL) return FALSE;
  
  printf("Writing to file >%s< ...",name);

  st = *s;
#ifdef BYTESWAP
  st.n_in_reg            = byteswap_int( st.n_in_reg );
  st.n_in_reg_2nd        = byteswap_int( st.n_in_reg_2nd );
  st.n_in_w              = byteswap_int( st.n_in_w );
  st.n_out               = byteswap_int( st.n_out );
  st.n_out_2nd           = byteswap_int( st.n_out_2nd );
  st.n_data              = byteswap_int( st.n_data );
  st.n_data_2nd          = byteswap_int( st.n_data_2nd );
  st.w_gen               = byteswap_double( st.w_gen );
  st.w_prune             = byteswap_double( st.w_prune );
  st.factor_prune        = byteswap_double( st.factor_prune );
  st.n_rf                = byteswap_int( st.n_rf );
  st.n_rf_allocated      = byteswap_int( st.n_rf_allocated );
  st.lwpr_used           = byteswap_int( st.lwpr_used );
  st.use_offsets         = byteswap_int( st.use_offsets );
  st.use_offsets_2nd     = byteswap_int( st.use_offsets_2nd );
  st.final_lambda        = byteswap_double( st.final_lambda );
  st.tau_lambda          = byteswap_double( st.tau_lambda );
  st.init_S2             = byteswap_double( st.init_S2 );
  st.diag_only           = byteswap_int( st.diag_only );
  st.penalty             = byteswap_double( st.penalty );
  st.allow_meta_learning = byteswap_int( st.allow_meta_learning );
  st.meta_learning_rate  = byteswap_double( st.meta_learning_rate );
  st.n_pruned_overlap    = byteswap_int( st.n_pruned_overlap );
  st.n_pruned_error      = byteswap_int( st.n_pruned_error );
  st.add_threshold       = byteswap_double( st.add_threshold );
  st.max_rfs             = byteswap_int( st.max_rfs );
  st.d_noise             = byteswap_double( st.d_noise );
  st.c_noise             = byteswap_double( st.c_noise );
  st.allow_d_update      = byteswap_int( st.allow_d_update );
  st.allow_addDim        = byteswap_int( st.allow_addDim );
  st.write_rfs           = byteswap_int( st.write_rfs );
  st.mean_n_proj         = byteswap_double( st.mean_n_proj );
  st.mean_n_proj_2nd     = byteswap_double( st.mean_n_proj_2nd );
  st.contrib_n_rf        = byteswap_double( st.contrib_n_rf );
  st.use_nn              = byteswap_int( st.use_nn );
  st.max_nn_comp         = byteswap_int( st.max_nn_comp );
  st.kernel_function     = byteswap_int( st.kernel_function );
  st.use_reg_2nd         = byteswap_int( st.use_reg_2nd );
  st.init_n_proj_2nd     = byteswap_int( st.init_n_proj_2nd );
  st.init_n_proj         = byteswap_int( st.init_n_proj );
  st.init_alpha          = byteswap_double( st.init_alpha );
#endif

  /* the defining main structure */
  num = 1;
  if (fwrite(&st,sizeof(LWPR),num,out)!= num) {
    printf( "cannot fwrite file.\n" );
    return FALSE;
  }

  /* norm, init_D,  mean_x, var_x */
  fwrite_vec(out,s->norm_in_w);
  fwrite_vec(out,s->norm_in_reg);
  fwrite_vec(out,s->norm_out);
  fwrite_mat(out,s->init_D);
  fwrite_vec(out,s->mean_x_w);
  fwrite_vec(out,s->var_x_w);
  fwrite_vec(out,s->mean_x_reg);
  fwrite_vec(out,s->var_x_reg);

  if (lwprs[ID].use_reg_2nd) {
    fwrite_vec(out,s->norm_in_reg_2nd);
    fwrite_vec(out,s->norm_out_2nd);
  }

  /* all the rfs */
  if (n_rf > 0) {
    for (i=1; i<=n_rf; ++i) {

      rt = s->rfs[i];
#ifdef BYTESWAP
      rt.w = byteswap_double(rt.w);
      rt.ww = byteswap_double(rt.ww);
      rt.n_proj = byteswap_int(rt.n_proj);
      rt.n_proj_2nd = byteswap_int(rt.n_proj_2nd);
      rt.trustworthy = byteswap_int(rt.trustworthy);
      rt.trustworthy_2nd = byteswap_int(rt.trustworthy_2nd);
      rt.sum_D2 = byteswap_double(rt.sum_D2);
      rt.sum_error_cv = byteswap_double(rt.sum_error_cv);
      rt.sum_error_cv_2nd = byteswap_double(rt.sum_error_cv_2nd);
      rt.n_updates = byteswap_int(rt.n_updates);
      rt.n_nn = byteswap_int(rt.n_nn);
      rt.n_nn_allocated = byteswap_int(rt.n_nn_allocated);
      rt.next_nn_check = byteswap_int(rt.next_nn_check);
#endif
      
      num = 1;
      if (fwrite(&rt,sizeof(RF),num,out)!= num) {
	printf( "cannot fwrite file.\n" );
	return FALSE;
      }
    }
  }

  /* the matrices of the rf */

  for (i=1; i<=n_rf; ++i) {

    r = &s->rfs[i];

    fwrite_mat(out,r->B);
    fwrite_mat(out,r->D);
    fwrite_mat(out,r->M);
    fwrite_vec(out,r->c);
    fwrite_mat(out,r->SXresYres);
    fwrite_vec(out,r->ss2);
    fwrite_vec(out,r->dof);
    fwrite_mat(out,r->SSYres);
    fwrite_mat(out,r->SSXres);
    fwrite_mat(out,r->alpha);
    fwrite_vec(out,r->mean_x_w);
    fwrite_vec(out,r->var_x_w);
    fwrite_vec(out,r->mean_x_reg);
    fwrite_vec(out,r->var_x_reg);
    fwrite_vec(out,r->mean_y);
    fwrite_vec(out,r->var_y);
    fwrite_vec(out,r->sum_weights);
    fwrite_vec(out,r->sum_error);
    fwrite_vec(out,r->sum_error_cvi);
    fwrite_vec(out,r->n_data);
    fwrite_vec(out,r->lambda);
    fwrite_mat(out,r->W);
    fwrite_vec(out,r->Wnorm);
    fwrite_mat(out,r->U);
    fwrite_mat(out,r->H);
    fwrite_vec(out,r->r);
    fwrite_mat(out,r->T);
    fwrite_mat(out,r->A);
    if (meta) {
      fwrite_mat(out,r->b);
      fwrite_mat(out,r->h);
    }
    fwrite_ivec(out,r->nn);
    fwrite_vec(out,r->nnw);
    fwrite_vec(out,r->s);

    if (lwprs[ID].use_reg_2nd) {
      fwrite_mat(out,r->B_2nd);
      fwrite_mat(out,r->SXresYres_2nd);
      fwrite_vec(out,r->ss2_2nd);
      fwrite_vec(out,r->dof_2nd);
      fwrite_mat(out,r->SSYres_2nd);
      fwrite_mat(out,r->SSXres_2nd);
      fwrite_vec(out,r->mean_x_reg_2nd);
      fwrite_vec(out,r->var_x_reg_2nd);
      fwrite_vec(out,r->mean_y_2nd);
      fwrite_vec(out,r->var_y_2nd);
      fwrite_vec(out,r->sum_weights_2nd);
      fwrite_vec(out,r->sum_error_2nd);
      fwrite_vec(out,r->sum_error_cvi_2nd);
      fwrite_vec(out,r->n_data_2nd);
      fwrite_vec(out,r->lambda_2nd);
      fwrite_mat(out,r->W_2nd);
      fwrite_vec(out,r->Wnorm_2nd);
      fwrite_mat(out,r->U_2nd);
      fwrite_vec(out,r->s_2nd);
    }

    fflush(out);

  }
  
  fflush(out);
  fclose(out);

  printf("done\n");
  
  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  readLWPR
 \date  January, 1994
 
 \remarks 
 
 reads the lwpr from the given name, by searching for a file name.lwpr,
 and assigns this lwpr to the lwpr with numer ID.
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ID     : the ID of the LWPR
 \param[in]     name   : the name of the LWPR, whose data is expected in the file
             name.lwpr
 
 ******************************************************************************/
int
readLWPR(int ID, char *name)
     
{

  FILE *in;
  int   diag_only;
  int   n_in_w;
  int   n_in_reg;
  int   n_out;
  int   n_in_reg_2nd;
  int   n_out_2nd;
  int   n_rf;
  int   n_rf_allocated;
  LWPR  *s,st;
  int   i,j,k;
  char  string[100];
  int   num;
  size_t length, number;
  RF   *r,rt;
  int   init_n_proj;

  if (lwprs[ID].lwpr_used) {
    printf("#LWPR=%d already existent!\n",ID);
    return FALSE;
  }

  /* open the file */

  i = strlen(name);
  if (name[i-5] == '.' && name[i-4] == 'l' && name[i-3] == 'w' &&
      name[i-2] == 'p' && name[i-1] == 'r') {
    strcpy(string,name);
    name[i-5] = '\0'; 
  } else {
    sprintf(string,"%s.lwpr",name);
  }

  in = fopen(string,"r");
  if (in == NULL) {
    printf("File >%s< not found -- LWPR not read!\n",name);
    return FALSE;
  }

  printf("Reading file >%s< ...",string);

  /* read the main structure */
  if (fread(&st,sizeof(LWPR),1,in)!= 1) {
    printf( "cannot fread file.\n" );
    return FALSE;
  }

#ifdef BYTESWAP
  st.n_in_reg            = byteswap_int( st.n_in_reg );
  st.n_in_reg_2nd        = byteswap_int( st.n_in_reg_2nd );
  st.n_in_w              = byteswap_int( st.n_in_w );
  st.n_out               = byteswap_int( st.n_out );
  st.n_out_2nd           = byteswap_int( st.n_out_2nd );
  st.n_data              = byteswap_int( st.n_data );
  st.n_data_2nd          = byteswap_int( st.n_data_2nd );
  st.w_gen               = byteswap_double( st.w_gen );
  st.w_prune             = byteswap_double( st.w_prune );
  st.factor_prune        = byteswap_double( st.factor_prune );
  st.n_rf                = byteswap_int( st.n_rf );
  st.n_rf_allocated      = byteswap_int( st.n_rf_allocated );
  st.lwpr_used           = byteswap_int( st.lwpr_used );
  st.use_offsets         = byteswap_int( st.use_offsets );
  st.use_offsets_2nd     = byteswap_int( st.use_offsets_2nd );
  st.final_lambda        = byteswap_double( st.final_lambda );
  st.tau_lambda          = byteswap_double( st.tau_lambda );
  st.init_S2             = byteswap_double( st.init_S2 );
  st.diag_only           = byteswap_int( st.diag_only );
  st.penalty             = byteswap_double( st.penalty );
  st.allow_meta_learning = byteswap_int( st.allow_meta_learning );
  st.meta_learning_rate  = byteswap_double( st.meta_learning_rate );
  st.n_pruned_overlap    = byteswap_int( st.n_pruned_overlap );
  st.n_pruned_error      = byteswap_int( st.n_pruned_error );
  st.add_threshold       = byteswap_double( st.add_threshold );
  st.max_rfs             = byteswap_int( st.max_rfs );
  st.d_noise             = byteswap_double( st.d_noise );
  st.c_noise             = byteswap_double( st.c_noise );
  st.allow_d_update      = byteswap_int( st.allow_d_update );
  st.allow_addDim        = byteswap_int( st.allow_addDim );
  st.write_rfs           = byteswap_int( st.write_rfs );
  st.mean_n_proj         = byteswap_double( st.mean_n_proj );
  st.mean_n_proj_2nd     = byteswap_double( st.mean_n_proj_2nd );
  st.contrib_n_rf        = byteswap_double( st.contrib_n_rf );
  st.use_nn              = byteswap_int( st.use_nn );
  st.max_nn_comp         = byteswap_int( st.max_nn_comp );
  st.kernel_function     = byteswap_int( st.kernel_function );
  st.use_reg_2nd         = byteswap_int( st.use_reg_2nd );
  st.init_n_proj_2nd     = byteswap_int( st.init_n_proj_2nd );
  st.init_n_proj         = byteswap_int( st.init_n_proj );
  st.init_alpha          = byteswap_double( st.init_alpha );
#endif


  /* some handy variables */
  diag_only      = st.diag_only;
  n_rf           = st.n_rf;
  n_rf_allocated = st.n_rf_allocated;
  n_in_w         = st.n_in_w;
  n_in_reg       = st.n_in_reg;
  n_out          = st.n_out;
  n_in_reg_2nd   = st.n_in_reg_2nd;
  n_out_2nd      = st.n_out_2nd;
  init_n_proj    = st.init_n_proj;

  /* overwrite the lwpr name */
  strcpy(st.lwpr_name,name);

  /* need the matrices init_D and stuff */
  st.init_D           = my_matrix(1,n_in_w,1,n_in_w);
  st.norm_in_w        = my_vector(1,n_in_w);
  st.norm_in_reg      = my_vector(1,n_in_reg);
  st.norm_out         = my_vector(1,n_out);
  st.mean_x_w         = my_vector(1,n_in_w);
  st.var_x_w          = my_vector(1,n_in_w);
  st.mean_x_reg       = my_vector(1,n_in_reg);
  st.var_x_reg        = my_vector(1,n_in_reg);
  if (st.use_reg_2nd) {
    st.norm_in_reg_2nd      = my_vector(1,n_in_reg_2nd);
    st.norm_out_2nd         = my_vector(1,n_out_2nd);
  }

  fread_vec(in,st.norm_in_w);
  fread_vec(in,st.norm_in_reg);
  fread_vec(in,st.norm_out);
  fread_mat(in,st.init_D);
  fread_vec(in,st.mean_x_w);
  fread_vec(in,st.var_x_w);
  fread_vec(in,st.mean_x_reg);
  fread_vec(in,st.var_x_reg);
  if (st.use_reg_2nd) {
    fread_vec(in,st.norm_in_reg_2nd);
    fread_vec(in,st.norm_out_2nd);
  }

  /* initialize the LWPR */

  if (!initLWPR(n_in_w, n_in_reg, st.n_in_reg_2nd, 
		st.n_out, st.n_out_2nd, diag_only,
		st.allow_meta_learning, 
		st.meta_learning_rate, st.penalty, 
		st.init_alpha,st.norm_in_w,st.norm_in_reg, st.norm_in_reg_2nd, 
		st.norm_out, st.norm_out_2nd, st.lwpr_name, ID, &st)) {
    return FALSE;
  }

  changeLWPRInitD(ID,st.init_D);

  s = &lwprs[ID];

  vec_equal(st.mean_x_w,s->mean_x_w);
  vec_equal(st.var_x_w,s->var_x_w);
  vec_equal(st.mean_x_reg,s->mean_x_reg);
  vec_equal(st.var_x_reg,s->var_x_reg);


  my_free_matrix(st.init_D,1,n_in_w,1,n_in_w);
  my_free_vector(st.norm_in_w,1,n_in_w);
  my_free_vector(st.norm_in_reg,1,n_in_reg);
  my_free_vector(st.norm_out,1,n_out);
  my_free_vector(st.mean_x_w,1,n_in_w);
  my_free_vector(st.var_x_w,1,n_in_w);
  my_free_vector(st.mean_x_reg,1,n_in_reg);
  my_free_vector(st.var_x_reg,1,n_in_reg);
  if (lwprs[ID].use_reg_2nd) {
    my_free_vector(st.norm_in_reg_2nd,1,n_in_reg_2nd);
    my_free_vector(st.norm_out_2nd,1,n_out_2nd);
  }

  /* add all rf to this structure */

  if (n_rf > 0) {

    length = sizeof(RF);
    number = n_rf_allocated;
    s->rfs = (RF *) calloc(number,length);
    
    if (s->rfs == NULL) {
      printf("No more memory for LWPR -- aborted\n");
      return FALSE;
    }

    s->rfs -= 1;
  }    

  s->n_rf_allocated = n_rf_allocated;

  /* read all the rf */

  if (n_rf > 0) {
    for (i=1; i<=n_rf; ++i) {
      num = 1;
      if (fread(&rt,sizeof(RF),num,in)!= num) {
	printf( "cannot fread file.\n" );
	return FALSE;
      }
#ifdef BYTESWAP
      rt.w = byteswap_double(rt.w);
      rt.ww = byteswap_double(rt.ww);
      rt.n_proj = byteswap_int(rt.n_proj);
      rt.n_proj_2nd = byteswap_int(rt.n_proj_2nd);
      rt.trustworthy = byteswap_int(rt.trustworthy);
      rt.trustworthy_2nd = byteswap_int(rt.trustworthy_2nd);
      rt.sum_D2 = byteswap_double(rt.sum_D2);
      rt.sum_error_cv = byteswap_double(rt.sum_error_cv);
      rt.sum_error_cv_2nd = byteswap_double(rt.sum_error_cv_2nd);
      rt.n_updates = byteswap_int(rt.n_updates);
      rt.n_nn = byteswap_int(rt.n_nn);
      rt.n_nn_allocated = byteswap_int(rt.n_nn_allocated);
      rt.next_nn_check = byteswap_int(rt.next_nn_check);
#endif
      s->rfs[i] = rt;

    }
  }

  /* now add the memory for the rfs; NOTE: this must come after reading the
     rfs since I read invalid pointers  in this step. */
  
  s->n_rf = 0;

  for (i=1; i<=n_rf; ++i) {
    if (!addRF(ID,s->rfs[i].n_proj,s->rfs[i].n_proj_2nd,s->rfs[i].n_nn_allocated))
      return FALSE;
  }

  /* the matrices of the rf */

  for (i=1; i<=n_rf; ++i) {

    r = &s->rfs[i];

    fread_mat(in,r->B);
    fread_mat(in,r->D);
    fread_mat(in,r->M);
    fread_vec(in,r->c);
    fread_mat(in,r->SXresYres);
    fread_vec(in,r->ss2);
    fread_vec(in,r->dof);
    fread_mat(in,r->SSYres);
    fread_mat(in,r->SSXres);
    fread_mat(in,r->alpha);
    fread_vec(in,r->mean_x_w);
    fread_vec(in,r->var_x_w);
    fread_vec(in,r->mean_x_reg);
    fread_vec(in,r->var_x_reg);
    fread_vec(in,r->mean_y);
    fread_vec(in,r->var_y);
    fread_vec(in,r->sum_weights);
    fread_vec(in,r->sum_error);
    fread_vec(in,r->sum_error_cvi);
    fread_vec(in,r->n_data);
    fread_vec(in,r->lambda);
    fread_mat(in,r->W);
    fread_vec(in,r->Wnorm);
    fread_mat(in,r->U);
    fread_mat(in,r->H);
    fread_vec(in,r->r);
    fread_mat(in,r->T);
    fread_mat(in,r->A);
    if (st.allow_meta_learning) {
      fread_mat(in,r->b);
      fread_mat(in,r->h);
    }
    fread_ivec(in,r->nn);
    fread_vec(in,r->nnw);
    fread_vec(in,r->s);

    if (lwprs[ID].use_reg_2nd) {
      fread_mat(in,r->B_2nd);
      fread_mat(in,r->SXresYres_2nd);
      fread_vec(in,r->ss2_2nd);
      fread_vec(in,r->dof_2nd);
      fread_mat(in,r->SSYres_2nd);
      fread_mat(in,r->SSXres_2nd);
      fread_vec(in,r->mean_x_reg_2nd);
      fread_vec(in,r->var_x_reg_2nd);
      fread_vec(in,r->mean_y_2nd);
      fread_vec(in,r->var_y_2nd);
      fread_vec(in,r->sum_weights_2nd);
      fread_vec(in,r->sum_error_2nd);
      fread_vec(in,r->sum_error_cvi_2nd);
      fread_vec(in,r->n_data_2nd);
      fread_vec(in,r->lambda_2nd);
      fread_mat(in,r->W_2nd);
      fread_vec(in,r->Wnorm_2nd);
      fread_mat(in,r->U_2nd);
      fread_vec(in,r->s_2nd);
    }

  }
  
  fclose(in);

  printf("done\n");
  
  return TRUE;


}

/*!*****************************************************************************
 *******************************************************************************
 \note  writeRFAscii
 \date  January, 1995
 
 \remarks 
 
 writes all data of a RF to an ascii file
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ID     : the ID of the LWPR
 \param[in]     rfID   : the ID of the RF 
 \param[in]     silent : print message about writing or not
 \param[in]     string : a message to be written in output

 ******************************************************************************/
int
writeRFAscii(int ID, int rfID, int silent, char *string)
     
{

  FILE *out;
  int   n_in_w;
  int   n_in_reg;
  int   n_proj;
  int   n_proj_2nd;
  int   n_out;
  int   diag_only;
  int   n_rf;
  LWPR  *s;
  int   i,j,k;
  char  name[100];
  int   num;

  /* some handy variables */
  
  n_in_w    = lwprs[ID].n_in_w;
  n_in_reg  = lwprs[ID].n_in_reg;
  n_out     = lwprs[ID].n_out;
  n_proj    = lwprs[ID].rfs[rfID].n_proj;
  n_proj_2nd= lwprs[ID].rfs[rfID].n_proj_2nd;
  diag_only = lwprs[ID].diag_only;
  n_rf      = lwprs[ID].n_rf;
  s         = &lwprs[ID];

  sprintf(name,"%s.%d.rf",s->lwpr_name,rfID);
  out = fopen(name,"w");
  if (out == NULL) return FALSE;
  
  if (!silent) printf("Writing to file >%s< ...",name);
  fprintf(out,"Data of RF #%d of LWPR >%s<:\n\n",rfID,s->lwpr_name);

  fprintf(out,"message             = %s\n",string);
  fprintf(out,"n_proj              = %d\n",n_proj);
  fprintf(out,"cost                = %f\n",-999.0);
  fprintf(out,"trustworthy         = %d\n",lwprs[ID].rfs[rfID].trustworthy);
  if (lwprs[ID].use_reg_2nd) {
    fprintf(out,"n_proj_2n           = %d\n",n_proj_2nd);
  }

  for (i=1;i<=n_proj; ++i) {
    fprintf(out,"%d.sum_weights      = %f\n",
	    i,lwprs[ID].rfs[rfID].sum_weights[i]);
    fprintf(out,"%d.sum_error        = %f\n",
	    i,lwprs[ID].rfs[rfID].sum_error[i]);
    fprintf(out,"%d.sum_error_cvi    = %f\n",
	    i,lwprs[ID].rfs[rfID].sum_error_cvi[i]);
    fprintf(out,"%d.n_data           = %f\n",
	    i,lwprs[ID].rfs[rfID].n_data[i]);
    fprintf(out,"%d.lambda           = %f\n",
	    i,lwprs[ID].rfs[rfID].lambda[i]);

    if (lwprs[ID].use_reg_2nd) {
      fprintf(out,"%d.sum_weights_2nd  = %f\n",
	      i,lwprs[ID].rfs[rfID].sum_weights_2nd[i]);
      fprintf(out,"%d.sum_error_2nd    = %f\n",
	      i,lwprs[ID].rfs[rfID].sum_error_2nd[i]);
      fprintf(out,"%d.sum_error_cvi_2nd= %f\n",
	      i,lwprs[ID].rfs[rfID].sum_error_cvi_2nd[i]);
      fprintf(out,"%d.n_data_2nd       = %f\n",
	      i,lwprs[ID].rfs[rfID].n_data_2nd[i]);
      fprintf(out,"%d.lambda_2nd       = %f\n",
	      i,lwprs[ID].rfs[rfID].lambda_2nd[i]);
    }
  }
  fprintf(out,"sum_D2              = %f\n",lwprs[ID].rfs[rfID].sum_D2);
  fprintf(out,"n_updates           = %ld\n",lwprs[ID].rfs[rfID].n_updates);

  fflush(out);

  fprint_vec(out,"RF-center c",s->rfs[rfID].c);
  fprint_mat(out,"Regression Coefficients B",s->rfs[rfID].B);
  fprint_mat(out,"Memory trace H",s->rfs[rfID].H);
  fprint_vec(out,"Memory trace r",s->rfs[rfID].r);
  fprint_mat(out,"Distance Metric D",s->rfs[rfID].D);
  fprint_vec(out,"Mean in Weight Input Space mean_x_w",s->rfs[rfID].mean_x_w);  
  fprint_vec(out,"Var in Weight Input Space var_x_w",s->rfs[rfID].var_x_w);  
  fprint_vec(out,"Mean in Regression Input Space mean_x_reg",s->rfs[rfID].mean_x_reg);  
  fprint_vec(out,"Var in Regression Input Space mean_x_reg",s->rfs[rfID].var_x_reg);  
  fprint_vec(out,"Mean in Output Space mean_y",s->rfs[rfID].mean_y);  
  fprint_vec(out,"Var in Output Space var_y",s->rfs[rfID].var_y);  

  fprint_vec(out,"Weighted Sum of Regression Var ss2",s->rfs[rfID].ss2);  
  fprint_vec(out,"Local Degree of freedom",s->rfs[rfID].dof);  
  fprint_mat(out,"Weighted Sum of Regression Cov SXresYres",
	     s->rfs[rfID].SXresYres);
  fprint_mat(out,"Weighted Sum of Regression Cov SSXres",
	     s->rfs[rfID].SSXres);
  fprint_mat(out,"Weighted Sum of Regression Cov SSYres",
	     s->rfs[rfID].SSYres);
  fprint_mat(out,"Distance Metric Learning Rate alpha",s->rfs[rfID].alpha);
  if (s->allow_meta_learning) {
    fprint_mat(out,"Distance Metric Learning Meta Rate b",s->rfs[rfID].b);
    fprint_mat(out,"Distance Metric Learning Trace Term h",s->rfs[rfID].h);
  }
  fprint_mat(out,"Projections W",s->rfs[rfID].W);
  fprint_mat(out,"Projections U",s->rfs[rfID].U);
  fprint_ivec(out,"Nearest Neighbors nn",s->rfs[rfID].nn);
  fprint_vec(out,"Nearest Neighbors nnw",s->rfs[rfID].nnw);

  if (lwprs[ID].use_reg_2nd) {
    fprint_mat(out,"Regression Coefficients B_2nd",s->rfs[rfID].B_2nd);
    fprint_vec(out,"Mean in Regression Input Space mean_x_reg_2nd",s->rfs[rfID].mean_x_reg_2nd);
    fprint_vec(out,"Var in Regression Input Space mean_x_reg_2nd",s->rfs[rfID].var_x_reg_2nd);  
    fprint_vec(out,"Mean in Output Space mean_y_2nd",s->rfs[rfID].mean_y_2nd);  
    fprint_vec(out,"Var in Output Space var_y_2nd",s->rfs[rfID].var_y_2nd);  
    
    fprint_vec(out,"Weighted Sum of Regression Var ss2_2nd",s->rfs[rfID].ss2_2nd);  
    fprint_vec(out,"Local Degree of freedom dof_2nd",s->rfs[rfID].dof_2nd);  
    fprint_mat(out,"Weighted Sum of Regression Cov SXresYres_2nd",
	       s->rfs[rfID].SXresYres_2nd);
    fprint_mat(out,"Weighted Sum of Regression Cov SSXres_2nd",
	       s->rfs[rfID].SSXres_2nd);
    fprint_mat(out,"Weighted Sum of Regression Cov SSYres_2nd",
	       s->rfs[rfID].SSYres_2nd);
    fprint_mat(out,"Projections W_2nd",s->rfs[rfID].W_2nd);
    fprint_mat(out,"Projections U_2nd",s->rfs[rfID].U_2nd);
  }

  fclose(out);
  
  if (!silent) printf("done\n");
  
  return TRUE;
  

} 

/*!*****************************************************************************
 *******************************************************************************
\note  getLWPRDistMetric
\date  Nov 3, 1992

\remarks 

       returns the distance metric for a specific RF


 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     ID    : lwpr ID
 \param[in]     rfID  : the rfID
 \param[out]    D     : the distance metric matrix


 ******************************************************************************/
void
getLWPRDistMetric(int ID, int rfID, double **D)

{

  int    i,j,k;
  RF    *rfptr;
  int    n_in_w;
  int    diag_only;
  double aux;

  /* handy variables */

  rfptr     = &lwprs[ID].rfs[rfID];
  n_in_w    = lwprs[ID].n_in_w;
  diag_only = lwprs[ID].diag_only;
  

  if (diag_only) {
    for (i=1; i<=n_in_w; ++i) {
      D[i][i] = sqr(rfptr->M[i][i]);
    }
  } else {
    for (i=1; i<=n_in_w; ++i) {
      for (j=i; j<=n_in_w; ++j) {
	D[i][j] = 0.;
	for (k=1; k<=i; ++k) {
	  D[i][j] += rfptr->M[k][i]*rfptr->M[k][j];
	}
	D[j][i] = D[i][j];
      }
    }
  }

}

/*!*****************************************************************************
 *******************************************************************************
 \note  addDimension
 \date  January, 1994
 
 \remarks 
 
 add a regression dimensions to the current receptive field
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ID     : the ID of the LWPR
 \param[in]     rfID   : the ID of the receptive field
 
 ******************************************************************************/
static int
addDimension(int ID, int rfID)

{
  int    i,j,m;
  RF    *rfptr;
  int    n_in_w;
  int    n_in_reg;
  int    n_proj;
  int    n_out;

  
  n_out      = lwprs[ID].n_out;
  n_in_w     = lwprs[ID].n_in_w;
  n_in_reg   = lwprs[ID].n_in_reg;
  rfptr      = &lwprs[ID].rfs[rfID];
  n_proj      = rfptr->n_proj;

  if (n_proj == n_in_reg) {
    if (DEBUG_FLAG) printf("Max.dimensions reached\n");
    return FALSE;
  }

  mat_add_shape(&(rfptr->B),1,0);
  vec_add_shape(&(rfptr->ss2),1);
  rfptr->ss2[n_proj+1]=lwprs[ID].init_S2;
  vec_add_shape(&(rfptr->dof),1);
  mat_add_shape(&(rfptr->SSYres),1,0);
  mat_add_shape(&(rfptr->SSXres),1,0);
  mat_add_shape(&(rfptr->SXresYres),1,0);

  mat_add_shape(&(rfptr->W),1,0);
  rfptr->W[n_proj+1][n_proj+1]=1.0;
  vec_add_shape(&(rfptr->Wnorm),1);

  mat_add_shape(&(rfptr->U),1,0);
  mat_add_shape(&(rfptr->H),1,0);
  vec_add_shape(&(rfptr->r),1);

  vec_add_shape(&(rfptr->sum_weights),1);
  rfptr->sum_weights[n_proj+1] = TINY_NUMBER;
  vec_add_shape(&(rfptr->sum_error),1);
  vec_add_shape(&(rfptr->sum_error_cvi),1);
  vec_add_shape(&(rfptr->n_data),1);
  vec_add_shape(&(rfptr->lambda),1);
  rfptr->lambda[n_proj+1]=lwprs[ID].init_lambda;
  vec_add_shape(&(rfptr->s),1);
  
  ++rfptr->n_proj;

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  checkAddDim
 \date  May 1999
 
 \remarks 
 
 checks whether to add dimensions to the regression
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ID     : the ID of the LWPR
 \param[in]     rfID   : the ID of the receptive field

 returns TRUE if dimension should be increased else returns FALSE
 
 ******************************************************************************/
static int
checkAddDim(int ID, int rfID)

{

  int    i,j,m;
  RF    *rfptr;
  int    n_in_w;
  int    n_in_reg;
  int    n_out;
  int    n_proj;
  double t;
  double prob;
  double mse_n_proj;
  double mse_n_proj_1;

  n_in_w    = lwprs[ID].n_in_w;
  n_in_reg  = lwprs[ID].n_in_reg;
  rfptr     = &lwprs[ID].rfs[rfID];
  n_proj    = rfptr->n_proj;

  if (n_proj >= n_in_reg)
    return FALSE;

  /* this is an attempt to use a t-test as a termination criterion */
  
  /*
  t = rfptr->B[n_proj][1] * sqrt(rfptr->ss2[n_proj]) 
    / (sqrt(rfptr->sum_error[n_proj]/rfptr->sum_weights[n_proj])+TINY_NUMBER);

  prob = 1.-prob_t(t,rfptr->sum_weights[n_proj]-rfptr->dof[n_proj]);

  if (prob < 0.001) 
    return TRUE;

    */

  /* here, the mean squared error of the current regression dimension
     is compared against the previous one. Only if there is a signficant
     improvement in MSE, another dimension gets added. Some additional
     heuristics had to be added to ensure that the MSE decision is 
     based on sufficient data */


  mse_n_proj   = rfptr->sum_error_cvi[n_proj]  / rfptr->sum_weights[n_proj];
  mse_n_proj_1 = rfptr->sum_error_cvi[n_proj-1]/ rfptr->sum_weights[n_proj-1];
 
  if (mse_n_proj/mse_n_proj_1 < lwprs[ID].add_threshold  && 
      rfptr->n_data[n_proj]/rfptr->n_data[n_proj-11] > 0.5 && 
	rfptr->n_data[n_proj]*(1.-rfptr->lambda[n_proj]) > 0.5) {
    /*
    printf("%f %f %f\n",mse_n_proj/mse_n_proj_1,
	   rfptr->n_data[n_proj]/rfptr->n_data[1],
	   rfptr->n_data[n_proj]*(1.-rfptr->lambda[n_proj]));
	   */
    return TRUE;
  }

  return FALSE;
     
}

/*!*****************************************************************************
 *******************************************************************************
 \note  updateDistanceMetric
 \date  April 1999
 
 \remarks 
 
 updates the distance metric of a receptive field. Appropriate variables
 must have been assigned before in the structure of the receptive field
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ID       : the ID of the LWPR
 \param[in]     rfID     : the receptive field ID
 \param[in]     x_w      : the input vector in weight space (only normalized)
 \param[in]     x_mz     : the mean zero input vector
 
 ******************************************************************************/
static void
updateDistanceMetric(int ID, int rfID, Vector x_w, Vector x_mz)

{
  RF       *rfptr;
  int       n_in_w, n_in_reg, n_out;
  int       i, j, k, n, m;
  int       n_proj;
  double   *s;
  double    w;
  double  **error_cv;
  double  **error;
  double    dJdw = 0;
  double    dSdw = 0;
  LWPR     *l;
  double    aux;
  double    penalty;
  double    max_M;
  Matrix    dwdM;
  Matrix    dpdM;
  Matrix    dwwdMdM;
  Matrix    dppdMdM;
  int       diag_only;
  double    dJdM;
  int      *derivatives_ok;
  double    ecv2;
  double    e2;
  double    eH;
  double    ecvb;
  double    ucr;
  double    transient_multiplier=0;
  Vector    Ps;
  double    PseH=0;
  double    Ps2r=0;
  double    sPs=0;
  double    W;
  
  rfptr    = &lwprs[ID].rfs[rfID];
  l        = &lwprs[ID];
  n_in_w   =  lwprs[ID].n_in_w;
  n_in_reg =  lwprs[ID].n_in_reg;
  diag_only=  lwprs[ID].diag_only;
  n_out    =  lwprs[ID].n_out;
  error_cv =  lwprs[ID].error_cv;
  error    =  lwprs[ID].error;
  s        =  rfptr->s;
  n_proj   =  rfptr->n_proj;
  w        =  rfptr->w;
  Ps       =  lwprs[ID].x_tempp;
  penalty  =  lwprs[ID].penalty;
  dwdM     =  lwprs[ID].dwdM;
  dpdM     =  lwprs[ID].dpdM;
  dwwdMdM  =  lwprs[ID].dwwdMdM;
  dppdMdM  =  lwprs[ID].dppdMdM;
  derivatives_ok = lwprs[ID].derivatives_ok;

  /* first I need to calculate dJ/dw, which is the sum of these 
     derivatives over all regression dimension. The update is very
     cheap since almost all calculations are scalar */

  /* check in which dimensions we trust the derivatives */
  for (i=1; i<=n_proj; ++i) 
    derivatives_ok[i] = 
      rfptr->n_data[i] > 0.1/(1.-rfptr->lambda[i]) && rfptr->trustworthy;

  if (!lwprs[ID].allow_d_update || !derivatives_ok[1]) {
    transient_multiplier=0;
    return;
  }

  /* precompute useful variables */
  ecv2 = vec_mult_inner_size(l->error_cv[n_proj],l->error_cv[n_proj],n_out);
  e2   = vec_mult_inner_size(l->error[n_proj],l->error[n_proj],n_out);
  W    = rfptr->sum_weights[1];

  transient_multiplier = sqr(sqr(e2/(ecv2+1.e-10))); /* this is a numerical safety heuristic */
  if (transient_multiplier > 1)
    transient_multiplier = 1;

  sPs = 0;
  for (i=1; i<=n_proj; ++i) {
    Ps[i] = s[i]/rfptr->ss2[i]*derivatives_ok[i];
    sPs += Ps[i]*s[i];
  }

  PseH = 0;
  Ps2r = 0;
  for (i=1; i<=n_proj; ++i) {
    for (j=1; j<=n_out; ++j) {
      PseH += Ps[i]*error[n_proj][j]*rfptr->H[i][j];
    }
    Ps2r += sqr(Ps[i])*rfptr->r[i];
  }

  dJdw = (ecv2	 
	  - rfptr->sum_error_cv/W
	  - 2. * Ps2r
	  - 2. * PseH ) / W;

  /* if required, calculated the second derivatives */
  if (lwprs[ID].allow_meta_learning) {
    
    /* second derivatives */
    dSdw =
      (-ecv2/W
       -2.*(-1./W-2.*sPs)*PseH
       +2*e2*sPs
      -(ecv2-2.*PseH)/W
       +2.*rfptr->sum_error_cv/sqr(W)) / W;
  }
  
  /* the memory traces are updated last */
  for (i=1; i<=n_proj; ++i) {
	
    if (derivatives_ok[i])
      rfptr->r[i] = rfptr->r[i] * LAMBDA(rfptr->lambda[i],w) + 
	sqr(w*s[i])*ecv2/(1.-w*sPs)*transient_multiplier;
	
    aux = w*s[i]/(1.-w*sPs);
    for (j=1; j<=n_out; ++j)
      if (derivatives_ok[i])
	rfptr->H[i][j] = 
	  rfptr->H[i][j]*LAMBDA(rfptr->lambda[i],w) + 
	  aux * l->error_cv[n_proj][j] * transient_multiplier;
	
    
    if (lwprs[ID].init_n_proj == 1) { /* the special one-projection case */
      
      ecvb = vec_mult_inner_size(l->error_cv[i],rfptr->B[i],n_out);
      
      for (j=1; j<=n_in_reg; ++j) {
	
	rfptr->T[i][j] = 
	  rfptr->T[i][j]*LAMBDA(rfptr->lambda[i],w) + 
	  w*(2.*w*ecv2*Ps[i]-ecvb)*x_mz[j]/(1.-w*sPs)*
	  transient_multiplier* derivatives_ok[1];
	
	rfptr->A[i][j] = 
	  rfptr->A[i][j]*LAMBDA(rfptr->lambda[i],w)+w*s[i]*x_mz[j]*
	  transient_multiplier* derivatives_ok[1];
	
      }
    }
  }

  /* adjust the derivative in case of the special one-projection case */
  
  if (lwprs[ID].init_n_proj == 1) { 
    ucr = vec_mult_inner_size(rfptr->W[1],l->cr,n_in_reg);
    aux = 0.0;
    for (j=1; j<=n_in_reg; ++j) {
      aux += (l->cr[j] - rfptr->W[1][j]*ucr) * 
	(rfptr->T[1][j] - rfptr->r[1]/rfptr->ss2[1]*rfptr->A[1][j]);
    }
    dJdw += aux / W / rfptr->Wnorm[1];
  }
  
  /* dividing the penalty by the number of inputs tries to
     compensate for the change of penalty due to very many inputs --
     despite the inputs don't matter, the system get penalized for
     those extra inputs -- this tries to avoid this effect */
  penalty *= w/W/(double)n_in_w;
  
  /* divide dJdw and dSdw by the number of outputs to normalize for the 
     number of outputs */
  dJdw /= (double)n_out;
  dSdw /= (double)n_out;


  /* calculate derivatives of weight w.r.t. to distance metric */
  calculateDWDM(ID, rfID, x_w, penalty, dwdM, dpdM, dppdMdM, dwwdMdM, &max_M);

  /*****************************************************************/
  /* update the meta learning rate, learning rate, distance metric */  
  /* and h-term in meta learning rate                              */
  /*****************************************************************/
  
  for (i=1; i<=n_in_w && lwprs[ID].allow_d_update; ++i) {
    for (j=i; j<=n_in_w; ++j) {
      
      /* the second part of the if statment allows us to use a bias
	 on D and set known elements to zero -- i.e., no weighting in
	 this direction -- numerical problems require that D be not
	 zero, thus the TINY_NUMBER stuff */
      if ((!diag_only || i==j) && 
	  rfptr->D[i][i]>=2.*TINY_NUMBER && rfptr->D[j][j]>=2.*TINY_NUMBER) {
	
	/*****************************************************************/
	/* update the distance metric: M = M - alpha * (dJdw*dwdM +      */
	/* dpdM)                                                         */
	/*****************************************************************/
	
	dJdM =  
	  (dJdw * dwdM[i][j] + dpdM[i][j]);

	/************************************************/
	/* second order learning schemes are tried here */
	/************************************************/
	
	if (lwprs[ID].allow_meta_learning) {
	  
	  /* the meta learning rate and the real learning rate */
	  /* b = b - META_LEARNING_RATE * dJdM * h             */
	  /* alpha = exp(b)                                    */
	  /* and also update the memory term h:                */
	  /* h = (1-alpha dJJdMdM) h - alpha dJdM              */
	  
	  aux = lwprs[ID].meta_learning_rate * rfptr->h[i][j] * 
	    dJdM * transient_multiplier;
	  
	  /* prevent too large steps */
	  
	  if (fabs(aux) > .1) aux = macro_sign(aux) * .1;
	  rfptr->b[i][j] -= aux;
	  
	  /* prevent numerical underflow and overflow*/
	  
	  if (rfptr->b[i][j] < -10) rfptr->b[i][j] = -10.;
	  if (rfptr->b[i][j] >  10) rfptr->b[i][j] =  10.;
	  
	  /* the new learning rate becomes */
	  
	  rfptr->alpha[i][j] = myExp(rfptr->b[i][j],FALSE);
	  
	  /* update the memory trace (note: S = dJdw */
	  
	  aux = 1. - rfptr->alpha[i][j]  *
	    (dSdw * sqr(dwdM[i][j]) + dJdw * dwwdMdM[i][j] +
	     dppdMdM[i][j]) * transient_multiplier;
	  if (aux < 0.0) aux = 0.0;
	  
	  rfptr->h[i][j] = rfptr->h[i][j] * aux - 
	    rfptr->alpha[i][j] * dJdM * transient_multiplier;
	  
	}
	
	
	/* the final update of the distance metric */
	
	aux = rfptr->alpha[i][j] * dJdM * transient_multiplier;
	
	/* limit the update to be not more than a fraction of
	   the magnitude of the max O matrix coefficient */
	
	if (fabs(aux) > 0.1*max_M) {
	  
	  /* as experience shows that such fast updates are really bad
	     I significantly limit the update under such a warning */
	  
	  /* reduce the learning rate */
	  
	  rfptr->alpha[i][j] /= 2.0;
	  aux = 0.0;
	  
	  /* zero the trace terms for meta learning */
	  if (lwprs[ID].allow_meta_learning) {
	    rfptr->h[i][j] = 0.0;
	    rfptr->b[i][j] = log( rfptr->alpha[i][j]);
	  }

	  if (lwprs[ID].write_rfs)
	    printf("WARNING:Large M[%d][%d]-Update: ID=%d rfID=%d n_data=%f\n",
		   i,j,ID,rfID,rfptr->n_data[n_proj]);
	}

	if (i==j && lwprs[ID].init_M[i][j]==0) 
	  ;
	else
	  rfptr->M[i][j] -= aux;

	if (my_isnan(rfptr->M[i][j])) {
	  writeRFAscii(ID,rfID,FALSE,"NaN in M");
	  printf("rfptr->M[%d][%d]=%f\n",i,j,rfptr->M[i][j]);
	  printf("%f %f\n",rfptr->alpha[i][j] ,dJdM);
	  exit(-1);
	  rfptr->M[i][j] = lwprs[ID].init_M[i][j];
	}
	if (i==j && rfptr->M[i][j] < TINY_NUMBER) rfptr->M[i][j]=TINY_NUMBER;
	if (rfptr->M[i][j] > BIG_NUMBER) rfptr->M[i][j]=BIG_NUMBER;
	
      }
    }
  }

  /* update the distance metric from the decomposed distance metric */
  getLWPRDistMetric(ID,rfID,rfptr->D);
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  calculateWeightp
\date  Nov 3, 1992

\remarks 

       calculates the activation of a given rf for a given input vector
       note: special function to void multiple process clash

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     ID    : lwpr ID
 \param[in]     rfID  : the rfID
 \param[in]     x     : the input vector

 ******************************************************************************/
static double
calculateWeightp(int ID, int rfID, Vector x)

{

  int    i,j,k;
  double mahal;
  double aux;


  /* get the mahalanobis distance */

  mahal = calculateMahalp(ID, rfID, x);

  switch (lwprs[ID].kernel_function) {

  case GAUSSIAN:
  case VONMISES:
  
    return (myExp(-0.5*mahal,USE_EXP_LOOKUP));

  case BISQUARE:

    if (mahal > 1.0) {
      return (0.0);
    } else {
      aux = 1.- mahal;
      return (aux * aux);
    }

  }

  return FALSE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  calculateMahalp
\date  Nov 3, 1992

\remarks 

       calculates the mahalanobis distance
       note: special function to void multiple process clash

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     ID    : lwpr ID
 \param[in]     rfID  : the rfID
 \param[in]     x     : the input vector

 ******************************************************************************/
static double
calculateMahalp(int ID, int rfID, Vector x)

{

  int    i,j,k;
  RF    *rfptr;
  int    diag_only;
  double mahal;
  double aux;
  Vector delta;
  int    iaux;

  /* hand variables */

  rfptr     = &lwprs[ID].rfs[rfID];
  diag_only = lwprs[ID].diag_only;
  delta     = lwprs[ID].delta2;
  
  /* the distance from the center */
  vec_sub(x,rfptr->c,delta);

  switch (lwprs[ID].kernel_function) {
  case VONMISES:
    for (i=1; i<=lwprs[ID].n_in_w; ++i) {
      delta[i] = sqrt(1.-cos(delta[i]));
    }
    mahal = mat_mahal(rfptr->D,delta);
    break;

  case GAUSSIAN:
  default:
    /* the mahalanobis distance */
    mahal = mat_mahal(rfptr->D,delta);
    break;
  }


  return mahal;

}


/*!*****************************************************************************
 *******************************************************************************
 \note  checkNeighborhood
 \date  June, 1995
 
 \remarks 
 checks whether two receptive fields overlap according to the cutoff 
 threshold
  
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ID     : the ID of the model
 \param[in]     rfID1  : ID of the one RF
 \param[in]     rfID2  : ID of other RF

 returns the maximal common activation between the two RFs
 
 ******************************************************************************/
static double
checkNeighborhood(int ID, int rfID1, int rfID2) 

{

  int     i,j,k;
  int     n_in_w;
  int     n_nn;
  RF     *rfptr1,*rfptr2;
  int    *nn;
  double  mahal=0;
  double  aux;
  double  w;


  if (rfID1 == rfID2 || rfID2 < 1 || rfID1 < 1) 
    return 0.0;

  rfptr1 = &lwprs[ID].rfs[rfID1];
  rfptr2 = &lwprs[ID].rfs[rfID2];
  n_in_w = lwprs[ID].n_in_w;

  /* compute the point of nearest distance between the receptive fields,
     but only from the diagonal distance metric */

  for (i=1; i<=n_in_w; ++i) {
    aux = (rfptr1->D[i][i]*rfptr1->c[i] + rfptr2->D[i][i]*rfptr2->c[i])/
      (rfptr1->D[i][i] + rfptr2->D[i][i] + TINY_NUMBER) - rfptr1->c[i];
    mahal += sqr(aux)*rfptr1->D[i][i];
  }

  w = myExp(-0.5*mahal,USE_EXP_LOOKUP);

  if (w > lwprs[ID].w_gen) {
    if (addNNToRF(ID,rfID1,rfID2,w))
      addNNToRF(ID,rfID2,rfID1,w);
  } else {
    if (deleteNNFromRF(ID,rfID1,rfID2))
      deleteNNFromRF(ID,rfID2,rfID1);
  }

  return w;
}

/*!*****************************************************************************
 *******************************************************************************
 \note  addNNToRF
 \date  June, 1995
 
 \remarks 
 
 adds a NN ID to a RF
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ID       : the ID of the SDC
 \param[in]     rfID     : ID of the RF to which we want to add the ID
 \param[in]     add_rfID : the ID to be added
 \param[in]     w        : the weight of neighborhood
 
 ******************************************************************************/
static int
addNNToRF(int ID, int rfID, int add_rfID, double w) 

{

  int      i,j,k;
  int      n_nn;
  RF      *rfptr;
  iVector  nn;
  Vector   nnw;

  if (rfID == add_rfID)
    return FALSE;

  n_nn = lwprs[ID].rfs[rfID].n_nn;
  rfptr = &lwprs[ID].rfs[rfID];

  /* is the add_rfID already present? */
  for (i=1; i<=n_nn; ++i) 
    if (rfptr->nn[i] == add_rfID) 
      return FALSE;

  /* do we need more memory? */
  if (n_nn+1 > rfptr->n_nn_allocated) {
    ivec_add_shape(&(rfptr->nn),NN_BLOCK);
    vec_add_shape(&(rfptr->nnw),NN_BLOCK);
    rfptr->n_nn_allocated += NN_BLOCK;
  }

  if (n_nn == 0 ) {

    rfptr->nn[1]  = add_rfID;
    rfptr->nnw[1] = w;

  } else {
    
    /* add the new neighbor in ascending distance */
    for (i=n_nn; i >= 1; --i) {
      if (w > rfptr->nnw[i]) {
	rfptr->nn[i+1]  = rfptr->nn[i];
	rfptr->nnw[i+1] = rfptr->nnw[i];
	if (i==1) {
	  rfptr->nn[i]  = add_rfID;
	  rfptr->nnw[i] = w;
	}
      } else {
	rfptr->nn[i+1]  = add_rfID;
	rfptr->nnw[i+1] = w;
	break;
      }
    }

  }

  ++rfptr->n_nn;

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  deleteNNFromRF
 \date  June, 1995
 
 \remarks 
 
 deletes a NN from a rf
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ID       : the ID of the SDC
 \param[in]     rfID     : ID of the RF from which we want to delete the ID
 \param[in]     del_rfID : the ID to be deleted
 
 ******************************************************************************/
static int
deleteNNFromRF(int ID, int rfID, int del_rfID) 

{

  int  i,j,k;
  RF  *rfptr;
  int *nn;

  if (rfID == del_rfID)
    return FALSE;

  rfptr = &lwprs[ID].rfs[rfID];

  for (i=1; i<=rfptr->n_nn; ++i) {
    if (rfptr->nn[i] == del_rfID) {
      for (j=i; j < rfptr->n_nn; ++j) {
	rfptr->nn[j]  = rfptr->nn[j+1];
	rfptr->nnw[j] = rfptr->nnw[j+1];
      }
      rfptr->nn[j] = 0;
      rfptr->nnw[j] = 0;
      --rfptr->n_nn;
      return TRUE;
    }
  }

  return FALSE;
  
}

/*!*****************************************************************************
 *******************************************************************************
 \note  swapNNID
 \date  June, 1995
 
 \remarks 
 
 replaces a NN ID with another NN ID
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ID       : the ID of the SDC
 \param[in]     rfID     : ID of the RF from which we want to delete the ID
 \param[in]     old_rfID : the ID to be swapped
 \param[in]     new_rfID : the new ID to be swapped in
 
 ******************************************************************************/
static int
swapNNID(int ID, int rfID, int old_rfID, int new_rfID) 

{

  int  i,j,k;
  RF  *rfptr;
  int *nn;

  if (old_rfID == new_rfID)
    return FALSE;

  rfptr = &lwprs[ID].rfs[rfID];

  for (i=1; i<=rfptr->n_nn; ++i) {
    if (rfptr->nn[i] == old_rfID) {
      rfptr->nn[i] = new_rfID;
      return TRUE;
    }
  }

  return FALSE;
  
}

/*!*****************************************************************************
 *******************************************************************************
 \note  checkLWPRNN
 \date  June 2000
 
 \remarks 
 
 exhaustively updates the entire NN structure
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ID       : the ID of the SDC
 
 ******************************************************************************/
int
checkLWPRNN(int ID) 

{

  int    i,j,k;
  double w;
  double aux, aux1;
  int    index_prune;
  int    index_non_prune;
  char   stmp[100];
  int   *pruneIDs;

  if (!checkLWPRExists(ID)) 
    return FALSE;

  if (!lwprs[ID].use_nn)
    return FALSE;

  pruneIDs = my_ivector(1,lwprs[ID].n_rf);

  for (i=1; i<=lwprs[ID].n_rf; ++i) {
    for (j=1; j<=i-1; ++j) {

      w = checkNeighborhood(ID,i,j);

      /* pruning if co-activation is too much */
      if (w > lwprs[ID].w_prune) {
	index_prune = j;
	index_non_prune = i;
	aux  = lwprs[ID].rfs[i].sum_error_cv;
	aux1 = lwprs[ID].rfs[j].sum_error_cv;
	if (aux > aux1) {
	  index_prune = i;
	  index_non_prune = j;
	}
	pruneIDs[index_prune]=index_non_prune;
	printf("Prune: %d (overlap with %d) -- activation = %f\n",index_prune,index_non_prune,w);
      }

    }
    lwprs[ID].rfs[i].next_nn_check = i-1;
  }

  /* prune RFs if necessary */
  for (i=lwprs[ID].n_rf; i>=1; --i) {

    if (pruneIDs[i] == 0)
      continue;

    sprintf(stmp,"overlap prune due to %d",pruneIDs[i]);
    if (lwprs[ID].write_rfs) 
      writeRFAscii(ID,i,TRUE,stmp);
    sprintf(stmp,"this RF caused overlap prune of %d",i);
    if (lwprs[ID].write_rfs) 
      writeRFAscii(ID,pruneIDs[i],TRUE,stmp);
    deleteRF(ID,i);
    ++lwprs[ID].n_pruned_overlap;
  }

  my_free_ivector(pruneIDs,1,lwprs[ID].n_rf);

  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
 \note  addRFManual
 \date  January, 1994
 
 \remarks 
 
 adds a receptive field to the given LWPR due to manual user request
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ID        : the ID of the LWPR
 \param[in]     x         : the location where to add the center
 \param[in]     D         : the distance metric to be assigned
 
 ******************************************************************************/
int 
addRFManual(int ID, double *x, double **D)
{

  int       i,j;
  LWPR     *s;
  int       rfIDaux;
  Vector    temp;  
  Matrix    M;

  s = &lwprs[ID];

  /**********************************************/
  /* normalize the input                        */
  /**********************************************/

  for (i=1; i<=s->n_in_w; ++i) {
    s->x[i] = x[i]/s->norm_in_w[i];
  }

  /* get the initial distance metric M by cholesky decomposition */
  temp = my_vector(1,s->n_in_w);
  M    = my_matrix(1,s->n_in_w,1,s->n_in_w);
  mat_equal_size(D,s->n_in_w,s->n_in_w,M);

  if (!my_choldc(M,s->n_in_w,temp)) {

    printf("ERROR: Matrix is not positive definite\n");
    return FALSE;

  } else {

    /* since choldc returns the M matrix as lower triangular matrix
       in M, and the diagonal elements in temp, I have
       to fix this nonsense */

    for (i=1; i<=s->n_in_w; ++i) {
      for (j=1; j<=s->n_in_w; ++j) {
	M[i][j] = M[j][i];
	if (i==j) {
	  M[i][j] = temp[i];
	} else if (i>j) {
	  M[i][j] = 0.0;
	}
      }
    }

  }

  // the add the RF 
  rfIDaux = 
    addRF(ID,s->init_n_proj,s->init_n_proj_2nd,NN_BLOCK);
  
  if (rfIDaux != FALSE) {
    initRF(ID, rfIDaux, s->x, M);
  }

  my_free_vector(temp,1,s->n_in_w);
  my_free_matrix(M,1,s->n_in_w,1,s->n_in_w);

  return TRUE;

}


/*!*****************************************************************************
 *******************************************************************************
 \note  calculateDWDM
 \date  December, 2004
 
 \remarks 
 
 calculate first and second derivates of the kernel weight w.r.t. the distance
 metric
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ID        : the ID of the LWPR
 \param[in]     x         : the location where to add the center
 \param[in]     D         : the distance metric to be assigned
 
 ******************************************************************************/
static void 
calculateDWDM(int ID, int rfID, Vector x_w, double penalty, Matrix dwdM, Matrix dpdM,
	      Matrix dppdMdM, Matrix dwwdMdM, double *max_M)

{

  RF       *rfptr;
  int       n_in_w;
  int       i, j, k, n, m;
  double    aux;
  double   *xx;
  double    sum_aux,sum_aux1=0;
  int       diag_only;
  double    w;
  
  rfptr    = &lwprs[ID].rfs[rfID];
  xx       =  lwprs[ID].x_temp_w;
  n_in_w   =  lwprs[ID].n_in_w;
  diag_only=  lwprs[ID].diag_only;
  w        =  rfptr->w;

  /* calculate the dwdM, the derivative of the weights with respect
     to the distance metric coefficients. For this, I first have to
     calculate the derivative dDdM */
  switch (lwprs[ID].kernel_function) {
  case VONMISES:
    for (i=1; i<=n_in_w; ++i)
      xx[i] = sqrt(1.-cos(x_w[i] - rfptr->c[i]));
    break;
  default:
    for (i=1; i<=n_in_w; ++i)
      xx[i] = x_w[i] - rfptr->c[i];
  }
    
  /* dD_ijdM_nm = M_im * kron_nj + M_jm * kron_ni */
  
  *max_M = 0.0;

  for (n=1; n<=n_in_w; ++n) {
    for (m=n; m<=n_in_w; ++m) {

      dwdM[n][m] = 0.;
      dpdM[n][m] = 0.;
      sum_aux    = 0.;
      
      if (lwprs[ID].allow_meta_learning) {
	sum_aux1   = 0.;
	dppdMdM[n][m] = 0.0;
      }
      
      /* now take the derivative of D with respect to nm_th element of M */
      
      if (diag_only && n==m) {
	
	if (fabs(rfptr->M[n][m]) > *max_M) *max_M = fabs(rfptr->M[n][m]);
	
	aux = rfptr->M[n][n] * 2.;
	dwdM[n][n] = xx[n] * xx[n] * aux;
	sum_aux = rfptr->D[n][n]*aux;
	if (lwprs[ID].allow_meta_learning) sum_aux1+= sqr(aux);
	  
      } else if (!diag_only){
	  
	if (fabs(rfptr->M[n][m]) > *max_M) *max_M = fabs(rfptr->M[n][m]);
	
	for (i=n; i<=n_in_w; ++i) {
	  
	  /* aux corresponds to the in_th (= ni_th) element of dDdM_nm */
	  /* this is directly processed for dwdM and dpenaltydo        */
	  
	  if (i == m) {
	    aux = rfptr->M[n][i] * 2.;
	    dwdM[n][m] += xx[i] * xx[m] * aux;
	    sum_aux += rfptr->D[i][m]*aux;
	    if (lwprs[ID].allow_meta_learning) sum_aux1+= sqr(aux);
	  } else {
	    aux = rfptr->M[n][i];
	    dwdM[n][m] += 2. * xx[i] * xx[m] * aux;
	    sum_aux += 2.*rfptr->D[i][m]*aux;
	    if (lwprs[ID].allow_meta_learning) sum_aux1+= 2.*sqr(aux);
	  }
	  
	}
	
      }	  

      switch (lwprs[ID].kernel_function) {
      case BISQUARE:
        dwdM[n][m] *= -sqrt(w);
	break;
      default:
	dwdM[n][m] *= -0.5*w;
      }
      
      dpdM[n][m]  = 2.*penalty*sum_aux;

      
      if (lwprs[ID].allow_meta_learning) {
	dppdMdM[n][m] = 2. * penalty * ( 2* rfptr->D[m][m] + sum_aux1 );
	dppdMdM[m][n] = dppdMdM[n][m];

	switch (lwprs[ID].kernel_function) {
	case BISQUARE:
	  dwwdMdM[n][m]   = sqr(dwdM[n][m])/w/2.0 - 2*sqrt(w)*sqr(xx[m]);
	  break;
	default:
	  dwwdMdM[n][m] = sqr(dwdM[n][m])/w - w*sqr(xx[m]);
	}
	dwwdMdM[m][n] = dwwdMdM[n][m];
      }
      
    }
  }
    
}

/*!*****************************************************************************
 *******************************************************************************
 \note  addDataToLWPRReg2nd
 \date  March 1999
 
 \remarks 
 
 Adds a data point to the 2nd regression model
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ID     : the ID of the LWPR
 \param[in]     x_w    : the input vector for the weights
 \param[in]     x      : the input vector for the regression
 \param[in]     y      : the output vector for the regression
 \param[in]     w_misc : additional weight for weighted regression
 \param[in]     flag   : FALSE if y is an output, TRUE if y is an error
 \param[in,out] rfID   : neighborhood rfID for input, max activated rfID for output
 
 ******************************************************************************/
int 
addDataToLWPRReg2ndPredictConf(int ID, 
			       Vector x_w, 
			       Vector x, 
			       Vector y, 
			       double w_misc,
			       int flag, 
			       int *rfID, 
			       Vector y_pred, 
			       Vector conf)
{
  int i,rc;
  double aux=0;

  rc = addDataToLWPRReg2nd(ID,x_w,x,y,w_misc,flag,rfID);

  for (i=1; i<=lwprs[ID].n_out_2nd; ++i) {
    y_pred[i] = lwprs[ID].sum_y_predicted_2nd[i];
    conf[i]   = lwprs[ID].sum_y_conf_2nd[i];
    aux += y_pred[i];
  }

  if (aux != 0.0)
    return rc;
  else
    return FALSE;

}

int 
addDataToLWPRReg2ndPredict(int ID, 
			   Vector x_w, 
			   Vector x, 
			   Vector y, 
			   double w_misc,
			   int flag, 
			   int *rfID, 
			   Vector y_pred)
{
  int i,rc;
  double aux=0;

  rc = addDataToLWPRReg2nd(ID,x_w, x,y,w_misc,flag,rfID);

  for (i=1; i<=lwprs[ID].n_out_2nd; ++i) {
    y_pred[i] = lwprs[ID].sum_y_predicted_2nd[i];
    aux += y_pred[i];
  }

  if (aux != 0.0)
    return rc;
  else
    return FALSE;

}

int 
addDataToLWPRReg2nd(int ID, 
		    Vector x_w, 
		    Vector x, 
		    Vector y, 
		    double w_misc,
		    int flag, 
		    int *rfID)
{
  int      i,j,k,r;
  double   max_activation=0;
  int      index_max;
  double   activation;
  int      rfIDaux;
  double   aux,aux1;
  LWPR     *s;
  char     stmp[100];
  double   dofs;

  static int count_updates = -1;

  s = &lwprs[ID];

  /**********************************************/
  /* normalize the inputs and outputs           */
  /**********************************************/

  for (i=1; i<=s->n_in_reg_2nd; ++i) {
    s->x_2nd[i] = x[i]/s->norm_in_reg_2nd[i];
  }

  for (i=1; i<=s->n_in_w; ++i) {
    s->x_w[i] = x_w[i]/s->norm_in_w[i];
  }

  for (i=1; i<=s->n_out_2nd; ++i) {
    s->y_2nd[i] = y[i]/s->norm_out_2nd[i];
  }

  /*****************************************************************/
  /* Run through all receptive fields and update their             */
  /* parameters appropriately                                      */
  /*****************************************************************/
     
  sum_activation      = 0;
  max_activation      = 0;
  index_max           = 0;
  s->mean_n_proj_2nd  = 0.0;
  s->contrib_n_rf     = 0.0;
  
  for (i=1; i<=s->n_out_2nd; ++i) {
    s->sum_y_predicted_2nd[i]  =0.0;
    s->sum_y_predicted2_2nd[i] =0.0;
    s->sum_y_conf_2nd[i]       =0.0;
  }

  for (r=1; r<=s->n_rf; ++r) {

    /* zero the weight to recognize which RF was updated */
    s->rfs[i].w = 0;

    /* fudge in the nearest neighbor add */
    if (s->use_nn && *rfID > 0 && *rfID <= s->n_rf) {
      if ( (r > s->max_nn_comp && max_activation > s->w_gen) || 
	   r > s->rfs[*rfID].n_nn)
	break;
      i = s->rfs[*rfID].nn[r];
    } else {
      i = r;
    }

    s->mean_n_proj_2nd  += (double) s->rfs[i].n_proj_2nd;

    /* get the activation weight */
    activation = s->rfs[i].w = calculateWeight(ID,i,s->x_w);
 
    /* should we update this local model ? */
    if (activation < ACTIVATION_THRESHOLD) {
      s->rfs[i].w = 0;
      continue;
    } 

    /* yes, we are going to update now */

   /* update the mean and variance, return the mean subtracted input for
      regression */
    updateMeanVar2nd(ID,i,s->x_2nd,s->x_mz_2nd);

    if (DEBUG_FLAG) {
      printf("w=%f\n",activation);
      print_vec("x_mz_2nd",s->x_mz_2nd);
    }

    /* create a fake training point if error-based learning is on */
    createFakeTarget2nd(ID,i,s->x_mz_2nd,s->y_2nd,flag,s->y_mz_2nd);
    
    /* update the mean and variance of the outputs, return the mean 
       subtracted output */
    updateRegMeanVar2nd(ID,i,s->y_mz_2nd,s->y_mz_2nd);


    if (DEBUG_FLAG) {
      print_vec("y_mz_2nd",s->y_mz_2nd);
      getchar();
    }

    /* calculate regression update */
    calculateRegression2nd(ID,i,s->x_mz_2nd,s->y_mz_2nd);
    
    /* update adminstrative variables */
    updateAdmVars2nd(ID,i);

    /* accumulate the sum of predictions */
    if (s->rfs[i].trustworthy) {
      sum_activation += activation;
      for (j=1; j<=s->n_out; ++j) {
	s->sum_y_predicted_2nd[j]  += s->ypred_2nd[j]*activation;
	s->sum_y_predicted2_2nd[j] += sqr(s->ypred_2nd[j])*activation;
	dofs = s->rfs[i].sum_weights_2nd[1];
	aux  = 1;
	for (k=1; k<=s->rfs[i].n_proj_2nd; ++k) {
	  dofs -= s->rfs[i].dof_2nd[k];
	  aux  += sqr(s->rfs[i].s_2nd[k])/s->rfs[i].ss2_2nd[k]*activation;
	}
	if (dofs < TINY_NUMBER)
	  dofs = TINY_NUMBER;
	s->sum_y_conf_2nd[j] += activation*s->rfs[i].sum_error_2nd[s->rfs[i].n_proj_2nd]/
	  dofs*aux;
      }
    } else {
      /* indicate that this RF did not contribute to predictions */
      s->rfs[i].w = 0;
    }

    /* check if another dimension has to be added */
    if (s->allow_addDim) {
      if (checkAddDim2nd(ID,i)){
	addDimension2nd(ID,i);
      }
    }

    /* I have to keep track of the largestactivation */
    
    if (activation > max_activation) {
      max_activation = activation;
      index_max      = i;
    }

 
  }

  /* finish some statistics */

  --r;
  if (r > 0) {
    s->mean_n_proj_2nd  /= (double) r;
  }
  s->contrib_n_rf = r;

  /* this is the current prediction for the newly added data point,
     AFTER the RFs have been updated */

  if (sum_activation > 0) {
    for (i=1; i<=s->n_out_2nd; ++i) {
      /* first computations without taking the output normalization into account */
      s->sum_y_predicted_2nd[i] = 
	s->sum_y_predicted_2nd[i]/sum_activation;
      aux = fabs(s->sum_y_predicted2_2nd[i]/sum_activation - 
	     sqr(s->sum_y_predicted_2nd[i]))/sum_activation;
      s->sum_y_conf_2nd[i] = sqrt(s->sum_y_conf_2nd[i]/sqr(sum_activation)+aux);

      /* now add the correction for output normalization */
      s->sum_y_predicted_2nd[i] *= s->norm_out_2nd[i];
      s->sum_y_conf_2nd[i] *= s->norm_out_2nd[i];

    }
  }

  *rfID = index_max;

  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  updateMeanVar2nd
\date  Sept, 1996

\remarks 

       update the weighted mean and variance of the 2nd model of 
       an RF and return a centered input vector for regression
            
	
       NOTE: the weight w must be assigned in the RF structure already

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     ID    : lwpr ID
 \param[in]     rfID  : the rfID
 \param[in]     x     : the input vector for regression
 \param[out]    xmz   : x-mean_x

 ******************************************************************************/
static void
updateMeanVar2nd(int ID, int rfID, Vector x, Vector xmz)
{

  int    i,j,k;
  RF    *rfptr;
  int    n_in_reg_2nd;
  double w, lambda_2nd;

  /* handy variables */

  rfptr         = &lwprs[ID].rfs[rfID];
  w             = rfptr->w;
  lambda_2nd    = LAMBDA(rfptr->lambda_2nd[1],w);
  n_in_reg_2nd  = lwprs[ID].n_in_reg_2nd;
  
  for (i=1; i<=n_in_reg_2nd; ++i) {

    if (lwprs[ID].use_offsets_2nd) {
      rfptr->mean_x_reg_2nd[i] = 
	(rfptr->sum_weights_2nd[1] * rfptr->mean_x_reg_2nd[i] * lambda_2nd + x[i]*w) /
	(rfptr->sum_weights_2nd[1] *lambda_2nd + w);
    }
      
    rfptr->var_x_reg_2nd[i] = 
      (rfptr->sum_weights_2nd[1] * rfptr->var_x_reg_2nd[i] * lambda_2nd  + 
		   sqr(x[i]-rfptr->mean_x_reg_2nd[i])*w)/
      (rfptr->sum_weights_2nd[1]  * lambda_2nd + w);

    xmz[i] = (x[i] - rfptr->mean_x_reg_2nd[i]);

  }

}

/*!*****************************************************************************
 *******************************************************************************
 \note  createFakeTarget2nd
 \date  June 1999
 
 \remarks 
 
 creates a fake target form an error based learning input for the second 
 model
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ID     : the ID of the LWPR
 \param[in]     rfID   : the receptive field ID
 \param[in]     xmz    : mean zero input
 \param[in]     err    : error
 \param[in]     flag   : TRUE if this is an error
 \param[out]    yfake  : faked output

 this function assumes that various quantities have been pre-computed
 
 ******************************************************************************/
static void
createFakeTarget2nd(int ID,int rfID, Vector xmz, Vector err,int flag, 
		    Vector yfake)
     
{

  int     j,k,n;
  int     n_in_reg_2nd;
  int     n_proj_2nd;
  int     n_out_2nd;
  RF     *rfptr;
  LWPR    *s;
  double  x_proj;
  double  w;

  s             = &lwprs[ID];
  n_in_reg_2nd  = s->n_in_reg_2nd;
  n_out_2nd     = s->n_out_2nd;

  rfptr      = &s->rfs[rfID];
  n_proj_2nd = rfptr->n_proj_2nd;
  
  w = rfptr->w;

  /* the constant part */
  if (!flag) {
    for (j=1; j<=n_out_2nd; ++j) 
      yfake[j] = err[j];
    return;
  }

  for (j=1; j<=n_out_2nd; ++j) 
    yfake[j] = rfptr->mean_y_2nd[j]+err[j];
  
  for (j=1; j<=n_in_reg_2nd; ++j) {
#if SPHERING
    s->x_temp_2nd[j] = xmz[j]/(sqrt(rfptr->var_x_reg_2nd[j])+TINY_NUMBER);
#else
    s->x_temp_2nd[j] = xmz[j];
#endif
  }

  for (k=1; k<=n_proj_2nd; ++k) {
    
    /* project the input onto the current projection */
    x_proj = vec_mult_inner_size(s->x_temp_2nd,rfptr->W_2nd[k],n_in_reg_2nd);
    
    /* do single variate prediction */
    for (j=1; j<=n_out_2nd; ++j) {
      yfake[j] += x_proj * rfptr->B_2nd[k][j];
    }
    
    /* adjust the input data */
    for (j=1; j<=n_in_reg_2nd; ++j) {
      s->x_temp_2nd[j] -= x_proj * rfptr->U_2nd[k][j];
    }
    
  }

  return;

}

/*!*****************************************************************************
 *******************************************************************************
\note  updateRegMeanVar2nd
\date  Sept, 1996

\remarks 

       update the weighted mean and variance of a RF for the 
       dependent variables for the 2nd model
	
       NOTE: the weight w must be assigned in the RF structure already

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     ID    : lwpr ID
 \param[in]     rfID  : the rfID
 \param[in]     y     : the output vector
 \param[out]    ymz   : y-mean_y

 ******************************************************************************/
static void
updateRegMeanVar2nd(int ID, int rfID, Vector y, Vector ymz)

{

  int    i,j,k;
  RF    *rfptr;
  int    n_out_2nd;
  double w;
  double lambda_2nd;

  /* handy variables */

  rfptr     = &lwprs[ID].rfs[rfID];
  n_out_2nd = lwprs[ID].n_out_2nd;
  w         = rfptr->w;
  lambda_2nd= LAMBDA(rfptr->lambda_2nd[1],w);
  
  for (i=1; i<=n_out_2nd; ++i) {

    if (lwprs[ID].use_offsets_2nd) {
      rfptr->mean_y_2nd[i] = 
	(rfptr->sum_weights_2nd[1] * rfptr->mean_y_2nd[i] * lambda_2nd + y[i]*w) /
	(rfptr->sum_weights_2nd[1] * lambda_2nd + w);
    }

    rfptr->var_y_2nd[i] = 
      (rfptr->sum_weights_2nd[1] * rfptr->var_y_2nd[i] * lambda_2nd  + 
		   sqr(y[i]-rfptr->mean_y_2nd[i])*w)/
      (rfptr->sum_weights_2nd[1]  * lambda_2nd + w);

    ymz[i] = (y[i] - rfptr->mean_y_2nd[i]);

  }


}


/*!*****************************************************************************
 *******************************************************************************
 \note  calculateRegression2nd
 \date  April 1999
 
 \remarks 
 
 recursively calculates the regressions in all directions of the 2nd model
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ID       : the ID of the LWPR
 \param[in]     rfID     : the receptive field ID
 \param[in]     x_mz : the input vector (zero mean)
 \param[in]     y_mz     : the output vector (mean zero)
 
 ******************************************************************************/
static void
calculateRegression2nd(int ID, int rfID, Vector x_mz, Vector y_mz)

{
  RF       *rfptr;
  int       n_in_reg_2nd, n_out_2nd;
  int       i, j, k;
  int       n_proj_2nd;
  double    aux;
  double    sum_val;
  double   *xres_2nd;
  double   *yres_2nd;
  double    s;
  double   *ypred_2nd;
  double    w;
  double    y_partial;
  double  **error_cv_2nd;
  double  **error_2nd;
  double   *xproj_2nd;
  Vector    cr_2nd;

  rfptr        = &lwprs[ID].rfs[rfID];
  n_in_reg_2nd =  lwprs[ID].n_in_reg_2nd;
  n_out_2nd    =  lwprs[ID].n_out_2nd;
  n_proj_2nd   =  rfptr->n_proj_2nd;
  xres_2nd     =  lwprs[ID].xres_2nd;
  yres_2nd     =  lwprs[ID].yres_2nd;
  w            =  rfptr->w;
  ypred_2nd    =  lwprs[ID].ypred_2nd;
  error_cv_2nd =  lwprs[ID].error_cv_2nd;
  error_2nd    =  lwprs[ID].error_2nd;
  xproj_2nd    =  rfptr->s_2nd;
  cr_2nd       =  lwprs[ID].cr_2nd;

  /* prepare residual variables */

  for (i=1; i<=n_in_reg_2nd; ++i)
#if SPHERING
    xres_2nd[i] = x_mz[i]/(sqrt(rfptr->var_x_reg_2nd[i])+TINY_NUMBER);
#else
    xres_2nd[i] = x_mz[i];
#endif
  for (i=1; i<=n_out_2nd; ++i) {
    yres_2nd[i]  = y_mz[i];
    ypred_2nd[i] = rfptr->mean_y_2nd[i];
  }

  /* loop over all regression outputs, find the projections, and update
     the regression parameters */
  for (i=1; i<=n_proj_2nd; ++i) {

    /* project the input onto the projection vector *before* updating the
       projections: this is like "synchronous" update for PLS while 
       classical PLS is asynchonous -- but synchronous updates keep
       cross validation for PLS cleaner */
    s=0.0;
    for (j=1; j<=n_in_reg_2nd; ++j)
      s += rfptr->W_2nd[i][j]*xres_2nd[j];

    /* update the projection direction; NOTE: this is not the proper
       version for multivariate outputs, but rather a quick and dirty
       version. Multivariate outputs are not recommended in any of the 
       LWPR methods, and in particular not in PLS */

    sum_val = 0.0;
    for (j=1; j<=n_in_reg_2nd; ++j) {
      cr_2nd[j] = 0.0;
      for (k=1; k<=n_out_2nd; ++k) {
	cr_2nd[j] += xres_2nd[j]*yres_2nd[k];
      }
      /* NOTE: for nonstationary learning problems, it turns out to be
	 useful if the projections change a bit more slowly than the
	 regression -- thus the weird formula below for lambda */
      rfptr->SXresYres_2nd[i][j] = 
	LAMBDA((1.-(1.-rfptr->lambda_2nd[i])/10.),w)*rfptr->SXresYres_2nd[i][j] + w*cr_2nd[j];
      sum_val += sqr(rfptr->SXresYres_2nd[i][j]);
    }
    sum_val = sqrt(sum_val)+TINY_NUMBER;
    rfptr->Wnorm_2nd[i] = sum_val;

    /* normalize this direction */
    for (j=1; j<=n_in_reg_2nd; ++j)
      rfptr->W_2nd[i][j] = rfptr->SXresYres_2nd[i][j]/sum_val;

    /* update sufficient statistics for regression based on this projection */
    rfptr->ss2_2nd[i] = LAMBDA(rfptr->lambda_2nd[i],w) * rfptr->ss2_2nd[i] + sqr(s) * w;
    for (j=1; j<=n_out_2nd; ++j) {
      rfptr->SSYres_2nd[i][j] = LAMBDA(rfptr->lambda_2nd[i],w) * rfptr->SSYres_2nd[i][j] +  
	w * yres_2nd[j] * s;
    }

    /* update the regression parameters for this dimension and predict
       the outputs */
    aux = 0;
    for (j=1; j<=n_out_2nd; ++j) {

      /* the cross validation error at this regression stage */
      y_partial          = rfptr->B_2nd[i][j]*s;
      error_cv_2nd[i][j] = yres_2nd[j] - y_partial;
      ypred_2nd[j]      += y_partial;
      yres_2nd[j]       -= y_partial;
      aux               += sqr(error_cv_2nd[i][j]);

      /* update the local model */
      rfptr->B_2nd[i][j] = rfptr->SSYres_2nd[i][j]/rfptr->ss2_2nd[i];


    }
    aux /= (double)n_out_2nd;  /* the mean squared cv error accross all outputs */

    /* update the sum_error term: this is a sum of CV terms */
    rfptr->sum_error_cvi_2nd[i]   = rfptr->sum_error_cvi_2nd[i] * 
      LAMBDA(rfptr->lambda_2nd[i],w) + w * aux;

    /* the total CV error of the RF needs to be accumulated separately as
       individual projections have different life times */
    if (i==n_proj_2nd)
      rfptr->sum_error_cv_2nd   = rfptr->sum_error_cv_2nd * LAMBDA(rfptr->lambda_2nd[1],w) 
	+ w * aux;

    /* the regression parameters on the inputs variables and
       prepare the variables for the next iteration */
    for (j=1; j<=n_in_reg_2nd; ++j) {
      rfptr->SSXres_2nd[i][j] = LAMBDA(rfptr->lambda_2nd[i],w) * rfptr->SSXres_2nd[i][j] +  
	w * xres_2nd[j] * s;
      xres_2nd[j] -= rfptr->U_2nd[i][j]*s;
      rfptr->U_2nd[i][j] = rfptr->SSXres_2nd[i][j]/rfptr->ss2_2nd[i];
    }
    
  }

  /* finally calulate the predicted regression output *after* updating */
  for (i=1; i<=n_out_2nd; ++i) {
    yres_2nd[i]  = y_mz[i];
    ypred_2nd[i] = rfptr->mean_y_2nd[i];
  }
  for (i=1; i<=n_in_reg_2nd; ++i)
#if SPHERING
    xres_2nd[i] = x_mz[i]/(sqrt(rfptr->var_x_reg_2nd[i])+TINY_NUMBER);
#else
    xres_2nd[i] = x_mz[i];
#endif

  for (i=1; i<=n_proj_2nd; ++i) {

    s=0.0;
    for (j=1; j<=n_in_reg_2nd; ++j)
      s += rfptr->W_2nd[i][j]*xres_2nd[j];

    xproj_2nd[i] = s;

    /* update the local degree of freedoms */
    rfptr->dof_2nd[i] = LAMBDA(rfptr->lambda_2nd[i],w) * rfptr->dof_2nd[i] + 
      sqr(w)*sqr(s)/rfptr->ss2_2nd[i];

    /* compute the predicted output */
    aux = 0;
    for (j=1; j<=n_out_2nd; ++j) {
      /* the non-CV error at this regression stage */
      y_partial   = rfptr->B_2nd[i][j]*s;
      error_2nd[i][j] = yres_2nd[j] - y_partial;
      ypred_2nd[j]   += y_partial;
      yres_2nd[j]    -= y_partial;
      aux        += sqr(error_2nd[i][j]);
    }
    aux /= (double)n_out_2nd;  /* the mean squared error accross all outputs */

    /* update the sum_error term: this is a sum of NON-CV terms */
    rfptr->sum_error_2nd[i]   = rfptr->sum_error_2nd[i] * LAMBDA(rfptr->lambda_2nd[i],w) 
      + w * aux;

    /* update the residual input for the next iteration */
    for (j=1; j<=n_in_reg_2nd; ++j) {
      xres_2nd[j] -= rfptr->U_2nd[i][j]*s;
    }

  }

  if (rfptr->n_data_2nd[1] > 2*n_in_reg_2nd)
    rfptr->trustworthy_2nd = TRUE; 


}

/*!*****************************************************************************
 *******************************************************************************
 \note  predictLWPROutput2nd
 \date  January, 2006
 
 \remarks 
 
 generates a prediction from a given input for the 2nd regression model
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ID     : the ID of the LWPR
 \param[in]     x_w    : the input vector for weights
 \param[in]     x      : the input vector for regression
 \param[in]     cutoff : a number between [0,1] which indicates at which threshold
             a receptive field is not used for averaging anymore
 \param[in]     blend  : blend predictions or use max only 
 \param[out]    y      : the output vector
 \param[in,out] rfID   : the rf for the neighborhood for input, returns the max activated rf 

 returns the max activation contributing to this output
 
 ******************************************************************************/
double
predictLWPROutputConf2nd(int ID, double *x_w, double *x, double cutoff, 
			 int blend, double *y, int *rfID, double *conf)
{
  int i;
  double rc;

  rc = predictLWPROutput2nd(ID, x_w, x, cutoff, blend, y, rfID);
  for (i=1; i<=lwprs[ID].n_out_2nd; ++i)
    conf[i] = lwprs[ID].y_conf_2nd[i];
  
  return rc;

}

double
predictLWPROutput2nd(int ID, double *x_w, double *x, double cutoff, 
		     int blend, double *y, int *rfID)
     
{
  
  int     i,j,k,n,r;
  double  max_activation = 0;
  double  sum_w = 0;
  double  w=0.;
  int     n_in_w;
  int     n_in_reg_2nd;
  int     n_proj_2nd;
  int     n_rf;
  int     n_out_2nd;
  int     index_max= -999;
  RF     *rfptr;
  int     count_rf=0;
  LWPR    *s;
  double  x_proj;
  Vector  xres_2nd;
  double  aux;
  double  dofs_2nd;

  if (!checkLWPRExists(ID)) 
    return FALSE;

  s             = &lwprs[ID];
  n_in_w        = s->n_in_w;
  n_in_reg_2nd  = s->n_in_reg_2nd;
  n_out_2nd     = s->n_out_2nd;
  n_rf          = s->n_rf;
  xres_2nd      = s->x_tempp_2nd;

  /* normalize the query point */
  for (j=1; j<=n_in_reg_2nd; ++j) {
    s->xp_2nd[j] = x[j]/s->norm_in_reg_2nd[j];
  }

  for (j=1; j<=n_in_w; ++j) {
    s->xp_w[j] = x_w[j]/s->norm_in_w[j];
  }

  /* reset the y[j] here since we sum over them */
  for (j=1; j<=n_out_2nd; ++j) {
    y[j] = 0.0;
    s->y_predicted_2nd[j]  = 0.0;
    s->y_predicted2_2nd[j] = 0.0;
    s->y_conf_2nd[j] = 0.0;
  }

  /* if there are no receptive fields */
  if (n_rf == 0)
    return max_activation;
  
  /* scan the receptive fields */

  for (r=1; r<=n_rf; ++r) {

    /* fudge in the nearest neighbor lookup */
    if (s->use_nn && *rfID > 0 && *rfID <= n_rf) {
      if ( (r > s->max_nn_comp && max_activation > cutoff) || 
	   r > s->rfs[*rfID].n_nn)
	break;
      i = s->rfs[*rfID].nn[r];
    } else {
      i = r;
    }
  
    rfptr      = &s->rfs[i];
    n_proj_2nd = rfptr->n_proj_2nd;

    if (!rfptr->trustworthy) continue;
    
    /* the mean subtracted input  */
    for (j=1; j<=n_in_reg_2nd; ++j) {
      s->x_mzp_2nd[j] = s->xp_2nd[j] - rfptr->mean_x_reg_2nd[j];
    }
    
    /* generate the weight for this RF */
    rfptr->ww = w = calculateWeightp(ID,i,s->xp_w);

    if (w > max_activation) {
      max_activation = w;
      index_max = i;
    }

    if (w > cutoff && blend) {

      for (j=1; j<=n_in_reg_2nd; ++j) {
#if SPHERING
	xres_2nd[j] = s->x_mzp_2nd[j]/(sqrt(rfptr->var_x_reg_2nd[j])+TINY_NUMBER);
#else
	xres_2nd[j] = s->x_mzp_2nd[j];
#endif
      }

      /* add the constant part */
      for (j=1; j<=n_out_2nd; ++j) {
	aux = rfptr->mean_y_2nd[j];
	s->y_predicted_2nd[j] = aux; 
	y[j] += aux * w;
      }


      for (k=1; k<=n_proj_2nd; ++k) {

	/* project the input onto the current projection */
	x_proj = vec_mult_inner_size(xres_2nd,rfptr->W_2nd[k],n_in_reg_2nd);
	
	/* do single variate prediction */
	for (j=1; j<=n_out_2nd; ++j) {
	  aux   = x_proj * rfptr->B_2nd[k][j];
	  y[j] += aux * w;
	  s->y_predicted_2nd[j] += aux;
	}
	
	/* adjust the input data */
	for (j=1; j<=n_in_reg_2nd; ++j) {
	  xres_2nd[j] -= x_proj * rfptr->U_2nd[k][j];
	}

      }
      
      /* prepare summary statistics and confidence intervals */
      sum_w += w;
      for (j=1; j<=n_out_2nd; ++j) {
	s->y_predicted2_2nd[j] += sqr(s->y_predicted_2nd[j]) * w;
	dofs_2nd = rfptr->sum_weights_2nd[1];
	aux  = 1;
	for (k=1; k<=n_proj_2nd; ++k) {
	  dofs_2nd -= rfptr->dof_2nd[k];
	  aux  += sqr(rfptr->s_2nd[k])/rfptr->ss2_2nd[k] * w;
	}
	if (dofs_2nd < TINY_NUMBER)
	  dofs_2nd = TINY_NUMBER;
	s->y_conf_2nd[j] += w * rfptr->sum_error_2nd[n_proj_2nd]/dofs_2nd*aux;
      }
      ++count_rf;

    }

  }


  /* finish up */

  if (blend) {

    if (sum_w < TINY_NUMBER) 
      sum_w = TINY_NUMBER;

    for (i=1; i<=n_out_2nd; ++i) {
      y[i] = y[i]/sum_w;
      aux = fabs(s->y_predicted2_2nd[i]/sum_w - sqr(y[i]))/sum_w;
      s->y_conf_2nd[i] = sqrt(s->y_conf_2nd[i]/sqr(sum_w)+aux);

      /* add output normalization */
      y[i] *= s->norm_out_2nd[i];
      s->y_conf_2nd[i] *= s->norm_out_2nd[i];
    }
    
  } else {
    
    /* predict y from the max activated rf */
    rfptr = &s->rfs[index_max];
    n_proj_2nd = rfptr->n_proj_2nd;

    for (j=1; j<=n_in_reg_2nd; ++j) {
      s->x_mzp_2nd[j] = s->xp_2nd[j] - rfptr->mean_x_reg_2nd[j];
#if SPHERING
      xres_2nd[j] = s->x_mzp_2nd[j]/(sqrt(rfptr->var_x_reg_2nd[j])+TINY_NUMBER);
#else
      xres_2nd[j] = s->x_mzp_2nd[j];
#endif
    }

    for (k=1; k<=n_proj_2nd; ++k) {
      
      /* project the input onto the current projection */
      x_proj = vec_mult_inner_size(xres_2nd,rfptr->W_2nd[k],n_in_reg_2nd);
      
      /* do single variate prediction */
      for (j=1; j<=n_out_2nd; ++j) {
	y[j] += x_proj * rfptr->B_2nd[k][j];
      }
      
      /* adjust the input data */
      for (j=1; j<=n_in_reg_2nd; ++j) {
	xres_2nd[j] -= x_proj * rfptr->U_2nd[k][j];
      }
    
    }
    
    /* add the constant part and compute confidence interval */
    for (j=1; j<=n_out_2nd; ++j) {
      y[j] = (y[j] + rfptr->mean_y_2nd[j])*s->norm_out_2nd[j]; 

      dofs_2nd = rfptr->sum_weights_2nd[1];
      aux  = 1;
      for (k=1; k<=n_proj_2nd; ++k) {
	dofs_2nd -= rfptr->dof_2nd[k];
	aux  += sqr(rfptr->s_2nd[k])/rfptr->ss2_2nd[k] * w;
      }
      if (dofs_2nd < TINY_NUMBER) 
	dofs_2nd = TINY_NUMBER;
      s->y_conf_2nd[j] = rfptr->sum_error_2nd[n_proj_2nd]/dofs_2nd*aux/w;
    }

    if (!s->rfs[index_max].trustworthy_2nd) max_activation = 0;
    
  }

  *rfID = index_max;

  return max_activation;

}

/*!*****************************************************************************
 *******************************************************************************
\note  updateAdmVars2nd
\date  April 1999

\remarks 

       updates several adminstrative variables for a receptive field for
       the 2nd model.
       This function should be called last since various other update
       function require the non-updated adminstrative variables.

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     ID        : lwpr ID
 \param[in]     rfID      : the rfID

 ******************************************************************************/
static void
updateAdmVars2nd(int ID, int rfID)

{

  int    i;
  RF    *rfptr;

  /* handy variables */
  rfptr     = &lwprs[ID].rfs[rfID];

  /* several trace values */
  for (i=1; i<=rfptr->n_proj_2nd; ++i) {

    rfptr->sum_weights_2nd[i] = LAMBDA(rfptr->lambda_2nd[i],rfptr->w)*
      rfptr->sum_weights_2nd[i] + rfptr->w;
    rfptr->n_data_2nd[i]      = rfptr->lambda_2nd[i] * rfptr->n_data_2nd[i] + 1;

    /* and finally update the forgetting factor lambda itself */
    if (lwprs[ID].final_lambda - rfptr->lambda_2nd[i] > TINY_NUMBER) {
      rfptr->lambda_2nd[i] = lwprs[ID].tau_lambda * rfptr->lambda_2nd[i] + 
	lwprs[ID].final_lambda*(1.-lwprs[ID].tau_lambda);
      
    }
  }


}

/*!*****************************************************************************
 *******************************************************************************
 \note  checkAddDim2nd
 \date  May 1999
 
 \remarks 
 
 checks whether to add dimensions to the regression of 2nd model
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ID     : the ID of the LWPR
 \param[in]     rfID   : the ID of the receptive field

 returns TRUE if dimension should be increased else returns FALSE
 
 ******************************************************************************/
static int
checkAddDim2nd(int ID, int rfID)

{

  int    i,j,m;
  RF    *rfptr;
  int    n_in_w;
  int    n_in_reg_2nd;
  int    n_out_2nd;
  int    n_proj_2nd;
  double mse_n_proj;
  double mse_n_proj_1;

  n_in_w        = lwprs[ID].n_in_w;
  n_in_reg_2nd  = lwprs[ID].n_in_reg_2nd;
  rfptr         = &lwprs[ID].rfs[rfID];
  n_proj_2nd    = rfptr->n_proj_2nd;

  if (n_proj_2nd >= n_in_reg_2nd)
    return FALSE;

  mse_n_proj   = rfptr->sum_error_cvi_2nd[n_proj_2nd]  /rfptr->sum_weights_2nd[n_proj_2nd];
  mse_n_proj_1 = rfptr->sum_error_cvi_2nd[n_proj_2nd-1]/rfptr->sum_weights_2nd[n_proj_2nd-1];
 
  if (mse_n_proj/mse_n_proj_1 < lwprs[ID].add_threshold && 
      rfptr->n_data_2nd[n_proj_2nd]/rfptr->n_data_2nd[1] > 0.99 && 
      rfptr->n_data_2nd[n_proj_2nd]*(1.-rfptr->lambda_2nd[n_proj_2nd]) > 0.5) {

    return TRUE;
  }

  return FALSE;
     
}

/*!*****************************************************************************
 *******************************************************************************
 \note  addDimension2nd
 \date  January, 1994
 
 \remarks 
 
 add a regression dimensions to the 2nd model of the current receptive field 
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ID     : the ID of the LWPR
 \param[in]     rfID   : the ID of the receptive field
 
 ******************************************************************************/
static int
addDimension2nd(int ID, int rfID)

{
  int    i,j,m;
  RF    *rfptr;
  int    n_in_w;
  int    n_in_reg_2nd;
  int    n_proj_2nd;
  int    n_out_2nd;

  
  n_out_2nd      = lwprs[ID].n_out_2nd;
  n_in_w         = lwprs[ID].n_in_w;
  n_in_reg_2nd   = lwprs[ID].n_in_reg_2nd;
  rfptr          = &lwprs[ID].rfs[rfID];
  n_proj_2nd     = rfptr->n_proj_2nd;

  if (n_proj_2nd == n_in_reg_2nd) {
    if (DEBUG_FLAG) printf("Max.dimensions reached\n");
    return FALSE;
  }

  mat_add_shape(&(rfptr->B_2nd),1,0);
  vec_add_shape(&(rfptr->ss2_2nd),1);
  rfptr->ss2_2nd[n_proj_2nd+1]=lwprs[ID].init_S2;
  vec_add_shape(&(rfptr->dof_2nd),1);
  mat_add_shape(&(rfptr->SSYres_2nd),1,0);
  mat_add_shape(&(rfptr->SSXres_2nd),1,0);
  mat_add_shape(&(rfptr->SXresYres_2nd),1,0);

  mat_add_shape(&(rfptr->W_2nd),1,0);
  rfptr->W_2nd[n_proj_2nd+1][n_proj_2nd+1]=1.0;
  vec_add_shape(&(rfptr->Wnorm_2nd),1);

  mat_add_shape(&(rfptr->U_2nd),1,0);

  vec_add_shape(&(rfptr->sum_weights_2nd),1);
  rfptr->sum_weights_2nd[n_proj_2nd+1] = TINY_NUMBER;
  vec_add_shape(&(rfptr->sum_error_2nd),1);
  vec_add_shape(&(rfptr->sum_error_cvi_2nd),1);
  vec_add_shape(&(rfptr->n_data_2nd),1);
  vec_add_shape(&(rfptr->lambda_2nd),1);
  rfptr->lambda_2nd[n_proj_2nd+1]=lwprs[ID].init_lambda;
  vec_add_shape(&(rfptr->s_2nd),1);
  
  ++rfptr->n_proj_2nd;

  return TRUE;

}

