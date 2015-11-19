/*!=============================================================================
  ==============================================================================

  \file    lwpr_script.c

  \author  Stefan Schaal
  \date    April 2006

  ==============================================================================
  \remarks
  
  Generates a LWPR model from parameters specified in a script file.
  
  ============================================================================*/

#include "stdio.h"
#include "math.h"
#include "utility.h"
#include "lwpr.h" 
#include "string.h"

/* functions */

/*!*****************************************************************************
 *******************************************************************************
 \note  readLWPRScriptMulti
 \date  February 1996
 
 \remarks 
 
 Reads multiple LWPR models using only one script file, i.e., assuming that
 the models share all the parameters except for the name. 
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     fname   : name of the script
 \param[in]     new_flag: TRUE = force the creation of a new LWPR
              FALSE = try to read the LWPR from file first
 \param[in]     IDs     : array of which ID number should be given to the lwprs
 \param[in]     names   : names of the lwpr models (pass NULL to use lwpr_name from script
              file -- but this is only useful for initializing one model)
 \param[in]     n_models: number of models to be initialized
 
 ******************************************************************************/
int
readLWPRScriptMulti(char *fname, int new_flag, int *IDs, char **names, int n_models)
     
{ 
  
  int      i,j,k,m,rc=TRUE;
  FILE    *in,*out;
  char     identification_string[100];
  char     string[100];
  char     vnames[50][100];
  int      ans;
  int      changerate=0;
  double   d_noise, c_noise;
  int      n_in_w=0;
  int      n_in_reg=0;
  int      n_in_reg_2nd=0;
  int      n_out;
  int      n_out_2nd=0;
  int      diag_only;
  int      allow_meta_learning;
  double   meta_learning_rate;
  double   penalty;
  double **init_D;
  double   init_alpha;
  double  *norm_in_w=NULL;
  double  *norm_in_reg=NULL;
  double  *norm_in_reg_2nd=NULL;
  double  *norm_out;
  double  *norm_out_2nd=NULL;
  double   w_gen;
  double   w_prune;
  double   factor_prune;
  double   init_lambda;
  double   final_lambda;
  double   tau_lambda;
  double   init_S2;
  char     lwpr_name[100];
  double   add_threshold;
  int      init_n_proj;
  int      init_n_proj_2nd;
  int      use_nn;
  int      max_nn_comp;
  int      composite_control;
  int      n_states;
  int      n_controls;
  int      linear_gx;
  int      use_offsets;
  int      use_offsets_2nd;
  int      kernel_function;
  int      flag_2nd = FALSE;
  
  long     max_rfs;
  
  /* debugging variables */

  int      allow_addDim;
  int      allow_add_rfs;
  int      allow_d_update;
  int      write_rfs;


  /* open the script file */

  in  = fopen_strip(fname);
  if (in == NULL) {
    printf("\nCouldn't read LWPR script file >%s<\n",fname);
    return FALSE;
  }

  /* check whether this is really a valid script file */
  
  rc=fscanf(in,"%s",identification_string);
  if (strcmp("#LWPR",identification_string) != 0) {
    fprintf(stderr,"\nError: Invalid script file!\n");
    fprintf(stderr,"A valid script file starts with >#LWPR<\n");
    return FALSE;
  }

  /* check for included files */
  if (find_keyword(in,"include")) {  
    char  fname_include[100];
    FILE *fp_include;

    rc=fscanf(in,"%s",fname_include);
    fp_include = fopen_strip(fname_include);
    fseek(fp_include, 0, SEEK_END);
    rewind(in);
    while ((rc=fgetc(in)) != EOF)
      fputc(rc,fp_include);

    fclose(in);
    in = fp_include;

    rewind(in);
  }
    

  /* All the names in file will be parsed. Here I define the names of the
     variables. Note that the variables defining the dimensionality of
     the LWPR must come first since we need them to allocate other 
     variables */


  i=0;

  /* this block has all variables needed to create a LWPR */
  sprintf(vnames[++i],"n_in_w");
  sprintf(vnames[++i],"n_in_reg");
  sprintf(vnames[++i],"n_out");
  sprintf(vnames[++i],"diag_only");
  sprintf(vnames[++i],"allow_meta_learning");
  sprintf(vnames[++i],"meta_learning_rate ");
  sprintf(vnames[++i],"penalty");
  sprintf(vnames[++i],"init_all_alpha ");
  sprintf(vnames[++i],"norm_in_w");
  sprintf(vnames[++i],"norm_in_reg");
  sprintf(vnames[++i],"norm_out");
  sprintf(vnames[++i],"lwpr_name");
  sprintf(vnames[++i],"n_in_reg_2nd");
  sprintf(vnames[++i],"n_out_2nd");
  sprintf(vnames[++i],"norm_in_reg_2nd");
  sprintf(vnames[++i],"norm_out_2nd");

  /* these variables are optional */
  sprintf(vnames[++i],"max_rfs");
  sprintf(vnames[++i],"init_all_diag_D"); 
  sprintf(vnames[++i],"init_diag_D"); 
  sprintf(vnames[++i],"init_D");
  sprintf(vnames[++i],"init_lambda");
  sprintf(vnames[++i],"init_n_proj");
  sprintf(vnames[++i],"init_n_proj_2nd");
  sprintf(vnames[++i],"init_S2");
  sprintf(vnames[++i],"w_gen");
  sprintf(vnames[++i],"w_prune");
  sprintf(vnames[++i],"factor_prune");
  sprintf(vnames[++i],"final_lambda");
  sprintf(vnames[++i],"tau_lambda");
  sprintf(vnames[++i],"allow_addDim");   
  sprintf(vnames[++i],"allow_add_rfs");   
  sprintf(vnames[++i],"allow_d_update"); 
  sprintf(vnames[++i],"write_rfs");
  sprintf(vnames[++i],"add_threshold");
  sprintf(vnames[++i],"d_noise");
  sprintf(vnames[++i],"c_noise");
  sprintf(vnames[++i],"use_nn");
  sprintf(vnames[++i],"max_nn_comp");
  sprintf(vnames[++i],"composite_control");
  sprintf(vnames[++i],"use_offsets");
  sprintf(vnames[++i],"use_offsets_2nd");
  sprintf(vnames[++i],"kernel_function");

  out = fopen("lwpr_varnames","w");
  for (j=1; j<=i; ++j) 
    fprintf(out,"%s\n",vnames[j]);
  fclose(out);

  /* first we need all the initial variables to create a LWPR */

  i = 0;

  if (!find_keyword(in,vnames[++i])) {
    printf("Could not find variable >%s<\n",vnames[i]);
    return FALSE;
  }
  rc=fscanf(in,"%d",&n_in_w);
  
  if (!find_keyword(in,vnames[++i])) {
    printf("Could not find variable >%s<\n",vnames[i]);
    return FALSE;
  }
  rc=fscanf(in,"%d",&n_in_reg);
  
  if (!find_keyword(in,vnames[++i])) {
    printf("Could not find variable >%s<\n",vnames[i]);
    return FALSE;
  }
  rc=fscanf(in,"%d",&n_out);
  
  if (!find_keyword(in,vnames[++i])) {
    printf("Could not find variable >%s<\n",vnames[i]);
    return FALSE;
  }
  rc=fscanf(in,"%d",&diag_only);
  
  if (!find_keyword(in,vnames[++i])) {
    printf("Could not find variable >%s<\n",vnames[i]);
    return FALSE;
  }
  rc=fscanf(in,"%d",&allow_meta_learning);
  
  if (!find_keyword(in,vnames[++i])) {
    printf("Could not find variable >%s<\n",vnames[i]);
    return FALSE;
  }
  rc=fscanf(in,"%lf",&meta_learning_rate);
  
  if (!find_keyword(in,vnames[++i])) {
    printf("Could not find variable >%s<\n",vnames[i]);
    return FALSE;
  }
  rc=fscanf(in,"%lf",&penalty);
  
  if (!find_keyword(in,vnames[++i])) {
    printf("Could not find variable >%s<\n",vnames[i]);
    return FALSE;
  }
  rc=fscanf(in,"%lf",&init_alpha);
  
  if (!find_keyword(in,vnames[++i])) {
    printf("Could not find variable >%s<\n",vnames[i]);
    return FALSE;
  }
  norm_in_w = my_vector(1,n_in_w);
  for (j=1; j<=n_in_w; ++j)
    rc=fscanf(in,"%lf",&norm_in_w[j]);
  
  if (!find_keyword(in,vnames[++i])) {
    printf("Could not find variable >%s<\n",vnames[i]);
    return FALSE;
  }
  norm_in_reg = my_vector(1,n_in_reg);
  for (j=1; j<=n_in_reg; ++j)
    rc=fscanf(in,"%lf",&norm_in_reg[j]);
  
  if (!find_keyword(in,vnames[++i])) {
    printf("Could not find variable >%s<\n",vnames[i]);
    return FALSE;
  }
  norm_out = my_vector(1,n_out);
  for (j=1; j<=n_out; ++j)
    rc=fscanf(in,"%lf",&norm_out[j]);
  
  if (!find_keyword(in,vnames[++i])) {
    printf("Could not find variable >%s<\n",vnames[i]);
    return FALSE;
  }
  rc=fscanf(in,"%s",lwpr_name);
  
  if (find_keyword(in,vnames[++i])) {
    rc=fscanf(in,"%d",&n_in_reg_2nd);
    if (n_in_reg_2nd != 0)
      flag_2nd = TRUE;
  }
  
  if (!find_keyword(in,vnames[++i])) {
    if (flag_2nd) {
      printf("Could not find variable >%s<\n",vnames[i]);
      return FALSE;
    }
  } else {
    if (flag_2nd) {
      rc=fscanf(in,"%d",&n_out_2nd);
    }
  }

  if (!find_keyword(in,vnames[++i])) {
    if (flag_2nd) {
      printf("Could not find variable >%s<\n",vnames[i]);
      return FALSE;
    }
  } else {
    if (flag_2nd) {
      norm_in_reg_2nd = my_vector(1,n_in_reg_2nd);
      for (j=1; j<=n_in_reg_2nd; ++j)
	rc=fscanf(in,"%lf",&norm_in_reg_2nd[j]);
    }
  }

  if (!find_keyword(in,vnames[++i])) {
    if (flag_2nd) {
      printf("Could not find variable >%s<\n",vnames[i]);
      return FALSE;
    }
  } else {
    if (flag_2nd) {
      norm_out_2nd = my_vector(1,n_out_2nd);
      for (j=1; j<=n_out_2nd; ++j)
	rc=fscanf(in,"%lf",&norm_out_2nd[j]);
    }
  }

  /* this allows to generate the LWPRs */
  for (j=1; j<=n_models; ++j) {

    ans = new_flag;
    if (names != NULL)
      strcpy(lwpr_name,names[j]);

    if (ans) {
    
      rc = initLWPR(n_in_w, n_in_reg, n_in_reg_2nd, n_out, n_out_2nd, diag_only, 
		    allow_meta_learning, meta_learning_rate,penalty, 
		    init_alpha, norm_in_w, norm_in_reg, norm_in_reg_2nd, 
		    norm_out, norm_out_2nd, lwpr_name, IDs[j], NULL);
    
    } else {

      rc = readLWPR(IDs[j],lwpr_name);
    
      if (!rc) {
	new_flag = TRUE;
	rc = initLWPR(n_in_w, n_in_reg, n_in_reg_2nd, n_out, n_out_2nd, 
		      diag_only, allow_meta_learning, meta_learning_rate,penalty, 
		      init_alpha, norm_in_w, norm_in_reg, norm_in_reg_2nd, 
		      norm_out, norm_out_2nd, 
		      lwpr_name, IDs[j], NULL);
      }
      changeLWPRPenalty(IDs[j],penalty );
      changeLWPRMetaLearningRate(IDs[j], meta_learning_rate);
      changeLWPRInitAlpha(IDs[j], init_alpha);
    
    }

  }

  /* check the optional values */

  if (find_keyword(in,vnames[++i])) {
    rc=fscanf(in,"%ld",&max_rfs);
    for (j=1; j<=n_models; ++j)
      changeLWPRMaxRFs(IDs[j],max_rfs);
  }

  if (find_keyword(in,vnames[++i])) {
    init_D = my_matrix(1,n_in_w,1,n_in_w);
    mat_zero(init_D);
    rc=fscanf(in,"%lf",&init_D[1][1]);
    for (j=2; j<=n_in_w; ++j) {
      init_D[j][j] = init_D[1][1];
    }
    for (j=1; j<=n_models; ++j)
      changeLWPRInitD(IDs[j],init_D);
  }
  
  if (find_keyword(in,vnames[++i])) {
    init_D = my_matrix(1,n_in_w,1,n_in_w);
    for (j=1; j<=n_in_w; ++j) {
      rc=fscanf(in,"%lf",&init_D[j][j]);
    }
    for (j=1; j<=n_models; ++j)
      changeLWPRInitD(IDs[j],init_D);
  }
  
  if (find_keyword(in,vnames[++i])) {
    init_D = my_matrix(1,n_in_w,1,n_in_w);
    for (j=1; j<=n_in_w; ++j) {
      for (k=1; k<=n_in_w; ++k) {
	rc=fscanf(in,"%lf",&init_D[j][k]);
      }
    }
    for (j=1; j<=n_models; ++j)
      changeLWPRInitD(IDs[j],init_D);
  }
  
  if (find_keyword(in,vnames[++i])) {
    rc=fscanf(in,"%lf",&init_lambda);
    for (j=1; j<=n_models; ++j)
      changeLWPRInitLambda(IDs[j],init_lambda);
  }

  if (find_keyword(in,vnames[++i])) {
    rc=fscanf(in,"%d",&init_n_proj);
    for (j=1; j<=n_models; ++j)
      changeLWPRInitNProj(IDs[j],init_n_proj);
  }

  if (find_keyword(in,vnames[++i])) {
    rc=fscanf(in,"%d",&init_n_proj_2nd);
    for (j=1; j<=n_models; ++j)
      changeLWPRInitNProj2nd(IDs[j],init_n_proj_2nd);
  }

  if (find_keyword(in,vnames[++i])) {
    rc=fscanf(in,"%lf",&init_S2);
    for (j=1; j<=n_models; ++j)
      changeLWPRInitS2(IDs[j],init_S2);
  }

  if (find_keyword(in,vnames[++i])) {
    rc=fscanf(in,"%lf",&w_gen);
    for (j=1; j<=n_models; ++j)
      changeLWPRWgen(IDs[j],w_gen);
  }

  if (find_keyword(in,vnames[++i])) {
    rc=fscanf(in,"%lf",&w_prune);
    for (j=1; j<=n_models; ++j)
      changeLWPRWprune(IDs[j],w_prune);
  }

  if (find_keyword(in,vnames[++i])) {
    rc=fscanf(in,"%lf",&factor_prune);
    for (j=1; j<=n_models; ++j)
      changeLWPRFactorPrune(IDs[j],factor_prune);
  }

  if (find_keyword(in,vnames[++i])) {
    rc=fscanf(in,"%lf",&final_lambda);
    for (j=1; j<=n_models; ++j)
      changeLWPRFinalLambda(IDs[j],final_lambda);
  }
  if (find_keyword(in,vnames[++i])) {
    rc=fscanf(in,"%lf",&tau_lambda);
    for (j=1; j<=n_models; ++j)
      changeLWPRTauLambda(IDs[j],tau_lambda);
  }
  
  if (find_keyword(in,vnames[++i])) {
    rc=fscanf(in,"%d",&allow_addDim);
    for (j=1; j<=n_models; ++j)
      changeLWPRAllowAddDim(IDs[j],allow_addDim);
  }

  if (find_keyword(in,vnames[++i])) {
    rc=fscanf(in,"%d",&allow_add_rfs);
    for (j=1; j<=n_models; ++j)
      changeLWPRAllowAddRFS(IDs[j],allow_add_rfs);
  }

  if (find_keyword(in,vnames[++i])) {
    rc=fscanf(in,"%d",&allow_d_update);
    for (j=1; j<=n_models; ++j)
      changeLWPRAllowDUpdate(IDs[j],allow_d_update);
  }

  if (find_keyword(in,vnames[++i])) {
    rc=fscanf(in,"%d",&write_rfs);
    for (j=1; j<=n_models; ++j)
      changeLWPRWriteRFs(IDs[j],write_rfs);
  }

  if (find_keyword(in,vnames[++i])) {
    rc=fscanf(in,"%lf",&add_threshold);
    for (j=1; j<=n_models; ++j)
      changeLWPRAddThreshold(IDs[j],add_threshold);
  }

  if (find_keyword(in,vnames[++i])) {
    rc=fscanf(in,"%lf",&d_noise);
    for (j=1; j<=n_models; ++j)
      changeLWPRDNoise(IDs[j],d_noise);
  }

  if (find_keyword(in,vnames[++i])) {
    rc=fscanf(in,"%lf",&c_noise);
    for (j=1; j<=n_models; ++j)
      changeLWPRCNoise(IDs[j],c_noise);
  }

  if (find_keyword(in,vnames[++i])) {
    rc=fscanf(in,"%d",&use_nn);
    for (j=1; j<=n_models; ++j)
      changeLWPRUseNN(IDs[j],use_nn);
  }

  if (find_keyword(in,vnames[++i])) {
    rc=fscanf(in,"%d",&max_nn_comp);
    for (j=1; j<=n_models; ++j)
      changeLWPRMaxNNComp(IDs[j],max_nn_comp);
  }

  if (find_keyword(in,vnames[++i])) {
    rc=fscanf(in,"%d %d %d %d",&composite_control,&n_states,&n_controls,&linear_gx);
    for (j=1; j<=n_models; ++j)
      changeLWPRCompositeControl(IDs[j],composite_control,n_states,n_controls,linear_gx);
  }

  if (find_keyword(in,vnames[++i])) {
    rc=fscanf(in,"%d",&use_offsets);
    for (j=1; j<=n_models; ++j)
      changeLWPRUseOffset(IDs[j],use_offsets);
  }

  if (find_keyword(in,vnames[++i])) {
    rc=fscanf(in,"%d",&use_offsets_2nd);
    for (j=1; j<=n_models; ++j)
      changeLWPRUseOffset2nd(IDs[j],use_offsets_2nd);
  }

  if (find_keyword(in,vnames[++i])) {
    rc=fscanf(in,"%d",&kernel_function);
    if (new_flag) 
      for (j=1; j<=n_models; ++j)
	changeLWPRKernelFunction(IDs[j],kernel_function);
  }


  fclose(in);
  remove_temp_file();

  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
 \note  readLWPRScript
 \date  February 1996
 
 \remarks 
 
 reads the script file which creates a LWPR
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     fname   : name of the script
 \param[in]     new_flag: TRUE = force the creation of a new LWPR
              FALSE = try to read the LWPR from file first
 \param[in]     ID      : which ID number should this LWPR have	    
 
 ******************************************************************************/
int
readLWPRScript(char *fname, int new_flag, int ID)
     
{ 
  
  int      i,j;
  int      IDs[1+1];

  IDs[1] = ID;

  return readLWPRScriptMulti(fname, new_flag, IDs, NULL, 1);
  
}



