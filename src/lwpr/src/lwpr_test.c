/*!=============================================================================
  ==============================================================================

  \file    lwpr_test.c

  \author  Stefan Schaal
  \date    April 2006

  ==============================================================================
  \remarks
  
  A framework to test various data sets with Local Adaptive Subspace 
  Regression (LWPR)

  ============================================================================*/

#include "stdio.h"
#include "math.h"
#include "utility.h"
#include "lwpr.h" 
#include "data_sets.h"
#include "string.h"


#define  INIT             1
#define  TRAIN            2
#define  EVALUATE         3
#define  RESULT           4
#define  PAUSE            5


#define  TOLERANCE        0.e-10

#define  LWPR1            1

/* global variables */
double  **Xw_train;      /* the input training data for the weights*/
double  **Xreg_train;    /* the input training data for the regression*/
double  **Xreg_train_2nd;/* the input training data for the regression*/
double  **Xw_test;       /* the input test data for the weights */
double  **Xreg_test;     /* the input test data for the regression*/
double  **Xreg_test_2nd; /* the input test data for the regression*/
double  **Y_train;       /* the output data */
double  **Y_train_2nd;   /* the output data */
double  **Y_test;        /* the test output data */
double  **Y_test_2nd;    /* the test output data */
double   *x_w;           /* the input for the weighting kernel */
double   *x_reg;         /* the input for the regression */
double   *x_reg_2nd;     /* the input for the regression */
double   *y;             /* the predicted outputs for each center */
double   *y_2nd;         /* the predicted outputs for each center */
double   *conf;          /* confidence intervals */
double   *conf_2nd;      /* confidence intervals */
double   *var_y;         /* variance of the outputs for normalizaton */
double   *var_y_2nd;     /* variance of the outputs for normalizaton */
double   *mean_y;        /* variance of the outputs for normalizaton */
double   *mean_y_2nd;    /* variance of the outputs for normalizaton */
char      fname_train_data[100];
char      fname_test_data[100];


/* import the variables from main */
extern    int   argc_global;
extern   char **argv_global;


/* steering variables */

int      action = INIT;
long     n_iterations;
long     max_iterations;
int      current_test_point;
int      index_function;
long     eval_time;
int      sampling_method;


/* general variables */
int      n_in_w = 0;
int      n_in_reg = 0;
int      n_in_reg_2nd = 0;
int      n_out;
int      n_out_2nd;
int      n_train_data;
int      n_test_data;
int      blending = TRUE;
double   cutoff = 0.001;
char     lwpr_name[100];
int      new_model = FALSE;

/* MSE variables */
double   nMSE;
double   nMSE_without;
double   last_nMSE;
double   MSE;
double   MSE_without;
double   nMSE_2nd;
double   nMSE_without_2nd;
double   last_nMSE_2nd;
double   MSE_2nd;
double   MSE_without_2nd;
double   n_without;

/* functions */

void initialize(void);
void read_script(void);
static int findIndex(char *name, char **vnames, int num);

  
/*!*****************************************************************************
 *******************************************************************************
 \note  runLWPR
 \date  January, 1994
 
 \remarks 
 
 runs the lwpr simulator in an event loop
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 none
 
 ******************************************************************************/
void 
runLWPR()

{
    
  int    i,j,k,n;
  static int    rfIDlookup=0,rfIDadd=0;
  FILE  *out;
  char   string[100];
  static FILE *out1=NULL;
  double activation;
  double tt[100];
  double aux;
  static int *ar=NULL;
  double mean_n_out_pca, mean_n_in_reg, mean_var_vel;
  static int last_test_point = 0;

    
  switch (action) {
	
  case INIT:
		
    initialize();
    action = TRAIN;
		
    break;
		
		
  case TRAIN:

    /* draw a new training data point */
    draw_data_point(Xw_train,Y_train,n_train_data,
		    n_in_w,n_out,sampling_method,index_function,&ar,
		    x_w,y,&current_test_point);

    for (i=1; i<=n_in_reg; ++i)
      x_reg[i] = Xreg_train[current_test_point][i];

    if (lwprs[LWPR1].use_reg_2nd) {
      for (i=1; i<=n_in_reg_2nd; ++i)
	x_reg_2nd[i] = Xreg_train_2nd[current_test_point][i];
      for (i=1; i<=n_out_2nd; ++i)
	y_2nd[i] = Y_train_2nd[current_test_point][i];
    }

    /* nearest neighbor training is only useful for sequential sampling,
       and needs to fixed when data sets repeats itself */
    if (current_test_point != last_test_point+1)
      rfIDadd = 0;

    addDataToLWPR(LWPR1, x_w, x_reg, y,FALSE,&rfIDadd);

    if (lwprs[LWPR1].use_reg_2nd) {
      addDataToLWPRReg2nd(LWPR1,x_w,x_reg_2nd,y_2nd,1.0,FALSE,&rfIDadd);
    }    

    if (++n_iterations%eval_time == 0) {
      action = EVALUATE;
    }

    if (n_iterations > max_iterations) {
      action = EVALUATE;
    }

    break;


  case EVALUATE:


    /* run throught the X-test data and generate a nMSE */

    last_nMSE = nMSE;
    nMSE = MSE = 0;
    nMSE_without = MSE_without = 0;
    n_without = 0.;
    
    for (i=1; i<=n_test_data; ++i) {

      for (j=1; j<=n_in_w; ++j) {
	x_w[j] = Xw_test[i][j];
      }

      for (j=1; j<=n_in_reg; ++j) {
	x_reg[j] = Xreg_test[i][j];
      }

      if (lwprs[LWPR1].use_reg_2nd) {
	for (j=1; j<=n_in_reg_2nd; ++j) {
	  x_reg_2nd[j] = Xreg_test_2nd[i][j];
	}
      }    

      rfIDlookup = 0;
      activation = 
	predictLWPROutput(LWPR1, x_w, x_reg, cutoff, blending, y, &rfIDlookup);
      if (activation < cutoff)
	activation = 0.0;

      for (j=1; j<=n_out; ++j) {
         nMSE += sqr(Y_test[i][j] - y[j])/var_y[j];
	 nMSE_without += activation * sqr(Y_test[i][j] - y[j]) / var_y[j];
         MSE += sqr(Y_test[i][j] - y[j]);
	 MSE_without += activation * sqr(Y_test[i][j] - y[j]);
	 n_without += activation;
      }

      if (lwprs[LWPR1].use_reg_2nd) {
	predictLWPROutput2nd(LWPR1, x_w, x_reg_2nd, cutoff, blending, y_2nd, 
			     &rfIDlookup);
	for (j=1; j<=n_out_2nd; ++j) {
	  nMSE_2nd += sqr(Y_test_2nd[i][j] - y_2nd[j])/var_y_2nd[j];
	  nMSE_without_2nd += activation * sqr(Y_test_2nd[i][j] - y_2nd[j]) / var_y_2nd[j];
	  MSE_2nd += sqr(Y_test_2nd[i][j] - y_2nd[j]);
	  MSE_without_2nd += activation * sqr(Y_test_2nd[i][j] - y_2nd[j]);
	}

      }    



    }

    nMSE /= (double) (n_test_data*n_out);
    MSE /= (double) (n_test_data*n_out);
    if (n_without > 0) {
      nMSE_without /= (double) (n_without);
      MSE_without /= (double) (n_without);
    }

    if (lwprs[LWPR1].use_reg_2nd) {
      nMSE_2nd /= (double) (n_test_data*n_out);
      MSE_2nd /= (double) (n_test_data*n_out);
      if (n_without > 0) {
	nMSE_without_2nd /= (double) (n_without);
	MSE_without_2nd /= (double) (n_without);
      }
    }

    printf("%s: iter=%7ld:    nMSE = % 8.6f (% 8.6f)      MSE = % 8.6f (% 8.6f)\n                 #rf=%ld  #tr=%ld  #proj=%f  #pre=%ld  #pro=%ld\n",
	   lwpr_name,
	   lwprs[LWPR1].n_data,
	   nMSE,
	   nMSE_without,
	   MSE,
	   MSE_without,
	   lwprs[LWPR1].n_rf,
	   lwprs[LWPR1].n_no_trust,
	   lwprs[LWPR1].mean_n_proj,
	   lwprs[LWPR1].n_pruned_error,
	   lwprs[LWPR1].n_pruned_overlap); 

    if (lwprs[LWPR1].use_reg_2nd) {
      printf("             nMSE_2nd = % 8.6f (% 8.6f)  MSE_2nd = % 8.6f (% 8.6f)\n                                #proj=%f\n",
	     nMSE_2nd,
	     nMSE_without_2nd,
	     MSE_2nd,
	     MSE_without_2nd,
	     lwprs[LWPR1].mean_n_proj_2nd);
    }
      
    if (out1 == NULL) {
      sprintf(string,"%s.trace",lwpr_name);
      if (new_model)
	out1 = fopen(string,"w");
      else 
	out1 = fopen(string,"a+");
    }

    fprintf(out1,"%ld %f %f %f %f %ld %f ",
	    lwprs[LWPR1].n_data,
	    nMSE,
	    nMSE_without,
	    MSE,
	    MSE_without,
	    lwprs[LWPR1].n_rf,
	    lwprs[LWPR1].mean_n_proj);

    if (lwprs[LWPR1].use_reg_2nd) {
      fprintf(out1,"%f %f %f %f %f",
	      nMSE_2nd,
	      nMSE_without_2nd,
	      MSE_2nd,
	      MSE_without_2nd,
	      lwprs[LWPR1].mean_n_proj_2nd);
    }

    fprintf(out1,"\n");

    fflush(out1);

    if (fabs(last_nMSE-nMSE) < TOLERANCE || n_iterations >= max_iterations) {
      action = RESULT;
    } else {
      action = TRAIN;
    }
		
    break;
		
		
  case RESULT:
		
    /* dump the LWPR */

    writeLWPR(LWPR1);

    /* evaluate the test set */

    sprintf(string,"%s.res",lwpr_name);
    out = fopen(string,"w");
    if (out == NULL) {
      printf("Could not open result file -- exiting!\n");
      exit(-1);
    }

    for (i=1; i<=n_test_data; ++i) {

      for (j=1; j<=n_in_w; ++j) {
	x_w[j] = Xw_test[i][j];
	fprintf(out,"%f ",x_w[j]);
      }
      
      for (j=1; j<=n_in_reg; ++j) {
	x_reg[j] = Xreg_test[i][j];
	fprintf(out,"%f ",x_reg[j]);
      }
      
      if (lwprs[LWPR1].use_reg_2nd) {
	for (j=1; j<=n_in_reg_2nd; ++j) {
	  x_reg_2nd[j] = Xreg_test_2nd[i][j];
	  fprintf(out,"%f ",x_reg_2nd[j]);
	}
      }

      rfIDlookup = 0;
      predictLWPROutputConf(LWPR1, x_w, x_reg, cutoff, blending, y, &rfIDlookup, conf);

      for (j=1; j<=n_out; ++j) {
	fprintf(out,"%f %f ",y[j],conf[j]);
      }

      if (lwprs[LWPR1].use_reg_2nd) {
	rfIDlookup = 0;
	predictLWPROutputConf2nd(LWPR1, x_w, x_reg_2nd, cutoff, blending, y_2nd, &rfIDlookup, 
				 conf_2nd);
	
	for (j=1; j<=n_out_2nd; ++j) {
	  fprintf(out,"%f %f ",y_2nd[j],conf_2nd[j]);
	}

      }

      fprintf(out,"\n");

    }

    fclose(out);

    action = PAUSE;
		
    break;
		
		
  case PAUSE:

    deleteLWPR(LWPR1);
    beep(1);
    exit(-1);
		
    break;
		
		
  default:
		
    break;
  }
    
    
}

/*!*****************************************************************************
 *******************************************************************************
 \note  read_script
 \date  January, 1994
 
 \remarks 
 
 reads in the script file which determines the task of this program
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 Returns the number of data points
 
 ******************************************************************************/
#define NORMALIZE_BY_TRAIN TRUE
void
read_script(void)
     
{ 
  
  int     i,j,k,m,rc;
  char    fname[100];
  double  dummy;
  int     idummy;
  FILE   *in,*out;
  int     n_train_data_columns;
  int     n_test_data_columns;
  Vector  row;
  char    identification_string[100];
  char    string[100];
  int     num;
  char    vnames[50][100];
  int     ans = 0;
  double  o_noise, c_noise;
  int     old_indx_flag = FALSE;

  Matrix  D_train=NULL;
  char  **vnames_train=NULL;
  char  **units_train=NULL;
  double  freq_train;
  int     n_cols_train;
  int     n_rows_train;
  
  Matrix  D_test=NULL;
  char  **vnames_test=NULL;
  char  **units_test=NULL;
  double  freq_test;
  int     n_cols_test;
  int     n_rows_test;
  

  /* I need the filename of the script file: first check whether the
     user provided it in the argv_global variables */

  if (argc_global > 0) {
    in = fopen(argv_global[1],"r");
    if (in != NULL) {
      fclose(in);
      strcpy(fname,argv_global[1]);
    } else {
      if (!getFile(fname)) exit(-1);
    }
  } else {
    if (!getFile(fname)) exit(-1);
  }

  /* this allows to generate the LWPR */
  if (argc_global > 1) {
    sscanf(argv_global[2],"%d",&new_model);
  } else {
    get_int("Generate new LWPR = 1; Read from file = 0",ans,&ans);
    if (ans) new_model = TRUE;
  }

  if (!readLWPRScript(fname,new_model,LWPR1)) {

    fprintf(stderr,"Error when reading script file >%s<\n",fname);
    exit(-1);

  }
  

  /* now read additional variables from the script */
  in  = fopen_strip(fname);

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
  sprintf(vnames[++i],"lwpr_name");
  sprintf(vnames[++i],"n_in_reg_2nd");
  sprintf(vnames[++i],"n_out_2nd");

  /* this block specifies all variables needed to get the data
     for training and testing, as well as some other parameters */

  sprintf(vnames[++i],"sampling_method");
  sprintf(vnames[++i],"index_function");
  sprintf(vnames[++i],"max_iterations");
  sprintf(vnames[++i],"eval_time");
  sprintf(vnames[++i],"cutoff");
  sprintf(vnames[++i],"blending");

  sprintf(vnames[++i],"file_name_train_data");
  sprintf(vnames[++i],"file_name_test_data");

  sprintf(vnames[++i],"name_train_in_w_columns");
  sprintf(vnames[++i],"name_train_in_reg_columns");
  sprintf(vnames[++i],"name_train_out_columns");
  sprintf(vnames[++i],"name_test_in_w_columns");
  sprintf(vnames[++i],"name_test_in_reg_columns");
  sprintf(vnames[++i],"name_test_out_columns");
  sprintf(vnames[++i],"name_train_in_reg_2nd_columns");
  sprintf(vnames[++i],"name_train_out_2nd_columns");
  sprintf(vnames[++i],"name_test_in_reg_2nd_columns");
  sprintf(vnames[++i],"name_test_out_2nd_columns");


  out = fopen("lwpr_test_varnames","w");
  for (j=1; j<=i; ++j) 
    fprintf(out,"%s\n",vnames[j]);
  fclose(out);
  remove_temp_file();

  /* parse keywords */
  i = 0;

  if (!find_keyword(in,vnames[++i])) {
    printf("Could not find variable >%s<\n",vnames[i]);
    exit(-i);
  }
  rc=fscanf(in,"%d",&n_in_w);

  if (!find_keyword(in,vnames[++i])) {
    printf("Could not find variable >%s<\n",vnames[i]);
    exit(-i);
  }
  rc=fscanf(in,"%d",&n_in_reg);

  if (!find_keyword(in,vnames[++i])) {
    printf("Could not find variable >%s<\n",vnames[i]);
    exit(-i);
  }
  rc=fscanf(in,"%d",&n_out);

  if (!find_keyword(in,vnames[++i])) {
    printf("Could not find variable >%s<\n",vnames[i]);
    exit(-i);
  }
  rc=fscanf(in,"%s",lwpr_name);

  if (!find_keyword(in,vnames[++i])) {
      if (lwprs[LWPR1].use_reg_2nd) {
	printf("Could not find variable >%s<\n",vnames[i]);
	exit(-i);
      }
  } else {
    rc=fscanf(in,"%d",&n_in_reg_2nd);
  }
    
  if (!find_keyword(in,vnames[++i])) {
    if (lwprs[LWPR1].use_reg_2nd) {
      printf("Could not find variable >%s<\n",vnames[i]);
      exit(-i);
    }
  } else {
    rc=fscanf(in,"%d",&n_out_2nd);
  }

  /* at last the parameters need to steer the training and testing of
     the LWPR */
  if (!find_keyword(in,vnames[++i])) {
    printf("Could not find variable >%s<\n",vnames[i]);
    exit(-i);
  }
  rc=fscanf(in,"%d",&sampling_method);

  if (!find_keyword(in,vnames[++i])) {
    printf("Could not find variable >%s<\n",vnames[i]);
    exit(-i);
  }
  rc=fscanf(in,"%d",&index_function);

  if (!find_keyword(in,vnames[++i])) {
    printf("Could not find variable >%s<\n",vnames[i]);
    exit(-i);
  }
  rc=fscanf(in,"%ld",&max_iterations);

  if (!find_keyword(in,vnames[++i])) {
    printf("Could not find variable >%s<\n",vnames[i]);
    exit(-i);
  }
  rc=fscanf(in,"%ld",&eval_time);

  if (find_keyword(in,vnames[++i])) {
    rc=fscanf(in,"%lf",&cutoff);
  }

  if (find_keyword(in,vnames[++i])) {
    rc=fscanf(in,"%d",&blending);
  }

  if (!find_keyword(in,vnames[++i]) && argc_global <= 2) {
    printf("Could not find variable >%s<\n",vnames[i]);
    exit(-i);
  }
  if (argc_global > 2) {
    strcpy(fname_train_data,argv_global[3]);
  } else {
    rc=fscanf(in,"%s",fname_train_data);
  }

  if (!find_keyword(in,vnames[++i]) && argc_global <= 3) {
    printf("Could not find variable >%s<\n",vnames[i]);
    exit(-i);
  }
  if (argc_global > 3) {
    strcpy(fname_test_data,argv_global[4]);
  } else {
    rc=fscanf(in,"%s",fname_test_data);
  }


  // at this point the data files can be read -- they are expected to be in
  // MRDPLOT format

  printf("Reading training data from >%s<...",fname_train_data);
  if (!mrdplot_convert(fname_train_data, &D_train, &vnames_train, &units_train, 
		       &freq_train, &n_cols_train, &n_rows_train)) {
    printf("Problems reading MRDPLOT file >%s<\n",fname_train_data);
    exit(-999);
  }
  printf("done\n");
  printf("%d rows with %d columns read\n",n_rows_train,n_cols_train);

  printf("Reading test data from >%s<...",fname_test_data);
  if (!mrdplot_convert(fname_test_data, &D_test, &vnames_test, &units_test, 
		       &freq_test, &n_cols_test, &n_rows_test)) {
    printf("Problems reading MRDPLOT file >%s<\n",fname_test_data);
    exit(-999);
  }
  printf("done\n");
  printf("%d rows with %d columns read\n",n_rows_test,n_cols_test);


  // allocate memory for all arrays
  Xw_train       = my_matrix(1,n_rows_train,1,n_in_w);      
  Xreg_train     = my_matrix(1,n_rows_train,1,n_in_reg);
  Xreg_train_2nd = my_matrix(1,n_rows_train,1,n_in_reg_2nd);
  Xw_test        = my_matrix(1,n_rows_test,1,n_in_w);
  Xreg_test      = my_matrix(1,n_rows_test,1,n_in_reg);
  Xreg_test_2nd  = my_matrix(1,n_rows_test,1,n_in_reg_2nd);
  Y_train        = my_matrix(1,n_rows_train,1,n_out);
  Y_train_2nd    = my_matrix(1,n_rows_train,1,n_out_2nd);
  Y_test         = my_matrix(1,n_rows_test,1,n_out);
  Y_test_2nd     = my_matrix(1,n_rows_test,1,n_out_2nd);
  x_w            = my_vector(1,n_in_w);
  x_reg          = my_vector(1,n_in_reg);
  x_reg_2nd      = my_vector(1,n_in_reg_2nd);
  y              = my_vector(1,n_out);
  y_2nd          = my_vector(1,n_out_2nd);
  conf           = my_vector(1,n_out);
  conf_2nd       = my_vector(1,n_out_2nd);
  var_y          = my_vector(1,n_out);
  var_y_2nd      = my_vector(1,n_out_2nd);
  mean_y         = my_vector(1,n_out);
  mean_y_2nd     = my_vector(1,n_out_2nd);

  // sort the test and training data into the appropriate arrays

  if (!find_keyword(in,vnames[++i])) {
    printf("Could not find variable >%s<\n",vnames[i]);
    exit(-i);
  }
  for (j=1; j<=n_in_w; ++j) {
    rc=fscanf(in,"%s",string);
    if (!(k=findIndex(string,vnames_train,n_cols_train))) {
      printf("Couldn't find column >%s< in training data\n",string);
      exit(-i);
    } else {
      for (m=1; m<=n_rows_train; ++m)
	Xw_train[m][j] = D_train[m][k];
    }
  }

  if (!find_keyword(in,vnames[++i])) {
    printf("Could not find variable >%s<\n",vnames[i]);
    exit(-i);
  }
  for (j=1; j<=n_in_reg; ++j) {
    rc=fscanf(in,"%s",string);
    if (!(k=findIndex(string,vnames_train,n_cols_train))) {
      printf("Couldn't find column >%s< in training data\n",string);
      exit(-i);
    } else {
      for (m=1; m<=n_rows_train; ++m)
	Xreg_train[m][j] = D_train[m][k];
    }
  }
  
  if (!find_keyword(in,vnames[++i])) {
    printf("Could not find variable >%s<\n",vnames[i]);
    exit(-i);
  }
  for (j=1; j<=n_out; ++j) {
    rc=fscanf(in,"%s",string);
    if (!(k=findIndex(string,vnames_train,n_cols_train))) {
      printf("Couldn't find column >%s< in training data\n",string);
      exit(-i);
    } else {
      for (m=1; m<=n_rows_train; ++m)
	Y_train[m][j] = D_train[m][k];
    }
  }
  

  if (!find_keyword(in,vnames[++i])) {
    printf("Could not find variable >%s<\n",vnames[i]);
    exit(-i);
  }
  for (j=1; j<=n_in_w; ++j) {
    rc=fscanf(in,"%s",string);
    if (!(k=findIndex(string,vnames_test,n_cols_test))) {
      printf("Couldn't find column >%s< in test data\n",string);
      exit(-i);
    } else {
      for (m=1; m<=n_rows_test; ++m)
	Xw_test[m][j] = D_test[m][k];
    }
  }

  if (!find_keyword(in,vnames[++i])) {
    printf("Could not find variable >%s<\n",vnames[i]);
    exit(-i);
  }
  for (j=1; j<=n_in_reg; ++j) {
    rc=fscanf(in,"%s",string);
    if (!(k=findIndex(string,vnames_test,n_cols_test))) {
      printf("Couldn't find column >%s< in test data\n",string);
      exit(-i);
    } else {
      for (m=1; m<=n_rows_test; ++m)
	Xreg_test[m][j] = D_test[m][k];
    }
  }
  
  if (!find_keyword(in,vnames[++i])) {
    printf("Could not find variable >%s<\n",vnames[i]);
    exit(-i);
  }
  for (j=1; j<=n_out; ++j) {
    rc=fscanf(in,"%s",string);
    if (!(k=findIndex(string,vnames_test,n_cols_test))) {
      printf("Couldn't find column >%s< in test data\n",string);
      exit(-i);
    } else {
      for (m=1; m<=n_rows_test; ++m)
	Y_test[m][j] = D_test[m][k];
    }
  }
  

  if (lwprs[LWPR1].use_reg_2nd) {

    if (!find_keyword(in,vnames[++i])) {
    printf("Could not find variable >%s<\n",vnames[i]);
    exit(-i);
  }
    for (j=1; j<=n_in_reg_2nd; ++j) {
      rc=fscanf(in,"%s",string);
      if (!(k=findIndex(string,vnames_train,n_cols_train))) {
	printf("Couldn't find column >%s< in training data\n",string);
	exit(-i);
      } else {
	for (m=1; m<=n_rows_train; ++m)
	  Xreg_train_2nd[m][j] = D_train[m][k];
      }
    }
    
    if (!find_keyword(in,vnames[++i])) {
    printf("Could not find variable >%s<\n",vnames[i]);
    exit(-i);
  }
    for (j=1; j<=n_out_2nd; ++j) {
      rc=fscanf(in,"%s",string);
      if (!(k=findIndex(string,vnames_train,n_cols_train))) {
	printf("Couldn't find column >%s< in training data\n",string);
	exit(-i);
      } else {
	for (m=1; m<=n_rows_train; ++m)
	  Y_train_2nd[m][j] = D_train[m][k];
      }
    }
    
    if (!find_keyword(in,vnames[++i])) {
    printf("Could not find variable >%s<\n",vnames[i]);
    exit(-i);
  }
    for (j=1; j<=n_in_reg_2nd; ++j) {
      rc=fscanf(in,"%s",string);
      if (!(k=findIndex(string,vnames_test,n_cols_test))) {
	printf("Couldn't find column >%s< in test data\n",string);
      exit(-i);
      } else {
	for (m=1; m<=n_rows_test; ++m)
	  Xreg_test_2nd[m][j] = D_test[m][k];
      }
    }
    
    if (!find_keyword(in,vnames[++i])) {
    printf("Could not find variable >%s<\n",vnames[i]);
    exit(-i);
  }
    for (j=1; j<=n_out_2nd; ++j) {
      rc=fscanf(in,"%s",string);
      if (!(k=findIndex(string,vnames_test,n_cols_test))) {
	printf("Couldn't find column >%s< in test data\n",string);
	exit(-i);
      } else {
	for (m=1; m<=n_rows_test; ++m)
	  Y_test_2nd[m][j] = D_test[m][k];
      }
    }

  }

  fclose(in);

  n_train_data = n_rows_train;
  n_test_data  = n_rows_test;


  if (NORMALIZE_BY_TRAIN) {

    for (i=1; i<=n_train_data; ++i) {
      vec_add_size(mean_y,Y_train[i],n_out,mean_y);
    }
    vec_mult_scalar(mean_y,1./(double)n_train_data,mean_y);

    for (i=1; i<=n_train_data; ++i) {
      for (j=1; j<=n_out; ++j) {
	var_y[j] += sqr(Y_train[i][j]-mean_y[j]);
      }
    }
    vec_mult_scalar(var_y,1./(double)n_train_data,var_y);

    if (lwprs[LWPR1].use_reg_2nd) {

      for (i=1; i<=n_train_data; ++i) {
	vec_add_size(mean_y_2nd,Y_train_2nd[i],n_out,mean_y_2nd);
      }
      vec_mult_scalar(mean_y_2nd,1./(double)n_train_data,mean_y_2nd);
      
      for (i=1; i<=n_train_data; ++i) {
	for (j=1; j<=n_out_2nd; ++j) {
	  var_y_2nd[j] += sqr(Y_train_2nd[i][j]-mean_y_2nd[j]);
	}
      }
      vec_mult_scalar(var_y_2nd,1./(double)n_train_data,var_y_2nd);

    }

  } else {

    for (i=1; i<=n_test_data; ++i) {
      vec_add_size(mean_y,Y_test[i],n_out,mean_y);
    }
    vec_mult_scalar(mean_y,1./(double)n_test_data,mean_y);

    for (i=1; i<=n_test_data; ++i) {
      for (j=1; j<=n_out; ++j) {
	var_y[j] += sqr(Y_test[i][j]-mean_y[j]);
      }
    }
    vec_mult_scalar(var_y,1./(double)n_test_data,var_y);

    if (lwprs[LWPR1].use_reg_2nd) {

      for (i=1; i<=n_test_data; ++i) {
	vec_add_size(mean_y_2nd,Y_test_2nd[i],n_out,mean_y_2nd);
      }
      vec_mult_scalar(mean_y_2nd,1./(double)n_test_data,mean_y_2nd);
      
      for (i=1; i<=n_train_data; ++i) {
	for (j=1; j<=n_out_2nd; ++j) {
	  var_y_2nd[j] += sqr(Y_test_2nd[i][j]-mean_y_2nd[j]);
	}
      }
      vec_mult_scalar(var_y_2nd,1./(double)n_train_data,var_y_2nd);

    }
  }

  
}


/*!*****************************************************************************
 *******************************************************************************
 \note  initialize
 \date  January, 1994
 
 \remarks 
 
 initializes all necessary things
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 none
 
 
 ******************************************************************************/
void
initialize(void)
     
{
  
  int i,rc,ans=0;
  
  /* read script file */
  
  rc=system("/bin/rm -f *.rf");	/* added -f,    1/30/00, Jorg Conradt */
  				/* so it doesn't complain about missing 
				 *.rf files */
  read_script();
  
  /* some general variable defaults */
  
  last_nMSE = 1.e+10;
  n_iterations    = 0;
  current_test_point = 0; 

}

/*!*****************************************************************************
 *******************************************************************************
 \note  findIndex
 \date  April 2006
 
 \remarks 
 
 Find the index of a string in an array of strings
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     name      : string to found
 \param[in]     vnames    : array of strings
 \param[in]     num       : how many elements in vnames

 returns either a postive index, or FALSE if failure
 
 
 ******************************************************************************/
static int
findIndex(char *name, char **vnames, int num)
     
{
  
  int i;

  for (i=1; i<=num; ++i) {
    if (strcmp(name,vnames[i])==0 && strlen(name)==strlen(vnames[i]))
      return i;
  }

  return FALSE;
}

