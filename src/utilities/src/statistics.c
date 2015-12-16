/*!=============================================================================
  ==============================================================================

  \file    statistics.c

  \author  Copyright Stefan Schaal
  \date    December 1995

  ==============================================================================

  \remarks

	general program for statistics programs
	NOTE: all programs operate on tranposed data matrices, i.e.,
	the columns are in rows in order to have them in consecutive
	memory

  ============================================================================*/

#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "utility.h"
#include "statistics.h"
#include "string.h"


/* filter stuff */

#define FILTER_ORDER 2
#define N_FILTERS    50

static double filters_a[N_FILTERS+1][FILTER_ORDER+1];
static double filters_b[N_FILTERS+1][FILTER_ORDER+1];


/*!*****************************************************************************
 *******************************************************************************
\note  filtfilt
\date  10/20/91 
   
\remarks 

        filters a vector with a second order butterworth filter
        with zero phase lag

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     vec        : vector of data
 \param[in]     nr         : number of element in vector
 \param[in]     cutoff     : cutoff in percent (integer) of Nyquist frequency
 \param[out]    vec_out    : filtered output vector
      

 ******************************************************************************/
int
filtfilt( double *vec, int nr, int cutoff, double *vec_out)
{

  static int firsttime = TRUE;
  FILE *filterfile;
  int count_v=0, count_r=0;
  int i,j,n;
  double *temp;
  double aux;
  char   fname[strlen(getenv("LAB_ROOT"))+100];
  int rc;

  if (cutoff == 0) { /* this takes the mean of all values */
    aux = 0;
    for (i=1; i<=nr; ++i) {
      aux += vec[i];
    }
    aux /= (double)nr;
    for (i=1; i<=nr; ++i) {
      vec_out[i] = aux;
    }
    return TRUE;
  }
  
  if (cutoff == 100) { /* just copy */
    for (i=1; i<=nr; ++i) {
      vec_out[i] = vec[i];
    }
    return TRUE;
  }
  
  if (cutoff > N_FILTERS || cutoff < 1) {
    printf("filter with cutoff %d not implemented! Exit ....\n",cutoff);
    exit(-999);
  }

  temp = my_vector(1,nr);

  for (i=1; i<=nr; ++i) {
    temp[i] = vec[i];
  }


  if (firsttime) {

    firsttime = FALSE;
    
    /* read in the filter file */
    
    sprintf(fname,"%s/utilities/filters/butterworth_table_order_2",
	    getenv("LAB_ROOT"));  
    filterfile = fopen(fname,"r");
    
    if (filterfile == NULL) {
      
      printf("Cannot find filter file >%s<\n",fname);
      printf("Hit return to exit ...\n");
      getchar();
      
      exit (-999);
      
    }
    
    for (i=1; i<=N_FILTERS; ++i) {
      
      ++count_r;
      
      for (j=0; j<= FILTER_ORDER; ++j) {
	
	++count_v;
	
	rc = fscanf(filterfile,"%lf",&(filters_a[i][j]));
	
      }
      
      for (j=0; j<= FILTER_ORDER; ++j) {
	
	++count_v;
	
	rc = fscanf(filterfile,"%lf",&(filters_b[i][j]));
	
      }
      
    }
    
    fclose (filterfile);
    
    printf("\nRead file of butterworth filter coefficients: #rows=%d  #val=%d\n",
	   count_r,count_v);


  }

  temp[1] = vec[1];
  temp[2] = vec[2];

  for (i=3; i<=nr; ++i) {
    
    temp[i] = 
      filters_b[cutoff][0] * vec[i] + 
	filters_b[cutoff][1] * vec[i-1] + 
	  filters_b[cutoff][2] * vec[i-2] -
	    (filters_a[cutoff][1] * temp[i-1] +
	     filters_a[cutoff][2] * temp[i-2]);
    
  }
  
 
  for (i=1; i<=nr; ++i) {
    vec_out[i] = temp[i];
  }

  for (i=nr-2; i>=1; --i) {
    
    vec_out[i] = 
      filters_b[cutoff][0] * temp[i] + 
	filters_b[cutoff][1] * temp[i+1] + 
	  filters_b[cutoff][2] * temp[i+2] -
	    (filters_a[cutoff][1] * vec_out[i+1] +
	     filters_a[cutoff][2] * vec_out[i+2]);
    
  }
  
  my_free_vector(temp,1,nr);

  return TRUE;


}

/*!*****************************************************************************
 *******************************************************************************
\note  findMax
\date  10/20/91 
   
\remarks 

        find the maximal value in an array

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     vec        : vector of data
 \param[in]     nr         : number of element in vector
 \param[out]    val        : the max value

      the function returns the index into the max element

 ******************************************************************************/
int
findMax( double *vec, int nr, double *val)

{

  int i,j=0;
  
  *val = -1.e+30;

  for (i=1; i<=nr; ++i) {
    if (vec[i] > *val) {
      *val = vec[i];
      j = i;
    }
  }

  return j;

}

/*!*****************************************************************************
 *******************************************************************************
\note  findMin
\date  10/20/91 
   
\remarks 

        find the minimal value in an array

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     vec        : vector of data
 \param[in]     nr         : number of element in vector
 \param[out]    val        : the min value

      the function returns the index into the min element

 ******************************************************************************/
int
findMin( double *vec, int nr, double *val)

{

  int i,j=0;
  
  *val = 1.e+30;

  for (i=1; i<=nr; ++i) {
    if (vec[i] < *val) {
      *val = vec[i];
      j = i;
    }
  }

  return j;

}

/*!*****************************************************************************
 *******************************************************************************
\note  diff
\date  10/20/91 
   
\remarks 

        differentiates a vector by a simple convulation

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     vec        : vector of data
 \param[in]     nr         : number of element in vector
 \param[in]     dt         : delta time between data points
 \param[out]    vec_out    : differntiatedoutput vector
      

 ******************************************************************************/
int
diff( double *vec, int nr, double dt, double *vec_out)

{

  int i,j,n;
  double *temp;
  double aux;
  
  temp = my_vector(1,nr);

  for (i=2; i<=nr-1; ++i) {
    temp[i] = (vec[i+1]-vec[i-1])/2.0/dt;
  }
  temp[1]  = temp[2];
  temp[nr] = temp[nr-1];
  
  for (i=1; i<=nr; ++i) {
    vec_out[i] = temp[i];
  }

  my_free_vector(temp,1,nr);

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  diff2
\date  February, 1996
   
\remarks 

        differentiates a vector twice by a simple convulation
	NOTE: this has more bandwidth than applying diff twice

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     vec        : vector of data
 \param[in]     nr         : number of element in vector
 \param[in]     dt         : delta time between data points
 \param[out]    vec_out    : differntiatedoutput vector
      

 ******************************************************************************/
int
diff2( double *vec, int nr, double dt, double *vec_out)

{

  int i,j,n;
  double *temp;
  double aux;
  
  temp = my_vector(1,nr);

  for (i=2; i<=nr-1; ++i) {
    temp[i] = (vec[i+1]-2.*vec[i]+vec[i-1])/sqr(dt);
  }
  temp[1]  = temp[2];
  temp[nr] = temp[nr-1];
  
  for (i=1; i<=nr; ++i) {
    vec_out[i] = temp[i];
  }

  my_free_vector(temp,1,nr);

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  meanvar
\date  10/20/91 
   
\remarks 

        calculates mean and variance of a vector

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     vec        : vector of data
 \param[in]     nr         : number of element in vector
 \param[in]     thres      : threshold for outliers
 \param[out]    mean       : the mean value
 \param[out]    var        : the variance
 \param[out]    n_outliers : number of potential outliers

 ******************************************************************************/
int
meanvar( double *vec, int nr, double thres, double *mean, double *var, 
	 int *n_outliers)

{

  int i,j,n;
  double aux;
  int count;

  /* calculate the mean */
  
  aux = 0;
  for (i=1; i<=nr; ++i) {
    aux += vec[i];
  }
  *mean = aux/(double)nr;

  /* calculate the variance */

  aux = 0;
  for (i=1; i<=nr; ++i) {
    aux += sqr(vec[i]- *mean);
  }
  *var = aux/(double)(nr-1);

  /* count potential outliers */

  aux = sqrt(*var * (1.-1./(double)nr));
  count = 0;
  for (i=1; i<=nr; ++i) {
    if (fabs(vec[i]- *mean)/aux > thres) ++count;
  }

  *n_outliers = count;

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  linear_regression
\date  December 1995
   
\remarks 

        performs a linear regression analysis on the given data and
	returns the normal regression statistics, outlier statistics,
	and SVD information.

	NOTE: the data matrices are in column form, i.e, one data
	point per column. This allows the calling program to shift
	data rows in time for ARMA like analyses (as the allocation 
	structure of my matrices are row based)

	NOTE: the "1" input dimensions must be added by the calling
	program.

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     X          : input data matrix
 \param[in]     Y          : output data matrix
 \param[in]     n_pts      : number of data points, i.e, number of columns
 \param[in]     n_in       : number of input dimensions
 \param[in]     n_out      : number of output dimensions
 \param[in]     outlier_threshold: statistical threshold according to normal
                            assumption (e.g., 1.96, 2.57, etc.)
      

 ******************************************************************************/
Regression_Statistics *
linearRegression(double **XX, double **YY, int n_pts, int n_in, int n_out,
		  double outlier_threshold)

{

  int      i,j,k,n,m;
  double **XTX;
  double **XTY;
  double **res;
  double **Yp;
  double **P;
  double **beta;
  double  *sum_outputs;
  double  *sum_residuals;
  double  *sum_regression;
  double  *mean_outputs;
  double  *mean_residuals;
  double  *mean_regression;
  double  *var_outputs;
  double  *var_residuals;
  double  *var_regression;
  double  *r_squared;
  double  *r_squared_adj;
  double   r_squared_total;
  double   r_squared_adj_total;
  double   f_reg_total;
  double   p_reg_total;
  double  *f_reg;
  double  *p_reg;
  double **var_coeff;
  double **std_coeff;
  double **t_coeff;
  double **p_coeff;
  int     *outliers;
  double   aux,value;
  double  *x;
  double **v_mat;
  double  *singular_values;
  int      n_outliers;
  double  *ss_inputs;

  Regression_Statistics *stat;

  /* allocate memory */

  XTX   = my_matrix(1,n_in,1,n_in);
  P     = my_matrix(1,n_in,1,n_in);
  XTY   = my_matrix(1,n_in,1,n_out);
  beta  = my_matrix(1,n_in,1,n_out);
  Yp    = my_matrix(1,n_out,1,n_pts);
  res   = my_matrix(1,n_out,1,n_pts);

  sum_outputs    = my_vector(1,n_out);
  sum_residuals  = my_vector(1,n_out);
  sum_regression = my_vector(1,n_out);
  mean_outputs    = my_vector(1,n_out);
  mean_residuals  = my_vector(1,n_out);
  mean_regression = my_vector(1,n_out);
  var_outputs    = my_vector(1,n_out);
  var_residuals  = my_vector(1,n_out);
  var_regression = my_vector(1,n_out);

  r_squared = my_vector(1,n_out);
  r_squared_adj = my_vector(1,n_out);

  f_reg = my_vector(1,n_out);
  p_reg = my_vector(1,n_out);

  var_coeff  = my_matrix(1,n_in,1,n_out);
  std_coeff  = my_matrix(1,n_in,1,n_out);
  t_coeff    = my_matrix(1,n_in,1,n_out);
  p_coeff    = my_matrix(1,n_in,1,n_out);

  outliers = my_ivector(1,n_pts);

  x = my_vector(1,n_in);

  v_mat = my_matrix(1,n_in,1,n_in);
  singular_values = my_vector(1,n_in);
  ss_inputs = my_vector(1,n_in);


  /******************************************************************/
  /* the regression calculation                                     */
  /******************************************************************/
  
  /* build the XTX matrix */

  for (n=1; n<=n_in; ++n) {
    for (m=n; m<=n_in; ++m) {
      XTX[n][m] = 0.0;
      for (i=1; i<=n_pts; ++i) {
	XTX[n][m] += XX[n][i]*XX[m][i];
      }
      XTX[m][n] = XTX[n][m];
    }
  }

  /* build the XTY matrix */

  for (n=1; n<=n_in; ++n) {
    for (m=1; m<=n_out; ++m) {
      XTY[n][m] = 0.0;
      for (i=1; i<=n_pts; ++i) {
	XTY[n][m] += XX[n][i]*YY[m][i];
      }
    }
  }


  /* invert the XTX matrix */

  my_inv_ludcmp(XTX, n_in, P);

  /* make sure P is symmetric */

  for (n=1; n<=n_in; ++n) {
    for (m=n+1; m<=n_in; ++m) {
      P[m][n] = P[n][m];
    }
  }

  /* get the regression parameters */

  mat_mult(P,XTY,beta);



  /******************************************************************/
  /* the regression statistics                                      */
  /******************************************************************/
  
  /* calculate the predicted values and the residuals */

  for (m=1; m<=n_out; ++m) {
    for (i=1; i<=n_pts; ++i) {
      Yp[m][i] = 0.0;
      for (n=1; n<=n_in; ++n) {
	Yp[m][i] += XX[n][i]*beta[n][m];
      }
      res[m][i] = YY[m][i] - Yp[m][i];
    }
  }


  /* calculate the sum of outputs, regression, and residuals */

  for (m=1; m<=n_out; ++m) {
    sum_outputs[m] = 0.0;
    sum_residuals[m] = 0.0;
    sum_regression[m] = 0.0;
    for (i=1; i<=n_pts; ++i) {
      sum_outputs[m] += YY[m][i];
      sum_regression[m] += Yp[m][i];
      sum_residuals[m] += res[m][i];
    }
  }
  
  /* the means of these values */

  for (m=1; m<=n_out; ++m) {
    mean_outputs[m]    = sum_outputs[m]/(double)n_pts;
    mean_residuals[m]  = sum_residuals[m]/(double)n_pts;
    mean_regression[m] = sum_regression[m]/(double)n_pts;
  }


  /* the variances of these values */
			     
  for (m=1; m<=n_out; ++m) {
    sum_outputs[m] = 0.0;
    sum_residuals[m] = 0.0;
    sum_regression[m] = 0.0;
    for (i=1; i<=n_pts; ++i) {
      sum_outputs[m] += sqr(YY[m][i]-mean_outputs[m]);;
      sum_regression[m] += sqr(Yp[m][i]-mean_regression[m]);
      sum_residuals[m] += sqr(res[m][i]-mean_residuals[m]);
    }
  }

  for (m=1; m<=n_out; ++m) {
    var_outputs[m]    = sum_outputs[m]/(double)(n_pts-1);
    var_residuals[m]  = sum_residuals[m]/(double)(n_pts-n_in);
    var_regression[m] = sum_regression[m]/(double)(n_in-1);
  }

  /******************************************************************/
  /* the r-squared values                                           */
  /******************************************************************/

  r_squared_total = 0.0;
  r_squared_adj_total = 0.0;

  for (m=1; m<=n_out; ++m) {

    r_squared[m] = (var_outputs[m]*(double)(n_pts-1) -
      var_residuals[m]*(double)(n_pts-n_in)) /
      (var_outputs[m]*(double)(n_pts-1));

    r_squared_adj[m] = (var_outputs[m] - var_residuals[m]) /
      (var_outputs[m]);

    r_squared_total += r_squared[m];
    r_squared_adj_total += r_squared_adj[m];
    
  }  

  r_squared_total /= (double)n_out;
  r_squared_adj_total /= (double)n_out;
  
  
  /******************************************************************/
  /* the F-test of the total regression                             */
  /******************************************************************/

  f_reg_total = 0;
  p_reg_total = 0;

  for (m = 1; m<=n_out; ++m) {

    f_reg[m] = var_regression[m] / var_residuals[m];
    p_reg[m] = prob_f(f_reg[m], (double)(n_in-1), (double)(n_pts-n_in));

    f_reg_total += f_reg[m];
    p_reg_total += p_reg[m];

  }

  f_reg_total /= (double) n_out;
  p_reg_total /= (double) n_out;


  /******************************************************************/
  /* the t-test of the regression coefficients                      */
  /******************************************************************/
  
  for (m=1; m <= n_out; ++m) {
    for (n=1; n <= n_in; ++n) {
      var_coeff[n][m] = var_residuals[m] * P[n][n];
      std_coeff[n][m] = sqrt(var_coeff[n][m]);
      t_coeff[n][m]   = beta[n][m] / std_coeff[n][m];
      p_coeff[n][m]   = 1.-prob_t(t_coeff[n][m], (double)(n_pts-n_in));
    }
  }


  /******************************************************************/
  /* outlier detection                                              */
  /******************************************************************/
  

  /* calculate the standarized PRESS residual */

  n_outliers = 0;

  for (i=1; i<=n_pts; ++i) {

    for (n=1; n<=n_in; ++n) {
      x[n] = XX[n][i];
    }
    aux = sqrt(1.-mat_mahal(P,x));

    outliers[i] = FALSE;

    for (m=1; m<=n_out; ++m) {
      value = fabs(res[m][i]/sqrt(var_residuals[m])/aux);
      if (value > outlier_threshold) {
	outliers[i] = TRUE;
      }
    }
    
    if (outliers[i]) outliers[++n_outliers] = i;

  }


  /******************************************************************/
  /* singular value decomposition                                   */
  /******************************************************************/

  /* scale XTX for this purpose */

  for (n=1; n<=n_in; ++n) {
    ss_inputs[n] = 0.0;
    for (i=1; i<=n_pts; ++i) {
      ss_inputs[n] += sqr(XX[n][i]);
    }
  }
  
  for (n=1; n<=n_in; ++n) {
    for (m=1; m<=n_in; ++m) {
      XTX[n][m] /= sqrt(ss_inputs[n])*sqrt(ss_inputs[m]);
    }
  }

  my_svdcmp(XTX,n_in,n_in,singular_values,v_mat);


  /******************************************************************/
  /* sort everything in the statistics structure                    */
  /******************************************************************/
 
  stat = my_calloc(1,sizeof(Regression_Statistics),MY_STOP);

  stat->n_in  = n_in;
  stat->n_out = n_out;
  stat->n_pts = n_pts;
  stat->outlier_threshold = outlier_threshold;
  stat->beta  = beta;
  stat->var_residuals = var_residuals;
  stat->var_outputs = var_outputs;
  stat->r_squared = r_squared;
  stat->r_squared_adj = r_squared_adj;
  stat->f_reg = f_reg;
  stat->p_reg = p_reg;
  stat->var_coeff = var_coeff;
  stat->std_coeff = std_coeff;
  stat->t_coeff = t_coeff;
  stat->p_coeff = p_coeff;
  stat->outliers = outliers;
  stat->n_outliers = n_outliers;
  stat->sv = singular_values;


  /* free memory */

  my_free_matrix(XTX,1,n_in,1,n_in);
  my_free_matrix(P,1,n_in,1,n_in);
  my_free_matrix(XTY,1,n_in,1,n_out);
  my_free_matrix(Yp,1,n_out,1,n_pts);
  my_free_matrix(res,1,n_out,1,n_pts);

  my_free_vector(sum_outputs,1,n_out);
  my_free_vector(sum_residuals,1,n_out);
  my_free_vector(sum_regression,1,n_out);
  my_free_vector(mean_outputs,1,n_out);
  my_free_vector(mean_residuals,1,n_out);
  my_free_vector(mean_regression,1,n_out);
  my_free_vector(var_regression,1,n_out);
  my_free_vector(x,1,n_in);

  my_free_matrix(v_mat,1,n_in,1,n_in);
  my_free_vector(ss_inputs,1,n_in);

  return stat;

}

/*!*****************************************************************************
 *******************************************************************************
\note  printRegressionStatistics
\date  December 1995
   
\remarks 

        dumps the regression statistics to a file

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     stat      : pointer to regression statistics
 \param[in]     fp        : pointer to output channel
 \param[in]     name      : a name attached to this output

 ******************************************************************************/
int
printRegressionStatistics(Regression_Statistics *stat,FILE *fp, char *name)

{

  FILE *out;
  int   i,j,k,m,n,r;
  double maxsv,minsv;

  fprintf(fp,"\n================================================\n");
  fprintf(fp,"STATISTICS OF >%s< FIT:\n",name);
  fprintf(fp,"================================================\n\n");
  
  fprintf(fp,"n_in=%d  n_out=%d  n_data=%d\n\n",
	  stat->n_in,stat->n_out,stat->n_pts);

  fprintf(fp,"R-Square:\n");
  for (i=1; i <= stat->n_out; ++i) {
    fprintf(fp,"         %d.Output: %8.4f\n", i, stat->r_squared[i]);
  }
  fprintf(fp,"\n");
  
  fprintf(fp,"Adjusted R-Square:\n");
  for (i = 1; i <= stat->n_out; ++i) {
    fprintf(fp,"         %d.Output: %8.4f\n", i,stat->r_squared_adj[i]);
  }
  fprintf(fp,"\n");
  
  fprintf(fp,"Total Regression Statistics:\n");
  for (i = 1; i <= stat->n_out; ++i) {
    fprintf(fp,"%d.Output: var_out=%8.4f  var_res=%8.4f  F-test=%8.4f  p=%8.4f\n", i,
	    stat->var_outputs[i],
	    stat->var_residuals[i],
	    stat->f_reg[i],
	    stat->p_reg[i]);
  }
  fprintf(fp,"\n");
  
  fprintf(fp,"Statistics for Regression Coefficients:\n");
  
  for (i = 1; i <= stat->n_out; ++i) {
    for (m = 1; m <= stat->n_in; ++m) {
      fprintf(fp,"%d.%d  coeff=%8.4f  var=%8.4f  std_err=%8.4f  t=%8.4f  p=%8.4f\n", i, m,
	      stat->beta[m][i],
	      stat->var_coeff[m][i],
	      stat->std_coeff[m][i],
	      stat->t_coeff[m][i],
	      stat->p_coeff[m][i]);
    }
    fprintf(fp,"\n");
  }
  fprintf(fp,"\n");

  
  maxsv = -1.e30;
  minsv =  1.e30;
  fprintf(fp,"Singular Values of **Scaled** Regression Matrix\n");
  for (r = 1; r <= stat->n_in; ++r) {
    fprintf(fp,"%d.SV:     %8.4f\n",r,stat->sv[r]);
    if (maxsv < stat->sv[r]) maxsv = stat->sv[r];
    if (minsv > stat->sv[r]) minsv = stat->sv[r];
  }
  fprintf(fp,"Condition Number: %g\n",maxsv/minsv);
      
  
  if (stat->n_outliers == 0) {
    
    fprintf(fp,"\nNo outliers detected!\n");
    
  } else {

    fprintf(fp,"\nThe following points are outliers, threshold=%7.4f:\n",
	    stat->outlier_threshold);
    for (i=1; i<=stat->n_outliers; ++i) {
      fprintf(fp,"%d ",stat->outliers[i]);
      if (i%10 == 0) fprintf(fp,"\n");
    }
    fprintf(fp,"\n");
  }
  
  fprintf(fp,"\n\n");

  return TRUE;
  
}
      
/*!*****************************************************************************
 *******************************************************************************
\note  predictRLS
\date  September 2006
   
\remarks 

        performs a prediction with RLS structure

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     rls        : data structure of rls
 \param[in]     n_data_min : minumum data in RLS to perform prediction
 \param[in]     x          : input data vector (without constant)
 \param[out]    y          : output data vector

 ******************************************************************************/
int
predictRLS(RLSStatisticsPtr rls, int n_data_min, double *x, double *y)
{
  int     i,j;

  // check for sufficient data
  if (rls->n_data < n_data_min)
    return FALSE;

  // compute the output
  for (j=1; j<=rls->n_out; ++j)
    if (rls->add_constant)
      y[j] = rls->beta[rls->n_in][j];
    else
      y[j] = 0.0;
    
  for (j=1; j<=rls->n_out; ++j)
    if (rls->add_constant)
      for (i=1; i<=rls->n_in-1; ++i)
	y[j] += rls->beta[i][j] * x[i];
    else
      for (i=1; i<=rls->n_in; ++i)
	y[j] += rls->beta[i][j] * x[i];

  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
\note  recursiveLeastSquares
\date  September 2006
   
\remarks 

        performs recursive least squares with forgetting factor and
        weight

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in,out] rls        : data structure of rls
 \param[in]     x          : input data vector (without constant)
 \param[in]     y          : output data vector
 \param[in]     w          : weight of this data point

 ******************************************************************************/
void
recursiveLeastSquares(RLSStatisticsPtr rls, double *x, double *y, double w)
{
  int    i,j;
  double xPx;
  double denom;

  // assign inputs and outputs to local data structures
  if (rls->add_constant) {
    for (i=1; i<=rls->n_in-1; ++i)
      rls->x[i] = x[i];
    rls->x[rls->n_in] = 1;
  } else {
    for (i=1; i<=rls->n_in; ++i)
      rls->x[i] = x[i];
  }

  for (i=1; i<=rls->n_out; ++i)
    rls->y[i] = y[i];

  // compute Px term
  mat_vec_mult(rls->P,rls->x,rls->Px);

  // compute xPx term
  xPx = vec_mult_inner(rls->x,rls->Px);
  denom = rls->lambda/w + xPx;

  // compute the entire P update
  for (i=1; i<=rls->n_in; ++i) {
    for (j=i; j<=rls->n_in; ++j) {
      rls->P[i][j] = (rls->P[i][j] - rls->Px[i]*rls->Px[j]/denom)/rls->lambda;
    }
  }

  // compute regression update
  mat_vec_mult(rls->P,rls->x,rls->Px);
  vec_mat_mult(rls->x,rls->beta,rls->pred);
  vec_sub(rls->y,rls->pred,rls->error);

  for (i=1; i<=rls->n_in; ++i) {
    for (j=1; j<=rls->n_out; ++j) {
      rls->beta[i][j] = rls->beta[i][j] + w*rls->Px[i]*rls->error[j];
    }
  }

  // update some statistics
  vec_mat_mult(rls->x,rls->beta,rls->pred);
  vec_sub(rls->y,rls->pred,rls->error);

  rls->n_data = rls->lambda*rls->n_data + 1;
  rls->nMSE   = 0;
  for (i=1; i<=rls->n_out; ++i) {
    rls->sum_error2[i] = rls->sum_error2[i]*rls->lambda + sqr(rls->error[i]);
    rls->sum_y[i]      = rls->sum_y[i]*rls->lambda + y[i];
    rls->sum_y2[i]     = rls->sum_y2[i]*rls->lambda + sqr(y[i]);
    rls->nMSE         += rls->sum_error2[i]/
      (rls->sum_y2[i] - sqr(rls->sum_y[i])/rls->n_data + 1.e-10);
  }
  rls->nMSE /= (double)rls->n_out;

}

/*!*****************************************************************************
 *******************************************************************************
\note  initRecursiveLeastSquares
\date  September 2006
   
\remarks 

        initializes a rls data structure and allocates all memory

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     n_in         : number of inputs (without constant)
 \param[in]     n_out        : number of outputs 
 \param[in]     add_constant : TRUE/FALSE to add constant to inputs
 \param[in]     ridge        : ridge factor
 \param[in]     lambda       : forgetting factor
 \param[in]     name         : name of rls model

 ******************************************************************************/
RLSStatisticsPtr
initRecursiveLeastSquares(int n_in, int n_out, int add_constant, 
			  double ridge, double lambda, char *name)
{
  int i;
  RLSStatisticsPtr rls;

  if (add_constant)
    ++n_in;

  // get data structure and memory
  rls       = (RLSStatisticsPtr) my_calloc(1,sizeof(RLSStatistics),MY_STOP);
  rls->P    = my_matrix_symm(1,n_in,1,n_in);
  rls->beta = my_matrix(1,n_in,1,n_out);
  rls->Px   = my_vector(1,n_in);
  rls->x    = my_vector(1,n_in);
  rls->y    = my_vector(1,n_out);
  rls->pred = my_vector(1,n_out);
  rls->error= my_vector(1,n_out);

  rls->sum_error2= my_vector(1,n_out);
  rls->sum_y2    = my_vector(1,n_out);
  rls->sum_y     = my_vector(1,n_out);

  // assign variables
  rls->n_in         = n_in;
  rls->n_out        = n_out;
  rls->ridge        = ridge;
  rls->add_constant = add_constant;
  rls->lambda       = lambda;
  rls->n_data       = 0;
  strcpy(rls->name,name);

  for (i=1; i<=n_in; ++i)
    rls->P[i][i] = 1./(ridge+1.e-10);

  return rls;
}

/*!*****************************************************************************
 *******************************************************************************
\note  writeRLS
\date  September 2006
   
\remarks 

        write the given RLS to file

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     rls      : data structure of rls

 ******************************************************************************/
int
writeRLS(RLSStatisticsPtr rls)
{
  FILE *out;
  char  name[100];

  sprintf(name,"%s.rls",rls->name);
  out = fopen(name,"w");
  if (out == NULL) {
    printf("Cannot open file >%s< for write\n",name);
    return FALSE;
  }
  
  printf("Writing to file >%s< ...",name);

  // the defining main structure
  if (fwrite(rls,sizeof(RLSStatistics),1,out)!= 1) {
    printf( "cannot fwrite file.\n" );
    return FALSE;
  }

  // now all the matrices and vectors
  fwrite_mat(out,rls->P);
  fwrite_mat(out,rls->beta);
  fwrite_vec(out,rls->sum_error2);
  fwrite_vec(out,rls->sum_y2);
  fwrite_vec(out,rls->sum_y);

  fclose(out);

  printf("done\n");

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  readRLS
\date  September 2006
   
\remarks 

        read the given RLS from file

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     rls      : data structure of rls

 ******************************************************************************/
RLSStatisticsPtr
readRLS(char *name)
{
  FILE *in;
  char  fname[100];
  RLSStatistics    rls;
  RLSStatisticsPtr rls_ptr;

  sprintf(fname,"%s.rls",name);
  in = fopen(fname,"r");
  if (in == NULL) {
    printf("Cannot open file >%s< for read\n",fname);
    return NULL;
  }
  
  printf("Reading file >%s< ...",fname);

  // the defining main structure
  if (fread(&rls,sizeof(RLSStatistics),1,in)!= 1) {
    printf( "cannot fread file.\n" );
    return NULL;
  }

  if (rls.add_constant == 1)
    --rls.n_in;

  rls_ptr = initRecursiveLeastSquares(rls.n_in, rls.n_out, rls.add_constant, 
				      rls.ridge, rls.lambda, rls.name);
  
  // now all the matrices and vectors
  fread_mat(in,rls_ptr->P);
  fread_mat(in,rls_ptr->beta);
  fread_vec(in,rls_ptr->sum_error2);
  fread_vec(in,rls_ptr->sum_y2);
  fread_vec(in,rls_ptr->sum_y);

  rls_ptr->nMSE = rls.nMSE;
  rls_ptr->n_data = rls.n_data;

  fclose(in);

  printf("done\n");

  return rls_ptr;

}

/*!*****************************************************************************
 *******************************************************************************
\note  initHistogram
\date  September 2006
   
\remarks 

        initializes a 1D histogram

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     min          : min value of histogram bins
 \param[in]     max          : max value of histogram bins
 \param[in]     n_bins       : the number of bins in histogram
 \param[in]     lambda       : forgetting factor
 \param[in]     name         : name of rls model

 ******************************************************************************/
HistogramStatisticsPtr
initHistogram(double min, double max, int n_bins,
	      double lambda, char *name)
{
  int i;
  HistogramStatisticsPtr hist;

  // get data structure and memory
  hist         = (HistogramStatisticsPtr) my_calloc(1,sizeof(HistogramStatistics),MY_STOP);
  hist->min    = min;
  hist->max    = max;
  hist->n_bins = n_bins;
  hist->lambda = lambda;
  hist->n_data = 0;
  strcpy(hist->name,name);

  hist->n_data_bin  = my_vector(1,n_bins);
  hist->n_success   = my_vector(1,n_bins);
  hist->p           = my_vector(1,n_bins);

  for (i=1; i<=n_bins; ++i)
    hist->p[i] = 0.5;

  return hist;
}

/*!*****************************************************************************
 *******************************************************************************
\note  writeHistogram
\date  September 2006
   
\remarks 

        write the given histogram to file

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     hist      : data structure of histogram

 ******************************************************************************/
int
writeHistogram(HistogramStatisticsPtr hist)
{
  FILE *out;
  char  name[100];

  sprintf(name,"%s.hist",hist->name);
  out = fopen(name,"w");
  if (out == NULL) {
    printf("Cannot open file >%s< for write\n",name);
    return FALSE;
  }
  
  printf("Writing to file >%s< ...",name);

  // the defining main structure
  if (fwrite(hist,sizeof(HistogramStatistics),1,out)!= 1) {
    printf( "cannot fwrite file.\n" );
    return FALSE;
  }

  // now all the matrices and vectors
  fwrite_vec(out,hist->n_data_bin);
  fwrite_vec(out,hist->n_success);
  fwrite_vec(out,hist->p);

  fclose(out);

  printf("done\n");

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  readHistogram
\date  September 2006
   
\remarks 

        read the given Histogram from file

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     name      : name of the histogram file (without .hist suffix)

 ******************************************************************************/
HistogramStatisticsPtr
readHistogram(char *name)
{
  FILE *in;
  char  fname[100];
  HistogramStatistics    hist;
  HistogramStatisticsPtr hist_ptr;

  sprintf(fname,"%s.hist",name);
  in = fopen(fname,"r");
  if (in == NULL) {
    printf("Cannot open file >%s< for read\n",fname);
    return NULL;
  }
  
  printf("Reading file >%s< ...",fname);

  // the defining main structure
  if (fread(&hist,sizeof(HistogramStatistics),1,in)!= 1) {
    printf( "cannot fread file.\n" );
    return NULL;
  }

  hist_ptr = initHistogram(hist.min, hist.max, hist.n_bins,
			   hist.lambda, hist.name);
  
  // now all the matrices and vectors
  fread_vec(in,hist_ptr->n_data_bin);
  fread_vec(in,hist_ptr->n_success);
  fread_vec(in,hist_ptr->p);

  // copy constant values
  hist_ptr->n_data = hist.n_data;

  fclose(in);

  printf("done\n");

  return hist_ptr;

}

/*!*****************************************************************************
 *******************************************************************************
\note  histogramClassifier
\date  September 2006
   
\remarks 

        a 1D histogram classifier

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in,out] hist       : data structure of histogram
 \param[in]     x          : input data 
 \param[in]     s          : s >= 0.5 success, s < 0.5 failure
 \param[in]     w          : weight of this data point

 ******************************************************************************/
void
histogramClassifier(HistogramStatisticsPtr hist, double x, double s, double w)
{
  int    i,j;
  double delta;
  int    n;

  // determine into which bin x falls
  delta = (hist->max - hist->min)/(double)hist->n_bins;
  n = (int) rint((x - hist->min)/delta);
  if (n < 1)
    n = 1;
  if (n > hist->n_bins)
    n = hist->n_bins;

  // update the statistics of this bin
  if ( s > 0.5) 
    s = 1.0;
  else
    s = 0.0;

  hist->n_data_bin[n]= hist->n_data_bin[n] * hist->lambda + w;
  hist->n_success[n] = hist->n_success[n] * hist->lambda + s*w;
  hist->p[n]         = hist->n_success[n]/hist->n_data_bin[n];

  ++hist->n_data;

}

/*!*****************************************************************************
 *******************************************************************************
\note  predictHistogram
\date  September 2006
   
\remarks 

        performs a prediction with the Histogram structure and neighborhood
        smoothing

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     hist       : data structure of histogram
 \param[in]     n_data_min : minumum data in bin to perform prediction
 \param[in]     n_neigbors : average over how many neighbor bins?
 \param[in]     x          : input data point
 \param[out]    y          : output (probability of success)

 ******************************************************************************/
int
predictHistogram(HistogramStatisticsPtr hist, int n_data_min, 
		 int n_neighbors, double x, double *y)
{
  int     i,j;
  double delta;
  int    n;
  double sum_w;
  double sum_data;
  double sum_data_bin;
  int    sn,en;
  double w;

  // check for sufficient data
  if (hist->n_data < n_data_min) {
    *y = 0.5;
    return FALSE;
  }

  // determine into which bin x falls
  delta = (hist->max - hist->min)/(double)hist->n_bins;
  n = (int) rint((x - hist->min)/delta);
  if (n < 1)
    n = 1;
  if (n > hist->n_bins)
    n = hist->n_bins;

  sn = n - n_neighbors;
  if (sn < 1)
    sn = 1;
  en = n + n_neighbors;
  if (en > hist->n_bins)
    en = hist->n_bins;

  sum_w        = 1.e-10;
  sum_data     = 0.0;
  sum_data_bin = 0.0;
  for (i=sn; i<=en; ++i) {
    w = ((n_neighbors - abs(n-i)) * 2.0 + 1)*hist->n_data_bin[i];
    sum_w += w;
    sum_data += hist->p[i] * w;
    sum_data_bin += hist->n_data_bin[i] * w;
  }

  // check for sufficient data in bin
  if (sum_data_bin/sum_w < n_data_min || sum_w == 0.0) {
    *y = 0.5;
    return FALSE;
  }

  *y = sum_data/sum_w;
  return TRUE;

}

