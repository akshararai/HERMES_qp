/*!=============================================================================
  ==============================================================================

  \file    stability.c

  \author  Stefan Schaal
  \date    April 1993

  ==============================================================================
  \remarks

	general program for several kinds of stability analysis

  ============================================================================*/

#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "utility.h"

#define N_STEPS_RENORMALIZATION 20

/*!*****************************************************************************
 *******************************************************************************
\note  liapunov_exp.c
\date  April, 1993

\remarks 

        Calculates the Liapunov exponents for a given trajectory of a
	given dynamic system

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     n_dim     : number of dimensions of the system (i.e., state space
                     variables)
 \param[in]     n_iter    : number of iterations for determining the exponents
 \param[in]     func      : function which takes an inital state and return the
                     the outcoming state and the Jacobian of it. 
 \param[in]     state     : the inital state at which the program shall start
                     running
 \param[in]     tau       : the time interval between two subsequent steps of
                     evaluation by function (for maps, set to tau=1)
 \param[out]    exponents : a vector of 


      note: program returns TRUE or FALSE, depending whether things 
            worked out or an error occured.
      note: the program obeys numerical recipies conventions, except that
            I always use double precision instead of foat.

 ******************************************************************************/
int
liapunov_exp(int n_dim, int n_iter, void (*func)(), double *state, 
	     double tau, double *exponents)
{

  double     count = 0.0;
  double    *x_n;        /* the state as local variable */
  double   **j;          /* the jacobian matrix */
  double    *x_n1;       /* the next state */
  double   **w;          /* the volume in space whose deformation will
			    be tracked in time to determine the
			    Liapunov exponents */
  double   **wn;         /* the volume in space after n iterations */
  double    *d;          /* the norm of each vector of w */

  int        i,n,m;
  double     aux;
  double    *temp;
  double     time;



  /* get the vectors and matrices which are needed */

  x_n       = my_vector(1,n_dim);
  x_n1      = my_vector(1,n_dim);
  d         = my_vector(1,n_dim);
  temp      = my_vector(1,n_dim);
  j         = my_matrix(1,n_dim,1,n_dim);
  w         = my_matrix(1,n_dim,1,n_dim);
  wn        = my_matrix(1,n_dim,1,n_dim);


  /* initalize these variables */

  for (i=1; i<=n_dim; ++i) {
    for (n=1; n<=n_dim; ++n) {
      w[i][n] = 0.0;
      if (i == n) w[i][n] = 1.0;
    }
  }

  for (i=1; i<=n_dim; ++i) {
    x_n[i] = state[i];
    d[i]   = 0.0;
  }

  mat_equal(w,wn);
  
  

  /* now do several iterations over the system and track the volume in time */

  for (i=1; i<=n_iter; ++i) {

    count += 1;
    time = count * tau;
    (*func)(x_n, time, j, x_n1);
    
    /* the first distortion of the volume is then given by; */
    
    mat_mult(j,wn,wn);
    
    if (count == N_STEPS_RENORMALIZATION || i == n_iter) {
      
      /* renormalize the volume by Gram-Schmidt procedure, and keep
	 track of the expansion so far */

      mat_trans(wn,wn);
	
      for (n=1; n<=n_dim; ++n) {
      
	vec_equal(wn[n], w[n]);
	
	for (m=1; m<=n-1; ++m) {
	
	  /* this is the Gram-Schmidt step */
	
	  aux = vec_mult_inner(w[m],wn[n]);
	
	  vec_mult_scalar(w[m],aux,temp);
	
	  vec_sub(w[n],temp,w[n]);
	
	}
	
	aux = vec_mult_inner(w[n],w[n]);
		
	aux = sqrt(aux);
		
	vec_mult_scalar(w[n],1./aux,w[n]);
		
	d[n] += log(aux);
		
      }

      count = 0;
      
      mat_equal(w,wn);

      mat_trans(wn,wn);

      /*
	for (n=1;n<=n_dim;++n) {
      	for (m=1;m<=n_dim; ++m) {
	printf("%f ",wn[n][m]);
      	}
      	printf("\n");
	}
	*/
    }
    
    vec_equal(x_n1,x_n);
  
  }

  

  /* calculate the Liapunov expontents */

  for (i=1; i<=n_dim; ++i) {

    exponents[i] = d[i] / ((double)n_iter * tau);

  }
  
  /* free the vectors and matrices */

  my_free_vector(x_n,1,n_dim);
  my_free_vector(x_n1,1,n_dim);
  my_free_vector(d,1,n_dim);
  my_free_vector(temp,1,n_dim);
  my_free_matrix(j,1,n_dim,1,n_dim);
  my_free_matrix(w,1,n_dim,1,n_dim);
  my_free_matrix(wn,1,n_dim,1,n_dim);

  return TRUE;

}
