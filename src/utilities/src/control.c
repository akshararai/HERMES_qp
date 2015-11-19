/*!=============================================================================
  ==============================================================================

  \file    control.c

  \author  Stefan Schaal
  \date    October 1992

  ==============================================================================
  \remarks

      program for various control calcultions

  ============================================================================*/

#define TRUE     1
#define FALSE    0
#define PI       3.141592654
#define EPSILON  0.01
#define LIMIT    10000

#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include "utility.h"


#define SVD_RELATIVE_THRESHOLD 1.0e-2
#define SVD_THRESHOLD 0.01

#define SVD            0
#define LUDCMP         1
#define INVERSE_METHOD LUDCMP

#define NORMAL_ITERATION 1
#define EIGEN_METHOD     2  /*!< incomplete yet; needs eigenvector computation */
#define HEWER_METHOD     3
#define DLQR_METHOD      NORMAL_ITERATION

/*!*****************************************************************************
 *******************************************************************************
\note  dlqr
\date  Oct. 25, 1992

\remarks 

      calculates the K matrix of a discrete LQR controller

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     fx         : A matrix of model x(n+1) = fx*x(n) + fu*u(n)
 \param[in]     fu         : B matrix of model 
 \param[in]     lxx        : weight matrix for x
 \param[in]     luu        : weight matrix for u
 \param[out]    d          : u(n) = -d * x(n)
 \param[in]     n_states   : number of states
 \param[in]     n_controls : number of controls
 \param[in,out] p           : the initial value function matrix

   note: q must be positive definite and r must be positive semi-definite
         matrices must start with index 1 (like numerical recipes)

   return  : TRUE or FALSE, i.e., calcuation successful or not

 ******************************************************************************/
int
dlqr(double **fx, double **fu, double **lxx, double **luu, double **d, 
     int n_states, int n_controls, double **p)

{

  double difference=99999;
  int i,j,m,n;
  int firsttime=TRUE;
  double info;
  static double **zxx,**zux,**zxu,**zuu,
  **fxt,**fut,**t1,**t2,**t3, **t4,**p_new, **zuu_inv;
  static double **diff;
  static int    *indx;
  int rc=FALSE;
  int count;
  double wmax, wmin;  
  static double **v, *w;
  static double **h;
  static double **luu_inv;
  static double **fx_invt;
  
  /* for Hewer's algorithm */

  static double **s;
  static double **wn;
  static double **vn;
  static double **phi;
  static double **eye;
  static double **f,**e,**ft;
  static int last_n_states = 0, last_n_controls = 0;



  /* I used the algorithm as outlined by Dyer and McReynolds 1970 */
  /* the entire algorithm is a multistage optimization problem in
     which the k matrix finally converges to a constant matrix. 
     It is not quite clear how long this actually takes. Thus I 
     stop the program after convergence or a certain LIMIT of
     iterations. */



  /* some temporary matrices */

  if (last_n_states != n_states || last_n_controls != n_controls) {

    if (last_n_states > 0) {
      my_free_matrix(zxx,1,last_n_states,1,last_n_states);
      my_free_matrix(zux,1,last_n_controls,1,last_n_states);
      my_free_matrix(zxu,1,last_n_states,1,last_n_controls);
      my_free_matrix(zuu,1,last_n_controls,1,last_n_controls);
      my_free_matrix(p_new,1,last_n_states,1,last_n_states);
      my_free_matrix(fxt,1,last_n_states,1,last_n_states);
      my_free_matrix(fut,1,last_n_controls,1,last_n_states);
      my_free_matrix(t1,1,last_n_states,1,last_n_states);
      my_free_matrix(t2,1,last_n_controls,1,last_n_states);
      my_free_matrix(t3,1,last_n_states,1,last_n_controls);
      my_free_matrix(t4,1,last_n_states,1,last_n_states);
      my_free_ivector(indx,1, last_n_controls);
      my_free_matrix(zuu_inv,1,last_n_controls,1,last_n_controls);
      my_free_matrix(diff,1,last_n_states,1,last_n_states);
      my_free_matrix(v,1,last_n_controls,1,last_n_controls);
      my_free_vector(w,1,last_n_controls);
      my_free_matrix(h,1,last_n_states+last_n_states,1,last_n_states+last_n_states);
      my_free_matrix(luu_inv,1,last_n_controls,1,last_n_controls);
      my_free_matrix(fx_invt,1,last_n_states,1,last_n_states);
      my_free_matrix(s,1,last_n_states,1,last_n_states);
      my_free_matrix(wn,1,last_n_states,1,last_n_states);
      my_free_matrix(vn,1,last_n_states,1,last_n_states);
      my_free_matrix(phi,1,last_n_states,1,last_n_states);
      my_free_matrix(eye,1,last_n_states,1,last_n_states);
      my_free_matrix(f,1,last_n_states,1,last_n_states);
      my_free_matrix(e,1,last_n_states,1,last_n_states);
      my_free_matrix(ft,1,last_n_states,1,last_n_states);
    }
    
    zxx     = my_matrix(1,n_states,1,n_states);
    zux     = my_matrix(1,n_controls,1,n_states);
    zxu     = my_matrix(1,n_states,1,n_controls);
    zuu     = my_matrix(1,n_controls,1,n_controls);
    p_new   = my_matrix(1,n_states,1,n_states);
    fxt     = my_matrix(1,n_states,1,n_states);
    fut     = my_matrix(1,n_controls,1,n_states);
    t1      = my_matrix(1,n_states,1,n_states);
    t2      = my_matrix(1,n_controls,1,n_states);
    t3      = my_matrix(1,n_states,1,n_controls);
    t4      = my_matrix(1,n_states,1,n_states);
    indx    = my_ivector(1, n_controls);
    zuu_inv = my_matrix(1,n_controls,1,n_controls);
    diff    = my_matrix(1,n_states,1,n_states);
    v       = my_matrix(1,n_controls,1,n_controls);
    w       = my_vector(1,n_controls);
    h       = my_matrix(1,n_states+n_states,1,n_states+n_states);
    luu_inv = my_matrix(1,n_controls,1,n_controls);
    fx_invt = my_matrix(1,n_states,1,n_states);
    s       = my_matrix(1,n_states,1,n_states);
    wn      = my_matrix(1,n_states,1,n_states);
    vn      = my_matrix(1,n_states,1,n_states);
    phi     = my_matrix(1,n_states,1,n_states);
    eye     = my_matrix(1,n_states,1,n_states);
    f       = my_matrix(1,n_states,1,n_states);
    e       = my_matrix(1,n_states,1,n_states);
    ft      = my_matrix(1,n_states,1,n_states);

  }

  last_n_states = n_states;
  last_n_controls = n_controls;


  mat_trans(fx,fxt);
  mat_trans(fu,fut);

  mat_zero(p);
  
  /* the identity matrix as n_states x n_states */

  for (i=1; i<=n_states; ++i) {
    for (j=1; j<=n_states; ++j) {
      eye[i][j] = 0.0;
      if (i==j) eye[i][j] = 1.0;
    }
  }

  count = 0;
  
  while (difference > EPSILON && count < LIMIT) {

    ++count;
    
    switch (DLQR_METHOD) {


    case NORMAL_ITERATION:
      
      mat_mult(fxt, p, t1);
      mat_mult(t1, fx, zxx);
      mat_add(lxx, zxx, zxx);
      
      mat_mult(t1, fu, zxu);
      
      mat_mult(fut, p, t2);
      mat_mult(t2, fu, zuu);
      mat_add(luu, zuu, zuu);
      
      mat_mult(t2, fx, zux);
      

      /* finally, I have to invert zuu */
      
      if (INVERSE_METHOD == LUDCMP) {
	
	/*	for (i=1; i<=n_controls; ++i) {
		for (j=1; j<=n_controls; ++j) {
		printf("  %f ",zuu[i][j]);
		}
		printf("\n");
		}
		*/
	if (!my_inv_ludcmp(zuu,  n_controls,  zuu_inv)) {
	  
	  /*		printf("count = %d\n",count);
			
			for (i=1; i<=n_states; ++i) {
			for (j=1; j<=n_states; ++j) {
			printf("%f ",fx[i][j]);
			}
			printf("\n");
			}
			for (i=1; i<=n_states; ++i) {
			for (j=1; j<=n_controls; ++j) {
			printf("%f ",fu[i][j]);
			}
			printf("\n");
			}
			for (i=1; i<=n_controls; ++i) {
			for (j=1; j<=n_controls; ++j) {
			printf("%f ",zuu[i][j]);
			}
			printf("\n");
			}
			*/	
	  printf("LUDCMP choked!!\n");
	  return FALSE;
	}
	
	
      } else if (INVERSE_METHOD == SVD) {
	
	/* make zuu_inv the identity matrix */
	
	for (i=1; i<=n_controls; ++i) {
	  for (j=1; j<=n_controls; ++j) {
	    zuu_inv[i][j] = 0.0;
	    if (i==j) zuu_inv[i][j] = 1.0;
	  }
	}
	
	/* do an svd of zuu */
	
	my_svdcmp( zuu, n_controls, n_controls, w, v );
	
	/* Choose threshold singular value. */
	
	wmax = 0.0;
	for( j = 1; j <= n_controls; j++ ) if( w[j] > wmax )  wmax = w[j];
	wmin = wmax * SVD_RELATIVE_THRESHOLD;
	if ( wmin > SVD_THRESHOLD ) wmin = SVD_THRESHOLD;
	
	/* Zero singular values < wmin */
	
	for( j = 1; j <= n_controls; j++ ) {
	  if ( w[j] < wmin ) {
	    w[j] = 0.0;
	  }
	}
	
	/* now generate the inverse. Due to the setup of matrices, I will
	   actuall create the transpose of the inverse */
	
	for (i=1; i<=n_controls; ++i) {
	  my_svbksb( zuu, w, v, n_controls, n_controls, zuu_inv[i], zuu_inv[i] );
	}
	
	
	/* now take the transpose of this inverse to make it what I want */
	
	mat_trans(zuu_inv, zuu_inv);
	
	
      } 

      /* finally calculate the update of p */
      
      mat_mult(zuu_inv, zux, t2);
      mat_mult(zxu, t2, p_new);
      mat_sub(zxx, p_new, p_new);
      
      mat_sub(p_new, p,  diff);
      
      mat_equal(p_new, p);
      
      difference = 0;
      for (i=1; i<=n_states; ++i) {
	for (j=1; j<=n_states; ++j) {
	  difference += fabs(diff[i][j]);
	}
      }
      
      
      /* now check the result of this show */
      /* NOTE: I calculate the control matrix also at the last step of
	 the iterations in case it is to be used even if the convergence
	 was not satistfying */
      
      if (difference < EPSILON || count == LIMIT) {
	
	/* this calculates the k-matrix */
	
	mat_mult(zuu_inv, zux, d);
	
	if (difference < EPSILON) rc = TRUE;
	
      } else {
	
	rc = FALSE;
	
      }
      
/*      
      printf("return code=%d difference=%f\n",rc,difference);
      for (i=1; i<=n_states; ++i) {
	for (j=1; j<=n_states; ++j) {
	  printf("%f ",d[i][j]);
	}
	printf("\n");
      }
*/      

      break;


    case HEWER_METHOD:

      /* this method I found in the 1974 paper of Kleinman */


      if (firsttime) {

	/* the extension which Kleinman contributed to Hewer's and other people's
	   algorithms is that he computes an initial matrix for the iterations
	   which guarantees to make the iteration stable */

	firsttime = FALSE;

	if (!my_inv_ludcmp(luu, n_controls, luu_inv)) {
	  
	  printf("weight matrix luu is singular\n");
	  return FALSE;
	  
	}
	mat_mult(fu,luu_inv,t3);
	mat_mult(t3,fut,s);

	/* some efficient computational tricks are involved here */

	n = (int)(log((double)n_states)/log(2.0)+.9999999);
	mat_equal(s,e);
	mat_equal(fx,f);
	mat_trans(f,ft);


	for (i=1; i<=n; ++i) {

	  mat_mult(ft,e,t1);
	  mat_mult(t1,f,t1);
	  mat_add(e,t1,e);
	  mat_mult(f,f,f);
	  mat_trans(f,ft);

	}


	if (!my_inv_ludcmp(e, n_states, t1)) {
	  
	  printf("special matrix >e< for Hewer initialization is singular\n");
	  return FALSE;
	  
	}	
	mat_mult(ft,t1,vn);
	mat_mult(vn,f,vn);

	/* vn is the crucial inital matrix for me to get started */

      } 
/*
      printf("vn-matrix:\n");
      for (i=1; i<=n_states; ++i) {
	for (j=1; j<=n_states; ++j) {
	  printf("%f ",vn[i][j]);
	}
	printf("\n");
      }
*/      
	printf("-------xxxxxxxxxxx\n");

      /* here begins the normal iterative algorithm */

      mat_mult(s,vn,t1);
      mat_add(eye,t1,t1);
      if (!my_inv_ludcmp(t1, n_states, t1)) {
	
	printf("special matrix >s  vn< for Hewer interation is singular\n");
	return FALSE;
	
      }	

      mat_mult(t1,fx,phi);

      mat_trans(phi,t4);
      mat_mult(t4,vn,t1);
      mat_mult(t1,s,t1);
      mat_mult(t1,vn,t1);
      mat_mult(t1,phi,t1);

      mat_add(t1,lxx,t1);

      mat_mult(t4,vn,t4);
      mat_mult(t4,phi,t4);

      mat_equal(vn, p);  /* temp store old vn */
      mat_add(t1,t4,vn);
      mat_sub(p, vn, diff);

      difference = 0;
      for (i=1; i<=n_states; ++i) {
	for (j=1; j<=n_states; ++j) {
	  difference += fabs(diff[i][j]);
	}
      }
      
      /* now check the result of this show */
      
      if (difference < EPSILON) {
	
	/* this calculates the k-matrix */
	
	mat_mult(fut, vn, t2);
	mat_mult(t2, fu, v);
	mat_add(v, luu, v);
	
	if (!my_inv_ludcmp(v, n_controls, v)) {
	  
	  printf("matrix to calculate k >r+b's b< for Hewer is singular\n");
	  return FALSE;
	  
	}	
	mat_mult(v, fut, t2);
	mat_mult(t2, vn, t2);
	mat_mult(t2, fx, d);
	

	rc = TRUE;
	
      } else {
	
	rc = FALSE;
	
      }
      
/*      
      printf("return code=%d difference=%f\n",rc,difference);
      for (i=1; i<=n_states; ++i) {
	for (j=1; j<=n_states; ++j) {
	  printf("%f ",d[i][j]);
	}
	printf("\n");
      }
*/      

      break;



    case EIGEN_METHOD:

      /* this methods uses the discrete Ricatti equation to generate the solution; 
	 it does not need iterations to find this solution; see Matlab */	
      
      /* step one: some inverse are needed */
      
      if (!my_inv_ludcmp(fx, n_states, fx_invt)) {
	
	printf("state matrix fu is singular\n");
	return FALSE;
	
      }
      
      mat_trans(fx_invt, fx_invt);
      
      if (!my_inv_ludcmp(luu, n_states, luu_inv)) {
	
	printf("weight matrix luu is singular\n");
	return FALSE;
	
      }
      
      /* build the Hamiltonian matrix; note: this is a bigger matrix and I will
	 fool around with the matrix pointers to place the necessary submatrices
	 in this matrix */
      
      mat_equal(fu,h);
      mat_mult(h,luu_inv,h);
      mat_mult(h,fut,h);
      mat_mult(h,fx_invt,h);
      
      for (i=1;i<=n_states;++i) for (j=1;j<=n_states;++j) h[i][j+n_states] = -h[i][j];
      
      mat_mult(h,lxx,h);
      mat_add(h,fx,h);
      
      mat_mult(fxt,lxx,t1);
      for (i=1;i<=n_states;++i) for(j=1;j<=n_states;++j) h[i+n_states][j] = -t1[i][j];
      
      for (i=1;i<=n_states;++i) for (j=1;j<=n_states;++j) 
	h[i+n_states][j+n_states] = fx_invt[i][j];
      
      
      /* now I need the eigenvalues and eigenvectors of this Hamiltonian */
      
      
      
        /************* not really implemented yet *****************/


      break;


    default:

      printf("wrong dlqr method\n hit continue to exit ....\n");
      getchar();
      exit(-1);

      break;

    }

  }
  
  if (count >= LIMIT) printf("DLWR choked because of LIMIT\n");
  
  return rc;

}


      
