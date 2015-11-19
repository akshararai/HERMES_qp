/*!=============================================================================
  ==============================================================================

  \file    parm_opt.c

  \author  Stefan Schaal
  \date    February 1994

  ==============================================================================
  \remarks

      program for parameter optimization with hard constraints, e.g.,
      typically modeled by Lagrange multipliers. Both Newton's method
      and normal gradient method are implemented

  ============================================================================*/
#include "stdio.h"
#include "math.h"
#include "utility.h"

#define NEWTON_THRESHOLD 0.0001


/*!*****************************************************************************
 *******************************************************************************
\note  parm_opt
\date  10/20/91

\remarks 
		this is the major optimzation program

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     tol      : error tolernance to be achieved
 \param[in]     n_parm   : number of parameters to be optimzed
 \param[in]     n_con    : number of contraints to be taken into account
 \param[in]     f_dLda   : function which calculates the derivative of the 
                    optimziation criterion with respect to the parameters;
		    must return vector
 \param[in]     f_dMda   : function which calculates the derivate of the 
                    constraints with respect to parameters
		    must return matrix
 \param[in]     f_M      : constraints function, must always be formulted to
                    return 0 for properly fulfilled constraints
 \param[in]     f_L      : function to calculate simple cost (i.e., constraint
	            cost NOT included), the constraint costs are added
		    by this program automatically, the function returns
		    a double scalar
 \param[in]     f_dLdada : second derivative of L with respect to the parameters, 
                    must be a matrix of dim n_parm x n_parm
 \param[in]     f_dMdada : second derivative of M with respect to parameters,
                    must be a matrix n_con*n_parm x n_parm
 \param[in]     use_newton: TRUE or FALSE to indicate that second derivatives
                    are given and can be used for Newton algorithm
 \param[in,out] a        : initial setting of parameters and also return of
                    optimal value (must be a vector, even if scalar)
 \param[out]    final_cost: the final cost
 \param[out]    err       : the sqrt of the squared error of all constraints
      				
      NOTE: - program returns TRUE if everything correct, otherwise FALSE
      			- always minimizes the cost!!!
      			-	algorithms come from Dyer McReynolds
      			
      NOTE: besides the possiblity of a bug, the Newton method seems to
      			sacrifice the validity of the constraint a little up to
      			quite a bit and should be used prudently

 ******************************************************************************/
int
parm_opt(double *a,int n_parm, int n_con, double *tol, void (*f_dLda)(), 
	void (*f_dMda)(), void (*f_M)(), double (*f_L)(), void (*f_dMdada)(),
	void (*f_dLdada)(), int use_newton, double *final_cost, double *err)

{
  
  register int i,j,n;
  double cost= 999.e30;
  double last_cost = 0.0;
  double *mult=NULL, *new_mult=NULL; /* this is the vector of Lagrange mulitplier */
  double **dMda=NULL, **dMda_t=NULL;
  double *dLda;
  double *K=NULL; /* the error in the constraints */
  double eps = 0.025; /* the learning rate */
  double **aux_mat=NULL; /* needed for inversion of matrix */
  double *aux_vec=NULL;
  double *new_a;
  double **dMdada=NULL;
  double **dLdada=NULL;
  double **A=NULL; /* big matrix, a combination of several other matrices */
  double *B=NULL;  /* a big vector */
  double **A_inv=NULL;
  int    rc=TRUE;
  long   count = 0;
  int    last_sign = 1;
  int    pending1 = FALSE, pending2 = FALSE;
  int    firsttime = TRUE;
  int    newton_active = FALSE;
  
  dLda    = my_vector(1,n_parm);
  new_a   = my_vector(1,n_parm);
  if (n_con > 0) {
    mult    = my_vector(1,n_con);
    dMda    = my_matrix(1,n_con,1,n_parm);
    dMda_t  = my_matrix(1,n_parm,1,n_con);
    K       = my_vector(1,n_con);
    aux_mat = my_matrix(1,n_con,1,n_con);
    aux_vec = my_vector(1,n_con);
  }

  
  if (use_newton) {
    
    dLdada   = my_matrix(1,n_parm,1,n_parm);
    A        = my_matrix(1,n_parm+n_con,1,n_parm+n_con);
    A_inv    = my_matrix(1,n_parm+n_con,1,n_parm+n_con);
    B        = my_vector(1,n_parm+n_con);

    if (n_con > 0) {
      dMdada   = my_matrix(1,n_con*n_parm,1,n_parm);
      new_mult = my_vector(1,n_con);
    }
    
    for (i=1+n_parm; i<=n_con+n_parm; ++i) {
      for (j=1+n_parm; j<=n_con+n_parm; ++j) {
	A[i][j] = 0.0;
      }
    }
    
  }
  
  
  while (fabs(cost-last_cost) > *tol) {
    
    ++count;
    pending1 = FALSE;
    pending2 = FALSE;
    
  AGAIN:
    
    /* calculate the current Lagrange multipliers */
    
    if (n_con > 0) {
      (*f_M)(a,K);       /* takes the parameters, returns residuals */
      (*f_dMda)(a,dMda); /* takes the parameters, returns the Jacobian */
    }
    (*f_dLda)(a,dLda); /* takes the parameters, returns the gradient */
    
    if (n_con > 0) {
      mat_trans(dMda,dMda_t);
    }
    
    
    if (newton_active) {
      if (n_con > 0) {
	(*f_dMdada)(a,dMdada);
      }
      (*f_dLdada)(a,dLdada);
    }
    
    
    /* the first step is always a gradient step */
    
    if (newton_active) {
      
      if (firsttime) {
	firsttime = FALSE;
	eps = 0.1; 
      }
      
      
      /* build the A matrix */
      
      for (i=1; i<=n_parm; ++i) {
	for (j=1; j<=n_parm; ++j) {
	  
	  A[i][j] = dLdada[i][j];
	  
	  for (n=1; n<=n_con; ++n) {
	    A[i][j] += mult[n]*dMdada[n+(i-1)*n_con][j];
	  }
	  
	}
      }
      
      
      for (i=1+n_parm; i<=n_con+n_parm; ++i) {
	for (j=1; j<=n_parm; ++j) {
	  A[j][i] = A[i][j] = dMda[i-n_parm][j];
	}
      }
      
      
      /* build the B vector */
      
      if (n_con > 0) {
	mat_vec_mult(dMda_t,mult,B);
      }
      
      
      for (i=1; i<=n_con; ++i) {
	B[i+n_parm] = K[i];
      }
      
      /* invert the A matrix */
      
      if (!my_inv_ludcmp(A, n_con+n_parm, A_inv)) {
	rc = FALSE;
	break;
      }
      
      mat_vec_mult(A_inv,B,B);
      vec_mult_scalar(B,eps,B);
      
      for (i=1; i<=n_parm; ++i) {
	new_a[i] = a[i] + B[i];
      }
      
      for (i=1; i<=n_con; ++i) {
	new_mult[i] = mult[i] + B[n_parm+i];
      }
      
      
      
    } else {
      
      
      if (n_con > 0) {

	/* the mulitpliers are updated according:
	   mult = (dMda dMda_t)^(-1) (K/esp - dMda dLda_t)       */

	mat_mult(dMda,dMda_t,aux_mat);
      
	if (!my_inv_ludcmp(aux_mat, n_con, aux_mat)) {
	  rc = FALSE;
	  break;
	}
      
	mat_vec_mult(dMda,dLda,aux_vec);
      
	vec_mult_scalar(K,1./eps,K);
      
	vec_sub(K,aux_vec,aux_vec);
      
	mat_vec_mult(aux_mat,aux_vec,mult);

      }
      
      
      /* the update step looks the following:
	 a_new = a - eps * (dLda + mult_t * dMda)_t     */
      
      if (n_con > 0) {

	vec_mat_mult(mult,dMda,new_a);
      
	vec_add(dLda,new_a,new_a);

      } else {

	vec_equal(dLda,new_a);

      }
      
      vec_mult_scalar(new_a,eps,new_a);
      
      vec_sub(a,new_a,new_a);
      
    }
    
    
    if (count == 1 && !pending1) {
      
      last_cost  = (*f_L)(a);

      if (n_con > 0) {
	(*f_M)(a,K);
	last_cost += vec_mult_inner(K,mult);
      }
      
    } else {
    
      last_cost = cost;

    }
    
    /* calculate the updated cost */
    
    
    cost = (*f_L)(new_a);
    /*printf("   %f\n",cost);*/

    if (n_con > 0) {

      (*f_M)(new_a,K);
      
      if (newton_active) {
	cost += vec_mult_inner(K,new_mult);
      } else {
	cost += vec_mult_inner(K,mult);
      }

    }
    
    /* printf("last=%f new=%f\n",last_cost,cost); */
    
    
    /* check out whether we reduced the cost */
    
    if (cost > last_cost && fabs(cost-last_cost) > *tol) {
      
      /* reduce the gradient climbing rate: sometimes a reduction of eps
	 causes an increase in cost, thus leave an option to increase
	 eps */
      
      cost = last_cost; /* reset last_cost */
      
      
      if (pending1 && pending2) {
	
	/* this means that either increase nor decrease
	   of eps helps, ==> leave the program */
	
	rc = TRUE;
	break;
	
      } else if (pending1) {
	
	eps *= 4.0;  /* the last cutting by half did not help, thus
			multiply by 2 to get to previous value, and
			one more time by 2 to get new value */
	pending2 = TRUE;
	
      } else {
	
	eps /= 2.0;
	pending1 = TRUE;
	
      }
      
      goto AGAIN;
      
    } else {
      
      vec_equal(new_a,a);
      if (newton_active && n_con > 0) {
	vec_equal(new_mult,mult);
      }
      if (use_newton && fabs(cost-last_cost) < NEWTON_THRESHOLD) 
	newton_active = TRUE;
      
    }
    
    
  }
  
  my_free_vector(dLda,1,n_parm);
  my_free_vector(new_a,1,n_parm);
  if (n_con > 0) {
    my_free_vector(mult,1,n_con);
    my_free_matrix(dMda,1,n_con,1,n_parm);
    my_free_matrix(dMda_t,1,n_parm,1,n_con);
    my_free_vector(K,1,n_con);
    my_free_matrix(aux_mat,1,n_con,1,n_con);
    my_free_vector(aux_vec,1,n_con);
  }
  
  if (use_newton) {
    
    my_free_matrix(dLdada,1,n_parm,1,n_parm);
    my_free_matrix(A,1,n_parm+n_con,1,n_parm+n_con);
    my_free_matrix(A_inv,1,n_parm+n_con,1,n_parm+n_con);
    my_free_vector(B,1,n_parm+n_con);
    if (n_con > 0) {
      my_free_matrix(dMdada,1,n_con*n_parm,1,n_parm);
      my_free_vector(new_mult,1,n_con);
    }
    
  }
  *final_cost = cost;
  *tol = fabs(cost-last_cost);
  if (n_con > 0) {
    *err = sqrt(vec_mult_inner(K,K));
  } else {
    *err = 0.0;
  }
/*  
  printf("count=%ld  rc=%d\n",count,rc);
*/ 
  return rc;
  
}
