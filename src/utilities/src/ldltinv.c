/*!=============================================================================
  ==============================================================================

  \file    ldltinv.c

  \author  Stefan Schaal
  \date    August 1993

  ==============================================================================
  \remarks

      general program for LDL' decomposition

  ============================================================================*/
#include "stdio.h"
#include "math.h"
#include "stdlib.h"

#include "utility.h"

#define TRUE 1
#define FALSE 0

/*!*****************************************************************************
 *******************************************************************************
\note  my_inv_ldlt
\date  August 93

\remarks 

       take the elements of A x = b and generates x.
       note: A must be symmetric

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

      none

 ******************************************************************************/
int
my_inv_ldlt(double **a_mat, double *b_vec, int n_dim, double *x_vec)


{

  double              tolerance = 1.0e-30;
  double              dk, lik;
  int                 i, j, k, p;
  double              zi;
  double              xi;



  double **ldlt_l;
  double  *ldlt_d;
  double  *ldlt_z;
  double  *ldlt_y;
  double  *ldlt_r;


  /* some temporary arrays and matrices */

  ldlt_l = my_matrix(1, n_dim, 1, n_dim);
  ldlt_d = my_vector(1,n_dim);
  ldlt_r = my_vector(1,n_dim);
  ldlt_y = my_vector(1,n_dim);
  ldlt_z = my_vector(1,n_dim);
 
  
  /* Compute the L matrix and D vector from a_mat */
  
  
  /* copy a_mat to l matrix */
  
  for (k = 1; k <= n_dim; ++k) {
    ldlt_l[k][k] = a_mat[k][k];
    for (j = k + 1; j <= n_dim; ++j) {
      ldlt_l[j][k] = ldlt_l[k][j] = a_mat[k][j];
    }
  }
  
  /* this does Gaussian elimination for the symmetric matrix;
  	 the result should be just a diagonal matrix */

  for (k = 1; k <= n_dim; ++k) {
    for (p = 1; p < k; ++p) {
      ldlt_r[p] = ldlt_d[p] * ldlt_l[k][p];
    }
    dk = ldlt_l[k][k];
    for (p = 1; p < k; ++p) {
      dk -= ldlt_l[k][p] * ldlt_r[p];
    }
    ldlt_d[k] = dk;
    if (fabs(dk) <= tolerance) {
      printf("\nLDLT singular matrix: d(%d) = %f\n", k, dk);
      return FALSE;
    }
    for (i = k + 1; i <= n_dim; ++i) {
      lik = ldlt_l[i][k];
      for (p = 1; p < k; ++p) {
		lik -= ldlt_l[i][p] * ldlt_r[p];
      }

      ldlt_l[i][k] = lik / dk;
    }
  }
  
  /* clean up l matrix */
  
    for (k = 1; k <= n_dim; k++) {
      ldlt_l[k][k] = 1.0;
      for (j = 1; j < k; j++) {
		ldlt_l[j][k] = 0.0;
      }
    }


  /* Compute Lz = b_vec  and  Dy = z: this is back substitution */

  for (i = 1; i <= n_dim; i++) {
    zi = b_vec[i];
    for (j = 1; j < i; j++) {
      zi -= ldlt_z[j] * ldlt_l[i][j];
    }
    ldlt_z[i] = zi;
  }
  
  
  for (i = 1; i <= n_dim; i++) {
    ldlt_y[i] = ldlt_z[i] / ldlt_d[i];
  }


  /* Compute L^T*x = y.: back substitution part two */


  for (i = n_dim; i >= 1; i--) {
    xi = ldlt_y[i];
    for (j = i + 1; j <= n_dim; ++j) {
      xi -= x_vec[j] * ldlt_l[j][i];
    }
    x_vec[i] = xi;
    
  }
  

  my_free_matrix(ldlt_l,1, n_dim, 1, n_dim);
  my_free_vector(ldlt_d,1,n_dim);
  my_free_vector(ldlt_r,1,n_dim);
  my_free_vector(ldlt_y,1,n_dim);
  my_free_vector(ldlt_z,1,n_dim);

  return TRUE;

}
