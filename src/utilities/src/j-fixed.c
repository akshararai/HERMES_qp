/*!=============================================================================
  ==============================================================================

  \file    j-fixed.c

  \author  Stefan Schaal
  \date    June 1993

  ==============================================================================
  \remarks

      handles singular value decompositions

  ============================================================================*/


#include "stdio.h"
#include "math.h"
#include "stdlib.h"
#include "utility.h"

#define TRUE 1
#define FALSE 0

int                 N_SVD;

/*!<  #define SVD_THRESHOLD 0.1 conservative setting */
#define SVD_THRESHOLD 0.001
#define SVD_RELATIVE_THRESHOLD 1.0e-3

int
check_svd(double **a, double **u, double *w, double **v);
int
check_x(double **a, double *x, double *b);


/*!*****************************************************************************
 *******************************************************************************
\note  correct_command_svd
\date  10/20/91

\remarks 

      the function takes the matrix a, the result vector b, and the unkown
      vector x. It does a svd on a, zeros out the singular values and computes
      then the vector x.

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     a    : matrix of linear equation a x = b
 \param[in]     b    : see above
 \param[in]     svdr : svd relative threshold
 \param[in]     svdt : svd threshold
 \param[out]    x    : see above
 \param[in]     n_dim: number of dimensions to regression

 ******************************************************************************/
int
correct_command_svd(double **a, double *b, double svdr, double svdt, double *x, int n_dim)
{
  double              wmax, wmin, **u, *w, **v;
  int                 i, j, n = 0;

  N_SVD = n_dim;


  /* the the matrices for the svd */

  u = my_matrix(1, N_SVD, 1, N_SVD);
  v = my_matrix(1, N_SVD, 1, N_SVD);
  w = my_vector(1, N_SVD);


  /* Copy a into u. */

  for (i = 1; i <= N_SVD; i++) {
    for (j = 1; j <= N_SVD; j++) {
      u[i][j] = a[i][j];
    }
  }


  /* Do singular value decomposition. */

  my_svdcmp(u, N_SVD, N_SVD, w, v);
  check_svd(a, u, w, v);

  /* printf( "svd: " ); for( j = 1; j <= N_SVD; j++ ) printf( "%g ", w[j] );
   * printf("\n"); */

  /* Choose threshold singular value. */

  wmax = 0.0;
  for (j = 1; j <= N_SVD; j++)
    if (w[j] > wmax)
      wmax = w[j];
  wmin = wmax * svdr;
  if (wmin > svdt)
    wmin = svdt;


  /* Zero singular values < wmin */
  for (j = 1; j <= N_SVD; j++) {
    if (w[j] < wmin) {
      w[j] = 0.0;
      ++n;
    }
  }

  /* printf( "Altered Singular values:\n" ); for( j = 1; j <= N_SVD; j++ )
   * printf( "%d %g\n", j, w[j] );
   * 
   * 
   * printf("Altered %d singular values to zero\n",n); */

  /* Do back substitution */

  my_svbksb(u, w, v, N_SVD, N_SVD, b, x);
  check_x(a, x, b);

  my_free_matrix(u, 1, N_SVD, 1, N_SVD);
  my_free_matrix(v, 1, N_SVD, 1, N_SVD);
  my_free_vector(w, 1, N_SVD);

  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
\note  check_svd
\date  8/4/92

\remarks 

      the matrices a, u, and v are reconstructed from the corrected matrix w.
      The squared errors denote how close the new matrices are to the
      original ones.

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     a : matrix of linear equation a x = b
 \param[in]     u : matrix of nullspace columns
 \param[in]     w : vector with values of the diagonal matrix w
 \param[in]     v : matrix of columns of range

 ******************************************************************************/
int
check_svd(double **a, double **u, double *w, double **v)
{
  double            **check_a, **check_u, **check_v;
  double              sum_a_errors, sum_u_errors, sum_v_errors;
  int                 i, j, k;

  check_a = my_matrix(1, N_SVD, 1, N_SVD);
  check_u = my_matrix(1, N_SVD, 1, N_SVD);
  check_v = my_matrix(1, N_SVD, 1, N_SVD);

  /* Check singular value decomposition. */
  sum_a_errors = 0.0;
  sum_u_errors = 0.0;
  sum_v_errors = 0.0;
  for (i = 1; i <= N_SVD; i++) {
    for (j = 1; j <= N_SVD; j++) {
      check_a[i][j] = 0.0;
      check_u[i][j] = 0.0;
      check_v[i][j] = 0.0;
      for (k = 1; k <= N_SVD; k++) {
	check_a[i][j] += u[i][k] * w[k] * v[j][k];	/* rebuilds matrix a */
	check_u[i][j] += u[k][i] * u[k][j];	/* since transpose equals
						 * inverse here this should
						 * become the Identity mat. */
	check_v[i][j] += v[k][i] * v[k][j];	/* the same as for u */
      }
      sum_a_errors += (check_a[i][j] - a[i][j]) * (check_a[i][j] - a[i][j]);
      /* printf( "%d %d %10.3g %10.3g %g\n", i, j, a[i][j], check_a[i][j],
       * check_a[i][j]-a[i][j] ); */
      if (i == j) {
	sum_u_errors += (check_u[i][j] - 1.0) * (check_u[i][j] - 1.0);
	sum_v_errors += (check_v[i][j] - 1.0) * (check_v[i][j] - 1.0);
      } else {
	sum_u_errors += check_u[i][j] * check_u[i][j];
	sum_v_errors += check_v[i][j] * check_v[i][j];
      }
    }
  }
/*
  printf( "Check_svd:\n" );
  printf( "Sum of errors squared in reconstructing a: %g\n", sum_a_errors );
  printf( "Sum of errors squared in checking u: %g\n", sum_u_errors );
  printf( "Sum of errors squared in checking v: %g\n", sum_v_errors );
*/

  my_free_matrix(check_v, 1, N_SVD, 1, N_SVD);
  my_free_matrix(check_u, 1, N_SVD, 1, N_SVD);
  my_free_matrix(check_a, 1, N_SVD, 1, N_SVD);

  return TRUE;
}

/****************************************************************************/
int
check_x(double **a, double *x, double *b)
{
  double             *check_x;
  double              sum_x_errors;
  int                 i, j;

  check_x = my_vector(1, N_SVD);

  /* Check your answer. */
  sum_x_errors = 0.0;
  for (i = 1; i <= N_SVD; i++) {
    check_x[i] = 0.0;
    for (j = 1; j <= N_SVD; j++)
      check_x[i] += a[i][j] * x[j];
    sum_x_errors += (check_x[i] - b[i]) * (check_x[i] - b[i]);
  }
/*  printf( "Check_x: Sum of X errors: %g\n", sum_x_errors );
  for( j = 1; j <= N_SVD; j++ )
    printf( "%3d %10.3g %10.3g %10.3g\n",
	   j, x[j], b[j], check_x[j] - b[j] );
	   */
  my_free_vector(check_x, 1, N_SVD);

  return TRUE;

}

/****************************************************************************/
