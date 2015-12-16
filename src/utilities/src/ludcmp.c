#include "math.h"
#include "utility.h"
#include "utility_macros.h"

#define TINY 1.0e-20;

#define  CALC_NO_DET   0
#define  CALC_DET      1
#define  CALC_DET_ONLY 2

extern int my_lubksb();

int my_ludcmp(double **a, int n, int *indx,double *d)
     
{
  int i,imax=0,j,k;
  double big,dum,sum,temp;
  double *vv;
  
  
  vv=my_vector(1,n);
  *d=1.0;
  for (i=1;i<=n;i++) {
    big=0.0;
    for (j=1;j<=n;j++)
      if ((temp=fabs(a[i][j])) > big) big=temp;
    if (big == 0.0) {
      printf("Singular matrix in routine LUDCMP\n");
      return 0;
    }
    vv[i]=1.0/big;
  }
  for (j=1;j<=n;j++) {
    for (i=1;i<j;i++) {
      sum=a[i][j];
      for (k=1;k<i;k++) sum -= a[i][k]*a[k][j];
      a[i][j]=sum;
    }
    big=0.0;
    for (i=j;i<=n;i++) {
      sum=a[i][j];
      for (k=1;k<j;k++)
	sum -= a[i][k]*a[k][j];
      a[i][j]=sum;
      if ( (dum=vv[i]*fabs(sum)) >= big) {
	big=dum;
	imax=i;
      }
    }
    if (j != imax) {
      for (k=1;k<=n;k++) {
	dum=a[imax][k];
	a[imax][k]=a[j][k];
	a[j][k]=dum;
      }
      *d = -(*d);
      vv[imax]=vv[j];
    }
    indx[j]=imax;
    if (a[j][j] == 0.0) a[j][j]=TINY;
    if (j != n) {
      dum=1.0/(a[j][j]);
      for (i=j+1;i<=n;i++) a[i][j] *= dum;
    }
  }
  my_free_vector(vv,1,n);
  return 1;
}

#undef TINY

/*!*****************************************************************************
 *******************************************************************************
 \note  my_inv_ludcmp_gen
 \date  Nov 2008 (Modified by Mrinal)
 
 \remarks 
 
 uses the LUDCMP to invert a square matrix.
 Also calculates the determinant if needed
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     mat        : matrix to be inverted
 \param[in]     size       : mat is a size x size matrix
 \param[out]    inv_mat    : inverse of this matrix
 \param[in]     calculate_determinant : one of CALC_NO_DET, CALC_DET, CALC_DET_ONLY
 \param[out]    det        : determinant of the matrix (if requested)
 
 note: - return code FALSE is given if not invertable
 - mat and inv_mat can be the same matrices; the program takes
 care of this
 - matrices are according to numerical recipes convention
 
 
 ******************************************************************************/
int 
my_inv_ludcmp_gen(double **mat, int size, double **inv_mat, int calculate_determinant, double* det)

{
  
  int             i,j,n;
  int             rc = 1;
  double          info;
  MY_MATRIX(aux, 1, size, 1, size);
  MY_IVECTOR(indx, 1, size);
  
  for (i=1; i<=size; ++i) for (j=1; j<=size; ++j) aux[i][j]=mat[i][j];
  
  if (!my_ludcmp(aux,size,indx,&info)) {
    
    rc = 0;
    
  } else {

    if (calculate_determinant != CALC_DET_ONLY) {
      
      /* make zuu_inv the identity matrix */
      
      for (i=1; i<=size; ++i) {
	for (j=1; j<=size; ++j) {
	  inv_mat[i][j] = 0.0;
	  if (i==j) inv_mat[i][j] = 1.0;
	}
      }
      
      /* now generate the inverse. Due to the setup of matrices, I will
	 actuall create the transpose of the inverse */
      
      for (i=1; i<=size; ++i) {
	my_lubksb(aux, size, indx, inv_mat[i]);
      }
      
      /* now take the transpose of this inverse to make it what I want */
      
      mat_trans(inv_mat, inv_mat);

    }

    if (calculate_determinant && det!=NULL) {
      
      calculate_determinant = CALC_NO_DET;

      *det = info;

      for (i=1; i<=size; ++i) *det *= aux[i][i];

    }
    
  }
  
  return rc;
}


/*!*****************************************************************************
 *******************************************************************************
 \note  my_inv_ludcmp
 \date  March 93
 
 \remarks 
 
 uses the LUDCMP to invert a square matrix
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     mat        : matrix to be inverted
 \param[in]     size       : mat is a size x size matrix
 \param[out]    inv_mat    : inverse of this matrix
 
 note: - return code FALSE is given if not invertable
 - mat and inv_mat can be the same matrices; the program takes
 care of this
 - matrices are according to numerical recipes convention
 
 
 ******************************************************************************/
int 
my_inv_ludcmp(double **mat, int size, double **inv_mat)

{
  return my_inv_ludcmp_gen(mat, size, inv_mat, CALC_NO_DET, NULL);
}

/*!*****************************************************************************
 *******************************************************************************
 \note  my_inv_ludcmp_solve
 \date  March 93
 
 \remarks 
 
 uses the LUDCMP to invert a square matrix and solve it for a given vector
 
             A x = b

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     mat        : matrix to be inverted
 \param[in]     b_vec      : vector for which system is to be solved
 \param[in]     size       : mat is a size x size matrix
 \param[out]    x_vec      : the result
 
 note: - return code FALSE is given if not invertable
 - mat and inv_mat can be the same matrices; the program takes
 care of this
 - matrices are according to numerical recipes convention
 
 
 ******************************************************************************/
int 
my_inv_ludcmp_solve(double **mat, double *b_vec, int size, double *x_vec)

{
  
  int             i,j,n;
  int             rc = 1;
  double          info;
  MY_MATRIX(aux, 1, size, 1, size);
  MY_IVECTOR(indx, 1, size);
  
  for (i=1; i<=size; ++i) for (j=1; j<=size; ++j) aux[i][j]=mat[i][j];
  
  if (!my_ludcmp(aux,size,indx,&info)) {
    
    rc = 0;
    
  } else {
    
    /* make zuu_inv the identity matrix */
    
    for (i=1; i<=size; ++i) {
      x_vec[i] = b_vec[i];
    }
    
    /* now generate the solution */
    
    my_lubksb(aux, size, indx, x_vec);
    
  }
  
  return rc;
}

/*!*****************************************************************************
 *******************************************************************************
 \note  my_inv_ludcmp_solve_many
 \date  March 93
 
 \remarks 
 
 uses the LUDCMP to invert a square matrix and solve it for a set of
 given vectors
 
             A x1 = b1   A x2 = b2 ....

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     mat        : matrix to be inverted
 \param[in]     b_vec      : matrix of vectors for which system is to be solved
                 NOTE: each ROW is used as a vector!!
 \param[in]     size       : mat is a size x size matrix
 \param[in]     n_vec      : how many vectors are to be solved for
 \param[out]    x_vec      : result matrix the result
                 NOTE: each ROW is used as a output vector!!
 
 note: - return code FALSE is given if not invertable
 - mat and inv_mat can be the same matrices; the program takes
 care of this
 - matrices are according to numerical recipes convention
 
 
 ******************************************************************************/
int 
my_inv_ludcmp_solve_many(double **mat, double **b_vec, int size, int n_vec,
			 double **x_vec)

{
  
  int             i,j,n;
  int             rc = 1;
  double          info;
  MY_MATRIX(aux, 1, size, 1, size);
  MY_IVECTOR(indx, 1, size);
  
  for (i=1; i<=size; ++i) for (j=1; j<=size; ++j) aux[i][j]=mat[i][j];
  
  if (!my_ludcmp(aux,size,indx,&info)) {
    
    rc = 0;
    
  } else {
    
    /* make zuu_inv the identity matrix */

    for (j=1; j<=n_vec; ++j) {
    
      for (i=1; i<=size; ++i) {
	x_vec[j][i] = b_vec[j][i];
      }
      
      /* now generate the solution */
      
      my_lubksb(aux, size, indx, x_vec[j]);

    }
    
  }
  
  return rc;
}

/*!*****************************************************************************
 *******************************************************************************
 \note  my_inv_ludcmp_det
 \date  March 93
 
 \remarks 
 
 uses the LUDCMP to invert a square matrix and calculates also the
 determinant
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     mat        : matrix to be inverted
 \param[in]     size       : mat is a size x size matrix
 \param[out]    inv_mat    : inverse of this matrix
 \param[out]    deter      : the determinant

 note: - return code FALSE is given if not invertable
       - mat and inv_mat can be the same matrices; the program takes
         care of this
       - matrices are according to numerical recipes convention
 
 
 ******************************************************************************/
int 
my_inv_ludcmp_det(double **mat, int size, double **inv_mat, double *deter)

{
  int rc,i;

  rc = my_inv_ludcmp_gen(mat, size, inv_mat, CALC_DET, deter);

  return rc;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  my_ludcmp_det
 \date  March 93
 
 \remarks 
 
 uses the LUDCMP to calculate the  determinant of the given matrix
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     mat        : matrix 
 \param[in]     size       : mat is a size x size matrix
 \param[out]    deter      : the determinant

 note: - return code FALSE is given if not invertable
       - matrices are according to numerical recipes convention
 
 ******************************************************************************/
int 
my_ludcmp_det(double **mat, int size, double *deter)

{
  int rc,i;
  MY_MATRIX(aux, 1, size, 1, size);
  
  rc = my_inv_ludcmp_gen(mat, size, aux, CALC_DET_ONLY, deter);

  return rc;

}
