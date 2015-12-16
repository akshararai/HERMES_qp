#include <math.h>
#include <float.h>
#define NRANSI
#include "nrutil.h"

/* NOTE: sschaal changed this function from void to int to get failure code */
int my_choldc(double **a, int n, double p[])
{
  int i,j,k;
  double sum;

  for (i=1;i<=n;i++) {
    for (j=i;j<=n;j++) {
      for (sum=a[i][j],k=i-1;k>=1;k--) sum -= a[i][k]*a[j][k];
      if (i == j) {
	if (sum <= 0.0) {
	  return 0;
	}
	p[i]=sqrt(sum);
      } else a[j][i]=sum/p[i];
    }
  }

  return 1;

}



