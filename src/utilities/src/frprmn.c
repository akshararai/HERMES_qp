#include "math.h"
#include "stdio.h"

#define EPS 1.0e-10
#define FREEALL my_free_vector(xi,1,n);my_free_vector(h,1,n);my_free_vector(g,1,n);

void 
my_frprmn(double *p,int n, double ftol, int *iter, double *fret, 
	  double (*func)(double *), void (*dfunc)(double *, double *))

{
  int j,its;
  double gg,gam,fp,dgg;
  double *g,*h,*xi,*my_vector();
  void my_dlinmin(),my_nrerror(),my_free_vector();
  int ITMAX;

  ITMAX = *iter;

  g=my_vector(1,n);
  h=my_vector(1,n);
  xi=my_vector(1,n);
  fp=(*func)(p);
  (*dfunc)(p,xi);
  for (j=1;j<=n;j++) {
    g[j] = -xi[j];
    xi[j]=h[j]=g[j];
  }
  for (its=1; its<=ITMAX;its++) {
    if (its%((int)(ITMAX/100)) == 1) printf("---- %d %f ----\n",its,fp);
    *iter=its;
    my_dlinmin(p,xi,n,fret,func,dfunc);
    if (2.0*fabs(*fret-fp) <= ftol*(fabs(*fret)+fabs(fp)+EPS)) {
      FREEALL
	return;
    }
    fp=(*func)(p);
    (*dfunc)(p,xi);
    dgg=gg=0.0;
    for (j=1;j<=n;j++) {
      gg += g[j]*g[j];
      /*		  dgg += xi[j]*xi[j];	*/
      dgg += (xi[j]+g[j])*xi[j];
    }
    if (gg == 0.0) {
      FREEALL
	printf("---- %d %f ----\n",its,fp);
	return;
    }
    gam=dgg/gg;
    for (j=1;j<=n;j++) {
      g[j] = -xi[j];
      xi[j]=h[j]=g[j]+gam*h[j];
    }
  }
  /* my_nrerror("Too many iterations in FRPRMN"); */
  FREEALL
    printf("---- %d %f ----\n",its,fp);
    
}

#undef ITMAX
#undef EPS
#undef FREEALL
