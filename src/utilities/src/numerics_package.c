#include "stdio.h"
#include "math.h"
#include "stdlib.h"
#include "utility.h"
 
  
/*! ******************************************************************************
	calculates a root of any passed function using a modifed Newton-Rapson
*******************************************************************************/
#define MAXIT 100

double my_rtsafe(funcd,x1,x2,xacc)

double x1,x2,xacc;
void   (*funcd)();

{
  int 		j;
  double 		df,dx,dxold,f,fh,fl;
  double 		swap,temp,xh,xl,rts;
  
  /* calculate the function value and the value of the first derivative 
     of the start values                                                 */
  (*funcd) (x1,&fl,&df);
  (*funcd) (x2,&fh,&df);
  if (fl*fh >= 0.0) {
    /* printf("Root must be bracketed in RTSAFE\n"); */
    return 999.;
  }
  
  
  if (fl < 0.0) {
    xl = x1;
    xh = x2;
  } else {
    xh = x1;
    xl = x2;
    swap = fl;
    fl = fh;
    fh = swap;
  }
  rts = 0.5 * (x1 + x2);
  dxold = fabs(x2-x1);
  dx = dxold;
  
  (*funcd) (rts,&f,&df);
  
  for (j=1; j<=MAXIT; ++j) {
    if ((((rts-xh)*df-f) * ((rts-xl)*df-f) >= 0.0) ||
	(fabs(2.0 * f) > fabs(dxold*df))) {
      dxold = dx;
      dx = 0.5 * (xh-xl);
      rts = xl + dx;
      if (xl == rts) return rts;
    } else {
      dxold = dx;
      dx = f/df;
      temp = rts;
      rts -= dx;
      if (temp == rts) return rts;
    }
    if (fabs(dx) < xacc) return rts;
    
    (*funcd) (rts,&f,&df);
    
    if (f < 0.0) {
      xl = rts;
      fl = f;
    } else {
      xh = rts;
      fh = f;
    }
  }
  
  printf("Maximum number of iterations exeeded in RTSAFE\n");
  return 999;
}


/*******************************************************************************
  Runge-Kutta method of 4th order
  *******************************************************************************/
void my_rk4(y,dydx,n,x,h,yout,derivs)
     
     double y[],dydx[],x,h,yout[];
     void   (*derivs)();
     int n;
     
{
  int 	i;
  double 	xh,hh,h6,*dym,*dyt,*yt,*my_vector();
  void   	my_free_vector();
  
  
  dym = my_vector(1,n);
  dyt = my_vector(1,n);
  yt  = my_vector(1,n);
  hh = h * 0.5;
  h6 = h/6;
  xh = x + hh;
  for (i=1;i<=n;++i) yt[i] = y[i] + hh * dydx[i];
  (*derivs)(xh,yt,dyt);
  for (i=1;i<=n;++i) yt[i] = y[i] + hh * dyt[i];
  (*derivs)(xh,yt,dym);
  for (i=1;i<=n;++i) {
    yt[i] 	= y[i] + h * dym[i];
    dym[i] += dyt[i];
  }
  (*derivs)(x+h,yt,dyt);
  for (i=1;i<=n;++i) 
    yout[i] = y[i]+h6*(dydx[i] + dyt[i] + 2.0 * dym[i]);
  
  my_free_vector(yt,1,n);
  my_free_vector(dyt,1,n);
  my_free_vector(dym,1,n);
}

