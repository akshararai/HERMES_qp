#include "stdio.h"
#include "math.h"

#define ITMAX 200
static double sqrarg;
/*!<  #define SQR(a) (sqrarg=(a),sqrarg*sqrarg) */
#define SQR(a) ((a)*(a))

void powell(p,xi,n,ftol,iter,fret,func)
double p[],**xi,ftol,*fret,(*func)();
int n,*iter;
{
	int i,ibig,j;
	double t,fptt,fp,del;
	double *pt,*ptt,*xit,*my_vector();
	void my_linmin(),my_nrerror(),my_free_vector();

	pt=my_vector(1,n);
	ptt=my_vector(1,n);
	xit=my_vector(1,n);
	*fret=(*func)(p);
	for (j=1;j<=n;j++) pt[j]=p[j];
	for (*iter=1;;(*iter)++) {
	  /*
	  printf( "POWELL ITERATION %d; XI:\n", *iter );  
	        for ( j = 1; j <= n; j++ ) 
		  { 
		    for ( i = 1; i <= n; i ++ ) 
		      printf( "%f ", xi[j][i] ); 
		    printf( "\n" ); 
		  }  */
		fp=(*fret);
		ibig=0;
		del=0.0;
		for (i=1;i<=n;i++) {
			for (j=1;j<=n;j++) xit[j]=xi[j][i];
			fptt=(*fret);
/*			printf( "SEARCHING ALONG DIRECTION %d:\n", i ); CGA
			for ( j = 1; j <= n; j++ ) 
			  printf( "%f ", xit[j] ); 
			printf( "\n\n" ); */
			my_linmin(p,xit,n,fret,func);
			if (fabs(fptt-(*fret)) > del) {
				del=fabs(fptt-(*fret));
				ibig=i;
			}
		}
		if (2.0*fabs(fp-(*fret)) <= ftol*(fabs(fp)+fabs(*fret))) {
			my_free_vector(xit,1,n);
			my_free_vector(ptt,1,n);
			my_free_vector(pt,1,n);
			return;
		}
		if (*iter == ITMAX) my_nrerror("Too many iterations in routine POWELL");
		for (j=1;j<=n;j++) {
			ptt[j]=2.0*p[j]-pt[j];
			xit[j]=p[j]-pt[j];
			pt[j]=p[j];
		}
/*	  printf( "TESTING WHETHER TO CHANGE DIRECTIONS\n" );*/ /* CGA */
		fptt=(*func)(ptt);
		if (fptt < fp) {
			t=2.0*(fp-2.0*(*fret)+fptt)*SQR(fp-(*fret)-del)-del*SQR(fp-fptt);
			if (t < 0.0) {
				my_linmin(p,xit,n,fret,func);
				for (j=1;j<=n;j++) xi[j][ibig]=xit[j];

				/*
			printf( "CHANGING DIRECTION %d\n", ibig );
				for ( j = 1; j <= n; j++ ) 
				  { 
				    for ( i = 1; i <= n; i++ ) 
				      { 
					if ( i == ibig ) 
					  printf( " *" ); 
					printf( "%f ", xi[j][i] ); 
				      } 
				    printf( "\n" ); 
				  } 
				  */
			}
		}
	}
}

#undef ITMAX
#undef SQR
