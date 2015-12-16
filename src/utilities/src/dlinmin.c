/*!<  CGA #define TOL 2.0e-4  This is too small? */
// #define TOL 2.0e-4
#define TOL 1.0e-6

int ncom=0;	/* defining declarations */
double *pcom=0,*xicom=0,(*nrfunc)();
void (*nrdfun)();

void my_dlinmin(p,xi,n,fret,func,dfunc)
     double p[],xi[],*fret,(*func)();
     void (*dfunc)();
     int n;
{
	int j;
	double xx,xmin,fx,fb,fa,bx,ax;
	double my_dbrent(),my_f1dim(),my_df1dim(),*my_vector();
	void my_mnbrak(),my_free_vector();

	ncom=n;
	pcom=my_vector(1,n);
	xicom=my_vector(1,n);
	nrfunc=func;
	nrdfun=dfunc;
	for (j=1;j<=n;j++) {
		pcom[j]=p[j];
		xicom[j]=xi[j];
	}
	ax=0.0;
	xx=1.0;
	bx=2.0;
	my_mnbrak(&ax,&xx,&bx,&fa,&fx,&fb,my_f1dim);
	*fret=my_dbrent(ax,xx,bx,my_f1dim,my_df1dim,TOL,&xmin);
	for (j=1;j<=n;j++) {
		xi[j] *= xmin;
		p[j] += xi[j];
	}
	my_free_vector(xicom,1,n);
	my_free_vector(pcom,1,n);
}

#undef TOL
