#define TOL 2.0e-4

static int ncom=0;	/* defining declarations */
static double *pcom=0,*xicom=0,(*nrfunc)();

void my_linmin(p,xi,n,fret,func)
double p[],xi[],*fret,(*func)();
int n;
{
	int j;
	double xx,xmin,fx,fb,fa,bx,ax;
	double my_brent(),my_f1dim(),*my_vector();
	void my_mnbrak(),my_free_vector();

	ncom=n;
	pcom=my_vector(1,n);
	xicom=my_vector(1,n);
	nrfunc=func;
	for (j=1;j<=n;j++) {
		pcom[j]=p[j];
		xicom[j]=xi[j];
	}
	ax=0.0;
	xx=1.0;
	bx=2.0;
	my_mnbrak(&ax,&xx,&bx,&fa,&fx,&fb,my_f1dim);
	*fret=my_brent(ax,xx,bx,my_f1dim,TOL,&xmin);
	for (j=1;j<=n;j++) {
		xi[j] *= xmin;
		p[j] += xi[j];
	}
	my_free_vector(xicom,1,n);
	my_free_vector(pcom,1,n);
}

#undef TOL
