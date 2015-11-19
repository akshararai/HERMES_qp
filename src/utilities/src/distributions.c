/* a collection of Numerical Recipes functions, all related to distributions */
/* Stefan Schaal, March 1993 */


#include "math.h"
#include "utility.h"

#define ITMAX 100
#define EPS   3.0e-7

/* function */
static double t_value_function(double *p);

/*******************************************************************************/

double my_betai(double a, double b, double x)

{
	double bt;
	double my_gammln(), my_betacf();
	void my_nrerror();
	
	if (x < 0.0 || x > 1.0) {
		printf("x = %f\n",x);
		printf("BAD x in routine BETAI");
		return MY_ERROR;
	}
	if (x == 0.0 || x == 1.0) bt = 0.0;
	else
		bt = exp(my_gammln(a+b)-my_gammln(a)-my_gammln(b)+a*log(x)+b*log(1.0-x));
	if (x < (a+1.0)/(a+b+2.0))
		return bt*my_betacf(a,b,x)/a;
	else 
		return 1.0-bt*my_betacf(b,a,1.0-x)/b;
}

/*******************************************************************************/

double my_betacf(double a, double b, double x)

{
	double qap,qam,qab,em,tem,d;
	double bz,bm=1.0,bp,bpp;
	double az=1.0,am=1.0,ap,app,aold;
	int m;
	void my_nrerror();

	qab=a+b;
	qap=a+1.0;
	qam=a-1.0;
	bz=1.0-qab*x/qap;
	for (m=1;m<=ITMAX;m++) {
		em=(double) m;
		tem=em+em;
		d=em*(b-em)*x/((qam+tem)*(a+tem));
		ap=az+d*am;
		bp=bz+d*bm;
		d = -(a+em)*(qab+em)*x/((qap+tem)*(a+tem));
		app=ap+d*az;
		bpp=bp+d*bz;
		aold=az;
		am=ap/bpp;
		bm=bp/bpp;
		az=app/bpp;
		bz=1.0;
		if (fabs(az-aold) < (EPS*fabs(az))) return az;
	}
	my_nrerror("a or b too big, or ITMAX too small in BETACF");

	return 0.0;
}

/*******************************************************************************/

double my_erf(double x)

{
  double my_gammp();

  if (x > 26.0) {
    return 1.0;
  } else if (x < -26.0) {
    return 0.0;
  } else {
    return x < 0.0 ? -my_gammp(0.5,x*x) : my_gammp(0.5,x*x);
  }
}

/*******************************************************************************/

double my_gammp(double a, double x)

{
	double gamser,gammcf,gln;
	void my_gser(),my_gcf(),my_nrerror();

	if (x < 0.0 || a <= 0.0) my_nrerror("Invalid arguments in routine GAMMP");
	if (x < (a+1.0)) {
		my_gser(&gamser,a,x,&gln);
		return gamser;
	} else {
		my_gcf(&gammcf,a,x,&gln);
		return 1.0-gammcf;
	}
}

/*******************************************************************************/

void my_gser(double *gamser, double a, double x, double *gln)

{
	int n;
	double sum,del,ap;
	double my_gammln();
	void my_nrerror();

	*gln=my_gammln(a);
	if (x <= 0.0) {
		if (x < 0.0) my_nrerror("x less than 0 in routine GSER");
		*gamser=0.0;
		return;
	} else {
		ap=a;
		sum=1.0/a;
		del=sum;
		for (n=1;n<=ITMAX;n++) {
			ap += 1.0;
			del *= x/ap;
			sum += del;
			if (fabs(del) < fabs(sum)*EPS) {
				*gamser=sum*exp(-x+a*log(x)-(*gln));
				return;
			}
		}
		my_nrerror("a too large, ITMAX too small in routine GSER");
		return;
	}
}

/*******************************************************************************/
/* 
 * NOTE (adsouza):
 * This function is deprecated. Use gammaln_scalar() instead (see below) 
 */
double my_gammln(double xx)

{
	double x,tmp,ser;
	static double cof[6]={76.18009173,-86.50532033,24.01409822,
		-1.231739516,0.120858003e-2,-0.536382e-5};
	int j;

	x=xx-1.0;
	tmp=x+5.5;
	tmp -= (x+0.5)*log(tmp);
	ser=1.0;
	for (j=0;j<=5;j++) {
		x += 1.0;
		ser += cof[j]/x;
	}
	return -tmp+log(2.50662827465*ser);
}

/*******************************************************************************/

void my_gcf(double *gammcf, double a, double x, double *gln)

{
	int n;
	double gold=0.0,g,fac=1.0,b1=1.0;
	double b0=0.0,anf,ana,an,a1,a0=1.0;
	double my_gammln();
	void my_nrerror();

	*gln=my_gammln(a);
	a1=x;
	for (n=1;n<=ITMAX;n++) {
		an=(double) n;
		ana=an-a;
		a0=(a1+a0*ana)*fac;
		b0=(b1+b0*ana)*fac;
		anf=an*fac;
		a1=x*a0+anf*a1;
		b1=x*b0+anf*b1;
		if (a1) {
			fac=1.0/a1;
			g=b1*fac;
			if (fabs((g-gold)/g) < EPS) {
				*gammcf=exp(-x+a*log(x)-(*gln))*g;
				return;
			}
			gold=g;
		}
	}
	my_nrerror("a too large, ITMAX too small in routine GCF");
}

/*******************************************************************************/

double my_factln(int n)

{
	static double a[101];
	double my_gammln();
	void my_nrerror();

	if (n < 0) my_nrerror("Negative factorial in routine FACTLN");
	if (n <= 1) return 0.0;
	if (n <= 100) return a[n] ? a[n] : (a[n]=my_gammln(n+1.0));
	else return my_gammln(n+1.0);
}

/*******************************************************************************/

/* calculates the probability of a t value for a certain t(n) distribution */
double prob_t(double t, double n)

{

	double a, b, x;
	
	a = n / 2.0;
	b = 0.5;
	x = n/(n + sqr(t));

	return (1.0-my_betai(a,b,x));
}

/*******************************************************************************/

/* calculates the probability of an F value for a certain F(m,n) distribution */
double prob_f(double f, double m, double n)

{

	double a, b, x;
	
	a = n / 2.0;
	b = m / 2.0;
	x = n / (n + m * f);

/* printf("f=%f m=%f n=%f x=%f\n",f,m,n,x ); */

	return (my_betai(a,b,x));
}


/*******************************************************************************/

double global_n;
double global_alpha;

/*  calculates the t value for a certain alpha and a certain number of
    degrees of freedom  */

double t_value(double alph, double n) 

{
  double    xn;
  double    fxn;
  double    dfdx;



  if ( n < 0.6) n = 0.6; /* avoid huge t values which are likely to
			    be numerically corrupted */

 
  /* since this function is well behaved, I find the point by the
     Newton-Raphson method */

  xn = .01;
  fxn = prob_t(xn,n)-alph;
  
  while(fabs(fxn) > .0001) {

    dfdx = (prob_t(xn+.01,n)-prob_t(xn-.01,n))/.02;
    xn = xn - fxn/dfdx;
    fxn = prob_t(xn,n)-alph;

  }

  return (xn);

}

/*!*****************************************************************************
 *******************************************************************************
 \note  digamma_scalar
 \date  July 07, 2002
 
 \remarks 
 
 Computes the digamma function for a scalar a.
 Digamma(a) = d/da log Gamma(a)

 Acknowledgements: Thanks to Matt Beal for the original matlab source

 *******************************************************************************
 Parameters:  (i/o = input/output)

 \param[in]     a		 : scalar a
 \param[out]    d		 : result of the digamma function

 ******************************************************************************/
int
digamma_scalar(double a, double *d){
  static double coef[] = {-1./12, 1./120, -1./252, 1./240,
                          -1./132, 691./32760, -1./12};
  double tempx, z, logz;
  int y;
  int i,j,k,count;

  if(a<=0){
    fprintf(stderr, "Digamma of -ve number not allowed in >digamma_scalar<\n");
    return FALSE;
  }

  y = ceil(0>(6-a)?0:(6-a));
  z = a + y;
  logz = log(z);
  
  (*d) = logz - 0.5/z;
  for (k = 1; k <= 7; k++)
    (*d) += coef[k-1]*exp(-2*k*logz);
  for (k = 1; k <= 6; k++)
    if (y>=k) (*d) -= 1/(a+k-1);
  
  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
 \note  digamma_vector
 \date  July 07, 2002
 
 \remarks 
 
 Computes the digamma function for a matrix a
 Digamma(a) = d/da log Gamma(a)

 Simply calls digamma_scalar() for each element of a

 *******************************************************************************
 Parameters:  (i/o = input/output)

 \param[in]     a		 : vector a
 \param[out]    d		 : result of the digamma function

 ******************************************************************************/
int
digamma_vector(Vector a, Vector d){
  int i;
  int ar,dr;
  
  ar = a[NR];
  dr = d[NR];

  if(ar!=dr){
    fprintf(stderr, "Input/output vector size differ in >digamma_vector<\n");
    return FALSE;
  }
  
  for(i=1; i<=ar; i++)
    digamma_scalar(a[i], &(d[i]));
  
  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
 \note  digamma_matrix
 \date  July 07, 2002
 
 \remarks 
 
 Computes the digamma function for a matrix a
 Digamma(a) = d/da log Gamma(a)

 Simply calls digamma_scalar() for each element of a

 *******************************************************************************
 Parameters:  (i/o = input/output)

 \param[in]     a		 : matrix a
 \param[out]    d		 : result of the digamma function

 ******************************************************************************/
int
digamma_matrix(Matrix a, Matrix d){
  int i,j;
  int ar,ac,dr,dc;
  int type;
  
  ar = a[0][NR];
  ac = a[0][NC];
  dr = d[0][NR];
  dc = d[0][NC];

  if(a[0][MAT_TYPE] < d[0][MAT_TYPE]){
    fprintf(stderr, "Incompatible size for destination matrix in >digamma_matrix<\n");
    return FALSE;
  }

  if(ar!=dr || ac!=dc){
    fprintf(stderr, "Input/output matrix size differ in >digamma_matrix<\n");
    return FALSE;
  }

  type = a[0][MAT_TYPE];

  switch(type){
  case IS_FULL:
    for(i=1; i<=ar; i++)
      for(j=1; j<=ac; j++)
        digamma_scalar(a[i][j], &(d[i][j]));
    break;

  case IS_SYMM:
    for(i=1; i<=ar; i++)
      for(j=i; j<=ac; j++)
        digamma_scalar(a[i][j], &(d[i][j]));
    if(d[0][MAT_TYPE]==IS_FULL)
      for(i=1; i<=ar; i++)
        for(j=i+1; j<=ac; j++)
          d[j][i] = d[i][j];
    break;

  case IS_DIAG:
    if(d[0][MAT_TYPE]==IS_FULL)
      for(i=1; i<=ar; i++)
        for(j=1; j<ac; j++)
          d[i][j] = 0;
    if(d[0][MAT_TYPE]==IS_SYMM)
      for(i=1; i<=ar; i++)
        for(j=i; j<ac; j++)
          d[i][j] = 0;
    for(i=1; i<=ar; i++)
      digamma_scalar(a[i][i], &(d[i][i]));
    break;

  default:
    fprintf(stderr, "Unknown matrix type in >digamma_matrix<");
    exit(-12);
  }

  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
 \note  gammaln_scalar
 \date  July 07, 2002
 
 \remarks 
 
 Computes the log Gamma function for a scalar a.
 gammaln(a) = log Gamma(a)

 Acknowledgements: Adapted from Numerical Recipes in C (pg. 214)
 
 Note: my_gammln() exists in distributions.c although it seems to be an
 older less accurate version. Ideally we should modify it to place a call
 to this function.

 *******************************************************************************
 Parameters:  (i/o = input/output)

 \param[in]     a		 : scalar a
 \param[out]    g		 : result of the gammaln function

 ******************************************************************************/
int
gammaln_scalar(double a, double *g){
  static double coef[] = {76.18009172947146,-86.50532032941677,
                          24.01409824083091,-1.231739572450155,
                          0.1208650973866179e-2,-0.5395239384953e-5};
  double x,y,tmp,ser;
  int i;
  
  if(a<=0){
    fprintf(stderr, "gammaln of -ve number not allowed in >gammaln_scalar<\n");
    return FALSE;
  }

  y = x = a;
  tmp = x+5.5;
  tmp -= (x+0.5)*log(tmp);
  ser = 1.000000000190015;
  for(i=0; i<=5; i++)
    ser += coef[i]/++y;

  (*g) = -tmp+log(2.5066282746310005*ser/x);
  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
 \note  gamma_entropy
 \date  July 07, 2002
 
 \remarks 
 
 Computes the entropy of a Gamma distribution with parameters a and b
 entropy = gammaln(a) - (a-1)*digamma(a) - log(b) + a;

 *******************************************************************************
 Parameters:  (i/o = input/output)

 \param[in]     a		 : +ve scalar a
 \param[in]     b		 : +ve scalar b
 \param[out]    e		 : entropy of Gamma(a,b)

 ******************************************************************************/
int
gamma_entropy(double a, double b, double *e){
  double tmp1,tmp2;

  if(a<=0 || b<=0){
    fprintf(stderr, "a, b should be > 0 in >gamma_entropy<");
    return FALSE;
  }

  gammaln_scalar(a,&tmp1);
  digamma_scalar(a,&tmp2);

  (*e) = tmp1 - (a-1)*tmp2 - log(b) + a;
  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
 \note  dirichlet_entropy
 \date  July 08, 2002
 
 \remarks 
 
 Computes the entropy of a Dirichlet distribution with parameter vector a

 *******************************************************************************
 Parameters:  (i/o = input/output)

 \param[in]     a		 : vector a
 \param[out]    e		 : entropy of Dirichlet(a)

 ******************************************************************************/
int
dirichlet_entropy(Vector a, double* e){
  int i;
  int ar;
  double sum_a;
  double tmp;

  ar = a[NR];

  for(i=1; i<=ar; i++)
    if(a[i]<=0.0){
      fprintf(stderr, "All Dirichlet parameters should be +ve in >dirichlet_entropy<\n");
      return FALSE;
    }
  
  sum_a = 0.0;
  (*e) = 0.0;
  for(i=1; i<=ar; i++){
    sum_a += a[i];
    gammaln_scalar(a[i], &tmp);
    (*e) += tmp;
    digamma_scalar(a[i], &tmp);
    (*e) -= (a[i]-1)*tmp;
  }
  gammaln_scalar(sum_a, &tmp);
  (*e) -= tmp;
  digamma_scalar(sum_a, &tmp);
  (*e) += (sum_a-ar)*tmp;
  
  return TRUE;
}
