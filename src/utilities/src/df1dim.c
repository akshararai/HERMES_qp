extern int ncom;	/* defined in DLINMIN */
extern double *pcom,*xicom,(*nrfunc)();
extern void (*nrdfun)();

double my_df1dim(x)
double x;
{
	int j;
	double df1=0.0;
	double *xt,*df,*my_vector();
	void my_free_vector();

	xt=my_vector(1,ncom);
	df=my_vector(1,ncom);
	for (j=1;j<=ncom;j++) xt[j]=pcom[j]+x*xicom[j];
	(*nrdfun)(xt,df);
	for (j=1;j<=ncom;j++) df1 += df[j]*xicom[j];
	my_free_vector(df,1,ncom);
	my_free_vector(xt,1,ncom);
	return df1;
}
