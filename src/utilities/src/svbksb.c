#include "utility.h"

void my_svbksb(double **u,double w[],double **v,int m,int n, double b[], double x[])
//double **u,w[],**v,b[],x[];
//int m,n;
{
	int jj,j,i;
	double s,*tmp,*my_vector();
	void my_free_vector();

	tmp=my_vector(1,n);
	for (j=1;j<=n;j++) {
		s=0.0;
		if (w[j]) {
			for (i=1;i<=m;i++) s += u[i][j]*b[i];
			s /= w[j];
		}
		tmp[j]=s;

	}
	for (j=1;j<=n;j++) {
		s=0.0;
		for (jj=1;jj<=n;jj++) s += v[j][jj]*tmp[jj];
		x[j]=s;
	}
	my_free_vector(tmp,1,n);
}
