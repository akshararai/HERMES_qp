/*!=============================================================================
  ==============================================================================

  \file    lwpr_error_test.c

  \author  Stefan Schaal
  \date    Feb. 98

  ==============================================================================
  \remarks
  
  a test to run RFWR based on error signals
  
  ============================================================================*/

#include "stdio.h"
#include "math.h"
#include "utility.h"
/* #include "rfwr_util.h" */
#include "lwpr.h" 


#define  LWPR1 1
#define  LWPR2 2

/* import the variables from main */

extern    int   argc_global;
extern   char **argv_global;


/* steering variables */

double   period = 1.0;
double   dt = 1./480.;

/* general variables */

int init_LWPR(void);

/* functions */

/*!*****************************************************************************
 *******************************************************************************
 \note  main
 \date  January, 1994
 
 \remarks 
 
 runs the rfwr simulator in an event loop
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 none
 
 ******************************************************************************/
int
main()

{
    
  int    i,j,n;
  double ds[3+1];
  double in[20+1];
  double out[1+1];
  double s[3+1];
  double err[1+1];
  double pred[1+1];
  double w;
  double ct=0.0;
  double command=0;
  double m=.1;
  double k=.2;
  double b=.3;
  FILE  *fptr;
  double store[100];
  int    n_delays=500;
  double aux=0;
  int    end=10000;
  double lambda = 0.999;
  double sse=0;
  double n_data = 0;
  double mse=0;

  w = 2.*PI/period;

  init_LWPR();

  for (i=1; i<=3; ++i) {
    ds[i] =0.0;
    s[i]  =0.0;
  }

  ds[1]=s[1]=0.5;

  get_int("how many iterations?",end,&end);

  fptr = fopen("temp.data","w");

  for (i=1; i<=end; ++i) {

    /* the current time */
    ct  += dt;

    /* the current desired state which is the result of the desired 
       acceleration from the previous step */
    ds[2] += ds[3]*dt;
    ds[1] += ds[2]*dt;
    s[2]  += s[3]*dt;
    s[1]  += s[2]*dt;
    
    /* the acceleration for the desired state */
    aux    = 0.2*aux + gaussian(0.0,1.0)*10.0;
    ds[3]  = ((-20.*ds[1] + 0.1* (0.25-sqr(ds[1]))*ds[2])+aux)/10.;
    
    /* the feedback error signal:
       compare the desired state with the current state:
       this was the error that the last command caused */

    
    store[n_delays] = (0.5*(ds[1]-s[1]) +0.1*(ds[2]-s[2]));
    err[1] = store[0];
    for (j=0;j<n_delays; ++j)
      store[j]=store[j+1];

    /* the perfect contoller: it applies the inv. dyn. command
       according to the current desired state and the desired
       acceleration, and add the feedback command due to the
       error in the previous motor command */

    /*
      err[1] = m*ds[3]+b*ds[2]+k*ds[1] + 0.1*(ds[1]-s[1]) + 
      0.1*(ds[2]-s[2]) - pred[1];  
      */

    if (i%8==0) /* avoid the initial tansient */
      addDataToLWPR(LWPR1,in,err,TRUE);

    /* build the new input state for the learning box */

    in[10]=in[5];
    in[9]=in[4];
    in[8]=in[3];
    in[7]=in[2];
    in[6]=in[1];
    in[1]=s[1];
    in[2]=s[2];
    in[3]=ds[1];
    in[4]=ds[2];
    in[5]=ds[3];

    predictLWPROutput(LWPR1, in, 0.001, TRUE, pred, &n);

    sse = sse*lambda + sqr(err[1]);
    n_data = n_data*lambda + 1;
    mse    = sqrt(sse/n_data);

    if (i%100==0) {
      printf("%f   %f  %f   %f %f\n",ct,mse,pred[1],ds[1],s[1]);
    }
    if (i%10==0) {
      fprintf(fptr,"%f   %f  %f   %f %f\n",ct,mse,pred[1],ds[1],s[1]);
    }

    command = err[1] + pred[1];

    s[3]  =(-b*s[2]-k*s[1] + command)/m;

  }

  writeLWPR(LWPR1);
  fclose(fptr);

  return TRUE;
    
}

int 
init_LWPR(void)

{
  int ans=0;
  int new_model;
  char fname[]="error_test.script";
  static int firsttime = TRUE;

  /* this allows to generate the SDC */

  get_int("Generate new LWPR = 1; Read from file = 0",ans,&ans);
  if (ans) new_model = TRUE;

  if (firsttime) {
    firsttime = FALSE;
  } else {
    deleteLWPR(LWPR1);
  }


  if (!readLWPRScript(fname,ans,LWPR1)) {

    fprintf(stderr,"Error when reading script file >%s<\n",fname);
    exit(-1);
  }
  return TRUE;

}

