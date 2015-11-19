/*!=============================================================================
  ==============================================================================

  \file    data_sets.c

  \author  Copyright Stefan Schaal
  \date    January 1996

  ==============================================================================
  \remarks
  
  some simple programs to generate data from data sets and generating
  functions
  
  ============================================================================*/

#include "stdio.h"
#include "math.h"
#include "utility.h"
#include "mdefs.h"

#include "data_sets.h"


void (*generating_function[MAX_GENERATING_FUNCTIONS+1])(double *x, double *y);


/*!*****************************************************************************
 *******************************************************************************
 \note  draw_data_point
 \date  January, 1994
 
 \remarks 
 
 draws a new training data point according to various critera
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     X      : the data matrix with input data
 \param[in]     Y      : the data matrix with output data
 \param[in]     n_data : number of data points in matrices
 \param[in]     n_in   : number of inputs
 \param[in]     n_out  : number of outputs
 \param[in]     index_function : index function to be used for GENERATING_FUNCTION
 \param[in]     sampling_method : RANDOM_SAMPLING, SEQUENTIAL_SAMPLING,
                      GENERATING_FUNCTION, PSEUDO_RANDOM_SAMPLING
 \param[in]     ind    : for pseudo random sampling ptr to an index array. Pass a variable
             initialized with NULL to make this program assign it
             appropriately		      
 \param[out]    x      : the output vector
 \param[out]    y      : the input vector
 \param[out]    current_data_point : the current data point
 
 
 ******************************************************************************/
int
draw_data_point(double **X, double **Y, int n_data, int n_in, int n_out,
		int sampling_method, int index_function, int **ind,
		double *x, double *y, int *current_data_point)

{

  int i;
  static int firsttime = TRUE;
  int num;


  if (firsttime) {

    firsttime = FALSE;
    
    /* define the generating functions */
    
    generating_function[1] = cross_3_noise;
    generating_function[2] = cross_8_noise;
    generating_function[3] = cross;
    generating_function[4] = squiggle;
    generating_function[5] = inv_dyn_2DOF;
    generating_function[6] = cross_nonsense_dim;
    generating_function[7] = cross_2_noise_1;
    generating_function[8] = cross_2_noise_01;
    generating_function[9] = cross_2_noise_001;
    generating_function[10] = cross_2_brown_10;
    generating_function[11] = cross_2_brown_25;
    generating_function[12] = cross_2_brown_50;
    generating_function[13] = cross_2_noise_2_brown;
    generating_function[14] = cross_1_noise_1_brown_1_duplicate;
    generating_function[15] = cross1;
    generating_function[16] = cross2;
    generating_function[17] = cross3;
    generating_function[18] = cross10d;
    generating_function[19] = cross10d_10_noise;

  
  }

  switch (sampling_method) {

  case RANDOM_SAMPLING:

    *current_data_point = (int) random_number(1,n_data);

    for (i=1; i<=n_in; ++i) {
      x[i] = X[*current_data_point][i];
    }
                
    for (i=1; i<=n_out; ++i) {
      y[i] = Y[*current_data_point][i];
    }
                
    break;
    

  case SEQUENTIAL_SAMPLING:

    if (++*current_data_point > n_data) *current_data_point = 1;

    for (i=1; i<=n_in; ++i) {
      x[i] = X[*current_data_point][i];
    }
                
    for (i=1; i<=n_out; ++i) {
      y[i] = Y[*current_data_point][i];
    }

    break;


  case GENERATING_FUNCTION:

    (*generating_function[index_function])(x,y);

    break;


  case PSEUDO_RANDOM_SAMPLING:

    if ( (*ind) == NULL) {
      (*ind)    = my_ivector(0,n_data);
      (*ind)[0] = n_data;
      for (i=1; i<=n_data; ++i) {
	(*ind)[i]=i;
      }
    }

    /* this rearanges the data array a lot, but it will ensure that
       each epoch each data point is exactly used once, with the
       additional randomization from swapping the index variables */

    num = random_number(1,(*ind)[0]);
    *current_data_point = (*ind)[num];
    (*ind)[num] = (*ind)[(*ind)[0]];
    (*ind)[(*ind)[0]] = *current_data_point;
    if (--((*ind)[0]) < 1) (*ind)[0]=n_data;

    for (i=1; i<=n_in; ++i) {
      x[i] = X[*current_data_point][i];
    }
                
    for (i=1; i<=n_out; ++i) {
      y[i] = Y[*current_data_point][i];
    }
    
    break;

  default:

    return FALSE;

  }


  return TRUE;


}

/*****************************************************************************
 ******************************************************************************

 all the generating functions are currently here
 
 *****************************************************************************/

void
squiggle(double *in, double *out)

{

  int i;
  int n = 0;
  double aux;

  in[++n] = uniform(0.5,0.2887);
  in[++n] = uniform(0.5,0.2887);

  aux     = in[1]-.5;
  out[1]  = (in[2]-.5) * sin(30.* Pi * in[1]* sin(sqr(aux)*aux));
  out[1] += gaussian(0.0,0.02);

}

void
cross(double *in, double *out)

{

  int n=0;
  double val=0.0;

  cross_n_noise_val(in, out, n, val);

}


void
cross10d(double *in, double *out)

{

  int i,j;
  int n=0;
  double val=0.0;
  static int firsttime=TRUE;
  static double **R;

  if (firsttime) {
    double T[]={0,
   1.5866155e-01,  -4.0970565e-01,   1.2437669e-01,  -4.7004816e-01,  -4.5033935e-02,   3.2980265e-01,  -3.1433783e-01,   4.4596580e-01,  -3.5238434e-01,   1.9464489e-01,
   5.3154002e-02,  -3.4126019e-02,   8.1903780e-01,   2.8636308e-01,  -1.8599783e-01,  -1.1114280e-01,  -2.5427668e-01,   1.3373057e-01,   1.1058119e-01,  -3.1853842e-01,
  -3.7785070e-01,  -9.8121296e-02,  -9.9138898e-02,  -6.7912728e-03,   1.3497418e-01,  -4.3836002e-01,  -1.2064505e-01,   5.7284087e-01,   4.6629616e-01,   2.5925832e-01,
   4.1026930e-01,  -4.1175361e-01,  -1.8141995e-01,   4.7971368e-01,  -7.1323546e-02,  -3.7385908e-01,   2.8983030e-01,   2.3907950e-01,  -3.3627133e-01,   1.7237891e-03,
  -2.3904578e-01,  -1.3020234e-01,  -3.0963723e-01,  -2.7410899e-01,  -5.4349236e-01,  -1.6451788e-01,  -2.3083771e-02,   7.7132581e-02,  -2.6421191e-02,  -6.5212197e-01,
   8.1315376e-02,  -4.8628922e-01,   1.2277770e-01,  -1.0765958e-01,   7.5792744e-02,   3.5251906e-01,   5.4723384e-01,  -1.0724859e-02,   5.3014852e-01,  -1.3997883e-01,
   1.1994873e-01,   1.2055965e-01,  -1.7107656e-01,   7.7250840e-02,   6.8490260e-01,   1.4187253e-01,  -1.5241727e-01,   2.8092646e-01,  -3.6416354e-02,  -5.8578713e-01,
   2.4292079e-01,  -3.3051703e-01,   7.6884381e-02,  -4.0062676e-01,   2.9464768e-01,  -5.4045122e-01,  -2.3055004e-01,  -4.7122965e-01,   9.2444517e-02,  -5.1705867e-02,
   3.7755380e-01,   5.0715076e-01,   1.9479501e-01,  -4.5133975e-01,  -4.4277090e-02,  -2.5487221e-01,   4.4956833e-01,   2.9802336e-01,  -1.7302781e-02,  -2.1195931e-02,
  -6.1742121e-01,  -1.3190896e-01,   2.9477572e-01,  -9.9880915e-02,   2.8473480e-01,  -1.3578056e-01,   3.9935511e-01,  -4.5448028e-02,  -4.9107994e-01,  -4.7982701e-02};
		  
    firsttime = FALSE;
    R = my_matrix(1,10,1,10);
    
    for (i=1; i<=10; ++i) {
      for (j=1; j<=10; ++j) {
	R[i][j] = T[10*(i-1)+j];
      }
    }
 
  }

  cross_n_noise_val(in, out, n, val);
  for (i=3; i<=10; ++i) in[i]=0.0;
  mat_vec_mult(R,in,in);

}


void
cross10d_10_noise(double *in, double *out)

{

  int i,j;
  int n=12;
  double val=0.025;
  static int firsttime=TRUE;
  static double **R;

  if (firsttime) {
    double T[]={0,
   1.5866155e-01,  -4.0970565e-01,   1.2437669e-01,  -4.7004816e-01,  -4.5033935e-02,   3.2980265e-01,  -3.1433783e-01,   4.4596580e-01,  -3.5238434e-01,   1.9464489e-01,
   5.3154002e-02,  -3.4126019e-02,   8.1903780e-01,   2.8636308e-01,  -1.8599783e-01,  -1.1114280e-01,  -2.5427668e-01,   1.3373057e-01,   1.1058119e-01,  -3.1853842e-01,
  -3.7785070e-01,  -9.8121296e-02,  -9.9138898e-02,  -6.7912728e-03,   1.3497418e-01,  -4.3836002e-01,  -1.2064505e-01,   5.7284087e-01,   4.6629616e-01,   2.5925832e-01,
   4.1026930e-01,  -4.1175361e-01,  -1.8141995e-01,   4.7971368e-01,  -7.1323546e-02,  -3.7385908e-01,   2.8983030e-01,   2.3907950e-01,  -3.3627133e-01,   1.7237891e-03,
  -2.3904578e-01,  -1.3020234e-01,  -3.0963723e-01,  -2.7410899e-01,  -5.4349236e-01,  -1.6451788e-01,  -2.3083771e-02,   7.7132581e-02,  -2.6421191e-02,  -6.5212197e-01,
   8.1315376e-02,  -4.8628922e-01,   1.2277770e-01,  -1.0765958e-01,   7.5792744e-02,   3.5251906e-01,   5.4723384e-01,  -1.0724859e-02,   5.3014852e-01,  -1.3997883e-01,
   1.1994873e-01,   1.2055965e-01,  -1.7107656e-01,   7.7250840e-02,   6.8490260e-01,   1.4187253e-01,  -1.5241727e-01,   2.8092646e-01,  -3.6416354e-02,  -5.8578713e-01,
   2.4292079e-01,  -3.3051703e-01,   7.6884381e-02,  -4.0062676e-01,   2.9464768e-01,  -5.4045122e-01,  -2.3055004e-01,  -4.7122965e-01,   9.2444517e-02,  -5.1705867e-02,
   3.7755380e-01,   5.0715076e-01,   1.9479501e-01,  -4.5133975e-01,  -4.4277090e-02,  -2.5487221e-01,   4.4956833e-01,   2.9802336e-01,  -1.7302781e-02,  -2.1195931e-02,
  -6.1742121e-01,  -1.3190896e-01,   2.9477572e-01,  -9.9880915e-02,   2.8473480e-01,  -1.3578056e-01,   3.9935511e-01,  -4.5448028e-02,  -4.9107994e-01,  -4.7982701e-02};
		  
    firsttime = FALSE;
    R = my_matrix(1,10,1,10);
    
    for (i=1; i<=10; ++i) {
      for (j=1; j<=10; ++j) {
	R[i][j] = T[10*(i-1)+j];
      }
    }
 
  }

  cross_n_noise_val(in, out, n, val);
  for (i=20; i>=10; --i)
    in[i] = in[i-8];
  for (i=3; i<=10; ++i) in[i]=0.0;
  mat_vec_mult_size(R,10,10,in,10,in);

}


void
cross1(double *in, double *out)

{

  int n=0;
  double val=0.0;
  static double r_max[3]={0.0,-0.2, 1.};
  static double r_min[3]={0.0,-1.0,-1.};

  cross_region(in, out, r_min,r_max);

}


void
cross2(double *in, double *out)

{

  int n=0;
  double val=0.0;
  static double r_max[3]={0.0, 0.4, 1.};
  static double r_min[3]={0.0,-0.4,-1.};

  cross_region(in, out, r_min,r_max);

}


void
cross3(double *in, double *out)

{

  int n=0;
  double val=0.0;
  static double r_max[3]={0.0, 1.0, 1.};
  static double r_min[3]={0.0, 0.2,-1.};

  cross_region(in, out, r_min,r_max);

}


void
cross_nonsense_dim(double *in, double *out)

{

  int n=2;
  double val=0.01;
  static double ttime=0;
  double dt = 0.01;
  double tau1=100.0,tau2=1.0;

  cross_n_noise_val(in, out, n, val);

  n = 4;

  ttime += dt;

  in[++n] = sin(2.*Pi/tau1*ttime)*0.1;
  in[++n] = sin(2.*Pi/tau2*ttime)*0.2;

}

void
cross_8_noise(double *in, double *out)

{

  int n=8;
  double val=0.025;

  cross_n_noise_val(in, out, n, val);
}

void
cross_3_noise(double *in, double *out)

{

  int n=3;
  double val=0.025;

  cross_n_noise_val(in, out, n, val);
}

void
cross_2_noise_001(double *in, double *out)

{
  int n=2;
  double val=0.001;

  cross_n_noise_val(in, out, n, val);
}

void
cross_2_noise_01(double *in, double *out)

{
  int n=2;
  double val=0.01;

  cross_n_noise_val(in, out, n, val);
}

void
cross_2_noise_1(double *in, double *out)

{
  int n=2;
  double val=0.1;

  cross_n_noise_val(in, out, n, val);
}

void
cross_n_noise_val(double *in, double *out, int n, double val)

{
  int i,j;
  double v1,v2,v3;

  j = 0;

  in[++j] = uniform(0.0,0.5774);
  in[++j] = uniform(0.0,0.5774);

  for (i=1; i<=n; ++i) {
    in[++j] = gaussian(0.0,val)+(i-1);
  }

  v1 = exp(-sqr(in[1])*10);
  v2 = exp(-sqr(in[2])*50);
  v3 = exp(-(sqr(in[1])+sqr(in[2]))*5) * 1.25;
  out[1] = v1;
  if (v2 > out[1]) out[1] = v2;
  if (v3 > out[1]) out[1] = v3;

  out[1] += gaussian(0.0,0.05);

}

void
cross_region(double *in, double *out, double *r_min, double *r_max)

{
  int i,j;
  double v1,v2,v3;

  j = 0;

  ++j;
  in[j] = uniform(0.0,0.5774)*(r_max[j]-r_min[j])/2.+(r_max[j]+r_min[j])/2.;
  ++j;
  in[j] = uniform(0.0,0.5774)*(r_max[j]-r_min[j])/2.+(r_max[j]+r_min[j])/2.;

  v1 = exp(-sqr(in[1])*10);
  v2 = exp(-sqr(in[2])*50);
  v3 = exp(-(sqr(in[1])+sqr(in[2]))*5) * 1.25;
  out[1] = v1;
  if (v2 > out[1]) out[1] = v2;
  if (v3 > out[1]) out[1] = v3;

  out[1] += gaussian(0.0,0.1);

}


void 
cross_2_brown_10(double *in, double *out)
{
  int n=0;
  int i;
  double val=0.0;
  static double x[2+1];
  double mult = 0.1;

  cross_n_noise_val(in, out, n, val);

  for (i=1; i<=2; ++i) {
    x[i] = cross_brown(x[i]);
    in[2+i] = x[i]*mult+(i-1);
  }

}

void 
cross_2_brown_50(double *in, double *out)
{
  int n=0;
  int i;
  double val=0.0;
  static double x[2+1];
  double mult = 0.50;

  cross_n_noise_val(in, out, n, val);

  for (i=1; i<=2; ++i) {
    x[i] = cross_brown(x[i]);
    in[2+i] = x[i]*mult+(i-1);
  }

}

void 
cross_2_brown_25(double *in, double *out)
{
  int n=0;
  int i;
  double val=0.0;
  static double x[2+1];
  double mult = 0.25;

  cross_n_noise_val(in, out, n, val);

  for (i=1; i<=2; ++i) {
    x[i] = cross_brown(x[i]);
    in[2+i] = x[i]*mult+(i-1);
  }

}

void 
cross_2_noise_2_brown(double *in, double *out)
{
  int n=2;
  int i;
  double val=0.01;
  static double x[2+1];
  double mult = 0.25;

  cross_n_noise_val(in, out, n, val);

  for (i=1; i<=2; ++i) {
    x[i] = cross_brown(x[i]);
    in[4+i] = x[i]*mult+(i-1);
  }
}

void 
cross_1_noise_1_brown_1_duplicate(double *in, double *out)
{
  int n=1;
  int i;
  double val=0.0316; /* this is var=0.001 */
  static double x[1+1];
  double mult = 0.1;

  cross_n_noise_val(in, out, n, val);

  in[3] += 0.1;

  for (i=1; i<=1; ++i) {
    x[i] = cross_brown(x[i]);
    in[3+i] = x[i]*mult+(i-1);
  }

  in[5] = in[1] + gaussian(0.0,0.05);

}

double
cross_brown(double x)
{
  int i;

  x += uniform(0.0,0.5774)*0.1;
  if (x >  1.0) x =  1.0 - (x - 1.0);
  if (x < -1.0) x = -1.0 - (x + 1.0);

  return x;
}

void
inv_dyn_2DOF(double *in, double *out)
{

  static double t=0.0;
  double        dt = .01;
  double        ltau,lamp;
  double        init_tau = 2.,tau;
  double        amplitude = 0.06;
  double        x ,y;
  double        setx= -.3,sety=.3;
  static double        last_th1, last_th2;
  static double        last_th1d, last_th2d;
  static int           firsttime = TRUE;
  int                  count=0, i;


 AGAIN:

  if (firsttime && count == 0) {
    last_th1 = setx;
    last_th2 = sety;
    last_th1d = 0;
    last_th2d = 0;
    t = 0.0;
  }

  tau = init_tau;

  ltau =  tau*PI/2.;;
  lamp =  amplitude*1.5;
  setx += sin(2.*Pi/ltau * t) * lamp;

  ltau =  tau*2.*PI;
  lamp =  amplitude*0.25;
  sety  += sin(2.*Pi/ltau * t) * lamp;

  ltau =  tau/2.;
  lamp =  amplitude;
  x  = setx + sin(2.*Pi/ltau * t) * lamp;

  ltau =  tau;
  lamp =  amplitude*1.5;
  y = sety + sin(2.*Pi/ltau * t) * lamp;

  inverse_kinematics_2DOF(x,y,&in[1],&in[2]);

  in[3] = (in[1] - last_th1)/dt;
  in[4] = (in[2] - last_th2)/dt;
  in[5] = (in[3] - last_th1d)/dt;
  in[6] = (in[4] - last_th2d)/dt;

  last_th1 = in[1];
  last_th2 = in[2];
  last_th1d = in[3];
  last_th2d = in[4];

  inverse_dynamics_2DOF(in[1],in[2],in[3],in[4],in[5],in[6],&out[1],&out[2]);

  out[1] = gaussian(out[1],.03);
  out[2] = gaussian(out[2],.03);

  t += dt;

  if (firsttime) {
    /* I need the numerical derivatives straightened out */
    if (++count >= 10) firsttime = FALSE;
    tau = init_tau;
    goto AGAIN;
  }

/*
  if (fabs(fmod(t,0.25)) < 0.1*dt) {
    printf("%f %f     ",x,y);
    for (i=1; i<=6; ++i) {
      printf("%f ",in[i]);
    }
    for (i=1; i<=2; ++i) {
      printf("%f ",out[i]);
    }
    printf("\n");
  }
  setx= .3;sety=.3;
  goto AGAIN;
*/


}

/***************************************************************************
 ***************************************************************************
 Function Name	: inverse_dynamics_2DOF
 Date		: January, 1994
 
 Remarks:
 
 the inverse dynamics of a planar 2DOF arm
 
 ***************************************************************************
 Paramters:  (i/o = input/output)
 
 th1,th1d,th1dd (i): states and accelerations for first joint
 th2,th2d,th2dd (i): states and accelerations for second joint
 control1       (o): command for first joint
 control2       (o): command for second joint 
 
 **************************************************************************/
void
inverse_dynamics_2DOF(double th1, double th2, double th1d, double th2d,
		      double th1dd, double th2dd, double *control1,
		      double *control2)

{

  double   m1 = 15.0;
  double   m2 = 15.0;
  double   Inertia1 = 0.05;
  double   Inertia2 = 0.05;
  double   s1 = 0.15;
  double   s2 = 0.15;
  double   l = .30;

  th1 -= PI/2.;

  *control1 = 
    Power(l,2)*m2*th1dd + (Inertia1 + m1*Power(s1,2))*th1dd + 
      (Inertia2 + m2*Power(s2,2))*(th1dd + th2dd) + G*m1*s1*Cos(th1) + 
	l*m2*s2*th1dd*Cos(th2) + l*m2*s2*(th1dd + th2dd)*Cos(th2) + 
	  G*m2*(l*Cos(th1) + s2*Cos(th1 + th2)) -l*m2*s2*th1d*th2d*Sin(th2) - 
	    l*m2*s2*th2d*(th1d + th2d)*Sin(th2);

  *control2 = 
    Inertia2*th1dd + m2*Power(s2,2)*th1dd + Inertia2*th2dd + 
      m2*Power(s2,2)*th2dd + l*m2*s2*th1dd*Cos(th2) + G*m2*s2*Cos(th1+ th2) + 
	l*m2*s2*Power(th1d,2)*Sin(th2);


}

/***************************************************************************
 ***************************************************************************
 Function Name	: forward_dynamics_2DOF
 Date		: January, 1994
 
 Remarks:
 
 the inverse dynamics of a planar 2DOF arm
 
 ***************************************************************************
 Paramters:  (i/o = input/output)
 
 th1,th1d       (i): states and accelerations for first joint
 th2,th2d       (i): states and accelerations for second joint
 control1       (i): command for first joint
 control2       (i): command for second joint 
 dt             (i): integration step
 nth1,nth1d     (o): the new states for the first joint
 nth2,nth2d     (o): the new states for the second joint
 
 **************************************************************************/
void
forward_dynamics_2DOF(double th1, double th2, double th1d, double th2d,
		      double control1, double control2, double dt,
		      double *nth1, double *nth2, double *nth1d,double *nth2d)

{

  double   m1 = 15.0;
  double   m2 = 15.0;
  double   Inertia1 = 0.05;
  double   Inertia2 = 0.05;
  double   s1 = 0.15;
  double   s2 = 0.15;
  double   l = .30;
  double   th1dd, th2dd;

  th1 -= PI/2.;

  th1dd =  
    -((-((-Inertia2 - m2*Power(s2,2) - l*m2*s2*Cos(th2))*
         (control2 - G*m2*s2*Cos(th2 + th1) - l*m2*s2*Power(th1d,2)*Sin(th2))
         ) + (-Inertia2 - m2*Power(s2,2))*
       (control1 - G*l*m2*Cos(th1) - G*m1*s1*Cos(th1) - 
	G*m2*s2*Cos(th2 + th1) + 2*l*m2*s2*th1d*th2d*Sin(th2) + 
	l*m2*s2*Power(th2d,2)*Sin(th2)))/
    (Inertia1*Inertia2 + Inertia2*Power(l,2)*m2 + Inertia2*m1*Power(s1,2) + 
     Inertia1*m2*Power(s2,2) + Power(l,2)*Power(m2,2)*Power(s2,2) + 
     m1*m2*Power(s1,2)*Power(s2,2) - 
     Power(l,2)*Power(m2,2)*Power(s2,2)*Power(Cos(th2),2)));

  th2dd = 
    -((control2 - G*m2*s2*Cos(th2 + th1) - l*m2*s2*Power(th1d,2)*Sin(th2))/
      (-Inertia2 - m2*Power(s2,2))) + 
	(-Inertia2 - m2*Power(s2,2) - l*m2*s2*Cos(th2))*
	  (-((-Inertia2 - m2*Power(s2,2) - l*m2*s2*Cos(th2))*
	     (control2 - G*m2*s2*Cos(th2 + th1) - l*m2*s2*Power(th1d,2)*Sin(th2))
	     ) + (-Inertia2 - m2*Power(s2,2))*
       (control1 - G*l*m2*Cos(th1) - G*m1*s1*Cos(th1) - 
	G*m2*s2*Cos(th2 + th1) + 2*l*m2*s2*th1d*th2d*Sin(th2) + 
	l*m2*s2*Power(th2d,2)*Sin(th2)))/
	  ((-Inertia2 - m2*Power(s2,2))*
	   (Inertia1*Inertia2 + Inertia2*Power(l,2)*m2 + Inertia2*m1*Power(s1,2) + 
	    Inertia1*m2*Power(s2,2) + Power(l,2)*Power(m2,2)*Power(s2,2) + 
	    m1*m2*Power(s1,2)*Power(s2,2) - 
	    Power(l,2)*Power(m2,2)*Power(s2,2)*Power(Cos(th2),2)));
	    
  *nth1  = th1d * dt + th1 + PI/2.;
  *nth2  = th2d * dt + th2;
  *nth1d = th1dd * dt + th1d;
  *nth2d = th2dd * dt + th2d;


}

/***************************************************************************
 ***************************************************************************
 Function Name	: inverse_kinematics_2DOF
 Date		: January, 1994
 
 Remarks:
 
 the inverse kinematics of a planar 2DOF arm
 
 ***************************************************************************
 Paramters:  (i/o = input/output)
 
 x,y            (i): cartesian inputs
 th1,th2        (o): joint states (one solution only)
 
 **************************************************************************/
void
inverse_kinematics_2DOF(double x, double y, double *th1, double *th2)

{

  double   l = .30;
  double   dist;
  double   angle1, angle2;

  dist = sqrt(sqr(x) + sqr(y));
  angle1 = acos(x/dist);
  if (y < 0.0) angle1 *= -1.;
  angle2 = acos(dist/2./l);
  
  *th1 = angle1-angle2+PI/2.;
  *th2 = 2.*angle2;

}

 
/***************************************************************************
 ***************************************************************************
 Function Name	: forward_kinematics_2DOF
 Date		: January, 1994
 
 Remarks:
 
 the forward kinematics of a planar 2DOF arm
 
 ***************************************************************************
 Paramters:  (i/o = input/output)
 
 th1,th2        (i): joint states (one solution only)
 x,y            (o): cartesian inputs
 
 **************************************************************************/
void
forward_kinematics_2DOF(double th1, double th2, double *x, double *y)

{

  double   l = .30;

  th1 -= PI/2.;
  *x = l * cos(th1) + l * cos(th1+th2);
  *y = l * sin(th1) + l * sin(th1+th2);

}

 
 
