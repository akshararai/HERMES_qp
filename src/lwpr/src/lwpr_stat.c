/*!=============================================================================
  ==============================================================================

  \file    lwpr_stat.c

  \author  Stefan Schaal
  \date    June 1995

  ==============================================================================
  \remarks
  
  A program which prints the statistics about a LWPR
  
  ============================================================================*/

#include "stdio.h"
#include "math.h"
#include "utility.h"
#include "lwpr.h" 
#include "string.h"

extern    int   argc_global;
extern   char **argv_global;

double *inp;
double *outp;
double **jac;
double activation;
double cutoff = 0.001;
int    blend = TRUE;

void 
generateMathematicaSurf();
void 
generateRBFData() ;

     
/*!*****************************************************************************
 *******************************************************************************
 \note  runLWPR
 \date  January, 1994
 
 \remarks 
 
 entry to this evaluation program
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 none
 
 ******************************************************************************/
void 
runLWPR()

{
    
  int    i,j,k,n,rc,r;
  char   fname[100]="inv_dynamics2DOF\0";
  FILE  *in;
  char   ans[100]="q";
  char   string[100];
  int    rfID=1;
  int    index_max=0;
  double max_error;
  int    answ;
  int    n_proj;


  /* I need the filename of the script file: first check whether the
     user provided it in the argv_global variables */

  if (argc_global > 0) {
    in = fopen(argv_global[1],"r");
    if (in != NULL) {
      fclose(in);
      strcpy(fname,argv_global[1]);
    } else {
      if (!getFile(fname)) exit(-1);
    }
  } else {
    if (!getFile(fname)) exit(-1);
  }

  /* read the LWPR */

  if (!readLWPR(1,fname)) exit(-1);

  inp  = my_vector(1,lwprs[1].n_in_reg);
  outp = my_vector(1,lwprs[1].n_out);
  jac  = my_matrix(1,lwprs[1].n_out,1,lwprs[1].n_in_reg);

  vec_zero(inp);

  printLWPRStatistics(1);

  
 AGAIN:

  printf("\nFURTHER OPTIONS:\n");
  printf("               Write RF                      --> 1\n");
  printf("               Write RF with largest error   --> 2\n");
  printf("               Write trust RF w.larg.error   --> 3\n");
  printf("               Lookup query point            --> 4\n");
  printf("               Delete a RF                   --> 5\n");
  printf("               Save LWPR                     --> 6\n");
  printf("               Mathematica Surface           --> 7\n");
  printf("               Matlab Output                 --> 8\n");
  printf("               RBF-like data                 --> 9\n");
  printf("               Update NN                     --> n\n");
  printf("               Quit                          --> q\n");
  if (!get_string("                                 Input",ans,ans)) exit(-1);

  switch (ans[0]) {

  case 'q':

    exit(-1);
    break;


  case '1':

    get_int("Which RF should be written?",rfID,&rfID);
    if (rfID < 1 || rfID > lwprs[1].n_rf) {
      printf("RF ID out of range 1-%ld\n",lwprs[1].n_rf);
      break;
    }
    printRFStatistics(1, rfID);
    writeRFAscii(1,rfID,FALSE,"Debugging Output");

    break;


  case '2':

    max_error = -999;
    for (i=1; i<=lwprs[1].n_rf;++i) {
      n_proj = lwprs[1].rfs[i].n_proj;
      if (lwprs[1].rfs[i].sum_error[n_proj]/lwprs[1].rfs[i].sum_weights[n_proj] 
	  > max_error) {
	max_error = 
	  lwprs[1].rfs[i].sum_error[n_proj]/lwprs[1].rfs[i].sum_weights[n_proj];
	  index_max = i;
      }
    }

    writeRFAscii(1,index_max,FALSE,"Debugging Output");

    break;


  case '3':

    max_error = -999;
    for (i=1; i<=lwprs[1].n_rf;++i) {
      if (!lwprs[1].rfs[i].trustworthy) continue;
      if (lwprs[1].rfs[i].sum_error[lwprs[1].rfs[i].n_proj] > max_error) {
	max_error = lwprs[1].rfs[i].sum_error[lwprs[1].rfs[i].n_proj];
	index_max = i;
      }
    }

    writeRFAscii(1,index_max,FALSE,"Debugging Output");

    break;

  case '4':

    if (lwprs[1].n_in_reg != lwprs[1].n_in_w) {
      printf("Inputs for regression and weighting need to be the same!\n");
      break;
    }

    get_double("Cutoff?",cutoff,&cutoff);
    get_int("Blending?",blend,&blend);

    for (i=1; i<=lwprs[1].n_in_reg;++i) {
      sprintf(string,"%d.query input",i);
      get_double(string,inp[i],&inp[i]);
    }

    activation = 
      predictLWPROutput(1, inp, inp, cutoff, blend, outp, &i);

    print_vec("Query Point",inp);
    print_vec("Prediction",outp);
    /* print_mat("Linear Model",jac,lwprs[1].n_out,lwprs[1].n_in); */

    break;

  case '5':

    if (!get_int("Which RF should be deleted?",rfID,&rfID))
      break;
    get_int("Are you sure?",0,&answ);
    if (answ != 1) 
      break;

    deleteRF(1,rfID);

    break;

  case '6':

    writeLWPR(1);

    break;

  case '7':

    if (lwprs[1].n_in_reg != lwprs[1].n_in_w) {
      printf("Inputs for regression and weighting need to be the same!\n");
      break;
    }


    if (lwprs[1].n_in_w==2) {
      generateMathematicaSurf();
    }

    break;

  case '8':

    /* write diagonal of distance metric */
    get_int("Diagonal=1 or Full Distance Metric=0",1,&answ);
    if (answ)
      in = fopen("Ddiag.data","w");
    else
      in = fopen("D.data","w");

    if (in != NULL ) {
      for (i=1; i<=lwprs[1].n_rf;++i) {
	for (j=1; j<=lwprs[1].n_in_w; ++j) {
	  if (answ) {
	    fprintf(in,"%f ",lwprs[1].rfs[i].D[j][j]);
	  } else {
	    for (r=1; r<=lwprs[1].n_in_w; ++r)
	      fprintf(in,"%f ",lwprs[1].rfs[i].D[j][r]);	      
	    fprintf(in,"\n");
	  }
	}
	fprintf(in,"\n");
      }
    }
    fclose(in);
    
    /* write all the projections */
    in = fopen("W.data","w");
    if (in != NULL ) {
      for (i=1; i<=lwprs[1].n_rf;++i) {
	for (r=1; r<=lwprs[1].rfs[i].n_proj; ++r) {
	  for (j=1; j<=lwprs[1].n_in_reg; ++j) {
	    fprintf(in,"%f ",lwprs[1].rfs[i].W[r][j]);
	  }
	  fprintf(in,"\n");
	}
	fprintf(in,"\n");
      }
    }
    fclose(in);
    
    /* write all the input space projections */
    in = fopen("U.data","w");
    if (in != NULL ) {
      for (i=1; i<=lwprs[1].n_rf;++i) {
	for (r=1; r<=lwprs[1].rfs[i].n_proj; ++r) {
	  for (j=1; j<=lwprs[1].n_in_reg; ++j) {
	    fprintf(in,"%f ",lwprs[1].rfs[i].U[r][j]);
	  }
	  fprintf(in,"\n");
	}
	fprintf(in,"\n");
      }
    }
    fclose(in);
    
    /* write all regression coefficients */
    in = fopen("B.data","w");
    if (in != NULL ) {
      for (i=1; i<=lwprs[1].n_rf;++i) {
	for (j=1; j<=lwprs[1].n_out; ++j) {
	  for (r=1; r<=lwprs[1].rfs[i].n_proj; ++r) {
	    fprintf(in,"%f ",lwprs[1].rfs[i].B[r][j]);
	  }
	  fprintf(in,"\n");
	}
	fprintf(in,"\n");
      }
    }
    fclose(in);
    
    /* write all centers */
    in = fopen("C.data","w");
    if (in != NULL ) {
      for (i=1; i<=lwprs[1].n_rf;++i) {
	for (j=1; j<=lwprs[1].n_in_w; ++j) {
	  fprintf(in,"%f ",lwprs[1].rfs[i].c[j]);
	}
	fprintf(in,"\n");
      }
    }
    fclose(in);
    
    
    break;

  case '9':

    if (lwprs[1].n_in_reg != lwprs[1].n_in_w) {
      printf("Inputs for regression and weighting need to be the same!\n");
      break;
    }

    generateRBFData();
    
    break;

  case 'n':

    checkLWPRNN(1);
    break;

  default:

    break;

  }

  goto AGAIN;
    
}

/*!*****************************************************************************
 *******************************************************************************
 \note  generateMathematicaSurf
 \date  January, 19941994
 
 \remarks 
 
  for 2D functions, generates polygons for display with mathematica
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 none
 
 ******************************************************************************/
void 
generateMathematicaSurf() 

{
  int     i,j,k,m,o;
  double *input;
  double *output;
  double**mat;
  FILE   *out;
  int     n_patch[10];
  double *inc;
  char    string[100];
  int     n_in, n_out;
  double *maxs, *mins;
  int     rfID;


  strcpy(string,lwprs[1].lwpr_name);
  n_in = lwprs[1].n_in_w;
  n_out = lwprs[1].n_out;

  input  = my_vector(1,n_in);
  output = my_vector(1,n_out);
  maxs   = my_vector(1,n_in);
  mins   = my_vector(1,n_in);
  inc    = my_vector(1,n_in);

  get_int("Number of Steps x",40.,&n_patch[1]);
  get_int("Number of Steps y",40.,&n_patch[2]);
  mat     = my_matrix(0,n_patch[1],0,n_patch[2]);

  for (i=1; i<=n_in; ++i) {
    sprintf(string,"Max. of %d.dim",i);
    maxs[i] = 1;
    get_double(string,maxs[i],&maxs[i]);
    sprintf(string,"Min. of %d.dim",i);
    mins[i] = -1;
    get_double(string,mins[i],&mins[i]);
    printf("\n");
    inc[i] = (maxs[i]-mins[i])/(double)n_patch[i];
  }


  for (i=0; i<=n_patch[1]; ++i) {
    for (j=0; j<=n_patch[2]; ++j) {
    
      input[1] = i*inc[1] + mins[1];
      input[2] = j*inc[2] + mins[2];
      
      if (predictLWPROutput(1, input, input, 0.001, TRUE, output,&rfID)) {
	  mat[i][j] = output[1];
      } else {
	mat[i][j] = 0;
      }
    }
  }

  out = fopen("mathematica.data","w");

  for (i=0; i<n_patch[1]; ++i) {
    for (j=0; j<n_patch[2]; ++j) {
       
      fprintf(out,"Polygon[{{%f,%f,%f},",
	      i*inc[1]+ mins[1],j*inc[2]+mins[2],mat[i][j]);
      fprintf(out,"{%f,%f,%f},",
	      (i+1)*inc[1]+ mins[1],j*inc[2]+mins[2],mat[i+1][j]);
      fprintf(out,"{%f,%f,%f},",
	      (i+1)*inc[1]+ mins[1],(j+1)*inc[2]+mins[2],mat[i+1][j+1]);
      fprintf(out,"{%f,%f,%f},",
	      (i)*inc[1]+ mins[1],(j+1)*inc[2]+mins[2],mat[i][j+1]);
      fprintf(out,"{%f,%f,%f}}]\n",
		i*inc[1]+ mins[1],j*inc[2]+mins[2],mat[i][j]);
    }
  }

  fclose(out);

  exit(1);
}
/*!*****************************************************************************
 *******************************************************************************
 \note  generateRBFData
 \date  August 1996
 
 \remarks 
 
  runs LWPR on a particular test data set, and generates a data set of
  weighted predictions of all RFs which can be used in a subsequent
  RBF-like net
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 none
 
 ******************************************************************************/
void 
generateRBFData() 

{

  FILE *in,*out;
  int   i,j,rc=0;
  char  fname[100];
  double w;
  double sum_w;

  if (!getFile(fname))
    return;

  in = fopen(fname,"r");
  sprintf(fname,"%s.rbf",fname);
  out = fopen(fname,"w");

  for (;;) {

    for (i=1; i<=lwprs[1].n_in_w; ++i) {
      rc = fscanf(in,"%lf",&inp[i]);
      if (rc == EOF)
	break;
    }
    if (rc == EOF)
      break;

    for (i=1; i<=lwprs[1].n_out; ++i) {
      rc = fscanf(in,"%lf",&outp[i]);
      if (rc == EOF)
	break;
    }
    if (rc == EOF)
      break;


    sum_w = 0;
    for (i=1; i<=lwprs[1].n_rf; ++i) {
      sum_w += predictLWPROutputRF(1, inp, inp, i, outp);
    }
    for (i=1; i<=lwprs[1].n_rf; ++i) {
      w = predictLWPROutputRF(1, inp, inp, i, outp);
      fprintf(out,"%f ",w*outp[1]/sum_w);
    }
    fprintf(out,"\n");

  }

  fclose(in);
  fclose(out);

}


















