/*!=============================================================================
  ==============================================================================

  \file    ascii2mrdplot.c

  \author  Stefan Schaal
  \date    April 2006

  ==============================================================================
  \remarks

      converts an ascii data file to mrdplot format, but names the columns
      according to the column number.

  ============================================================================*/
#include "stdio.h"
#include "stdlib.h"
#include "utility.h"
#include "string.h"


/*!*****************************************************************************
 *******************************************************************************
\note  main
\date  02/25/92 
   
\remarks 
           
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     argc : standard arguments to give some initial input to the program
 \param[in]     arv  : s.o.

 ******************************************************************************/
int
main(int argc, char **argv)
{
  int    i,j,k;
  int    rc;
  char   fname[200];
  char   fname_out[200];
  FILE  *fp;
  int    count;
  char  *string;
  double aux;
  int    n_cols;
  int    n_rows;
  Matrix D;
  char   **vnames;
  char   **units;
  char   special[5];
  char  *strptr;
  
  

  for (i=1; i<argc; ++i) {

    strcpy(fname,argv[i]);
    printf("Processing >%s< ... ",fname);
    
    // open the file
    fp = fopen(fname,"r");
    if (fp == NULL) {
      printf("Cannot open file >%s< -- file skipped!\n",fname);
      continue;
    }

    // find out how many columns by searching for the \n charcter
    count = 0;
    rc=fgetc(fp);
    while (rc != '\n' && rc != EOF) {
      ++count;
      rc=fgetc(fp);
    }

    string = my_calloc(count+1,sizeof(char),MY_STOP);

    rewind(fp);

    for (j=1; j<=count; ++j)
      string[j-1] = fgetc(fp);
    string[j] = '\0';


    count = 0;
    special[0] = ' ';
    special[1] = '\t';
    special[2] = ';';
    special[3] = ',';
    special[4] = '\0';
    strptr = string;
    while (strtok(strptr,special) != NULL) {
      ++count;
      strptr = NULL;
    }

    n_cols = count;

    rewind(fp);
    count = 0;
    while ((rc=fscanf(fp,"%lf",&aux)) == 1) {
      ++count;
    }

    n_rows = count/n_cols;
    printf("found %d columns and %d rows ... ",n_cols,n_rows);

    // allocate memory
    D = my_matrix(1,n_rows,1,n_cols);
    vnames = (char **) my_calloc(n_cols+1,sizeof(char *),MY_STOP);
    units  = (char **) my_calloc(n_cols+1,sizeof(char *),MY_STOP);
    for (j=1; j<=n_cols; ++j) {
      vnames[j] = (char *) my_calloc(20,sizeof(char),MY_STOP);
      units[j]  = (char *) my_calloc(20,sizeof(char),MY_STOP);
      sprintf(vnames[j],"%d",j);
      sprintf(units[j],"-");
    }

    rewind(fp);
    for (j=1; j<=n_rows; ++j)
      for (k=1; k<=n_cols; ++k)
	rc=fscanf(fp,"%lf",&(D[j][k]));

    fclose(fp);

    sprintf(fname_out,"d%s",fname);

    mrdplot_gen(fname_out,D,vnames,units,1.0,n_cols,n_rows);

    for (j=1; j<=n_cols; ++j) {
      free(vnames[j]);
      free(units[j]);
    }
    free(vnames);
    free(units);
    my_free_matrix(D,1,n_cols,1,n_rows);

    printf("output in >%s< ... done.\n",fname);
    
  }
  	
  return TRUE;

}
