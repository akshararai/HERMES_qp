/*!=============================================================================
  ==============================================================================

  \file    lookup_table.c

  \author  Stefan Schaal
  \date    August 1993

  ==============================================================================
  \remarks

      implements various forms of lookup tables

  ============================================================================*/

#include "stdio.h"
#include "math.h"
#include "string.h"

/* private includes */

#include "utility.h"

#define N_TABLES_MAX 10
#define N_CHAR_STRING 100

#define MAX 1
#define MIN 2


typedef struct Lookup_Table {

  char    table_name[N_CHAR_STRING];  
  int     table_ID;
  double **ranges;  /*!< the max and min values of each input dim */
  float **sum_inputs;
  float **sum_results;
  long   *count_points;
  double  *bin_size;
  int     n_dim_in;
  int     n_dim_out;
  int     n_bins;

} Lookup_Table;


Lookup_Table lookup_tables[N_TABLES_MAX+1];


/* functions */


/*!*****************************************************************************
 *******************************************************************************
\note  init_lookup_table
\date  10/20/91 
   
\remarks 

        initializes all necessary things

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     ident      : ident number to be assigned to table
 \param[in]     name       : give the table a name
 \param[in]     n_dim_in   : how many input dimensions
 \param[in]     n_dim_out  : how many output dimensions
 \param[in]     n_bins     : how many bins per input dimension

 ******************************************************************************/
int
init_lookup_table(int ident,char *name, int n_dim_in, int n_dim_out, 
		  int n_bins,double **ranges)

{
  int i, j, n;
  size_t number;
  size_t length;
  long   all_bins;
  

 
  /* get memory for this sucker */

  
  all_bins = pow((double)n_bins,(double)n_dim_in);

   fprintf(stdout,"Initalize: %s %d %d %d %d %ld\n",
	   name,
	   ident,
	   n_dim_in,
	   n_dim_out,
	   n_bins,
	   all_bins);


  lookup_tables[ident].sum_inputs = 
    my_fmatrix(1,all_bins,1,n_dim_in);
  
  lookup_tables[ident].sum_results = 
    my_fmatrix(1,all_bins,1,n_dim_out);

  lookup_tables[ident].count_points = 
    my_livector(1,all_bins);
  

  for (i=1; i <= all_bins; ++i) {

    lookup_tables[ident].count_points[i] = 0;

    for (j=1; j<=n_dim_in; ++j) {
      lookup_tables[ident].sum_inputs[i][j] = 0.0;
    }
    for (j=1; j<=n_dim_out; ++j) {
      lookup_tables[ident].sum_results[i][j] = 0.0;
    }
      
  }


  lookup_tables[ident].ranges = my_matrix(1,n_dim_in,1,2);
   
  lookup_tables[ident].bin_size  = my_vector(1,n_dim_in);


  for (i=1; i<=n_dim_in; ++i) {

    for (j=1; j<=2; ++j) {

      lookup_tables[ident].ranges[i][j] = ranges[i][j];
	
    }

    /* calculate bin sizes */
   
    lookup_tables[ident].bin_size[i] = 
      (ranges[i][MAX] - ranges[i][MIN]) / (double) n_bins;

  }
  

  strcpy(lookup_tables[ident].table_name,name);

  lookup_tables[ident].table_ID = ident;

  
  lookup_tables[ident].n_dim_in  = n_dim_in;
  lookup_tables[ident].n_dim_out = n_dim_out;
  lookup_tables[ident].n_bins    = n_bins;

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  add_to_table
\date  August 7, 1993
   
\remarks 

        add a value to a given table:
	the value is added in the appropriate bin; if the bin is already
	occupied, nothing is done (this could be changed to do be an
	overwrite or an averaging if necessary in future)

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     ident : the table ID of the table
 \param[in]     in    : the input vector
 \param[in]     out   : the output vector
 \param[in]     silent: don't print

 ******************************************************************************/
int
add_to_table(int ident,double *in,double *out,int silent)

{
  
  int j, i;
  long bin=0;
  int  iaux=0;
  double mult;


  /* determine the bin in which this point falls */

  mult = 1.0;
  bin  = 1;


  for (i=1; i<=lookup_tables[ident].n_dim_in; ++i) {

    iaux = (in[i] - lookup_tables[ident].ranges[i][MIN]) /
      lookup_tables[ident].bin_size[i] + 1;

    if (iaux > lookup_tables[ident].n_bins) 
      iaux = lookup_tables[ident].n_bins;

    if (iaux > lookup_tables[ident].n_bins || iaux < 1) {

      if (!silent) {
	printf("Point %d = %f out of table range %f to %f - point not added\n",
	       i,in[i],lookup_tables[ident].ranges[i][MIN],
	       lookup_tables[ident].ranges[i][MAX]);
      }
      
      return FALSE;

    }

    bin  += (iaux-1) * mult;
    mult *= (double)lookup_tables[ident].n_bins;

  }


  lookup_tables[ident].count_points[bin] += 1;

  for (j=1; j<=lookup_tables[ident].n_dim_out; ++j) {

    lookup_tables[ident].sum_results[bin][j] += out[j];

  }

  for (j=1; j<=lookup_tables[ident].n_dim_in; ++j) {

    lookup_tables[ident].sum_inputs[bin][j] += in[j];

  }

  if (!silent) 
    printf("Added values to bin %ld which has now average of %ld values\n",
	 bin,lookup_tables[ident].count_points[bin]);
  
  return TRUE;

}


/*!*****************************************************************************
 *******************************************************************************
\note  lookup_in_table
\date  August 7, 1993
   
\remarks 

        lookup of a value in a table

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     ident : the table ID of the table
 \param[in]     in    : the input vector
 \param[out]    out   : the output vector
 \param[out]    center: the vector characterizing the center of the bin
 \param[in]     interpolate : interpolate or not
 \param[in]     silen : don't print anything

 ******************************************************************************/
int
lookup_in_table(int ident,double *in,double *out,
		double *center, int interpolate,int silent)

{
  
  int j, i,n;
  long bin=0;
  int  iaux=0, irem, iend;
  double weight;
  static int  nn_bins[100];
  static double weights[100];
  static double distance_normalization;
  static double normalized_remainders[30];
  static int    index_bins;
  static double mult,mult1;
  static long   lookup_bin;
  static long   current_bin;
  static double sum_weights;


  /* determine the bin in which this point falls */

  mult = 1.0;
  bin  = 1;

  for (i=1; i<=lookup_tables[ident].n_dim_in; ++i) {

    /* if the point lies at the bound of the lookup table in this dimension
       and its remainder is towards the bounds, I'll set the remainder
       to zero such that the interpolation neglects interpolation in
       this direction */

    if ((in[i] - lookup_tables[ident].ranges[i][MIN]) <
	lookup_tables[ident].bin_size[i]/2.               ||
	lookup_tables[ident].ranges[i][MAX] - in[i]  < 
	lookup_tables[ident].bin_size[i]/2.) {

      normalized_remainders[i] = 0.0;

    } else {

      normalized_remainders[i] = 
	fmod((in[i] - lookup_tables[ident].ranges[i][MIN]) /
	 lookup_tables[ident].bin_size[i],1.0) - 0.5;

    }

    iaux = (in[i] - lookup_tables[ident].ranges[i][MIN]) /
      lookup_tables[ident].bin_size[i] + 1;

    if (iaux == lookup_tables[ident].n_bins+1) 
      iaux = lookup_tables[ident].n_bins;

    if (iaux > lookup_tables[ident].n_bins || iaux < 1) {

      if (!silent) {
	printf("Point out of table range:\n");
	for (j=1; j<=lookup_tables[ident].n_dim_in;++j) {
	  printf("      %d.input = %f\n",j,in[j]);
	}
	printf("\n");
      }
      return FALSE;
      
    }

    center[i] = lookup_tables[ident].ranges[i][MIN] + 
      (iaux-0.5) * lookup_tables[ident].bin_size[i];

    bin  += (iaux-1) * mult;
    mult *= (double)lookup_tables[ident].n_bins;

  }

  lookup_bin = bin;

  if (lookup_tables[ident].count_points[lookup_bin] == 0) {
    
    if(!silent) {
      printf("No data for:\n");
      for (j=1; j<=lookup_tables[ident].n_dim_in;++j) {
	printf("      %d.input = %f\n",j,in[j]);
      }
      printf("\n");
    }
    return FALSE;

  }


  /* interpolate */

  /* first, determine all bins which contribute to interpolation */


  if (interpolate) {
    
    /* the members of the hyper cube can be thought of as
       binary numbers from 0 to 2^n_in_dim-1; the ONEs are 
       treated as a dimension being used, the ZEROs are
       dimensions which are not used */

    index_bins = 0;
    sum_weights = 0;

    iend = 1 << lookup_tables[ident].n_dim_in;
    
    for (i=0; i< iend ; ++i) {
    
      mult        = 1;
      current_bin = lookup_bin;
      iaux        = i;
      weight      = 1;

      for (j=1; j<=lookup_tables[ident].n_dim_in; ++j) {

	irem  = iaux%2;
	iaux /= 2;

	if (irem == 1) {
	  current_bin += macro_sign(normalized_remainders[j])*mult;
	  weight *= fabs(normalized_remainders[j]);
	} else {
	  weight *= (1.-fabs(normalized_remainders[j]));
	}

	mult *= (double)lookup_tables[ident].n_bins;

      }

   
      if (weight != 0.) {

	if (lookup_tables[ident].count_points[current_bin] != 0) {
	  
	  nn_bins[++index_bins] = current_bin;
	  weights[index_bins]   = weight;

	  sum_weights += weight;

	}

      }

    }
    
  } else {

    index_bins = 0;
    nn_bins[++index_bins] = lookup_bin;
    
    weights[index_bins] = 1.0;
    sum_weights = 1.0;

  }

  /* the final interpolation */


  for (j=1; j<=lookup_tables[ident].n_dim_out; ++j) {
    out[j] = 0.0;
  }
  
  for (i=1; i<=index_bins; ++i) {
    for (j=1; j<=lookup_tables[ident].n_dim_out; ++j) {
      out[j] += (lookup_tables[ident].sum_results[nn_bins[i]][j]/
		 (double)lookup_tables[ident].count_points[nn_bins[i]]) 
	* weights[i];
    }
  }

  for (j=1; j<=lookup_tables[ident].n_dim_out; ++j) {
    out[j] /= sum_weights;
  }

  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
\note  write_lookup_table_ascii
\date  August 7, 1993
   
\remarks 

        dumps a lookup table to an ascii file with the table name

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     ident : the table ID of the table

 ******************************************************************************/
int
write_lookup_table_ascii(int ident)


{

  FILE *out;
  Lookup_Table  *t;
  int i,j;
  long   all_bins;
  static char   string[100];
  
  t = &(lookup_tables[ident]);
  
  all_bins = pow((double)t->n_bins,(double)t->n_dim_in);
  
  sprintf(string,"%s.ascii",t->table_name);
  out = fopen(string,"w");

  if (out == NULL) return FALSE;
  
  fprintf(out,"%s %d %d %d %d\n",
	  t->table_name,
	  t->table_ID,
	  t->n_dim_in,
	  t->n_dim_out,
	  t->n_bins);

  
  for (i=1; i<=t->n_dim_in; ++i) {
    
    fprintf(out,"%f %f %f\n",t->bin_size[i],
	    t->ranges[i][MIN],t->ranges[i][MAX]);
    
  }
  
  for ( i=1; i<=all_bins; ++i) {
    
    fprintf(out,"%ld    ",t->count_points[i]);
      
      for (j=1; j<=t->n_dim_in; ++j) {
	fprintf(out,"%f ",t->sum_inputs[i][j]);
      }
      for (j=1; j<=t->n_dim_out; ++j) {
	fprintf(out,"%f ",t->sum_results[i][j]);
      }

    fprintf(out,"\n");
    
  }
  
  fclose(out);

  
  return TRUE;
    
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  write_lookup_table
\date  August 7, 1993
   
\remarks 

        dumps a lookup table to a biniary format file with the table name

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     ident : the table ID of the table

 ******************************************************************************/
int
write_lookup_table(int ident)


{

  FILE *out;
  Lookup_Table  *t;
  int i,j;
  long   all_bins;
  
  t = &(lookup_tables[ident]);
  
  all_bins = pow((double)t->n_bins,(double)t->n_dim_in);
  

  out = fopen(t->table_name,"w");

  if (out == NULL) return FALSE;
  
  if (fwrite(t->table_name,sizeof(char),N_CHAR_STRING,out)!= N_CHAR_STRING) {
    printf( "cannot fwrite file.\n" );
    return FALSE;
  }
  if (fwrite(&(t->table_ID),sizeof(int),1,out)!= 1) {
    printf( "cannot fwrite file.\n" );
    return FALSE;
  }
  if (fwrite(&(t->n_dim_in),sizeof(int),1,out)!= 1) {
    printf( "cannot fwrite file.\n" );
    return FALSE;
  }
  if (fwrite(&(t->n_dim_out),sizeof(int),1,out)!= 1) {
    printf( "cannot fwrite file.\n" );
    return FALSE;
  }
  if (fwrite(&(t->n_bins),sizeof(int),1,out)!= 1) {
    printf( "cannot fwrite file.\n" );
    return FALSE;
  }

  if (fwrite(&(t->ranges[1][1]),sizeof(double),t->n_dim_in*2,out)!= 
      t->n_dim_in*2) {
    printf( "cannot fwrite file.\n" );
    return FALSE;
  }

  if (fwrite(&(t->bin_size[1]),sizeof(double),t->n_dim_in,out)!= 
      t->n_dim_in) {
    printf( "cannot fwrite file.\n" );
    return FALSE;
  }

  if (fwrite(&(t->count_points[1]),sizeof(long),all_bins,out)!= all_bins) {
    printf( "cannot fwrite file.\n" );
    return FALSE;
  }

  if (fwrite(&(t->sum_inputs[1][1]),sizeof(float),all_bins*t->n_dim_in,out)!= 
      all_bins*t->n_dim_in) {
    printf( "cannot fwrite file.\n" );
    return FALSE;
  }

  if (fwrite(&(t->sum_results[1][1]),sizeof(float),
	     all_bins*t->n_dim_out,out)!=all_bins*t->n_dim_out) {
    printf( "cannot fwrite file.\n" );
    return FALSE;
  }

  
  fclose(out);

  
  return TRUE;
    
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  read_lookup_table
\date  August 7, 1993
   
\remarks 

        reads a lookup table from an ascii file with the table name

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     ident : ident number to be assigned
 \param[in]     fname : the name of the file containing the table

 ******************************************************************************/
int
read_lookup_table(int ident,char *fname)

{

  FILE *in;
  Lookup_Table  t;
  int i,j;
  static double bin_size[100];
  double **ranges;
  long   all_bins;
  


  ranges = my_matrix(1,100,1,2);


  in = fopen(fname,"r");

  if (in == NULL) return FALSE;
  
  if (fread(t.table_name,sizeof(char),N_CHAR_STRING,in)!= N_CHAR_STRING) {
    printf( "cannot fread file.\n" );
    return FALSE;
  }
  if (fread(&(t.table_ID),sizeof(int),1,in)!= 1) {
    printf( "cannot fread file.\n" );
    return FALSE;
  }
  if (fread(&(t.n_dim_in),sizeof(int),1,in)!= 1) {
    printf( "cannot fread file.\n" );
    return FALSE;
  }
  if (fread(&(t.n_dim_out),sizeof(int),1,in)!= 1) {
    printf( "cannot fread file.\n" );
    return FALSE;
  }
  if (fread(&(t.n_bins),sizeof(int),1,in)!= 1) {
    printf( "cannot fread file.\n" );
    return FALSE;
  }

  if (fread(&(ranges[1][1]),sizeof(double),t.n_dim_in*2,in)!= t.n_dim_in*2) {
    printf( "cannot fread file.\n" );
    return FALSE;
  }


 /* create the table */

  
  init_lookup_table(ident,t.table_name,t.n_dim_in, t.n_dim_out, 
		    t.n_bins, ranges);
  

  all_bins = pow((double)t.n_bins,(double)t.n_dim_in);
  
  
  /* read the table data */

  if (fread(&(lookup_tables[ident].bin_size[1]),sizeof(double),t.n_dim_in,in)!= 
      t.n_dim_in) {
    printf( "cannot fread file.\n" );
    return FALSE;
  }

  if (fread(&(lookup_tables[ident].count_points[1]),sizeof(long),all_bins,in)
      != all_bins) {
    printf( "cannot fread file.\n" );
    return FALSE;
  }

  if (fread(&(lookup_tables[ident].sum_inputs[1][1]),sizeof(float),
	    all_bins*t.n_dim_in,in)!= all_bins*t.n_dim_in) {
    printf( "cannot fread file.\n" );
    return FALSE;
  }

  if (fread(&(lookup_tables[ident].sum_results[1][1]),sizeof(float),
	    all_bins*t.n_dim_out,in)!= all_bins*t.n_dim_out) {
    printf( "cannot fread file.\n" );
    return FALSE;
  }


  fclose(in);

  my_free_matrix(ranges,1,100,1,2);

  return TRUE;
    
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  read_lookup_table_ascii
\date  August 7, 1993
   
\remarks 

        reads a lookup table from an ascii file with the table name

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     fname : the name of the file containing the table

 ******************************************************************************/
int
read_lookup_table_ascii(int ident,char *fname)

{

  FILE *in;
  Lookup_Table  t;
  int i,j;
  int rc;
  static double bin_size[100];
  static double **ranges;
  static long   all_bins;
  static char   string[100];
  

  ranges = my_matrix(1,100,1,2);

  sprintf(string,"%s.ascii",fname);
  in = fopen(string,"r");

  if (in == NULL) return FALSE;
  
  rc = fscanf(in,"%s %d %d %d %d\n",
	  t.table_name,
	  &t.table_ID,
	  &t.n_dim_in,
	  &t.n_dim_out,
	  &t.n_bins);



  for (i=1; i<=t.n_dim_in; ++i) {
    
    rc = fscanf(in,"%lf %lf %lf\n",&bin_size[i],
	   &ranges[i][MIN],&ranges[i][MAX]);
    
  }
    
  /* create the table */

  init_lookup_table(ident,t.table_name, t.n_dim_in, 
		      t.n_dim_out, t.n_bins, ranges);
  

  all_bins = pow((double)t.n_bins,(double)t.n_dim_in);
  

  for ( i=1; i<=all_bins; ++i) {
    
    rc = fscanf(in,"%ld    ",&lookup_tables[ident].count_points[i]);
    
    for (j=1; j<=t.n_dim_in; ++j) {
      rc = fscanf(in,"%f",&lookup_tables[ident].sum_inputs[i][j]);
    }
    for (j=1; j<=t.n_dim_out; ++j) {
      rc = fscanf(in,"%f",&lookup_tables[ident].sum_results[i][j]);
    }
    
  }
  
  
  fclose(in);

  my_free_matrix(ranges,1,100,1,2);

  return TRUE;
    
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  get_mean_and_center_bin
\date  August 7, 1993
   
\remarks 

        returns the input mean value and the input center value of the bin
	which belongs to the given input value

	returns FALSE if bin has no data or out of table range

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     ident : the table ID of the table
 \param[in]     in    : the input vector
 \param[out]    mean  : the mean input
 \param[out]    center: the center value
 \param[in]     silent: don't print anything

 ******************************************************************************/
int
get_mean_and_center_bin(int ident,double *in,double *mean,
			double *center, int silent)

{
  
  int j, i,n;
  long bin=0;
  double mult;
  long   lookup_bin;
  long   iaux;


  /* determine the bin in which this point falls */

  mult = 1.0;
  bin  = 1;

  for (i=1; i<=lookup_tables[ident].n_dim_in; ++i) {

    iaux = (in[i] - lookup_tables[ident].ranges[i][MIN]) /
      lookup_tables[ident].bin_size[i] + 1;

    if (iaux == lookup_tables[ident].n_bins+1) 
      iaux = lookup_tables[ident].n_bins;

    if (iaux > lookup_tables[ident].n_bins || iaux < 1) {

      if (!silent) {
	printf("Point out of table range:\n");
	for (j=1; j<=lookup_tables[ident].n_dim_in;++j) {
	  printf("      %d.input = %f\n",j,in[j]);
	}
	printf("\n");
      }
      return FALSE;
      
    }

    center[i] = lookup_tables[ident].ranges[i][MIN] + 
      (iaux-0.5) * lookup_tables[ident].bin_size[i];

    bin  += (iaux-1) * mult;
    mult *= (double)lookup_tables[ident].n_bins;

  }

  lookup_bin = bin;

  if (lookup_tables[ident].count_points[lookup_bin] == 0) {
    
    if(!silent) {
      printf("No data for:\n");
      for (j=1; j<=lookup_tables[ident].n_dim_in;++j) {
	printf("      %d.input = %f\n",j,in[j]);
      }
      printf("\n");
    }
    return FALSE;

  }

  
  for (j=1; j<=lookup_tables[ident].n_dim_in; ++j) {
    mean[j] = (lookup_tables[ident].sum_inputs[lookup_bin][j]/
	       (double)lookup_tables[ident].count_points[lookup_bin]);
  }

  return TRUE;
}

