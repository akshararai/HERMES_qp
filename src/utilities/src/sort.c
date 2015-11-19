#include "stdio.h"
#include "utility.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"

#define  N_VARS  40

/* current (April 92) output of data:

		1. side
		2. time
		3. file name
		4. hit number
		5. stick ad (real)
		6. stick yd (real)
		7. stick xd (real)
		8. stick a  (real)
		9. stick y  (real)
		10.stick x  (real)
		11.stick p  (real)
		12.stick ad (nominal)
		13.stick yd (nominal)
		14.stick xd (nominal)
		15.stick a  (nominal)
		16.stick y  (nominal)
		17.stick x  (nominal)
		18.stick p  (nominal)
		19.command yd
		20.command xd
		21.desired p 
		22.command y (real position)
		23.command x (real position)
		24.trigger velocity
		25.wait time
		26.stick result ad (real)
		27.stick result yd (real)
		28.stick result xd (real)
		29.stick result a  (real)
		30.stick result y  (real)
		31.stick result x  (real)
		32.stick result p  (real)
		33.stick result ad (nominal)
		34.stick result yd (nominal)
		35.stick result xd (nominal)
		36.stick result a  (nominal)
		37.stick result y  (nominal)
		38.stick result x (nominal)
		39.stick result p  (nominal)
		40.valid
		
*/
 


extern int get_string (char *comment, char *defaultvalue, char *value);
extern int get_double (char *comment, double defaultvalue, double *value);

main(void) {


FILE    *in,*out;
int     i,rc,which_side=0;
char    name[100];
double  v[100],aux;
int     count=0;
int	index;
int     v_out[100] ;

v_out[ 0] = 18;
v_out[ 1] = 15;
v_out[ 2] = 14;
v_out[ 3] = 13;
v_out[ 4] = 12;
v_out[ 5] = 39;
v_out[ 6] = 36;
v_out[ 7] = 35;
v_out[ 8] = 34;
v_out[ 9] = 33;
v_out[10] = 20;
v_out[11] = 19;
v_out[12] = 23;
v_out[13] = 22;
v_out[14] = 24;
v_out[15] = 999;



get_string("Name of Input File\0","c02735.hits\0",name);

if ((in = fopen(name,"r")) == NULL) {
	printf("File >%s< not found\n",name);
	exit (-1);
}

get_string("Name of Output File\0","temp.data\0",name);

if ((out = fopen(name,"w")) == NULL) {
	printf("File open error for >%s<\n",name);
	exit (-1);
}

get_double("Extract which side? LEFT=0 RIGHT=1",(double)which_side,&aux);
which_side = aux;


name[0] = ' ';  /* reset first element of name */
index = 0;


while ((rc=fgetc(in)) != EOF) {
	
	if (rc == '\n' || rc == ' ' || rc == '\t') {
				
		name[index+1] = '\0'; /* just a hack to avoid stuff behind index */
		name[index+2] = 'm';
		name[index+3] = '\0';
		
		if (++count > N_VARS) {
		  if (v[1] == which_side) {
		    for(i=0; v_out[i] != 999; ++i) {
		      fprintf(out,"%f ",v[v_out[i]]);
		    }
		    fprintf(out,"\n");
		  }
		  count = 1;
		}
		
		if (sscanf(name,"%lf",&v[count]) == 1)	{
			/* printf("%f \n",v[count]); */
		} else {
			--count;
		}
		index = 0;
	
	} else {
	
		++index;
		name[index] = rc;
		name[index+1] = '\0';
		
	}
	
}

fclose(in);
fclose(out);

}


