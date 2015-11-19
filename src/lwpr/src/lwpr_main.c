/*!=============================================================================
  ==============================================================================

  \file    lwpr_main.c

  \author  Sethu Vijayakumar & Stefan Schaal
  \date    September 1996

  ==============================================================================
  \remarks

      template main() program 

  ============================================================================*/
#include "stdio.h"
#include "stdlib.h"
#include "unistd.h"
#include "time.h"

#define  TRUE  1
#define  FALSE 0

char   **argv_global;
int      argc_global;

void runLWPR();

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
  int   i;
  long  t;
  
  /* copy the input arguments */

  argc_global = argc;
  argv_global = argv;
  
  /* subtract 1 from argv to get rid of the 0-th argument which is the
     function name itself */
  
  argc_global -= 1;
  t = time(NULL);

  
  while (TRUE) {
    runLWPR();
  }
	
}
