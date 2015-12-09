#include <native/heap.h>

int main (int argc, char *argv[])

{

  RT_HEAP heap_desc;

  int err, i;
  
  /* Bind to a shared heap which has been created elsewhere, either
     in kernel or user-space. Here we cannot wait and the heap must
     be available at once, since the caller is not a Xenomai-enabled
     thread. The heap should have been created with the H_SHARED
     mode set. */

  for(i=1; i<argc; i++)
    {
      err = rt_heap_bind(&heap_desc, argv[i], TM_NONBLOCK);
      if(err)
	{
	 printf("error %d in binding to memory named %s\n",err,argv[i]); 
	}
      else
	{
	  err = rt_heap_delete(&heap_desc);
	  if(err)
	    {
	      printf("error %d in deleting the memory named %s\n", err, argv[i]);
	    }
	  else
	    {
	      printf("deleted memory %s\n", argv[i]);
	    }
	}
    }
  
  return 1;
}
