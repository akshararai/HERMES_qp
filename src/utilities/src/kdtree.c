/*!=============================================================================
  ==============================================================================

  \file    kdtree.c

  \author  Stefan Schaal
  \date    August 1993

  ==============================================================================
  \remarks
  
  implements kdtree structuring of data
  
  ============================================================================*/

#include "stdio.h"
#include "math.h"
#include "string.h"

/* private includes */

#include "utility.h"

#define N_KDTREES_MAX 10
#define N_CHAR_STRING 100

#define MAX 1
#define MIN 2

#define MAX_ELEMENTS 2000
#define CHUNK_SIZE   10


typedef struct KDLeaf {
  
  int      n_points;                /*!< how many points in leaf */
  int      n_allocated_points;      /*!< for how many points do we have memory */
  double **inputs;                  /*!< matrix of inputs */
  double **outputs;                 /*!< matrix of associated outputs */
  double  *sum_inputs;              /*!< sum of inputs for means */
  double  *sum_outputs;             /*!< sum of outputs for means */
  double  *max;                     /*!< max values in partition */
  double  *min;                     /*!< min values in partition */
  int     *max_min_vertices;        /*!< index to max and min vertices */
  int     *neighbor_vertices;       /*!< indices of all neighbor vertices */
  int      n_neighbor_vertices;
  int      n_allocated_neighbor_vertices;
  double  *median;
  double  *next_lower_median;
  double  *next_greater_median;
  int      n_elements_extra;        /*!< this allows to add an extra vector
				       to each leaf which can contain 
				       whatever you want */
  double  *extra;
  int      prediction_status;       /*!< if set to FALSE, partition will
				       be disabled for predictions */

} KDLeaf;


typedef struct KDNode {
  
  double  *mean_inputs;
  double  *mean_outputs;
  double  *median;
  double  *middle;
  int      split_dim;  /*!< this is the dimension perpendicularly 
			  to which the split is */
  int      next_larger;
  int      next_smaller;
  int      is_leaf;
  int      leaf_index;
  
} KDNode;


typedef struct KDTree {
  
  int      ident;
  int      split_type;
  char     kdtree_name[N_CHAR_STRING];  
  int      kdtree_ID;
  KDLeaf  *leaves;
  KDNode  *nodes;
  int      n_leaves;
  int      n_nodes;
  int      n_dim_in;
  int      n_dim_out;
  double  *max_inputs;
  double  *min_inputs;
  double **all_vertices;     /*!< since vertices are shared,
				I list all of them here and the
				leaves just point to these data
				points */
  double **all_vertices_output; /*!< each vertex can associate an output */
  int     *all_vertices_output_status; /*!< indicates whether output exists */
  int     *n_elements_all_vertices_extra; /*!< how many elements per extra? */
  double **all_vertices_extra;  /*!< each vertex can associate an extra vector */
  int      n_all_vertices;
  int      n_allocated_all_vertices;
  
} KDTree;


KDTree kdtrees[N_KDTREES_MAX+1];

/* functions */

static int init_kdtree_leaf(int ident, KDLeaf *lptr, int n_points, int zero);
static int init_kdtree_node(int ident, KDNode *nptr, int zero);
static int free_kdtree_leaf(int ident, KDLeaf *lptr);
static int free_kdtree_node(int ident, KDNode *nptr);
static int find_partition(int ident,double *in, int *leafID, KDLeaf **lptr, 
			  KDNode **nptr);
static int add_vertex_to_database(int ident, double *vector);
static int generate_max_min_vertices(int ident, KDLeaf *lptr);
static int add_neighbor_vertex_by_index(int ident,KDLeaf *lptr, int ind);
static int add_neighbor_vertex(int ident,KDLeaf *lptr,double *vert, int *ind);


/*!*****************************************************************************
 *******************************************************************************
 \note  init_kdtree
 \date  10/20/91 
 
 \remarks 
 
 initializes all necessary things
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ident      : ident number to be assigned to table
 \param[in]     name       : give the kdtree a name
 \param[in]     n_dim_in   : how many input dimensions
 \param[in]     n_dim_out  : how many output dimensions
 \param[in]     split_type : SPLIT_AT_MEDIAN, SPLIT_AT_MEAN, or SPLIT_AT_MIDDLE
 \param[in]     ranges     : the max input dimensions of the kdtree
 
 ******************************************************************************/
int
  init_kdtree(int ident,char *name, int n_dim_in, int n_dim_out, 
	      int split_type, double **ranges)

{
  int i, j, n;
  size_t number;
  size_t length;
  
  
  /* get memory for this sucker */
  
  fprintf(stdout,"Initalize: %s ID=%d n_in=%d n_out=%d\n",
	  name,
	  ident,
	  n_dim_in,
	  n_dim_out);
  
  strcpy(kdtrees[ident].kdtree_name,name);
  
  kdtrees[ident].ident = ident;
  kdtrees[ident].split_type = split_type;
  
  number = MAX_ELEMENTS+1;
  length = sizeof(KDLeaf);
  kdtrees[ident].leaves = (KDLeaf *) my_calloc(number,length,MY_STOP);
  
  number = MAX_ELEMENTS+1;
  length = sizeof(KDNode);
  kdtrees[ident].nodes = (KDNode *) my_calloc(number,length,MY_STOP);
  
  kdtrees[ident].max_inputs  = my_vector(1,n_dim_in);
  kdtrees[ident].min_inputs  = my_vector(1,n_dim_in);
  
  kdtrees[ident].all_vertices =my_matrix(1,CHUNK_SIZE,1,n_dim_in);
  kdtrees[ident].all_vertices_output =my_matrix(1,CHUNK_SIZE,1,n_dim_out);
  kdtrees[ident].all_vertices_output_status =my_ivector(1,CHUNK_SIZE);
  for (i=1; i<=CHUNK_SIZE; ++i) 
    kdtrees[ident].all_vertices_output_status[i]=0;

  number = CHUNK_SIZE+1;
  length = sizeof(double *);
  kdtrees[ident].all_vertices_extra = 
    (double **) my_calloc(number,length,MY_STOP);

  kdtrees[ident].n_elements_all_vertices_extra = my_ivector(1,CHUNK_SIZE);
  for (i=1; i<=CHUNK_SIZE; ++i) 
    kdtrees[ident].n_elements_all_vertices_extra[i]=0;
  kdtrees[ident].n_allocated_all_vertices  = CHUNK_SIZE;
  kdtrees[ident].n_all_vertices  = 0;
 
  for (i=1; i<=n_dim_in; ++i) {
    kdtrees[ident].max_inputs[i] = ranges[i][MAX];
    kdtrees[ident].min_inputs[i] = ranges[i][MIN];
  }

  kdtrees[ident].n_leaves = 1;
  kdtrees[ident].n_nodes  = 1;
  
  kdtrees[ident].n_dim_in  = n_dim_in;
  kdtrees[ident].n_dim_out = n_dim_out;
  
  init_kdtree_node(ident,&(kdtrees[ident].nodes[1]),TRUE);
  kdtrees[ident].nodes[1].is_leaf      = TRUE;
  kdtrees[ident].nodes[1].leaf_index   = 1;
  
  init_kdtree_leaf(ident,&(kdtrees[ident].leaves[1]),CHUNK_SIZE,TRUE);
  generate_max_min_vertices(ident,&(kdtrees[ident].leaves[1]));


  for (i=1; i<=n_dim_in; ++i) {
    kdtrees[ident].nodes[1].middle[i] = 
      (kdtrees[ident].leaves[1].max[i] + kdtrees[ident].leaves[1].min[i])/2.;
  }
  
  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
 \note  init_kdtree_leaf
 \date  August 7, 1993
 
 \remarks 
 
 initializes all memory structure for a given leaf pointer
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ident   : the table ID of the kdtree
 \param[in]     lptr    : pointer to the leaf to be initialized
 \param[in]     n_points: the data matrices are allocated for n_points data points
 \param[in]     zero    : set values to default values if TRUE
 
 ******************************************************************************/
static int
  init_kdtree_leaf(int ident, KDLeaf *lptr, int n_points, int zero)

{
  
  KDTree *kd;
  int     i;
  int     n_allocated_points;
  
  kd = &(kdtrees[ident]);

  n_allocated_points = n_points;
  
  lptr->n_allocated_points   = n_allocated_points;
  lptr->sum_inputs           = my_vector(1,kd->n_dim_in);
  lptr->sum_outputs          = my_vector(1,kd->n_dim_out);
  lptr->inputs               = my_matrix(1,n_allocated_points,1,kd->n_dim_in);
  lptr->outputs              = my_matrix(1,n_allocated_points,1,kd->n_dim_out);
  lptr->max                  = my_vector(1,kd->n_dim_in);
  lptr->min                  = my_vector(1,kd->n_dim_in);
  lptr->max_min_vertices     = my_ivector(1, (1 << kd->n_dim_in));
  lptr->median               = my_vector(1,kd->n_dim_in);
  lptr->next_lower_median    = my_vector(1,kd->n_dim_in);
  lptr->next_greater_median  = my_vector(1,kd->n_dim_in);
  lptr->neighbor_vertices    = my_ivector(1,CHUNK_SIZE);
  
  if (zero) {
    
    lptr->n_points             = 0;
    lptr->n_neighbor_vertices  = 0;
    lptr->n_elements_extra     = 0;
    lptr->prediction_status    = FALSE;
    lptr->n_allocated_neighbor_vertices  = CHUNK_SIZE;
    
    for (i=1; i<=kd->n_dim_in; ++i) {
      lptr->sum_inputs[i] = 0.0;
      lptr->max[i] = kd->max_inputs[i];
      lptr->min[i] = kd->min_inputs[i];
    }
    
    for (i=1; i<=kd->n_dim_out; ++i) {
      lptr->sum_outputs[i] = 0.0;
    }

  }
  
  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
 \note  free_kdtree_leaf
 \date  August 7, 1993
 
 \remarks 
 
 free the data arrays of the given leaf pointer
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ident   : the table ID of the kdtree
 \param[in]     lptr    : pointer to the leaf to be freed

 
 ******************************************************************************/
static int
  free_kdtree_leaf(int ident, KDLeaf *lptr)

{
  
  KDTree *kd;
  int     i;
  
  kd = &(kdtrees[ident]);
  
  my_free_vector(lptr->sum_inputs,1,kd->n_dim_in);
  my_free_vector(lptr->sum_outputs,1,kd->n_dim_out);
  my_free_matrix(lptr->inputs,1,lptr->n_allocated_points,1,kd->n_dim_in);
  my_free_matrix(lptr->outputs,1,lptr->n_allocated_points,1,kd->n_dim_out);
  my_free_vector(lptr->max,1,kd->n_dim_in);
  my_free_vector(lptr->min,1,kd->n_dim_in);
  my_free_ivector(lptr->max_min_vertices, 1, (1 << kd->n_dim_in));
  my_free_vector(lptr->median,1,kd->n_dim_in);
  my_free_vector(lptr->next_lower_median,1,kd->n_dim_in);
  my_free_vector(lptr->next_greater_median,1,kd->n_dim_in);
  my_free_ivector(lptr->neighbor_vertices,1,
		  lptr->n_allocated_neighbor_vertices);
  
  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
 \note  init_kdtree_node
 \date  August 7, 1993
 
 \remarks 
 
 initializes all memory structure for a given node pointer
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ident   : the table ID of the kdtree
 \param[in]     nptr    : pointer to the node to be initialized
 \param[in]     zero    : set values to default values if TRUE
 
 
 ******************************************************************************/
static int
  init_kdtree_node(int ident, KDNode *nptr, int zero)

{
  
  KDTree *kd;
  
  kd = &(kdtrees[ident]);
  
  nptr->mean_inputs  = my_vector(1,kd->n_dim_in);
  nptr->mean_outputs = my_vector(1,kd->n_dim_out);
  nptr->median       = my_vector(1,kd->n_dim_in);
  nptr->middle       = my_vector(1,kd->n_dim_in);
  
  if (zero) {
    
    nptr->is_leaf      = TRUE;
    nptr->leaf_index   = -999; /* must be changed by calling program */
    
  }
  
  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
 \note  free_kdtree_node
 \date  August 7, 1993
 
 \remarks 
 
 frees the data arrays of the given node poiter
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ident   : the table ID of the kdtree
 \param[in]     nptr    : pointer to the node to be initialized
 
 ******************************************************************************/
static int
  free_kdtree_node(int ident, KDNode *nptr)

{
  
  KDTree *kd;
  
  kd = &(kdtrees[ident]);
  
  my_free_vector(nptr->mean_inputs,1,kd->n_dim_in);
  my_free_vector(nptr->mean_outputs,1,kd->n_dim_out);
  my_free_vector(nptr->median,1,kd->n_dim_in);
  my_free_vector(nptr->middle,1,kd->n_dim_in);
  
  
  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
 \note  add_to_kdtree
 \date  August 7, 1993
 
 \remarks 
 
 add a value to a given kdtree:
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ident   : the table ID of the kdtree
 \param[in]     in      : the input vector
 \param[in]     out     : the output vector
 \param[out]    part_ID : the ID number of the node to whose leaf data was added
 \param[out]    n_pts   : the number of points in this partion after point was added

 returns TRUE or FALSE if added or not
 
 ******************************************************************************/
int
  add_to_kdtree(int ident,double *in,double *out,int *part_ID, int *n_pts)

{
  
  int j, i;
  KDLeaf *leaf;
  KDNode *node;
  KDTree *kd;
  double **ptr_in;
  double **ptr_out;
  double aux1,aux2,aux3;
  double distance;
  
  kd = &kdtrees[ident];
  
  /* find the partion in which the point is to be added */
  
  if (!find_partition(ident,in, part_ID, &leaf, &node)) return FALSE;
 
  
  /* add this data point */
  
  if (++leaf->n_points > leaf->n_allocated_points) {
    
    ptr_in  = my_matrix(1,leaf->n_allocated_points+CHUNK_SIZE,1,kd->n_dim_in);
    ptr_out = my_matrix(1,leaf->n_allocated_points+CHUNK_SIZE,1,kd->n_dim_out);
    
    /* copy the old data */
    
    memcpy(&(ptr_in[1][1]),&(leaf->inputs[1][1]),
	   (size_t)((leaf->n_allocated_points)*(kd->n_dim_in)*sizeof(double)));
    memcpy(&(ptr_out[1][1]),&(leaf->outputs[1][1]),
	   (size_t)((leaf->n_allocated_points)*(kd->n_dim_out)*sizeof(double)));
    
    my_free_matrix(leaf->inputs,1,leaf->n_allocated_points,1,kd->n_dim_in);
    my_free_matrix(leaf->outputs,1,leaf->n_allocated_points,1,kd->n_dim_out);
    
    leaf->inputs  = ptr_in;
    leaf->outputs = ptr_out;
    
    leaf->n_allocated_points += CHUNK_SIZE;
    
  }
  
  for (i=1; i<=kd->n_dim_in; ++i) {
    leaf->inputs[leaf->n_points][i] = in[i];
    leaf->sum_inputs[i] += in[i];
    node->mean_inputs[i] = leaf->sum_inputs[i]/(double)leaf->n_points;
  }
  
  for (i=1; i<=kd->n_dim_out; ++i) {
    leaf->outputs[leaf->n_points][i] = out[i];
    leaf->sum_outputs[i] += out[i];
    node->mean_outputs[i] = leaf->sum_outputs[i]/(double)leaf->n_points;
  }
  
  /* update the median */

  if (leaf->n_points == 1) {
    
    for (i=1; i<=kd->n_dim_in; ++i) {
      leaf->median[i] = in[i];
      leaf->next_lower_median[i] = in[i];
      leaf->next_greater_median[i] = in[i];
      node->median[i] = in[i];
    }

  } else if (leaf->n_points == 2) {
    
    for (i=1; i<=kd->n_dim_in; ++i) {
      leaf->median[i] = node->mean_inputs[i];
      if (in[i] > leaf->median[i]) {
	leaf->next_greater_median[i] = in[i];
	leaf->next_lower_median[i] = leaf->inputs[1][i];
      } else {
	leaf->next_greater_median[i] = leaf->inputs[1][i];
	leaf->next_lower_median[i] = in[i];
      }
      node->median[i] = leaf->median[i];
    }

  } else {

    for (i=1; i<=kd->n_dim_in; ++i) {
    
      if (in[i] > leaf->median[i]) {

	aux1 = in[i] - leaf->median[i];
	aux2 = leaf->next_greater_median[i] - leaf->median[i];

	if (aux1 > aux2) {

	  if (leaf->n_points%2 == 0) {

	    leaf->median[i]=(leaf->next_greater_median[i]+leaf->median[i])/2.;

	    /* leaf->next_greater_median[i] stays the same,
	     and the old median becomes the lower bound */

	    leaf->next_lower_median[i] = node->median[i];

	  } else {

	    leaf->median[i] = leaf->next_greater_median[i];
	    /* leaf->next_lower_median[i] remains the same;
	       but the next_greater_median has to be searched for */
	    
	    distance = 1.e30;

	    for (j=1; j<=leaf->n_points; ++j) {
	      aux3 = leaf->inputs[j][i] - leaf->median[i];
	      if (aux3 < distance && aux3 > 0) {
		distance = aux3;
		leaf->next_greater_median[i] = leaf->inputs[j][i];
	      }
	    }

	  }

	} else {  /* i.e., if (aux1 < aux2) */

	  if (leaf->n_points%2 == 0) {

	    leaf->median[i]=(in[i] + leaf->median[i]) / 2.;
	    leaf->next_greater_median[i] = in[i];
	    leaf->next_lower_median[i] = node->median[i];

	  } else {

	    leaf->median[i] = in[i];
	    /* leaf->next_lower_median[i] and greater remain the same */
	    
	  }

	}

      } else { /* i.e, if (in[i] < leaf->median[i]) */

	aux1 = leaf->median[i] - in[i];
	aux2 = leaf->median[i] - leaf->next_lower_median[i];

	if (aux1 > aux2) {

	  if (leaf->n_points%2 == 0) {

	    leaf->median[i]=(leaf->next_lower_median[i]+leaf->median[i])/2.;

	    /* leaf->next_lower_median[i] stays the same,
	     and the old median becomes the greater bound */

	    leaf->next_greater_median[i] = node->median[i];

	  } else {

	    leaf->median[i] = leaf->next_lower_median[i];
	    /* leaf->next_greater_median[i] remains the same;
	       but the next_lower_median has to be searched for */
	    
	    distance = 1.e30;

	    for (j=1; j<=leaf->n_points; ++j) {
	      aux3 = leaf->median[i] - leaf->inputs[j][i];
	      if (aux3 < distance && aux3 > 0) {
		distance = aux3;
		leaf->next_lower_median[i] = leaf->inputs[j][i];
	      }
	    }

	  }

	} else {  /* i.e., if (aux1 < aux2) */

	  if (leaf->n_points%2 == 0) {

	    leaf->median[i]=(in[i] + leaf->median[i]) / 2.;
	    leaf->next_lower_median[i] = in[i];
	    leaf->next_greater_median[i] = node->median[i];

	  } else {

	    leaf->median[i] = in[i];
	    /* leaf->next_lower_median[i] and greater remain the same */
	    
	  }

	}

      }
      
      node->median[i] = leaf->median[i];

    }  /* end for (i... loop */

  } /* if n_points > 2) */



  *n_pts = leaf->n_points;
  
  return TRUE;
  
}


/*!*****************************************************************************
 *******************************************************************************
 \note  split_kdtree_partition
 \date  August 7, 1993
 
 \remarks 
 
 splits the given partition perpendicular to the given dimension
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ident   : the kdtree ID
 \param[in]     part_ID : the ID of the partition to be split
 \param[in]     split   : the split dimension
 \param[out]    larger  : the new leaf containing data above the split criterion
 \param[out]    smaller : the new leaf containing data below the split criterion
 
 ******************************************************************************/
int
  split_kdtree_partition(int ident,int part_ID,int split,int *larger, int *smaller)

{
  KDTree *kd;
  KDLeaf *leaf;
  KDNode *node;
  int     leafID;
  
  KDLeaf  *larger_leaf;
  KDLeaf  *smaller_leaf;
  KDNode  *larger_node;
  KDNode  *smaller_node;

  KDLeaf   store_leaf;
  
  int      store_leaf_index;
  double   split_criterion;
  
  int      pID;
  int      npts;
  
  int      i,m,j,n;
  int      iaux, irem;
  double  *vector1,*vector2;

  long     neighbor_index;
  int      add_by_index_flag;
  
  /* first, check whether partition has more than zero data points.
     NOTE: mean and median splitting are data driven and need a mean
           or a median to identify a split. Thus I cannot split
	   an empty partition, since I would not know where to
	   split it. For middle splitting, this is not true and
	   the algorithm can proceed */
  
  kd   = &kdtrees[ident];
  node = &(kd->nodes[part_ID]);
  leaf = &(kd->leaves[node->leaf_index]);
  store_leaf_index = node->leaf_index;
  
  if (leaf->n_points < 2 && kd->split_type != SPLIT_AT_MIDDLE) {
    printf("Number of Points in Partition %d = %d -- too few for splitting\n",
	   part_ID,leaf->n_points);
    return FALSE;
  }
  
  /* store the pointers to the data matrices of the leaf data */
  
  store_leaf = *leaf;


  /* what is the split criterion ? */

  switch (kd->split_type) {
  case SPLIT_AT_MEDIAN:
    split_criterion = node->median[split];
    break;
  case SPLIT_AT_MEAN:
    split_criterion = node->mean_inputs[split];
    break;
  case SPLIT_AT_MIDDLE:
    split_criterion = node->middle[split];
    break;
  default:
    split_criterion = node->median[split];
    break;
  }
  
  /* initialize two new nodes and two new leaves */
  
  node->split_dim = split;
  node->is_leaf   = FALSE;
  node->leaf_index = -999;

  if (kd->n_nodes+2 > MAX_ELEMENTS) {
    printf("ran out of memory for nodes\n");
    getchar();
    exit(-1);
  }
  
  larger_node = &(kd->nodes[++kd->n_nodes]);
  node->next_larger = kd->n_nodes;
  init_kdtree_node(ident,larger_node,TRUE);
  *larger = node->next_larger;
  
  smaller_node = &(kd->nodes[++kd->n_nodes]);
  node->next_smaller = kd->n_nodes;
  init_kdtree_node(ident,smaller_node,TRUE);
  *smaller = node->next_smaller;
  
  larger_leaf = leaf;  /* recycle the old leaf */
  larger_node->leaf_index = store_leaf_index;
  init_kdtree_leaf(ident,larger_leaf,CHUNK_SIZE,TRUE);
  
  for (i=1; i<=kd->n_dim_in; ++i) {
    larger_leaf->max[i] = store_leaf.max[i];
    larger_leaf->min[i] = store_leaf.min[i];
  }

  /* copy also the vertex indices from max_min_vertices */

  j = 1 << kd->n_dim_in;

  for (i=1; i<= j; ++i) {
    larger_leaf->max_min_vertices[i] = store_leaf.max_min_vertices[i];
  }

  larger_leaf->min[split] = split_criterion;

  for (i=1; i<=kd->n_dim_in; ++i) {
    larger_node->middle[i] = (larger_leaf->max[i]+larger_leaf->min[i])/2.;
  }


  /* the same stuff for the smaller leaf */
  
  if (kd->n_leaves+1 > MAX_ELEMENTS) {
    printf("ran out of memory for leaves\n");
    getchar();
    exit(-1);
  }
  
  smaller_leaf = &(kd->leaves[++kd->n_leaves]);
  smaller_node->leaf_index = kd->n_leaves;
  init_kdtree_leaf(ident,smaller_leaf,CHUNK_SIZE,TRUE);
  
  for (i=1; i<=kd->n_dim_in; ++i) {
    smaller_leaf->max[i] = store_leaf.max[i];
    smaller_leaf->min[i] = store_leaf.min[i];
  }

  /* copy also the vertex indices from max_min_vertices */

  j = 1 << kd->n_dim_in;

  for (i=1; i<= j; ++i) {
    smaller_leaf->max_min_vertices[i] = store_leaf.max_min_vertices[i];
  }

  smaller_leaf->max[split] = split_criterion;

  for (i=1; i<=kd->n_dim_in; ++i) {
    smaller_node->middle[i] = (smaller_leaf->max[i]+smaller_leaf->min[i])/2.;
  }
  
  
  /* now the kdtree is initialized, and only the old data has to be
     filled in */
  
  for (i=1; i<=store_leaf.n_points; ++i) {
    
    add_to_kdtree(ident,store_leaf.inputs[i],store_leaf.outputs[i],&pID, &npts);
    
  }



  /* now I have to sort the the old neighbor vertices into the two
     new leaves */

  for (i=1; i<=store_leaf.n_neighbor_vertices; ++i) {

    j = store_leaf.neighbor_vertices[i];

    if (kd->all_vertices[j][split] > split_criterion) {
      
      add_neighbor_vertex_by_index(ident,larger_leaf,j);

    } else {

      add_neighbor_vertex_by_index(ident,smaller_leaf,j);

    }

  }


  /* determine the vertices that must be added to other
     partitions */

  vector1 = my_vector(1,kd->n_dim_in);
  vector2 = my_vector(1,kd->n_dim_in);

  m = 1 << (kd->n_dim_in-1);

  for (i=0; i<m; ++i) {

    iaux = i;
    
    for (j=1; j<=kd->n_dim_in; ++j) {

      if (j == split) {
	vector1[j] = split_criterion; 
	continue;
      }

      irem  = iaux%2;
      iaux /= 2;
      
      if (irem == 1) {
	vector1[j] = store_leaf.max[j];
      } else {
	vector1[j] = store_leaf.min[j];
      }

    }


    /* add this vector to the global data base */

    neighbor_index = add_vertex_to_database(ident,vector1);


    /* and overwrite the appropriate entry in max_min_vertices of
       both the smaller and the larger leaf */

    n = 1 << kd->n_dim_in;

    for (j=1; j<=n; ++j) {
      if (kd->all_vertices[smaller_leaf->max_min_vertices[j]][split] >
	  split_criterion) {
	smaller_leaf->max_min_vertices[j] = neighbor_index;
	break;
      }
    }

    for (j=1; j<=n; ++j) {
      if (kd->all_vertices[larger_leaf->max_min_vertices[j]][split] <
	  split_criterion) {
	larger_leaf->max_min_vertices[j] = neighbor_index;
	break;
      }
    }

    
    /* now I have to vary this vector by adding a very small number to
       make it stick out of the current partition. Then I can find
       the partition it is in. I unfortunately have to do this for
       all combinations of adding/subracting to really cover all
       possible partitions */


    for (n=0; n<m; ++n) {

      iaux = n;

      for (j=1; j<=kd->n_dim_in; ++j) {
      
	if (j == split) {
	  vector2[j] = vector1[j]; 
	  continue;
	}

	irem  = iaux%2;
	iaux /= 2;
	
	if (irem == 1) {
	  vector2[j] = vector1[j]+.000001;
	} else {
	  vector2[j] = vector1[j]-.000001;
	}
      }

      find_partition(ident,vector2,&leafID,&leaf,&node);

      if (leaf != larger_leaf && leaf != smaller_leaf) {

	/* as soon as it was determined which index a point gets
	   int the all_vertices array, I can add it by index
	   afterwards */
	
	add_neighbor_vertex_by_index(ident,leaf,neighbor_index);
	
      }

    }
    
  }
      
  /* finally free the old data matrices */
  
  free_kdtree_leaf(ident,&store_leaf);
  my_free_vector(vector1,1,kd->n_dim_in);
  my_free_vector(vector2,1,kd->n_dim_in);


  
  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
 \note  get_kdtree_leaf_info
 \date  August 7, 1993
 
 \remarks 
 
 returns all vertices of the leaf partition as well as its
 means and medians
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ident       : the kdtree ID
 \param[in]     node_ID     : the ID of the leaf
 \param[out]    vertices    : ptr to matrix in which the vertices are to be stored
 \param[out]    ind_ar      : the ID numbers of all the vertices
 \param[out]    mean_inputs : vector of means of inputs
 \param[out]    mean_outputs: vector of means of outputs
 \param[out]    median      : vector of median of inputs
 
 ******************************************************************************/
int
get_kdtree_leaf_info(int ident,int node_ID,double **vertices,
		     int *ind_ar, double *mean_inputs, 
		     double *mean_outputs, double *median)
     
{
  int     i,j;
  KDTree *kd;
  KDLeaf *leaf;
  KDNode *node;
  int     n_vertex;
  int     iaux, irem;
  
  
  kd   = &(kdtrees[ident]);
  
  if (node_ID > kd->n_nodes || node_ID < 1) return FALSE;

  node = &(kd->nodes[node_ID]);
  
  if (!node->is_leaf) return FALSE;
  
  leaf = &(kd->leaves[node->leaf_index]);
  
  /* the means are very simple to calculte */
  
  for (i=1; i<=kd->n_dim_in; ++i) {
    mean_inputs[i] = leaf->sum_inputs[i]/(double)leaf->n_points;
    median[i]      = leaf->median[i];
  }
  for (i=1; i<=kd->n_dim_out; ++i) {
    mean_outputs[i] = leaf->sum_outputs[i]/(double)leaf->n_points;
  }

  /* get the vertices */

  n_vertex = 1 << kd->n_dim_in;
  
  for (i=1; i <= n_vertex; ++i) {
    
    for (j=1; j<=kd->n_dim_in; ++j) {
      vertices[i][j] = kd->all_vertices[leaf->max_min_vertices[i]][j];;
    }

    ind_ar[i] = leaf->max_min_vertices[i];
    
  }
  
  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
 \note  write_kdtree_ascii
 \date  August 7, 1993
 
 \remarks 
 
 dumps a kdtree to an ascii file with the kdtree name
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ident : the kdtree ID
 
 ******************************************************************************/
int
  write_kdtree_ascii(int ident)


{

  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
 \note  write_kdtree
 \date  August 7, 1993
 
 \remarks 
 
 dumps a kdtree to a binary format file with the kdtree name
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ident : the kdtree ID
 
 ******************************************************************************/
int
  write_kdtree(int ident)


{
  
  FILE    *out;
  KDTree  *kd;
  int      i,j;
  int      n_in;
  int      n_out;
  
  kd    = &(kdtrees[ident]);
  n_in  = kd->n_dim_in;
  n_out = kd->n_dim_out;
  
  out = fopen(kd->kdtree_name,"w");
  
  if (out == NULL) return FALSE;
  
  if (fwrite(kd,sizeof(KDTree),1,out)!= 1) {
    printf( "cannot fwrite file.\n" );
    return FALSE;
  }
  
  
  if (fwrite(&(kd->max_inputs[1]),sizeof(double),n_in,out)!= n_in) {
    printf( "cannot fwrite file.\n" );
    return FALSE;
  }
  
  if (fwrite(&(kd->min_inputs[1]),sizeof(double),n_in,out)!= n_in) {
    printf( "cannot fwrite file.\n" );
    return FALSE;
  }
  
  
  if (fwrite(kd->leaves,sizeof(KDLeaf),kd->n_leaves+1,out)!= kd->n_leaves+1) {
    printf( "cannot fwrite file.\n" );
    return FALSE;
  }
  
  if (fwrite(kd->nodes,sizeof(KDNode),kd->n_nodes+1,out)!= kd->n_nodes+1) {
    printf( "cannot fwrite file.\n" );
    return FALSE;
  }
  
  if (fwrite(&(kd->all_vertices[1][1]),
	     sizeof(double), kd->n_all_vertices * n_in,
	     out)!=kd->n_all_vertices * n_in) {
    printf( "cannot fwrite file.\n" );
    return FALSE;
  }

  if (fwrite(&(kd->all_vertices_output[1][1]),
	     sizeof(double), kd->n_all_vertices * n_out,
	     out)!=kd->n_all_vertices * n_out) {
    printf( "cannot fwrite file.\n" );
    return FALSE;
  }

  if (fwrite(&(kd->all_vertices_output_status[1]),
	     sizeof(int), kd->n_all_vertices,
	     out)!=kd->n_all_vertices) {
    printf( "cannot fwrite file.\n" );
    return FALSE;
  }

  if (fwrite(&(kd->n_elements_all_vertices_extra[1]),
	     sizeof(int), kd->n_all_vertices,
	     out)!=kd->n_all_vertices) {
    printf( "cannot fwrite file.\n" );
    return FALSE;
  }

  for (i=1; i<=kd->n_all_vertices; ++i) {

    if (kd->n_elements_all_vertices_extra[i] > 0) {

      if (fwrite(&(kd->all_vertices_extra[i][1]),
		 sizeof(double), kd->n_elements_all_vertices_extra[i],
		 out)!=kd->n_elements_all_vertices_extra[i]) {
	printf( "cannot fwrite file.\n" );
	return FALSE;
      }

    }

  }

  

  
  /* now all the memory for the leaves */
  

  for (i=1; i<=kd->n_leaves; ++i) {
    
    if (fwrite(&(kd->leaves[i].inputs[1][1]),sizeof(double),
	       n_in * kd->leaves[i].n_points,out) != 
	n_in * kd->leaves[i].n_points) {
      printf( "cannot fwrite file.\n" );
      return FALSE;
    }
    
    if (fwrite(&(kd->leaves[i].outputs[1][1]),sizeof(double),
	       n_out * kd->leaves[i].n_points,out) != 
	n_out * kd->leaves[i].n_points) {
      printf( "cannot fwrite file.\n" );
      return FALSE;
    }
    
    if (fwrite(&(kd->leaves[i].sum_inputs[1]),sizeof(double),n_in,out) 
	!= n_in) {
      printf( "cannot fwrite file.\n" );
      return FALSE;
    }
    
    if (fwrite(&(kd->leaves[i].sum_outputs[1]),sizeof(double),n_out,out) != 
	n_out) {
      printf( "cannot fwrite file.\n" );
      return FALSE;
    }
    
    if (fwrite(&(kd->leaves[i].max[1]),sizeof(double),n_in,out) != n_in) {
      printf( "cannot fwrite file.\n" );
      return FALSE;
    }
    
    if (fwrite(&(kd->leaves[i].min[1]),sizeof(double),n_in,out) != n_in) {
      printf( "cannot fwrite file.\n" );
      return FALSE;
    }
    
    if (fwrite(&(kd->leaves[i].max_min_vertices[1]),sizeof(int),1 <<n_in,out)!= 
	1 << n_in) {
      printf( "cannot fwrite file.\n" );
      return FALSE;
    }
    
    if (fwrite(&(kd->leaves[i].neighbor_vertices[1]), sizeof(int),
	       kd->leaves[i].n_neighbor_vertices, out) != 
	kd->leaves[i].n_neighbor_vertices) {
      printf( "cannot fwrite file.\n" );
      return FALSE;
    }
    
    if (fwrite(&(kd->leaves[i].median[1]),sizeof(double),n_in,out) != n_in) {
      printf( "cannot fwrite file.\n" );
      return FALSE;
    }

    if (fwrite(&(kd->leaves[i].next_greater_median[1]),sizeof(double),n_in,out) 
	!= n_in) {
      printf( "cannot fwrite file.\n" );
      return FALSE;
    }

    if (fwrite(&(kd->leaves[i].next_lower_median[1]),sizeof(double),n_in,out) 
	!= n_in) {
      printf( "cannot fwrite file.\n" );
      return FALSE;
    }

    if (kd->leaves[i].n_elements_extra > 0) {

      if (fwrite(&(kd->leaves[i].extra[1]),sizeof(double),
		 kd->leaves[i].n_elements_extra,out) 
	  != kd->leaves[i].n_elements_extra) {
	printf( "cannot fwrite file.\n" );
	return FALSE;
      }
      
    }
    
  }
  
  
  for (i=1; i<=kd->n_nodes; ++i) {
    
    if (fwrite(&(kd->nodes[i].mean_inputs[1]),sizeof(double),n_in,out)!= n_in) {
      printf( "cannot fwrite file.\n" );
      return FALSE;
    }
    
    if (fwrite(&(kd->nodes[i].mean_outputs[1]),sizeof(double),n_out,out)
	!=n_out) {
      printf( "cannot fwrite file.\n" );
      return FALSE;
    }
    
    if (fwrite(&(kd->nodes[i].median[1]),sizeof(double),n_in,out) != n_in) {
      printf( "cannot fwrite file.\n" );
      return FALSE;
    }
    
    if (fwrite(&(kd->nodes[i].middle[1]),sizeof(double),n_in,out) != n_in) {
      printf( "cannot fwrite file.\n" );
      return FALSE;
    }
    
  }
  
  
  fclose(out);
  
  
  return TRUE;
  
  
}

/*!*****************************************************************************
 *******************************************************************************
 \note  read_kdtree
 \date  August 7, 1993
 
 \remarks 
 
 reads a kdtree from binary file with the kdtree name
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ident : ident number to be assigned
 \param[in]     fname : the name of the file containing the table
 
 ******************************************************************************/
int
  read_kdtree(int ident,char *fname)

{
  
  FILE    *in;
  KDTree  *kd;
  int      i,j;
  int      n_in;
  int      n_out;
  double **ranges;
  double  *vec;
  int      n_leaves, n_nodes, n_all_vertices, n_allocated_all_vertices;
  size_t   number;
  size_t   length;
  
  
  in = fopen(fname,"r");
  
  if (in == NULL) return FALSE;
  
  kd    = &(kdtrees[ident]);
  
  if (fread(kd,sizeof(KDTree),1,in) != 1) {
    printf( "cannot fread file.\n" );
    return FALSE;
  }
  
  n_in                     = kd->n_dim_in;
  n_out                    = kd->n_dim_out;
  n_leaves                 = kd->n_leaves;
  n_nodes                  = kd->n_nodes;
  n_all_vertices           = kd->n_all_vertices;
  n_allocated_all_vertices = kd->n_allocated_all_vertices;

  ranges = my_matrix(1,n_in,1,2);
  vec    = my_vector(1,n_in);
  
  if (fread(&(vec[1]),sizeof(double),n_in,in)!= n_in) {
    printf( "cannot fread file.\n" );
    return FALSE;
  }

  for (i=1; i<=n_in; ++i) ranges[i][MAX] = vec[i];
  
  if (fread(&(vec[1]),sizeof(double),n_in,in)!= n_in) {
    printf( "cannot fread file.\n" );
    return FALSE;
  }

  for (i=1; i<=n_in; ++i) ranges[i][MIN] = vec[i];
  
  /* initialize the primary structure for this kdtree */
  
  init_kdtree(ident, kd->kdtree_name, n_in, n_out,kd->split_type, ranges);
  
  my_free_matrix(ranges,1,n_in,1,2);
  my_free_vector(vec,1,n_in);

  kd->n_leaves                 = n_leaves;
  kd->n_nodes                  = n_nodes;
  kd->n_all_vertices           = n_all_vertices;
  kd->n_allocated_all_vertices = n_allocated_all_vertices;


 /* and free the memory of the first leaf and first node, since
     this was initialized by default */
  
  free_kdtree_leaf(ident,&(kd->leaves[1]));
  free_kdtree_node(ident,&(kd->nodes[1]));


  if (fread(kd->leaves,sizeof(KDLeaf),kd->n_leaves+1,in)!= kd->n_leaves+1) {
    printf( "cannot fread file.\n" );
    return FALSE;
  }
  
  if (fread(kd->nodes,sizeof(KDNode),kd->n_nodes+1,in)!= kd->n_nodes+1) {
    printf( "cannot fread file.\n" );
    return FALSE;
  }

  my_free_matrix(kd->all_vertices,1,CHUNK_SIZE,1,kd->n_dim_in);
  kd->all_vertices = my_matrix(1,kd->n_allocated_all_vertices,1,kd->n_dim_in);

  my_free_matrix(kd->all_vertices_output,1,CHUNK_SIZE,1,kd->n_dim_out);
  kd->all_vertices_output = my_matrix(1,kd->n_allocated_all_vertices,1,
				      kd->n_dim_out);

  my_free_ivector(kd->all_vertices_output_status,1,CHUNK_SIZE);
  kd->all_vertices_output_status = my_ivector(1,kd->n_allocated_all_vertices);

  my_free_ivector(kd->n_elements_all_vertices_extra,1,CHUNK_SIZE);
  kd->n_elements_all_vertices_extra =my_ivector(1,kd->n_allocated_all_vertices);

  free(kd->all_vertices_extra);
  number = kd->n_allocated_all_vertices+1;
  length = sizeof(double *);
  kd->all_vertices_extra = (double **) my_calloc(number,length,MY_STOP);


  if (fread(&(kd->all_vertices[1][1]),
	     sizeof(double), kd->n_all_vertices * n_in,
	     in) != kd->n_all_vertices * n_in) {
    printf( "cannot fread file.\n" );
    return FALSE;
  }
  
  if (fread(&(kd->all_vertices_output[1][1]),
	     sizeof(double), kd->n_all_vertices * n_out,
	     in) != kd->n_all_vertices * n_out) {
    printf( "cannot fread file.\n" );
    return FALSE;
  }
  
  if (fread(&(kd->all_vertices_output_status[1]),
	     sizeof(int), kd->n_all_vertices,
	     in) != kd->n_all_vertices) {
    printf( "cannot fread file.\n" );
    return FALSE;
  }
  
  if (fread(&(kd->n_elements_all_vertices_extra[1]),
	     sizeof(int), kd->n_all_vertices,
	     in)!=kd->n_all_vertices) {
    printf( "cannot fread file.\n" );
    return FALSE;
  }

  for (i=1; i<=kd->n_all_vertices; ++i) {

    if (kd->n_elements_all_vertices_extra[i] > 0) {

      kd->all_vertices_extra[i] = 
	my_vector(1,kd->n_elements_all_vertices_extra[i]);

      if (fread(&(kd->all_vertices_extra[i][1]),
		 sizeof(double), kd->n_elements_all_vertices_extra[i],
		 in)!=kd->n_elements_all_vertices_extra[i]) {
	printf( "cannot fread file.\n" );
	return FALSE;
      }

    }

  }


  /* now all the leaves */

  for (i=1; i<=kd->n_leaves; ++i) {
    
    init_kdtree_leaf(ident,&(kd->leaves[i]),
		     kd->leaves[i].n_allocated_points,FALSE);
    
    if (fread(&(kd->leaves[i].inputs[1][1]),sizeof(double),
	      n_in * kd->leaves[i].n_points,in) != 
	n_in * kd->leaves[i].n_points) {
      printf( "cannot fread file.\n" );
      return FALSE;
    }
    
    if (fread(&(kd->leaves[i].outputs[1][1]),sizeof(double),
	      n_out * kd->leaves[i].n_points,in) != n_out * kd->leaves[i].n_points) {
      printf( "cannot fread file.\n" );
      return FALSE;
    }
    
    if (fread(&(kd->leaves[i].sum_inputs[1]),sizeof(double),n_in,in) != n_in) {
      printf( "cannot fread file.\n" );
      return FALSE;
    }
    
    if (fread(&(kd->leaves[i].sum_outputs[1]),sizeof(double),n_out,in)!=n_out) {
      printf( "cannot fread file.\n" );
      return FALSE;
    }
    
    if (fread(&(kd->leaves[i].max[1]),sizeof(double),n_in,in) != n_in) {
      printf( "cannot fread file.\n" );
      return FALSE;
    }
    
    if (fread(&(kd->leaves[i].min[1]),sizeof(double),n_in,in) != n_in) {
      printf( "cannot fread file.\n" );
      return FALSE;
    }
    
    if (fread(&(kd->leaves[i].max_min_vertices[1]),sizeof(int),1 << n_in,in) != 
	1 << n_in) {
      printf( "cannot fread file.\n" );
      return FALSE;
    }


    my_free_ivector(kd->leaves[i].neighbor_vertices,1,CHUNK_SIZE);
    kd->leaves[i].neighbor_vertices = 
      my_ivector(1,kd->leaves[i].n_allocated_neighbor_vertices);
    
    if (fread(&(kd->leaves[i].neighbor_vertices[1]), sizeof(int),
	       kd->leaves[i].n_neighbor_vertices, in) != 
	kd->leaves[i].n_neighbor_vertices) {
      printf( "cannot fread file.\n" );
      return FALSE;
    }
    
    if (fread(&(kd->leaves[i].median[1]),sizeof(double),n_in,in) != n_in) {
      printf( "cannot fread file.\n" );
      return FALSE;
    }

    if (fread(&(kd->leaves[i].next_greater_median[1]),sizeof(double),n_in,in) 
	!= n_in) {
      printf( "cannot fread file.\n" );
      return FALSE;
    }

    if (fread(&(kd->leaves[i].next_lower_median[1]),sizeof(double),n_in,in) 
	!= n_in) {
      printf( "cannot fread file.\n" );
      return FALSE;
    }

    if (kd->leaves[i].n_elements_extra > 0) {
      kd->leaves[i].extra = my_vector(1,kd->leaves[i].n_elements_extra);

      if (fread(&(kd->leaves[i].extra[1]),sizeof(double),
		 kd->leaves[i].n_elements_extra,in) 
	  != kd->leaves[i].n_elements_extra) {
	printf( "cannot fread file.\n" );
	return FALSE;
      }
      
    }

  }
  
  
  for (i=1; i<=kd->n_nodes; ++i) {
    
    init_kdtree_node(ident,&(kd->nodes[i]),FALSE);
    
    if (fread(&(kd->nodes[i].mean_inputs[1]),sizeof(double),n_in,in) != n_in) {
      printf( "cannot fread file.\n" );
      return FALSE;
    }
    
    if (fread(&(kd->nodes[i].mean_outputs[1]),sizeof(double),n_out,in) != 
	n_out) {
      printf( "cannot fread file.\n" );
      return FALSE;
    }
    
    if (fread(&(kd->nodes[i].median[1]),sizeof(double),n_in,in) != n_in) {
      printf( "cannot fread file.\n" );
      return FALSE;
    }

    if (fread(&(kd->nodes[i].middle[1]),sizeof(double),n_in,in) != n_in) {
      printf( "cannot fread file.\n" );
      return FALSE;
    }
  }
  
  
  fclose(in);
  
  
  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
 \note  read_kdtree_ascii
 \date  August 7, 1993
 
 \remarks 
 
 reads a kdtree from an ascii file with the kdtree name
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     fname : the name of the file containing the kdtree
 
 ******************************************************************************/
int
read_kdtree_ascii(int ident,char *fname)

{
  
  FILE *in;
  
  return TRUE;
  
  
}

/*!*****************************************************************************
 *******************************************************************************
 \note  add_neighbor_vertex
 \date  August 7, 1993
 
 \remarks 
 
 add a neigbor vertex to a given leaf pointer
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ident   : which kdtree
 \param[in]     lptr    : the leaf pointer
 \param[in]     vert    : vertex vector (n_dim_in)
 \param[out]    ind     : the index in the all_neighbors array which this point
              was assigned.
 
 ******************************************************************************/
static int
  add_neighbor_vertex(int ident,KDLeaf *lptr,double *vert, int *ind)

{

  int     *ptr_in;
  KDTree  *kd;
  int      i,j;
  int      only_add_index_flag = FALSE;

  kd    = &(kdtrees[ident]);

  /* check whether this point does not coincide with
     one of the partion's vertices */

  for (i=1; i<=kd->n_dim_in; ++i) {
    if (lptr->min[i] != vert[i] && lptr->max[i] != vert[i]) goto HOP1;
  }

  /* if this point is reached, the vert vector was composed just of
     min and max values of this leaf box ==> don't add */

  return FALSE;

 HOP1:

  /* check whether this vertex already exists in the all_vertices
     data base and add it if appropriate */

  *ind = add_vertex_to_database(ident,vert);


  /* is this point already included in this leaf ? */
  
  for (i=1; i<=lptr->n_neighbor_vertices; ++i) {
    
    /* if the point exists, just return FALSE */
    
    if (*ind == lptr->neighbor_vertices[i]) return FALSE;
    
  }
  

  /* now we are sure the vertex is new: add this data point */
  
  if (++lptr->n_neighbor_vertices > lptr->n_allocated_neighbor_vertices) {
    
    ptr_in  = my_ivector(1,lptr->n_allocated_neighbor_vertices+CHUNK_SIZE);
    
    /* copy the old data */
    
    memcpy(&(ptr_in[1]),&(lptr->neighbor_vertices[1]),
	   (size_t)((lptr->n_allocated_neighbor_vertices)*sizeof(int)));
    
    my_free_ivector(lptr->neighbor_vertices,1,
		   lptr->n_allocated_neighbor_vertices);
    
    lptr->neighbor_vertices  = ptr_in;
    
    lptr->n_allocated_neighbor_vertices += CHUNK_SIZE;
    
  }

  /* register this index in the leaf */

  lptr->neighbor_vertices[lptr->n_neighbor_vertices] = *ind;


  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  add_neighbor_vertex_by_index
 \date  August 7, 1993
 
 \remarks 
 
 add a neigbor vertex to a given leaf pointer using the index of the
 vertex in the global database
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ident   : which kdtree
 \param[in]     lptr    : the leaf pointer
 \param[in]     ind     : the index in the all_neighbors array which this point
              was assigned.
 
 ******************************************************************************/
static int
  add_neighbor_vertex_by_index(int ident,KDLeaf *lptr, int ind)

{

  int     *ptr_in;
  KDTree  *kd;
  int      i,j;

  kd    = &(kdtrees[ident]);

  /* check whether this point does not coincide with
     one of the partion's vertices */

  j = 1 << kd->n_dim_in;

  for (i=1; i<=kd->n_dim_in; ++i) {
    if (lptr->max_min_vertices[i] == ind) return FALSE;
  }


  /* check whether the point is not already included in the neighbor
     vertex array of this leaf */

  for (i=1; i<=lptr->n_neighbor_vertices; ++i) {
    if (lptr->neighbor_vertices[i] == ind) return FALSE;
  }


  /* now we are sure the vertex is new: add this data point */
  
  if (++lptr->n_neighbor_vertices > lptr->n_allocated_neighbor_vertices) {
    
    ptr_in  = my_ivector(1,lptr->n_allocated_neighbor_vertices+CHUNK_SIZE);
    
    /* copy the old data */
    
    memcpy(&(ptr_in[1]),&(lptr->neighbor_vertices[1]),
	   (size_t)((lptr->n_allocated_neighbor_vertices)*sizeof(int)));
    
    my_free_ivector(lptr->neighbor_vertices,1,
		   lptr->n_allocated_neighbor_vertices);
    
    lptr->neighbor_vertices  = ptr_in;
    
    lptr->n_allocated_neighbor_vertices += CHUNK_SIZE;
    
  }

  /* register this index in the leaf */

  lptr->neighbor_vertices[lptr->n_neighbor_vertices] = ind;


  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  find_partition
 \date  August 7, 1993
 
 \remarks 
 
 given an input vector, finds the partition to which the vector
 belongs
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ident   : the table ID of the kdtree
 \param[in]     in      : the input vector
 \param[out]    leafID  : the ID of the leaf to which the input vector belongs
 \param[out]    lptr    : the pointer of this leaf
 \param[out]    nptr    : the pointer of the associated node

 returns FALSE if not belonging to kdtree
 
 ******************************************************************************/
static int
  find_partition(int ident,double *in, int *leafID, KDLeaf **lptr, KDNode **nptr)

{

  int     j, i;
  int     partition;
  KDTree *kd;
  int     sd;
  double  aux1=0.0;
  
  kd = &kdtrees[ident];

  /* is the partition within the range of the current kdtree? */

  for (i=1; i<=kd->n_dim_in; ++i) {
    if (in[i] > kd->max_inputs[i] || in[i] < kd->min_inputs[i]) {
      return FALSE;
    }
  }
  
  /* find the partion in which the point is to be added */
  
  partition = 1;
  
  while (!kd->nodes[partition].is_leaf) {
    
    sd = kd->nodes[partition].split_dim;

    switch(kd->split_type) {
      
    case SPLIT_AT_MEDIAN:
      aux1 = kd->nodes[partition].median[sd];
      break;

    case SPLIT_AT_MEAN:
      aux1 = kd->nodes[partition].mean_inputs[sd];
      break;

    case SPLIT_AT_MIDDLE:
      aux1 = kd->nodes[partition].middle[sd];
      break;

    default:
      exit(-1);

    }
    
    if (in[sd] > aux1) {
      partition = kd->nodes[partition].next_larger;
    } else {
      partition = kd->nodes[partition].next_smaller;
    }
    
  }
  
  /* the leaf is thus determined to be: */
  
  *(nptr) = &(kd->nodes[partition]);
  *(lptr) = &(kd->leaves[(*nptr)->leaf_index]);
  *leafID = partition;
  
  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  get_kdtree_parm
 \date  August 7, 1993
 
 \remarks 
 
 returns some of the important kd-tree parameters 
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ident      : the table ID of the kdtree
 \param[out]    n_leaves   : number of leaves
 \param[out]    n_nodes    : number of nodes
 
 ******************************************************************************/
int
  get_kdtree_parm(int ident, int *n_leaves, int *n_nodes)

{
  
  KDTree *kd;
  
  kd = &(kdtrees[ident]);
  
  *n_leaves = kd->n_leaves;
  *n_nodes  = kd->n_nodes;

  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
 \note  get_kdtree_leaf_vertices
 \date  August 7, 1993
 
 \remarks 
 
 returns all vertices in the vertices array. The max number of 
 elements in array is given in n_v, and it is overwritten by the
 number of vertices filled in. The corner vertices are filled into
 vertices first, and then the neighboring vertices.
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ident      : the table ID of the kdtree
 \param[in]     leafID     : ID of the leaf to checked out
 \param[in,out] vertices    : matrix of vertices
 \param[in,out] n_v         : max number which fits into vertices, returns the
                  real number of vertices put inot vertices
 \param[out]    ind_ar      : array of numbers which index the vertices (also
                  characterized by n_v

		  returns TRUE if all vertices fit into the vertices
		  array, an returns FALSE if array too small
 
 ******************************************************************************/
int
  get_kdtree_leaf_vertices(int ident, int leafID, double **vertices, int *n_v, 
			   int *ind_ar)

{
  
  KDTree *kd;
  KDLeaf *leaf;
  int     n_vertices,i,j;
  int     count=0;
  int     rc = TRUE;
  
  kd = &(kdtrees[ident]);
  if (leafID > kd->n_leaves || leafID < 0) return FALSE;
  leaf = &(kd->leaves[leafID]);


  n_vertices = 1 << kd->n_dim_in;

  for (i=1; i<=n_vertices; ++i) {

    if (++count > *n_v) {
      rc = FALSE;
      count = *n_v;
      break;
    }
    
    for (j=1; j<=kd->n_dim_in; ++j) {
      
	vertices[count][j] = kd->all_vertices[leaf->max_min_vertices[i]][j];
      
    }

    ind_ar[count] = leaf->max_min_vertices[i];
    
  }

  for (i=1; i<=leaf->n_neighbor_vertices; ++i) {

    if (++count > *n_v) {
      rc = FALSE;
      count = *n_v;
      break;
    }
    
    for (j=1; j<=kd->n_dim_in; ++j) {
      
	vertices[count][j] = kd->all_vertices[leaf->neighbor_vertices[i]][j];
      
    }
    
    ind_ar[count] = leaf->neighbor_vertices[i];

  }

  *n_v = count;

  return rc;
  
}

/*!*****************************************************************************
 *******************************************************************************
 \note  generate_max_min_vertices
 \date  August 7, 1993
 
 \remarks 
 
 generates all vertex vector which correspond to the corners of the
 kd-tree partition
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ident      : the table ID of the kdtree
 \param[in]     lptr       : the pointer to the leaf
 
 ******************************************************************************/
static int
  generate_max_min_vertices(int ident, KDLeaf *lptr)

{
  
  KDTree *kd;
  int     iaux,irem,n_vertex,i,j,n,k;
  double *vector;
  int     vertex_index;
  int     only_add_index_flag;
  
  kd = &(kdtrees[ident]);

  vector = my_vector(1,kd->n_dim_in);

  n_vertex = 1 << kd->n_dim_in;
  
  for (i=0; i < n_vertex; ++i) {

    /* generate the vertex vector */
    
    iaux = i;
    
    for (j=1; j <= kd->n_dim_in; ++j) {
      
      irem  = iaux%2;
      iaux /= 2;
      
      if (irem == 1) {
	
	vector[j] = lptr->max[j];
	
      } else {
	
	vector[j] = lptr->min[j];
	
      }
      
    }

    /* check whether it exists already in the vertex data base and
       add it if appropriate */

    vertex_index = add_vertex_to_database(ident, vector);

    /* and register the index in the appropriate array */

    lptr->max_min_vertices[i+1] = vertex_index;


  }

  my_free_vector(vector,1,kd->n_dim_in);

  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
 \note  add_vertex_to_database
 \date  August 7, 1993
 
 \remarks 
 
 add a vertex to the global data base of vertices if it does not exist
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ident      : the table ID of the kdtree

         returns the index of the added vertex
 
 ******************************************************************************/
static int
  add_vertex_to_database(int ident, double *vector)

{

  KDTree *kd;
  int     i,j,k;
  int     vertex_index;
  double **ptr_in;
  int     *iptr_in;
  size_t   number;
  size_t   length;

  kd = &(kdtrees[ident]);

  for (j=1; j <= kd->n_all_vertices; ++j) {
    
    for (k=1; k <= kd->n_dim_in; ++k) {
      if (kd->all_vertices[j][k] != vector[k]) goto HOP;
    }
    
    /* if this point is reached, the vert vector coincides with
       a previously added one ==> make a note of its index */
    
    vertex_index = j;
    
    return vertex_index;
    
  HOP:
    ;
  }

  /* otherwise add the vertex */


  if (++kd->n_all_vertices > kd->n_allocated_all_vertices) {
    
    ptr_in  = my_matrix(1,kd->n_allocated_all_vertices+10*CHUNK_SIZE,
			1,kd->n_dim_in);
    
    /* copy the old data */
    
    memcpy(&(ptr_in[1][1]),&(kd->all_vertices[1][1]),
	   (size_t)((kd->n_allocated_all_vertices)*
		    (kd->n_dim_in)*sizeof(double)));
    
    my_free_matrix(kd->all_vertices,1,
		   kd->n_allocated_all_vertices,1,kd->n_dim_in);


    kd->all_vertices  = ptr_in;
    


    ptr_in  = my_matrix(1,kd->n_allocated_all_vertices+10*CHUNK_SIZE,
			1,kd->n_dim_out);
    
    /* copy the old data */
    
    memcpy(&(ptr_in[1][1]),&(kd->all_vertices_output[1][1]),
	   (size_t)((kd->n_allocated_all_vertices)*
		    (kd->n_dim_out)*sizeof(double)));
    
    my_free_matrix(kd->all_vertices_output,1,
		   kd->n_allocated_all_vertices,1,kd->n_dim_out);


    kd->all_vertices_output  = ptr_in;




    iptr_in  = my_ivector(1,kd->n_allocated_all_vertices+10*CHUNK_SIZE);
    for (i=kd->n_allocated_all_vertices; 
	 i<=kd->n_allocated_all_vertices+10*CHUNK_SIZE; ++i) 
      iptr_in[i] = 0;
    
    /* copy the old data */
    
    memcpy(&(iptr_in[1]),&(kd->all_vertices_output_status[1]),
	   (size_t)((kd->n_allocated_all_vertices)*sizeof(int)));
    
    my_free_ivector(kd->all_vertices_output_status,1,
		   kd->n_allocated_all_vertices);


    kd->all_vertices_output_status  = iptr_in;

    


    iptr_in  = my_ivector(1,kd->n_allocated_all_vertices+10*CHUNK_SIZE);
    for (i=kd->n_allocated_all_vertices; 
	 i<=kd->n_allocated_all_vertices+10*CHUNK_SIZE; ++i) 
      iptr_in[i] = 0;
    
    /* copy the old data */
    
    memcpy(&(iptr_in[1]),&(kd->n_elements_all_vertices_extra[1]),
	   (size_t)((kd->n_allocated_all_vertices)*sizeof(int)));
    
    my_free_ivector(kd->n_elements_all_vertices_extra,1,
		   kd->n_allocated_all_vertices);


    kd->n_elements_all_vertices_extra  = iptr_in;

    


    number = kd->n_allocated_all_vertices+10*CHUNK_SIZE+1;
    length = sizeof(double *);
    ptr_in = (double **) my_calloc(number,length,MY_STOP);
    
    /* copy the old data */

    memcpy(ptr_in, kd->all_vertices_extra,
	   (size_t)((kd->n_allocated_all_vertices+1)*sizeof(double *)));
    
    free(kd->all_vertices_extra);
    
    kd->all_vertices_extra  = ptr_in;



    

    kd->n_allocated_all_vertices += 10*CHUNK_SIZE;
    
  }
  
  /* add the vertex to the data array */

  for (i=1; i<=kd->n_dim_in; ++i) {
    kd->all_vertices[kd->n_all_vertices][i] = vector[i];
  }

  kd->all_vertices_output_status[kd->n_all_vertices] = FALSE;

  vertex_index = kd->n_all_vertices;

  return vertex_index;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  get_kdtree_all_vertices_ptr
 \date  August 7, 1993
 
 \remarks 
 
     returns the pointer to kd->all_vertices
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ident       : the kdtree ID
 \param[out]    ptr         : the ptr to the vertices
 \param[out]    n_elements  : the number of elements in ptr
 
 ******************************************************************************/
int
  get_kdtree_all_vertices_ptr(int ident, double ***ptr, int *n_elements)

{

  *ptr = kdtrees[ident].all_vertices;
  *n_elements = kdtrees[ident].n_all_vertices;
  
  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  add_kdtree_output_to_vertex
 \date  August 7, 1993
 
 \remarks 
 
     adds a given output to a vertex given by the vertex index
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ident       : the kdtree ID
 \param[in]     v_index     : the index of the vertex
 \param[in]     vec         : the vector to be added
 
 ******************************************************************************/
int
  add_kdtree_output_to_vertex(int ident, int v_index, double *vec)

{

  KDTree *kd;
  int     i,j,k;

  kd = &(kdtrees[ident]);

  for (i=1; i<=kd->n_dim_out; ++i) {
    kd->all_vertices_output[v_index][i] = vec[i];
  }

  kd->all_vertices_output_status[v_index] = TRUE;

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  delete_kdtree_output_from_vertex
 \date  August 7, 1993
 
 \remarks 
 
     deletes an output from a vertex given by the vertex index
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ident       : the kdtree ID
 \param[in]     v_index     : the index of the vertex
 
 ******************************************************************************/
int
  delete_kdtree_output_from_vertex(int ident, int v_index)

{

  KDTree *kd;

  kd = &(kdtrees[ident]);

  kd->all_vertices_output_status[v_index] = FALSE;

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  get_kdtree_output_from_vertex
 \date  August 7, 1993
 
 \remarks 
 
     retrives an output from a vertex given by the vertex index
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ident       : the kdtree ID
 \param[in]     v_index     : the index of the vertex
 \param[out]    vec         : the vector to be added

 returns FALSE if no output assigned
 
 ******************************************************************************/
int
  get_kdtree_output_from_vertex(int ident, int v_index, double *vec)

{

  KDTree *kd;
  int     i,j,k;

  kd = &(kdtrees[ident]);

  if (!kd->all_vertices_output_status[v_index]) return FALSE;

  for (i=1; i<=kd->n_dim_out; ++i) {
    vec[i] = kd->all_vertices_output[v_index][i];
  }

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  add_kdtree_extra
 \date  August 7, 1993
 
 \remarks 
 
 add a value to a given kdtree:
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ident   : the table ID of the kdtree
 \param[in]     part_ID : the ID number of the node to whose leaf the extra info 
               will be added
 \param[in]     extra   : the extra vector
 \param[in]     n_el    : the number of elements in the extra vector
 
 ******************************************************************************/
int
  add_kdtree_extra(int ident, int part_ID, double *extra, int n_el)

{
  
  int j, i;
  KDLeaf *leaf;
  KDNode *node;
  KDTree *kd;

  kd   = &kdtrees[ident];
  if (part_ID > kd->n_nodes || part_ID < 1) return FALSE;
  node = &(kd->nodes[part_ID]);
  if (!node->is_leaf) return FALSE;
  leaf = &(kd->leaves[node->leaf_index]);

  /* note, I simply overwrite when some vector already exists */

  if (leaf->n_elements_extra != n_el) {

    if (leaf->n_elements_extra > 0) {

      my_free_vector(leaf->extra,1,leaf->n_elements_extra);

    }

    leaf->extra = my_vector(1,n_el);
    leaf->n_elements_extra = n_el;

  }


  /* add the vector */

  for (i=1; i<=n_el; ++i) {
    leaf->extra[i] = extra[i];
  }

  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
 \note  get_kdtree_extra
 \date  August 7, 1993
 
 \remarks 
 
 add a value to a given kdtree:
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ident   : the table ID of the kdtree
 \param[in]     part_ID : the ID number of the node to whose leaf the extra info 
               will be added
 \param[out]    extra   : the extra vector
 \param[in,out] n_el    : the number of elements in the extra vector and how
                many elements are provided by the vector extra as
		input


 returns FALSE if no data was found
 
 ******************************************************************************/
int
  get_kdtree_extra(int ident, int part_ID, double *extra, int *n_el)

{
  
  int j, i;
  KDLeaf *leaf;
  KDNode *node;
  KDTree *kd;

  kd   = &kdtrees[ident];
  if (part_ID > kd->n_nodes || part_ID < 1) return FALSE;
  node = &(kd->nodes[part_ID]);
  if (!node->is_leaf) return FALSE;
  leaf = &(kd->leaves[node->leaf_index]);

  /* note, I simply overwrite when some vector already exists */

  if (leaf->n_elements_extra <= 0) return FALSE;

  /* retrieve the data */

  /* a safety measure; the user should know what he added */
  
  if (*n_el >= leaf->n_elements_extra) 
     *n_el = leaf->n_elements_extra;


  for (i=1; i <= *n_el; ++i) {
    extra[i] = leaf->extra[i];
  }

  /* finally set the number of elements which extra actually has */

  *n_el = leaf->n_elements_extra;

  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
 \note  delete_kdtree_data_from_partition
 \date  August 7, 1993
 
 \remarks 
 
 deletes all data in a partition give by the node ID
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ident   : the table ID of the kdtree
 \param[in]     part_ID : the ID number of the node 
 
 ******************************************************************************/
int
  delete_kdtree_data_from_partition(int ident, int part_ID)

{
  
  int j, i;
  KDLeaf *leaf;
  KDNode *node;
  KDTree *kd;

  kd   = &kdtrees[ident];

  if (part_ID > kd->n_nodes || part_ID < 1) return FALSE;

  node = &(kd->nodes[part_ID]);

  if (!node->is_leaf) return FALSE;

  leaf = &(kd->leaves[node->leaf_index]);

  my_free_matrix(leaf->inputs,1,leaf->n_allocated_points,1,kd->n_dim_in);
  my_free_matrix(leaf->outputs,1,leaf->n_allocated_points,1,kd->n_dim_out);

  leaf->n_allocated_points = leaf->n_points = 0;

  return TRUE;
  
}


/*!*****************************************************************************
 *******************************************************************************
 \note  predict_kdtree_output
 \date  August 7, 1993
 
 \remarks 
 
 given an input vector, the function either returns the extra vector
 of the corresponding leaf, or it interpolates among the vertex data.
 I amy want to add more options here like nearest neighbors or so.
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ident   : the table ID of the kdtree
 \param[in]     in      : the input vector
 \param[in]     opt     : the retrieve options
 \param[in,out] n_el    : howmany elements does out maximal have, and the number
               of elements which are found are returned
 \param[out]    partID  : the ID of the leaf to which the input vector belongs
 \param[out]    out     : the output vector

 NOTE: the output vector must be big enough for INTERPOLATE_EXTRA

 returns TRUE or FALSE if successful or not
 
 ******************************************************************************/
int
  predict_kdtree_output(int ident,double *in, int opt, int *n_el,
			int *partID, double *out)

{

  int     j, i, k;
  KDTree *kd;
  KDLeaf *leaf;
  KDNode *node;
  double **vec_in;
  double **vec_out;
  int     n_in;
  int     n_pts;
  int     n_out;
  
  kd = &kdtrees[ident];
  
  if (!find_partition(ident,in, partID, &leaf, &node)) {
    return FALSE;
  }
  
  /* if this was successful, distinguish according to the users
     wishes */

  if (!leaf->prediction_status) {
    return FALSE;
  }


  switch (opt) {


  case KDTREE_EXTRA:

    /* this just retrieves the extra vector */

    if (!get_kdtree_extra(ident,*partID, out, n_el)) return FALSE;

    break;


  case KDTREE_INTERPOLATE:

    /* currently I just interpolate among the major vertices of the
       partition and not among both the major vertices and the
       neighbor vertices. This is because I could not find a 
       straighforward way to interpolate among all these points
       without excessive computation */

    /* shuffle the vertices into an array */

    n_in = kd->n_dim_in;
    n_out = kd->n_dim_out;
    n_pts = 1 << n_in;
    
    vec_in  = my_matrix(1,n_pts,1,n_in);
    vec_out = my_matrix(1,n_pts,1,n_out);
    
    for (i=1; i<=n_pts; ++i) {
      
      k = leaf->max_min_vertices[i];
      
      /* if no output values were attached to vertex, return FALSE */
      
      if (!kd->all_vertices_output_status[k]) return FALSE;
      
      for (j=1; j<=n_in; ++j) {
	vec_in[i][j] = kd->all_vertices[k][j];
      }
      
      for (j=1; j<=n_out; ++j) {
	vec_out[i][j] = kd->all_vertices_output[k][j];
      }
      
    }
    
    /* interpolate */
    
    if (!interpolate(vec_in, n_in, vec_out, n_out, in, out)) {
      return FALSE;
    }
     
    my_free_matrix(vec_in,1,n_pts,1,n_in);
    my_free_matrix(vec_out,1,n_pts,1,n_out);
    
    break;
    
    
  case KDTREE_INTERPOLATE_EXTRA:

    /* interpolate among the extra vertices */

    /* shuffle the vertices into an array */

    n_in = kd->n_dim_in;
    n_out = kd->n_elements_all_vertices_extra[leaf->max_min_vertices[1]];
    if (n_out <= 0) return FALSE;
    n_pts = 1 << n_in;
    
    vec_in  = my_matrix(1,n_pts,1,n_in);
    vec_out = my_matrix(1,n_pts,1,n_out);

    for (i=1; i<=n_pts; ++i) {

      k = leaf->max_min_vertices[i];

      /* if no output values were attached to vertex, return FALSE */

      if (!kd->all_vertices_output_status[k]) return FALSE;

      for (j=1; j<=n_in; ++j) {
	vec_in[i][j] = kd->all_vertices[k][j];
      }

      /* check whether all vertices extra vector have the same lenght */

      *n_el = kd->n_elements_all_vertices_extra[k];
      if (kd->n_elements_all_vertices_extra[k] != n_out) return FALSE;

      for (j=1; j<=n_out; ++j) {
	vec_out[i][j] = kd->all_vertices_extra[k][j];
      }

    } 

    /* interpolate */

    if (!interpolate(vec_in, n_in, vec_out, n_out, in, out)) {
      return FALSE;
    }

    my_free_matrix(vec_in,1,n_pts,1,n_in);
    my_free_matrix(vec_out,1,n_pts,1,n_out);
    
    break;


  default:

    return FALSE;


  }


  return TRUE;

}


/*!*****************************************************************************
 *******************************************************************************
 \note  set_kdtree_prediction_status
 \date  August 7, 1993
 
 \remarks 
 
 set a binary value which indicates whether the partition is good for
 prediciton or not
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ident   : the table ID of the kdtree
 \param[in]     part_ID : the ID number of the node to whose leaf the extra info 
               will be added
 \param[in]     status  : the status to be set
 
 ******************************************************************************/
int
  set_kdtree_prediction_status(int ident, int part_ID, int status)

{
  
  int j, i;
  KDLeaf *leaf;
  KDNode *node;
  KDTree *kd;

  kd   = &kdtrees[ident];
  if (part_ID > kd->n_nodes || part_ID < 1) return FALSE;
  node = &(kd->nodes[part_ID]);
  if (!node->is_leaf) return FALSE;
  leaf = &(kd->leaves[node->leaf_index]);

  leaf->prediction_status = status;

  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
 \note  get_kdtree_prediction_status
 \date  August 7, 1993
 
 \remarks 
 
 returns the status value for this leaf
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ident   : the table ID of the kdtree
 \param[in]     part_ID : the ID number of the node to whose leaf the extra info 
               will be added
 
 returns FALSE or TRUE according to the status	
    
 ******************************************************************************/
int
  get_kdtree_prediction_status(int ident, int part_ID)

{
  
  int j, i;
  KDLeaf *leaf;
  KDNode *node;
  KDTree *kd;

  kd   = &kdtrees[ident];
  if (part_ID > kd->n_nodes || part_ID < 1) return FALSE;
  node = &(kd->nodes[part_ID]);
  if (!node->is_leaf) return FALSE;
  leaf = &(kd->leaves[node->leaf_index]);

  return leaf->prediction_status;
  
}

/*!*****************************************************************************
 *******************************************************************************
 \note  add_kdtree_extra_to_vertex
 \date  August 7, 1993
 
 \remarks 
 
     adds a given extra vector to a vertex given by the vertex index
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ident       : the kdtree ID
 \param[in]     v_index     : the index of the vertex
 \param[in]     vec         : the vector to be added
 \param[in]     n_el        : the number of elements of the vector
 
 ******************************************************************************/
int
  add_kdtree_extra_to_vertex(int ident, int v_index, double *vec, int n_el)

{

  KDTree *kd;
  int     i,j,k;

  kd = &(kdtrees[ident]);

  /* allocate the memory if not present */

  if (kd->n_elements_all_vertices_extra[v_index] != n_el) {

    if (kd->n_elements_all_vertices_extra[v_index] > 0) {
      my_free_vector(kd->all_vertices_extra[v_index],1,
		     kd->n_elements_all_vertices_extra[v_index]);
    }
     
    kd->all_vertices_extra[v_index] = my_vector(1,n_el);

  }

  for (i=1; i<=n_el; ++i) {
    kd->all_vertices_extra[v_index][i] = vec[i];
  }

  kd->n_elements_all_vertices_extra[v_index] = n_el;

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  get_kdtree_extra_from_vertex
 \date  August 7, 1993
 
 \remarks 
 
     retrieves an extra vector from a vertex given by the vertex index
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     ident       : the kdtree ID
 \param[in]     v_index     : the index of the vertex
 \param[in]     vec         : the vector to be added
 \param[in,out] n_el        : the number of elements of the vector, and howmany elements
                   the user provided in vec

 returns FALSE if no vector exists for this vertex
 
 ******************************************************************************/
int
  get_kdtree_extra_from_vertex(int ident, int v_index, double *vec, int *n_el)

{

  KDTree *kd;
  int     i,j,k;

  kd = &(kdtrees[ident]);

  /* allocate the memory if not present */

  if (kd->n_elements_all_vertices_extra[v_index] <= 0) return FALSE;

  /* a safety measure */

  if (kd->n_elements_all_vertices_extra[v_index] <= *n_el)
    *n_el = kd->n_elements_all_vertices_extra[v_index];

  for (i=1; i<=*n_el; ++i) {
    vec[i] =  kd->all_vertices_extra[v_index][i];
  }

  *n_el = kd->n_elements_all_vertices_extra[v_index];

  return TRUE;

}

