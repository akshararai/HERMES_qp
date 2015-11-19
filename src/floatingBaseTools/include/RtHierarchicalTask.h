/*********************************************************************
 Autonomous Motion Department
 Max-Planck Intelligent Systems
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         RtHierarchicalTask.h

 \author       Alexander Herzog
 \date         Apr 19, 2013

 *********************************************************************/

#ifndef RTHIERARCHICALTASK_H_
#define RTHIERARCHICALTASK_H_

#include "NonRtEiquadprog.h"
#include "RtHierarchicalTask.hh"

namespace floating_base_utilities
{

template<int All_A_Rows, int Max_Num_Vars, int Max_B_Rows>
const double RtHierarchicalTask<All_A_Rows, Max_Num_Vars, Max_B_Rows>::diag_addition_for_psd_hessian_;

template<int Max_A_Rows, int max_num_vars, int Max_B_Rows>
RtHierarchicalTask<Max_A_Rows, max_num_vars, Max_B_Rows>::RtHierarchicalTask()
{
  reset();
}

template<int Max_A_Rows, int max_num_vars, int Max_B_Rows>
RtHierarchicalTask<Max_A_Rows, max_num_vars, Max_B_Rows>::~RtHierarchicalTask(){};

template<int Max_A_Rows, int max_num_vars, int Max_B_Rows>
void RtHierarchicalTask<Max_A_Rows, max_num_vars, Max_B_Rows>::reset()
{
  RtMatrixXUtils::setZero(A_mat_);
  RtVectorXUtils::setZero(a_vec_);
  RtMatrixXUtils::setZero(B_prev_Z_, 0, max_num_vars);
  RtMatrixXUtils::setIdentity(prev_Z_);
  RtVectorXUtils::setZero(solution_);
}

/**
 * Accessors
 */
template<int Max_A_Rows, int max_num_vars, int Max_B_Rows>
const typename RtVectorX<max_num_vars>::d& RtHierarchicalTask<Max_A_Rows, max_num_vars, Max_B_Rows>::solution() const {return solution_;};
template<int Max_A_Rows, int max_num_vars, int Max_B_Rows>
const typename RtVectorX<Max_A_Rows + Max_B_Rows>::d& RtHierarchicalTask<Max_A_Rows, max_num_vars, Max_B_Rows>::slack() const {return slack_;};
template<int Max_A_Rows, int max_num_vars, int Max_B_Rows>
const typename RtMatrixX<Max_A_Rows, max_num_vars>::d& RtHierarchicalTask<Max_A_Rows, max_num_vars, Max_B_Rows>::AMat() const{return A_mat_;};
template<int Max_A_Rows, int max_num_vars, int Max_B_Rows>
const typename RtVectorX<Max_A_Rows>::d& RtHierarchicalTask<Max_A_Rows, max_num_vars, Max_B_Rows>::aVec() const {return a_vec_;};

template<int Max_A_Rows, int max_num_vars, int Max_B_Rows>
const typename RtMatrixX<Max_B_Rows, max_num_vars>::d RtHierarchicalTask<Max_A_Rows, max_num_vars, Max_B_Rows>::prevBZMat() const {return B_prev_Z_;};
template<int Max_A_Rows, int max_num_vars, int Max_B_Rows>
const typename RtMatrixX<max_num_vars, max_num_vars>::d RtHierarchicalTask<Max_A_Rows, max_num_vars, Max_B_Rows>::prevZMat() const {return prev_Z_;};


template<int Max_A_Rows, int max_num_vars, int Max_B_Rows>
RtQuadraticProgram<max_num_vars+Max_B_Rows+Max_A_Rows, Max_B_Rows, Max_A_Rows>& RtHierarchicalTask<Max_A_Rows, max_num_vars, Max_B_Rows>::solver() {return qp_solver_;};
template<int Max_A_Rows, int max_num_vars, int Max_B_Rows>
int RtHierarchicalTask<Max_A_Rows, max_num_vars, Max_B_Rows>::numVars() const{return A_mat_.cols();};


} /* namespace floating_base_utilities */
#endif /* RTHIERARCHICALTASK_H_ */
