/*********************************************************************
 Autonomous Motion Department
 Max-Planck Intelligent Systems
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         RtHierarchicalTask.hh

 \author       Alexander Herzog
 \date         Jul 13, 2013

 *********************************************************************/

#ifndef RTHIERARCHICALTASK_HH_
#define RTHIERARCHICALTASK_HH_

#include "RtQuadraticProgram.h"
//#include "RtQpOasis.h"
#include "NonRtEiquadprog.hh"

namespace floating_base_utilities
{


// Max_Num_Mats = #{A, B1, B2, .., B(N-1)} = N
template<int Max_A_Rows, int max_num_vars, int Max_B_Rows>
class RtHierarchicalTask
{
public:
//  template <int _A_Rows, int _A_Cols, int _Options, int _MaxRows, int _MaxCols>
  RtHierarchicalTask();

  virtual ~RtHierarchicalTask();

  void reset();

  template <typename AMatDer, typename aVecDer>
  void reset(const Eigen::MatrixBase<AMatDer>& mat_A,
       const Eigen::MatrixBase<aVecDer>& vec_a)
  {
    RtMatrixXUtils::setZero(A_mat_, mat_A.rows(), mat_A.cols());
    RtVectorXUtils::setZero(a_vec_, mat_A.rows());
    A_mat_ = mat_A;
    a_vec_ = vec_a;

    RtMatrixXUtils::setZero(B_prev_Z_, 0, numVars());
    RtMatrixXUtils::setIdentity(prev_Z_, numVars());
    RtVectorXUtils::setZero(solution_, numVars());
  }

  bool solveInitial();

  template <typename BMatDer, typename bVecDer>
  bool solveNextTask(const Eigen::MatrixBase<BMatDer>& B_mat,
        const Eigen::MatrixBase<bVecDer>& b_vec);


  /**
   * Accessors
   */
  const typename RtVectorX<max_num_vars>::d& solution() const;
  const typename RtVectorX<Max_A_Rows + Max_B_Rows>::d& slack() const;
  const typename RtMatrixX<Max_A_Rows, max_num_vars>::d& AMat() const;
  const typename RtVectorX<Max_A_Rows>::d& aVec() const;

  const typename RtMatrixX<Max_B_Rows, max_num_vars>::d prevBZMat() const;
  const typename RtMatrixX<max_num_vars, max_num_vars>::d prevZMat() const;

private:
  static const double diag_addition_for_psd_hessian_ = 0.000001;
  typename RtMatrixX<Max_A_Rows, max_num_vars>::d A_mat_;
  typename RtVectorX<Max_A_Rows>::d a_vec_;

  typename RtMatrixX<Max_B_Rows, max_num_vars>::d B_prev_Z_;
  typename RtMatrixX<max_num_vars, max_num_vars>::d prev_Z_;
  typename RtVectorX<max_num_vars>::d solution_;
  typename RtVectorX<Max_A_Rows + Max_B_Rows>::d slack_;
//  RtQpOasis<max_num_vars+Max_B_Rows+Max_A_Rows, Max_B_Rows, Max_A_Rows> qp_solver_;  //this should be replaceable with other solver
  NonRtEiquadprog<max_num_vars+Max_B_Rows+Max_A_Rows, Max_B_Rows, Max_A_Rows> qp_solver_;  //this should be replaceable with other solver

  RtQuadraticProgram<max_num_vars+Max_B_Rows+Max_A_Rows, Max_B_Rows, Max_A_Rows>& solver();
  int numVars() const;

};

template<int Max_A_Rows, int max_num_vars, int Max_B_Rows>
bool RtHierarchicalTask<Max_A_Rows, max_num_vars, Max_B_Rows>::solveInitial()
{
  const int num_aux_vars = A_mat_.rows();
  // set solver
  typename RtVectorX<max_num_vars+Max_B_Rows+Max_A_Rows>::d sol_tmp;
  sol_tmp = solver().solution();
  solver().reset(num_aux_vars+numVars(), 0, num_aux_vars);
  solver().solution() = sol_tmp;
  solver().objectiveQuadPart().block(numVars(), numVars(), num_aux_vars, num_aux_vars).setIdentity();

  // add slack on diagonal of hessian to make it positive definite
  solver().objectiveQuadPart().diagonal().array() +=  diag_addition_for_psd_hessian_;

  // create problem
  typename RtMatrixX<Max_A_Rows, max_num_vars+Max_A_Rows>::d init_ineq_mat;
  RtMatrixXUtils::resize(init_ineq_mat, A_mat_.rows(), num_aux_vars+numVars());
  init_ineq_mat.block(0,0, A_mat_.rows(), numVars()) = A_mat_;
  init_ineq_mat.block(0, numVars(), num_aux_vars, num_aux_vars).setIdentity();
  solver().appendInequalities(init_ineq_mat, a_vec_);

  // solve
  solver().solve();
#ifndef RTEIG_NO_ASSERTS
  assert(((!solver().isSolved() ) || (solver().isSolved() && solver().checkSolution(false)) ) &&  "Hierarchical initial task could not be solved");
#endif
  slack_ = solver().solution().block(numVars(), 0, num_aux_vars, 1);
  a_vec_ = a_vec_ + slack_;

  return solver().isSolved();
}

template<int Max_A_Rows, int max_num_vars, int Max_B_Rows>
template <typename BMatDer, typename bVecDer>
bool RtHierarchicalTask<Max_A_Rows, max_num_vars, Max_B_Rows>::solveNextTask(const Eigen::MatrixBase<BMatDer>& B_mat, const Eigen::MatrixBase<bVecDer>& b_vec)
{
#ifndef RTEIG_NO_ASSERTS
  assert(B_mat.rows() <= Max_B_Rows && "HierarchicalTask exceeds max rows!");
  assert(B_mat.cols() == numVars() && "HierarchicalTask has wrong number of columns!");
#endif

  const int num_aux_vars = B_mat.rows();
  // compute nullspace projector of previous B matrix
  if(B_prev_Z_.rows() > 0)
  {
    // update nullspace projector
    typename RtMatrixX<max_num_vars, Max_B_Rows>::d B_prev_Z_pinv;
    RtMatrixXUtils::resize(B_prev_Z_pinv, B_prev_Z_.cols(), B_prev_Z_.rows());
    typename RtMatrixX<max_num_vars, max_num_vars>::d B_prev_Z_null;
    RtMatrixXUtils::resize(B_prev_Z_null, B_prev_Z_.cols(), B_prev_Z_.cols());
    int dim_range = RtMatrixXUtils::computePseudoInv(B_prev_Z_, B_prev_Z_pinv, B_prev_Z_null);
    prev_Z_ = prev_Z_*B_prev_Z_null;
  }
  B_prev_Z_ = B_mat * prev_Z_;


  // set solver
  solver().reset(num_aux_vars+numVars(), B_mat.rows(), A_mat_.rows());
  solver().objectiveQuadPart().block(numVars(), numVars(), num_aux_vars, num_aux_vars).setIdentity();

  // add slack on diagonal of hessian to make it positive definite
  solver().objectiveQuadPart().diagonal().array() +=  diag_addition_for_psd_hessian_;

  // setup inequalities
  typename RtMatrixX<Max_A_Rows, max_num_vars+Max_B_Rows>::d ineq_mat;
  RtMatrixXUtils::setZero(ineq_mat, A_mat_.rows(), num_aux_vars+numVars());
  ineq_mat.block(0,0, A_mat_.rows(), numVars()) = A_mat_*prev_Z_;
//  typename RtVectorX<Max_A_Rows>::d ineq_vec;
//  RtVectorXUtils::resize(ineq_vec, A_mat_.rows());
//  ineq_vec = a_vec_ + A_mat_ * solution_;
  solver().appendInequalities(ineq_mat, a_vec_ + A_mat_ * solution_);

  // setup equalities
  typename RtMatrixX<Max_B_Rows, max_num_vars+Max_B_Rows>::d eq_mat;
  RtMatrixXUtils::setZero(eq_mat, B_mat.rows(), num_aux_vars+numVars());
  eq_mat.block(0,0, B_mat.rows(), numVars()) = B_mat*prev_Z_;
  eq_mat.block(0,numVars(), num_aux_vars, num_aux_vars).setIdentity();
  eq_mat.block(0,numVars(), num_aux_vars, num_aux_vars) *= -1.0;
//  typename RtVectorX<Max_A_Rows>::d eq_vec;
//  RtVectorXUtils::resize(eq_vec, B_mat.rows());
//  eq_vec = b_vec + B_mat * solution_;
  solver().appendEqualities(eq_mat, b_vec + B_mat * solution_);

  // solve
  solver().solve();
#ifndef RTEIG_NO_ASSERTS
  assert(((!solver().isSolved() ) || (solver().isSolved() && solver().checkSolution(false)) ) && "Hierarchical task could not be solved");
#endif
  solution_ += prev_Z_ * solver().solution().block(0, 0, numVars(), 1);
//  RtMatrixXUtils::resize(B_prev_Z_, B_mat.rows(), B_mat.cols());
//  B_prev_Z_ = B_mat*prev_Z_;
  slack_ = solver().solution().block(numVars(), 0, num_aux_vars, 1);

  return solver().isSolved();
}

}  //namespace
#endif /* RTHIERARCHICALTASK_HH_ */
