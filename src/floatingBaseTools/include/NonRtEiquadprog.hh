/*********************************************************************
 Autonomous Motion Department
 Max-Planck Intelligent Systems
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         NonRtEquadprog.h

 \author       Alexander Herzog
 \date         Apr 24, 2013

 *********************************************************************/

#ifndef NONRTEIQUADPROG_HH_
#define NONRTEIQUADPROG_HH_

#include <limits>
#include <Eigen/Dense>
#include "RtQuadraticProgram.hh"
#include "EigenQpSolver.h"

namespace floating_base_utilities
{

template<int max_dim_var, int max_eq_constr, int max_ineq_constr>
class NonRtEiquadprog : public RtQuadraticProgram<max_dim_var, max_eq_constr,
      max_ineq_constr>
{
public:

  static const bool must_have_equality_constraints_ = true;
  typedef RtQuadraticProgram<max_dim_var,
      max_eq_constr, max_ineq_constr> BaseClass;

  NonRtEiquadprog();
  virtual ~NonRtEiquadprog();

  inline void reset();
  inline void reset(int dim_qp, int num_eq, int num_ineq);

  bool solve();
  bool isSolved() const;

  typename RtMatrixX<max_dim_var, max_dim_var>::d Hessian_factor_inv_;
  bool is_inverse_provided_;
private:
  double solver_return_;

  template<int nVars, int nIneqCon>
  void solveFixedSized();

};



} /* namespace floating_base_utilities */
#endif /* NONRTEIQUADPROG_HH_ */
