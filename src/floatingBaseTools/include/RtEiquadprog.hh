/*********************************************************************
 Autonomous Motion Department
 Max-Planck Intelligent Systems
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         RtEiquadprog.hh

 \author       Alexander Herzog
 \date         Aug 4, 2013

 *********************************************************************/

#ifndef RTEIQUADPROG_HH_
#define RTEIQUADPROG_HH_

#include <Eigen/Dense>
#include "RtMatrixX.h"
#include "RtQuadraticProgram.hh"

namespace floating_base_utilities
{

template<int nVars, int nEqCon, int nIneqCon>
class RtEiquadprog : public RtQuadraticProgram<nVars, nEqCon,
      nIneqCon>
{
public:
  static const bool must_have_equality_constraints_ = true;
  typedef RtQuadraticProgram<nVars, nEqCon, nIneqCon> BaseClass;

  RtEiquadprog();
  virtual ~RtEiquadprog();

  inline void reset();
  inline void reset(int dim_qp, int num_eq, int num_ineq);

  bool solve();
  bool isSolved() const;

  /**
   * solves the problem
   * min. x' Hess x + 2 g0' x
   * s.t. CE x + ce0 = 0
   *      CI x + ci0 <= 0
   */
  inline double solve_quadprog(
      const typename RtMatrixX<nVars,nVars>::d & Hess, const typename RtVectorX<nVars>::d & g0,
      const typename RtMatrixX<nEqCon, nVars>::d & CE, const typename RtVectorX<nEqCon>::d & ce0,
      const typename RtMatrixX<nIneqCon, nVars>::d & CI, const typename RtVectorX<nIneqCon>::d & ci0,
      typename RtVectorX<nVars>::d & x);

  typename RtMatrixX<nVars,nVars>::d Hessian_factor_inv_; // Hessian_factor_inv_ * Hessian_factor_inv_' = Hessian
  bool is_inverse_provided_;
  bool cleanup_ineqs_;
private:
  Eigen::LLT<typename RtMatrixX<nVars,nVars>::d,Eigen::Lower> chol_;
  double solver_return_;

  template<typename Scalar>
  inline Scalar distance(Scalar a, Scalar b)
  {
          Scalar a1, b1, t;
          a1 = std::abs(a);
          b1 = std::abs(b);
          if (a1 > b1)
          {
                  t = (b1 / a1);
                  return a1 * std::sqrt(1.0 + t * t);
          }
          else
                  if (b1 > a1)
                  {
                          t = (a1 / b1);
                          return b1 * std::sqrt(1.0 + t * t);
                  }
          return a1 * std::sqrt(2.0);
  }

  inline void compute_d(typename RtVectorX<nVars>::d & d,
      const typename RtMatrixX<nVars,nVars>::d & J, const typename RtVectorX<nVars>::d & np)
  {
    d = J.adjoint() * np;
  }

  inline void update_z(typename RtVectorX<nVars>::d & z,
      const typename RtMatrixX<nVars,nVars>::d & J,
      const typename RtVectorX<nVars>::d & d, int iq)
  {
      z = J.rightCols(J.cols()-iq) * d.tail(J.cols()-iq);
  }

  inline void update_r(const typename RtMatrixX<nVars,nVars>::d & R,
      typename RtVectorX<nIneqCon+nEqCon>::d& r, const typename RtVectorX<nVars>::d& d,
      int iq)
  {
    r.head(iq)= R.topLeftCorner(iq,iq)
                 .template triangularView<Eigen::Upper>()
                 .solve(d.head(iq));
  }

  inline bool add_constraint(
      typename RtMatrixX<nVars,nVars>::d & R, typename RtMatrixX<nVars,nVars>::d & J,
      typename RtVectorX<nVars>::d & d,
      int& iq, double& R_norm);
  inline void delete_constraint(
      typename RtMatrixX<nVars,nVars>::d& R, typename RtMatrixX<nVars,nVars>::d& J,
      typename RtVectorX<nIneqCon+nEqCon>::i & A, typename RtVectorX<nIneqCon+nEqCon>::d & u,
      int p, int& iq, int l);
};

} /* namespace floating_base_utilities */
#endif /* RTEIQUADPROG_HH_ */
