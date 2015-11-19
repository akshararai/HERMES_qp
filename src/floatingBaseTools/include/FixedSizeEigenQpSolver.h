/*********************************************************************
 Autonomous Motion Department
 Max-Planck Intelligent Systems
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         FixedSizeEigenQpSolver.h

 \author       Alexander Herzog
 \date         Aug 3, 2013

 *********************************************************************/

#include <Eigen/Core>
#include <Eigen/Cholesky>

#include "FixedSizeEigenQpSolver.hh"

namespace floating_base_utilities
{

template<int nVars,  int nIneqCon>
FixedSizeEigenQpSolver<nVars, nIneqCon>::FixedSizeEigenQpSolver(){}

template<int nVars,  int nIneqCon>
FixedSizeEigenQpSolver<nVars, nIneqCon>::~FixedSizeEigenQpSolver(){}


template<int nVars,  int nIneqCon>
double FixedSizeEigenQpSolver<nVars, nIneqCon>::solve_quadprog(Eigen::Matrix<double,nVars,nVars> & Hessian,
      Eigen::Matrix<double,nVars,1> & g0,
      const Eigen::Matrix<double,nVars,nIneqCon> & CI, const Eigen::Matrix<double,nIneqCon,1> & ci0,
      Eigen::Matrix<double,nVars,1> & x)
{
  /* decompose the matrix Hessian in the form LL^T */
  Eigen::LLT<Eigen::Matrix<double,nVars,nVars>, Eigen::Lower> chol;
  chol.compute(Hessian);

  /* compute the inverse of the factorized matrix Hessian^-1, this is the initial value for H */
  // Hessian_factor_inv = L^-T
  Eigen::Matrix<double,nVars,nVars> Hessian_factor_inv = Eigen::Matrix<double,nVars,nVars>::Identity();
  Hessian_factor_inv = chol.matrixU().solve(Hessian_factor_inv);

  return solve_quadprog(Hessian, g0, CI, ci0, x, Hessian_factor_inv);
}

} /* namespace momentum_balance_control */
