/*********************************************************************
 Autonomous Motion Department
 Max-Planck Intelligent Systems
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         EigenQpSolver.cpp

 \author       Alexander Herzog
 \date         Aug 1, 2013

 *********************************************************************/

#include "EigenQpSolver.h"

using namespace Eigen;

namespace floating_base_utilities
{

EigenQpSolver::EigenQpSolver()
{
  // TODO Auto-generated constructor stub

}

EigenQpSolver::~EigenQpSolver()
{
  // TODO Auto-generated destructor stub
}

double EigenQpSolver::solve_quadprog(Eigen::MatrixXd & Hessian,  Eigen::VectorXd & g0,
                             const Eigen::MatrixXd & CE, const Eigen::VectorXd & ce0,
                             const Eigen::MatrixXd & CI, const Eigen::VectorXd & ci0,
                             Eigen::VectorXd& x)
{  /* decompose the matrix Hessian in the form LL^T */
  LLT<MatrixXd,Lower> chol(Hessian.cols());
  chol.compute(Hessian);

  Eigen::MatrixXd Hessian_factor_inv;
  Hessian_factor_inv.setIdentity(Hessian.rows(), Hessian.rows());
  Hessian_factor_inv = chol.matrixU().solve(Hessian_factor_inv);

  return solve_quadprog(Hessian, g0, CE, ce0, CI, ci0, x, Hessian_factor_inv);
}

} /* namespace floating_base_utilities */
