/*********************************************************************
 Autonomous Motion Department
 Max-Planck Intelligent Systems
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         NonRtEiquadprog.cpp

 \author       Alexander Herzog
 \date         Apr 24, 2013

 *********************************************************************/

#ifndef NONRTEIQUADPROG_H_
#define NONRTEIQUADPROG_H_

#include "EigenQpSolver.h"
#include "NonRtEiquadprog.hh"
#include "RtQuadraticProgram.h"
#include "FixedSizeEigenQpSolver.h"

namespace floating_base_utilities
{

template<int max_dim_var, int max_eq_constr, int max_ineq_constr>
const bool NonRtEiquadprog<max_dim_var, max_eq_constr, max_ineq_constr>::must_have_equality_constraints_;

template<int max_dim_var, int max_eq_constr, int max_ineq_constr>
NonRtEiquadprog<max_dim_var, max_eq_constr, max_ineq_constr>::NonRtEiquadprog() :
      solver_return_(std::numeric_limits<double>::infinity())
{
  is_inverse_provided_ = false;
};

template<int max_dim_var, int max_eq_constr, int max_ineq_constr>
NonRtEiquadprog<max_dim_var, max_eq_constr, max_ineq_constr>::~NonRtEiquadprog(){};


template<int max_dim_var, int max_eq_constr, int max_ineq_constr>
template<int nVars, int nIneqCon>
void NonRtEiquadprog<max_dim_var, max_eq_constr, max_ineq_constr>::solveFixedSized()
{

  Eigen::Matrix<double,nVars,nVars>  Hess = BaseClass::objectiveQuadPart();
  Eigen::Matrix<double,nVars,1>  g0 = BaseClass::objectiveLinPart();
  Eigen::Matrix<double,nVars,nIneqCon>  CI = -BaseClass::ineqConstraintsMat().transpose();
  Eigen::Matrix<double,nIneqCon,1>  ci0 = -BaseClass::ineqConstraintsVec();
  Eigen::Matrix<double,nVars,1>  sol_x = BaseClass::solution();


  if(is_inverse_provided_)
  {
//    Eigen::Matrix<double,nVars,nVars>  Hessian_fac_inv = Hessian_factor_inv_;
//    solver_return_ = FixedSizeEigenQpSolver<nVars, nIneqCon>::solve_quadprog(Hess, g0, CI, ci0, sol_x, Hessian_fac_inv);
    solver_return_ = FixedSizeEigenQpSolver<nVars, nIneqCon>::solve_quadprog(Hess, g0, CI, ci0, sol_x, Hessian_factor_inv_);
  }
  else
  {
    solver_return_ = FixedSizeEigenQpSolver<nVars, nIneqCon>::solve_quadprog(Hess, g0, CI, ci0, sol_x);
  }

  if(isSolved())
    BaseClass::solution() = sol_x.block(0, 0, BaseClass::numVariables(), 1);
}

template<int max_dim_var, int max_eq_constr, int max_ineq_constr>
bool NonRtEiquadprog<max_dim_var, max_eq_constr, max_ineq_constr>::solve()
{
  if(BaseClass::objectiveQuadPart().rows() == 0)
    return true;

  const int nVars = BaseClass::objectiveQuadPart().rows();
  const int nEqCon = BaseClass::eqConstraintsMat().rows();
  const int nIneqCon = BaseClass::ineqConstraintsMat().rows();
  bool is_handled_as_fixed_size = false;

  if(nEqCon == 0)
  {
//    if(nIneqCon == 116)
//    {
//      switch(nVars)
//      {
//        case 159:
//          solveFixedSized<159,116>();
//          is_handled_as_fixed_size = true;
//          break;
//        case 25:
//          solveFixedSized<25,116>();
//          is_handled_as_fixed_size = true;
//          break;
//        case 22:
//          solveFixedSized<22,116>();
//          is_handled_as_fixed_size = true;
//          break;
//        case 6:
//          solveFixedSized<6,116>();
//          is_handled_as_fixed_size = true;
//          break;
//        default:
//        break;
//      }
//    }
//    if(nIneqCon == 152)
//    {
//      switch(nVars)
//      {
//       case 204:
//         solveFixedSized<204,152>();
//         is_handled_as_fixed_size = true;
//         break;
//       case 34:
//         solveFixedSized<34,152>();
//         is_handled_as_fixed_size = true;
//         break;
//       case 31:
//         solveFixedSized<31,152>();
//         is_handled_as_fixed_size = true;
//         break;
//       case 6:
//         solveFixedSized<6,152>();
//         is_handled_as_fixed_size = true;
//         break;
//       default:
//         break;
//      }
//    }
//    else if(nIneqCon == 144)
//    {
//      switch(nVars)
//      {
//        case 190:
//          solveFixedSized<190,144>();
//          is_handled_as_fixed_size = true;
//          break;
//        case 34:
//          solveFixedSized<34,144>();
//          is_handled_as_fixed_size = true;
//          break;
//        case 25:
//          solveFixedSized<25,144>();
//          is_handled_as_fixed_size = true;
//          break;
//      }
//    }
  }

  if(!is_handled_as_fixed_size)
  {
    Eigen::MatrixXd Hess = BaseClass::objectiveQuadPart();
    Eigen::VectorXd g0 = BaseClass::objectiveLinPart();
    Eigen::MatrixXd CE = BaseClass::eqConstraintsMat().transpose();
    Eigen::VectorXd ce0 = BaseClass::eqConstraintsVec();
    Eigen::MatrixXd CI = -BaseClass::ineqConstraintsMat().transpose();
    Eigen::VectorXd ci0 = -BaseClass::ineqConstraintsVec();
    Eigen::VectorXd sol_x;
    sol_x = BaseClass::solution();
//    std::cout << "solving problem of size: " << Hess.rows() << "/" << Hess.cols() <<
//        "/" << g0.rows() << "/" << CE.rows() << "/" <<  CE.cols() << "/" <<
//        ce0.size() << "/" <<  CI.rows() << "/" <<  CI.cols() << "/" <<
//        ci0.size() << "/" << sol_x.rows() << std::endl;

    //  solver_return_ = Eigen::solve_quadprog(Hess, g0, CE, ce0, CI, ci0, sol_x);
    if(is_inverse_provided_)
      solver_return_ = EigenQpSolver::solve_quadprog(Hess, g0, CE, ce0, CI, ci0, sol_x, Hessian_factor_inv_);
    else
      solver_return_ = EigenQpSolver::solve_quadprog(Hess, g0, CE, ce0, CI, ci0, sol_x);

    if(isSolved())
      BaseClass::solution() = sol_x.block(0, 0, BaseClass::numVariables(), 1);
  }

  return isSolved();
}


template<int max_dim_var, int max_eq_constr, int max_ineq_constr>
bool NonRtEiquadprog<max_dim_var, max_eq_constr, max_ineq_constr>::isSolved() const
{
  return solver_return_ != std::numeric_limits<double>::infinity();
};


template<int max_dim_var, int max_eq_constr, int max_ineq_constr>
void NonRtEiquadprog<max_dim_var, max_eq_constr, max_ineq_constr>::reset()
{
  BaseClass::reset();
  RtMatrixXUtils::setZero(Hessian_factor_inv_, max_dim_var, max_dim_var);
}

template<int max_dim_var, int max_eq_constr, int max_ineq_constr>
void NonRtEiquadprog<max_dim_var, max_eq_constr, max_ineq_constr>::reset(int dim_qp, int num_eq, int num_ineq)
{
  BaseClass::reset(dim_qp, num_eq, num_ineq);
  RtMatrixXUtils::setZero(Hessian_factor_inv_, dim_qp, dim_qp);
}

}  // namespace
#endif //NONRTEIQUADPROG_H_

