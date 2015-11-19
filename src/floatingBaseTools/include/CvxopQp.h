/*********************************************************************
 Autonomous Motion Department
 Max-Planck Intelligent Systems
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         CvxopQp.h

 \author       Alexander Herzog
 \date         Nov 17, 2013

 *********************************************************************/

#ifndef CVXOPQP_H_
#define CVXOPQP_H_

#include "RtQuadraticProgram.h"
#include "CvxopQp.hh"

namespace floating_base_utilities
{

template<int nVars, int nEqCon, int nIneqCon>
CvxopQp<nVars, nEqCon, nIneqCon>::CvxopQp()
{
  py_shell_ = &py_cpp_interface::PyShell::shell();
  py_shell_->exec("from cvxopt import matrix");
  py_shell_->exec("from cvxopt.solvers import qp");
  py_shell_->exec("from cvxopt.solvers import options");
  py_shell_->exec("options['show_progress'] = False");
}


template<int nVars, int nEqCon, int nIneqCon>
bool CvxopQp<nVars, nEqCon, nIneqCon>::isSolved() const
{
  return solver_return_;
}

template<int nVars, int nEqCon, int nIneqCon>
bool CvxopQp<nVars, nEqCon, nIneqCon>::solve()
{
  if(BaseClass::objectiveQuadPart().rows() == 0)
    return true;

  if(BaseClass::qp_properties_ & BaseClass::ePP_MightZeroIneqs)
  {
    RtAffineUtils::removeZeroRows<nIneqCon, nVars>(BaseClass::ineqConstraintsMat(),
          BaseClass::ineqConstraintsVec());
  }
  using namespace py_cpp_interface;
  py_shell_->main_namespace_["H"] = py_shell_->matEigenToNumpy(BaseClass::objectiveQuadPart());
  py_shell_->main_namespace_["g"] = py_shell_->matEigenToNumpy(BaseClass::objectiveLinPart());
  py_shell_->main_namespace_["E"] = py_shell_->matEigenToNumpy(BaseClass::eqConstraintsMat());
  py_shell_->main_namespace_["e"] = py_shell_->matEigenToNumpy(BaseClass::eqConstraintsVec());
  py_shell_->main_namespace_["I"] = py_shell_->matEigenToNumpy(BaseClass::ineqConstraintsMat());
  py_shell_->main_namespace_["i"] = py_shell_->matEigenToNumpy(BaseClass::ineqConstraintsVec());
  py_shell_->exec("qp_sol = qp(matrix(H), matrix(g), matrix(I), -matrix(i), matrix(E), -matrix(e))");
  py_shell_->exec("tmp = np.array(qp_sol['x'])");
  py_shell_->matNumpyToEigen(boost::python::extract<boost::python::numeric::array>(
      py_shell_->main_namespace_["tmp"]), BaseClass::solution());
  py_shell_->exec("is_qp_ok = qp_sol['status']=='optimal'");
  solver_return_ = boost::python::extract<bool>(py_shell_->main_namespace_["is_qp_ok"]);

  return isSolved();
}

} /* namespace floating_base_utilities */

#endif
