/*********************************************************************
 Autonomous Motion Department
 Max-Planck Intelligent Systems
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         test_CvxoptQp.cpp

 \author       Alexander Herzog
 \date         Nov 17, 2013

 *********************************************************************/

#include <Eigen/Dense>
#include <RtEiquadprog.h>
#include "CvxopQp.h"
#include "py_cpp_interface/PyShell.h"

using namespace floating_base_utilities;
using namespace py_cpp_interface;

std::string PyRank(const std::string& varname)
{
  return std::string("abs(np.linalg.svd(") +varname+ std::string(")[1] - np.linalg.svd(") +varname+ std::string(")[1][0]/1e7).argmin()");
}

int main()
{
  PyShell& py_shell = PyShell::shell();

  py_shell.exec("from cvxopt import matrix");
  py_shell.exec("from cvxopt.solvers import qp");
  py_shell.exec("from cvxopt.solvers import options");

  int nVars =5;
  int nEqs =2;
  int nIneqs = 3;

  py_shell.main_namespace_["nVars"] = nVars;
  py_shell.main_namespace_["nEqs"] = nEqs;
  py_shell.main_namespace_["nIneqs"] = nIneqs;

  Eigen::MatrixXd hess(nVars, nVars);
  Eigen::VectorXd linp(nVars);
  Eigen::MatrixXd eq_mat(nEqs, nVars);
  Eigen::VectorXd eq_vec(nEqs);
  Eigen::MatrixXd ineq_mat(nIneqs, nVars);
  Eigen::VectorXd ineq_vec(nIneqs);

  int eq_rank = 0;
  int stack_rank = 0;
  for(int trials=0; trials < 100 && (eq_rank+1 < nEqs || stack_rank+1 < nVars); ++trials)
  {
    hess.setRandom();
    hess = hess * hess.transpose() + Eigen::MatrixXd::Identity(nVars, nVars);
    linp.setRandom();
    eq_mat.setRandom() + Eigen::MatrixXd::Identity(nEqs, nVars);;
    eq_vec.setRandom();
    ineq_mat.setRandom();
    ineq_vec.setRandom();

    py_shell.main_namespace_["H"] = py_shell.matEigenToNumpy(hess);
    py_shell.main_namespace_["g"] = py_shell.matEigenToNumpy(linp);
    py_shell.main_namespace_["E"] = py_shell.matEigenToNumpy(eq_mat);
    py_shell.main_namespace_["e"] = py_shell.matEigenToNumpy(eq_vec);
    py_shell.main_namespace_["I"] = py_shell.matEigenToNumpy(ineq_mat);
    py_shell.main_namespace_["i"] = py_shell.matEigenToNumpy(ineq_vec);

    py_shell.exec(std::string("E_rank = ")+PyRank("E"));
    py_shell.exec("E_rank=int(E_rank)");
    py_shell.getVar("E_rank", eq_rank);
    std::cout << "E rank is " << eq_rank << std::endl;

    py_shell.exec("Test = np.zeros((nVars+nEqs+nIneqs,nVars))");
    py_shell.exec("Test[:nVars,:] = H");
    py_shell.exec("Test[nVars:nVars+nIneqs,:] = I");
    py_shell.exec("Test[nVars+nIneqs:,:] = E");
    py_shell.exec(std::string("Test_rank = ")+PyRank("Test"));
    py_shell.exec("Test_rank=int(Test_rank)");
    py_shell.getVar("Test_rank", stack_rank);
    std::cout << "[H; I; E] rank is " << stack_rank << std::endl;
  }

  CvxopQp<5,2,3> cvx_solver;
  cvx_solver.reset();
  cvx_solver.objectiveQuadPart() = hess;
  cvx_solver.objectiveLinPart() = linp;
  cvx_solver.eqConstraintsMat() = eq_mat;
  cvx_solver.eqConstraintsVec() = eq_vec;
  cvx_solver.ineqConstraintsMat() = ineq_mat;
  cvx_solver.ineqConstraintsVec() = ineq_vec;
  cvx_solver.solve();

  RtEiquadprog<5,2,3> rtsolver;
  rtsolver.reset();
  rtsolver.objectiveQuadPart() = hess;
  rtsolver.objectiveLinPart() = linp;
  rtsolver.eqConstraintsMat() = eq_mat;
  rtsolver.eqConstraintsVec() = eq_vec;
  rtsolver.ineqConstraintsMat() = ineq_mat;
  rtsolver.ineqConstraintsVec() = ineq_vec;
  rtsolver.solve();

  py_shell.exec("qp_sol = qp(matrix(H), matrix(g), matrix(I), -matrix(i), matrix(E), -matrix(e))");
//  py_shell.exec("qp_sol = qp(matrix(H), matrix(g))");

  py_shell.exec("is_qp_ok = qp_sol['status']=='optimal'");
  bool is_sol_ok = boost::python::extract<bool>(py_shell.main_namespace_["is_qp_ok"]);
  std::cout << "optimal?: " << is_sol_ok << std::endl;
  std::cout << "True: " << true << std::endl;
  std::cout << "False: " << false << std::endl;
  py_shell.exec("embIPython.ipsh()");
//  py_shell.exec("import IPython");
//  py_shell.exec("IPython.start_ipython()");
  py_shell.print("'qp_sol:', qp_sol");

  Eigen::VectorXd sol(nVars);

  py_shell.exec("tmp = np.array(qp_sol['x'])");
  py_shell.matNumpyToEigen(boost::python::extract<boost::python::numeric::array>(py_shell.main_namespace_["tmp"]), sol);
  std::cout << "diff norm = " << (rtsolver.solution() - cvx_solver.solution()).norm() << std::endl;

  return 0;
}
