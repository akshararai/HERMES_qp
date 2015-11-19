/*********************************************************************
 Autonomous Motion Department
 Max-Planck Intelligent Systems
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         CvxopQp.hh

 \author       Alexander Herzog
 \date         Nov 17, 2013

 *********************************************************************/

#ifndef CVXOPQP_HH_
#define CVXOPQP_HH_

#include "RtQuadraticProgram.hh"
#include "py_cpp_interface/PyShell.h"

namespace floating_base_utilities
{

template<int nVars, int nEqCon, int nIneqCon>
class CvxopQp : public RtQuadraticProgram<nVars, nEqCon,
      nIneqCon>
{
public:
  typedef RtQuadraticProgram<nVars, nEqCon, nIneqCon> BaseClass;

  CvxopQp();
  virtual ~CvxopQp(){};

  bool solve();
  bool isSolved() const;

private:
  py_cpp_interface::PyShell * py_shell_;
  bool solver_return_;

};

} /* namespace floating_base_utilities */
#endif /* CVXOPQP_H_ */
