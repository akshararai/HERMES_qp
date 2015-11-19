/*********************************************************************
 Autonomous Motion Department
 Max-Planck Intelligent Systems
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         RtQpOasis.h

 \author       Alexander Herzog
 \date         Apr 8, 2013

 *********************************************************************/

#ifndef RTQPOASIS_H_
#define RTQPOASIS_H_

#include <qpOASES.hpp>
#include "RtQuadraticProgram.h"

namespace floating_base_utilities
{

template<int max_dim_var, int max_eq_constr, int max_ineq_constr>
class RtQpOasis : public RtQuadraticProgram<max_dim_var,
      max_eq_constr, max_ineq_constr>
{
public:
  RtQpOasis() : solver_(max_dim_var, max_eq_constr + max_ineq_constr)
  {
    solver_options_ = solver_.getOptions();
    solver_options_.printLevel = qpOASES::PL_NONE;
    resetOasesData(true);
  };
  virtual ~RtQpOasis(){};

  bool virtual solve();

  typedef RtQuadraticProgram<max_dim_var,
      max_eq_constr, max_ineq_constr> BaseClass;
  inline void reset()
  {
    bool reset_solver = !(max_dim_var == BaseClass::numVariables() &&
        max_eq_constr == BaseClass::numEqConstr() &&
        max_ineq_constr == BaseClass::numIneqConstr());
    BaseClass::reset();
    resetOasesData(reset_solver);
  }
  inline void reset(int dim_qp, int num_eq, int num_ineq)
  {
    bool reset_solver = !(dim_qp == BaseClass::numVariables() &&
        num_eq == BaseClass::numEqConstr() &&
        num_ineq == BaseClass::numIneqConstr());
    BaseClass::reset(dim_qp, num_eq, num_ineq);
    resetOasesData(dim_qp,num_eq, num_ineq, reset_solver);
  }

  bool virtual checkSolution(bool print = true, double eq_threashold = 0.001, double ineq_threashold = 0.001);


  bool isSolved() const{return (solver_status_ == qpOASES::SUCCESSFUL_RETURN);}

  /**!
   * Accessors
   */
  const qpOASES::SQProblem& solver() const{return solver_;};
  qpOASES::SQProblem& solver() {return solver_;};
  const typename RtVectorX<max_dim_var>::d& upperBounds() const{return upper_bounds_;};
  typename RtVectorX<max_dim_var>::d& upperBounds() {return upper_bounds_;};
  const typename RtVectorX<max_dim_var>::d& lowerBounds() const{return lower_bounds_;};
  typename RtVectorX<max_dim_var>::d& lowerBounds() {return lower_bounds_;};
  const double* constraints() const{return constraints_;};
  double* constraints() {return constraints_;};
  const double* constrLowerBounds() const{return constr_lower_bounds_;};
  double* constrLowerBounds() {return constr_lower_bounds_;};
  const double* constrUpperBound() const{return constr_upper_bounds_;};
  double* constrUpperBound() {return constr_upper_bounds_;};
  const qpOASES::returnValue& solverStatus() const{return solver_status_;};
  qpOASES::returnValue& solverStatus() {return solver_status_;};

  const double* quadPartArray() const{return BaseClass::objectiveQuadPart().data();};
  double* quadPartArray() {return BaseClass::objectiveQuadPart().data();};
  const double* linPartArray() const{return BaseClass::objectiveLinPart().data();};
  double* linPartArray() {return BaseClass::objectiveLinPart().data();};

  const double* varUpperBoundArray() const{return upper_bounds_.data();};
  double* varUpperBoundArray() {return upper_bounds_.data();};
  const double* varLowerBoundArray() const{return lower_bounds_.data();};
  double* varLowerBoundArray() {return lower_bounds_.data();};

private:
  qpOASES::SQProblem solver_;
  qpOASES::Options solver_options_;

  typename RtVectorX<max_dim_var>::d upper_bounds_;
  typename RtVectorX<max_dim_var>::d lower_bounds_;
  double constraints_[max_dim_var*(max_eq_constr + max_ineq_constr)];
  double constr_lower_bounds_[max_eq_constr + max_ineq_constr];
  double constr_upper_bounds_[max_eq_constr + max_ineq_constr];
  qpOASES::returnValue solver_status_;


  void toDoubleArrays();

  inline void resetOasesData(bool reset_solver = true)
  {
    upper_bounds_ = 1000000*Eigen::Matrix<double, max_dim_var, 1>::Ones();
    lower_bounds_ = -upper_bounds_;
    solver_status_ = qpOASES::TERMINAL_LIST_ELEMENT;

    if(reset_solver)
    {
      solver_ = qpOASES::SQProblem(max_dim_var, max_eq_constr + max_ineq_constr);
      solver_.setOptions(solver_options_);
    }
  }
  inline void resetOasesData(int dim_qp, int num_eq, int num_ineq,
                             bool reset_solver = true)
  {
    RtVectorXUtils::setConstant(upper_bounds_, dim_qp, 1000000);
    lower_bounds_ = -upper_bounds_;
    solver_status_ = qpOASES::TERMINAL_LIST_ELEMENT;

    if(reset_solver)
    {
      solver_ = qpOASES::SQProblem(dim_qp, num_eq + num_ineq);
      solver_.setOptions(solver_options_);
    }
  }

};



template<int max_dim_var, int max_eq_constr, int max_ineq_constr>
void RtQpOasis<max_dim_var, max_eq_constr, max_ineq_constr>::toDoubleArrays()
{
  const int num_eq_constr = BaseClass::numEqConstr();
  const int num_ineq_constr = BaseClass::numIneqConstr();
  const int num_vars = BaseClass::numVariables();

  for(int i=0; i<num_eq_constr; i++)
    for(int j=0; j<num_vars; j++)
        constraints_[i*num_vars + j] = BaseClass::eqConstraintsMat()(i,j);

  typename RtVectorX<max_eq_constr>::d eq_bnd_tmp = -BaseClass::eqConstraintsVec();
  std::memcpy(constr_upper_bounds_, eq_bnd_tmp.data(), eq_bnd_tmp.size()*sizeof(double));
  std::memcpy(constr_lower_bounds_, eq_bnd_tmp.data(), eq_bnd_tmp.size()*sizeof(double));

  for(int i=0; i<num_ineq_constr; i++)
    for(int j=0; j<num_vars; j++)
        constraints_[num_eq_constr*num_vars + i*num_vars+j] = BaseClass::ineqConstraintsMat()(i,j);

  typename RtVectorX<max_ineq_constr>::d inq_bnd_tmp = -BaseClass::ineqConstraintsVec();
  std::memcpy(&(constr_upper_bounds_[num_eq_constr]), inq_bnd_tmp.data(), inq_bnd_tmp.size()*sizeof(double));
  RtVectorXUtils::setOnes(inq_bnd_tmp, num_ineq_constr);
  inq_bnd_tmp *= -1000000;
  std::memcpy(&(constr_lower_bounds_[num_eq_constr]), inq_bnd_tmp.data(), inq_bnd_tmp.size()*sizeof(double));

}

template<int max_dim_var, int max_eq_constr, int max_ineq_constr>
bool RtQpOasis<max_dim_var, max_eq_constr, max_ineq_constr>::solve()
{
  //convert to qpOases data-structures
  toDoubleArrays();

  int nWSR = 1000;
  if(!solver_.isInitialised())
          solver_status_ = solver_.init(quadPartArray(), linPartArray(), constraints(), varLowerBoundArray(),
                        varUpperBoundArray(), constrLowerBounds(), constrUpperBound(), nWSR, 0);
  else
          solver_status_ = solver_.hotstart(quadPartArray(), linPartArray(), constraints(), varLowerBoundArray(),
                        varUpperBoundArray(), constrLowerBounds(), constrUpperBound(), nWSR, 0);

  if(isSolved())
          solver_.getPrimalSolution(BaseClass::solution().data());

  return isSolved();
}

template<int max_dim_var, int max_eq_constr, int max_ineq_constr>
bool RtQpOasis<max_dim_var, max_eq_constr, max_ineq_constr>::checkSolution(bool print, double eq_threashold,
    double ineq_threashold)
{
  bool base_result = BaseClass::checkSolution(print, eq_threashold, ineq_threashold);

  typename RtVectorX<max_dim_var>::d lower_dist = lowerBounds() - BaseClass::solution();
  for(int i=0; i<BaseClass::numVariables(); ++i)
    lower_dist[i] *= lower_dist[i] <= 0.0 ? 0.0 : 1.0;
  const double lower_norm = lower_dist.norm();

  typename RtVectorX<max_dim_var>::d upper_dist = BaseClass::solution() - upperBounds();
  for(int i=0; i<BaseClass::numVariables(); ++i)
    upper_dist[i] *= upper_dist[i] <= 0.0 ? 0.0 : 1.0;
  const double upper_norm = upper_dist.norm();

  bool is_sol_valid = base_result && lower_norm <=ineq_threashold && upper_norm <= ineq_threashold;

  if(print || (!is_sol_valid && isSolved()))
  {
	  if(base_result && !is_sol_valid)
	  {
		  BaseClass::checkSolution(true, eq_threashold, ineq_threashold);
	  }
    std::cout << "  lower bound squared dist: " << lower_norm << std::endl;
    std::cout << "  upper bound squared dist: " << upper_norm << std::endl;
    std::cout << "  solver status: " << solver_status_ << std::endl;
  }

  return is_sol_valid;
}

} /* namespace floating_base_utilities */
#endif /* RTQPOASIS_H_ */
