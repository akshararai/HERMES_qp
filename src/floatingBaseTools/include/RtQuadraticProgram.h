/*********************************************************************
 Autonomous Motion Department
 Max-Planck Intelligent Systems
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         RtQuadraticProgram.h

 \author       Alexander Herzog
 \date         Apr 7, 2013

 *********************************************************************/


#ifndef RTQUADRATICPROGRAM_H_
#define RTQUADRATICPROGRAM_H_

#include "ConfigUtils.h"
#include "RtQuadraticProgram.hh"

namespace floating_base_utilities
{

/***!
 * Accessors
 */
template <int max_dim_qp, int max_num_eq, int max_num_ineq>
const typename RtMatrixX<max_dim_qp, max_dim_qp>::d& RtQuadraticProgram<max_dim_qp, max_num_eq, max_num_ineq>::objectiveQuadPart() const{return H_;};
template <int max_dim_qp, int max_num_eq, int max_num_ineq>
typename RtMatrixX<max_dim_qp, max_dim_qp>::d& RtQuadraticProgram<max_dim_qp, max_num_eq, max_num_ineq>::objectiveQuadPart() {return H_;};
template <int max_dim_qp, int max_num_eq, int max_num_ineq>
const typename RtVectorX<max_dim_qp>::d& RtQuadraticProgram<max_dim_qp, max_num_eq, max_num_ineq>::objectiveLinPart() const{return g_;};
template <int max_dim_qp, int max_num_eq, int max_num_ineq>
typename RtVectorX<max_dim_qp>::d& RtQuadraticProgram<max_dim_qp, max_num_eq, max_num_ineq>::objectiveLinPart() {return g_;};
template <int max_dim_qp, int max_num_eq, int max_num_ineq>
const typename RtMatrixX<max_num_eq, max_dim_qp>::d& RtQuadraticProgram<max_dim_qp, max_num_eq, max_num_ineq>::eqConstraintsMat() const{return Eq_mat_;};
template <int max_dim_qp, int max_num_eq, int max_num_ineq>
typename RtMatrixX<max_num_eq, max_dim_qp>::d& RtQuadraticProgram<max_dim_qp, max_num_eq, max_num_ineq>::eqConstraintsMat() {return Eq_mat_;};
template <int max_dim_qp, int max_num_eq, int max_num_ineq>
const typename RtVectorX<max_num_eq>::d& RtQuadraticProgram<max_dim_qp, max_num_eq, max_num_ineq>::eqConstraintsVec() const{return eq_vec_;};
template <int max_dim_qp, int max_num_eq, int max_num_ineq>
typename RtVectorX<max_num_eq>::d& RtQuadraticProgram<max_dim_qp, max_num_eq, max_num_ineq>::eqConstraintsVec() {return eq_vec_;};
template <int max_dim_qp, int max_num_eq, int max_num_ineq>
const typename RtMatrixX<max_num_ineq, max_dim_qp>::d& RtQuadraticProgram<max_dim_qp, max_num_eq, max_num_ineq>::ineqConstraintsMat() const{return Ineq_mat_;};
template <int max_dim_qp, int max_num_eq, int max_num_ineq>
typename RtMatrixX<max_num_ineq, max_dim_qp>::d& RtQuadraticProgram<max_dim_qp, max_num_eq, max_num_ineq>::ineqConstraintsMat() {return Ineq_mat_;};
template <int max_dim_qp, int max_num_eq, int max_num_ineq>
const typename RtVectorX<max_num_ineq>::d& RtQuadraticProgram<max_dim_qp, max_num_eq, max_num_ineq>::ineqConstraintsVec() const{return ineq_vec_;};
template <int max_dim_qp, int max_num_eq, int max_num_ineq>
typename RtVectorX<max_num_ineq>::d& RtQuadraticProgram<max_dim_qp, max_num_eq, max_num_ineq>::ineqConstraintsVec() {return ineq_vec_;};
template <int max_dim_qp, int max_num_eq, int max_num_ineq>
const typename RtVectorX<max_dim_qp>::d& RtQuadraticProgram<max_dim_qp, max_num_eq, max_num_ineq>::solution() const{return sol_;};
template <int max_dim_qp, int max_num_eq, int max_num_ineq>
typename RtVectorX<max_dim_qp>::d& RtQuadraticProgram<max_dim_qp, max_num_eq, max_num_ineq>::solution() {return sol_;};
template <int max_dim_qp, int max_num_eq, int max_num_ineq>
int RtQuadraticProgram<max_dim_qp, max_num_eq, max_num_ineq>::numVariables() const{return H_.rows();};
template <int max_dim_qp, int max_num_eq, int max_num_ineq>
int RtQuadraticProgram<max_dim_qp, max_num_eq, max_num_ineq>::numEqConstr() const{return Eq_mat_.rows();};
template <int max_dim_qp, int max_num_eq, int max_num_ineq>
int RtQuadraticProgram<max_dim_qp, max_num_eq, max_num_ineq>::numIneqConstr() const{return Ineq_mat_.rows();};


template <int max_dim_qp, int max_num_eq, int max_num_ineq>
RtQuadraticProgram<max_dim_qp, max_num_eq, max_num_ineq>::RtQuadraticProgram()
{
	reset();

#ifndef RTEIG_NO_ASSERT
	if(!ConfigUtils::setVarFromConfig(rtqp_collect_problems_, "RTQP_COLLECT_PROBLEMS"))
	  rtqp_collect_problems_ = 0;
	if(!ConfigUtils::setVarFromConfig(rtqp_collect_rate_, "RTQP_COLLECT_RATE"))
	  rtqp_collect_rate_ = 1;

	if(1 == rtqp_collect_problems_)
	{
          qp_fs_.resize(eQLT_SIZE_);
          for(int i=0; i<eQLT_SIZE_; ++i)
          {
            qp_fs_[i].reset(new FileSequence());
                  qp_fs_[i]->setFolder("qp_problem");
          }
          qp_fs_[eQLT_HESS_]->setFilePrefix("hessian_");
          qp_fs_[eQLT_LIN_PART_]->setFilePrefix("linear_part_");
          qp_fs_[eQLT_EQ_MAT_]->setFilePrefix("equality_mat_");
          qp_fs_[eQLT_EQ_VEC_]->setFilePrefix("equality_vec_");
          qp_fs_[eQLT_INEQ_MAT_]->setFilePrefix("inequality_mat_");
          qp_fs_[eQLT_INEQ_VEC_]->setFilePrefix("inequality_vec_");
          qp_fs_[eQLT_SOL_]->setFilePrefix("solution_");
	}
#endif
};


template <int max_dim_qp, int max_num_eq, int max_num_ineq>
void RtQuadraticProgram<max_dim_qp, max_num_eq, max_num_ineq>::printProblemCondition(std::ostream& stream)
{
  stream << "QP:" << std::endl;
  stream << "  num Variables: " << numVariables() << std::endl;
  stream << "  num Equalities: " << numEqConstr() << std::endl;
  stream << "  num Inequalities: " << numIneqConstr() << std::endl;

  Eigen::JacobiSVD<typename RtMatrixX<max_dim_qp, max_dim_qp>::d > Hess_svd;
  if(objectiveQuadPart().rows() > 0)
  {
    Hess_svd.compute(objectiveQuadPart());
    stream << "  Hessian condition: " << Hess_svd.singularValues().maxCoeff()/
        Hess_svd.singularValues().minCoeff() << " = " << Hess_svd.
        singularValues().maxCoeff() << " / " << Hess_svd.singularValues().minCoeff() << std::endl;
  }
  Eigen::JacobiSVD<typename RtMatrixX<max_num_eq, max_dim_qp>::d > EqConstr_svd;
  if(eqConstraintsMat().rows() > 0)
  {
    EqConstr_svd.compute(eqConstraintsMat());
    stream << "  Equality Matrix condition: " << EqConstr_svd.singularValues().maxCoeff()/
        EqConstr_svd.singularValues().minCoeff() << " = " << EqConstr_svd.
        singularValues().maxCoeff() << " / " << EqConstr_svd.singularValues().minCoeff() << std::endl;
  }
  Eigen::JacobiSVD<typename RtMatrixX<max_num_ineq, max_dim_qp>::d > IneqConstr_svd;
  if(ineqConstraintsMat().rows() > 0)
  {
    IneqConstr_svd.compute(ineqConstraintsMat());
    stream << "  Inequality Matrix condition: " << IneqConstr_svd.singularValues().maxCoeff()/
          IneqConstr_svd.singularValues().minCoeff() << " = " << IneqConstr_svd.
          singularValues().maxCoeff() << " / " << IneqConstr_svd.singularValues().minCoeff() << std::endl;
  }

  if(numVariables() < 10)
  {
    stream << "  Hessian: " << objectiveQuadPart() << std::endl;
    stream << "  linear part: " << objectiveLinPart().transpose() << std::endl;
    stream << "  Equality Matrix: " << eqConstraintsMat() << std::endl;
    stream << "  Equality Vector: " << eqConstraintsVec().transpose() << std::endl;
    stream << "  Inequality Matrix: " << ineqConstraintsMat() << std::endl;
    stream << "  Inequality Vector: " << ineqConstraintsVec().transpose() << std::endl;
  }
};


template <int max_dim_qp, int max_num_eq, int max_num_ineq>
RtQuadraticProgram<max_dim_qp, max_num_eq, max_num_ineq>::~RtQuadraticProgram(){};

template <int max_dim_qp, int max_num_eq, int max_num_ineq>
void RtQuadraticProgram<max_dim_qp, max_num_eq, max_num_ineq>::reset()
{
  qp_properties_ = 0;
  H_ = Eigen::Matrix<double, max_dim_qp, max_dim_qp>::Zero();
  g_ = Eigen::Matrix<double, max_dim_qp, 1>::Zero();
  sol_ = Eigen::Matrix<double, max_dim_qp, 1>::Zero();

  RtMatrixXUtils::setZero(Eq_mat_, 0, max_dim_qp);
  RtVectorXUtils::setZero(eq_vec_, 0);
  RtMatrixXUtils::setZero(Ineq_mat_, 0, max_dim_qp);
  RtVectorXUtils::setZero(ineq_vec_, 0);
};

template <int max_dim_qp, int max_num_eq, int max_num_ineq>
void RtQuadraticProgram<max_dim_qp, max_num_eq, max_num_ineq>::reset(int dim_qp, int num_eq, int num_ineq)
{
  qp_properties_ = 0;
  RtMatrixXUtils::setZero(H_, dim_qp, dim_qp);
  RtVectorXUtils::setZero(g_, dim_qp);
  RtMatrixXUtils::setZero(Eq_mat_, 0, dim_qp);
  RtVectorXUtils::setZero(eq_vec_, 0);
  RtMatrixXUtils::setZero(Ineq_mat_, 0, dim_qp);
  RtVectorXUtils::setZero(ineq_vec_, 0);
  RtVectorXUtils::setZero(sol_, dim_qp);
};

template <int max_dim_qp, int max_num_eq, int max_num_ineq>
bool RtQuadraticProgram<max_dim_qp, max_num_eq, max_num_ineq>::checkSolution(bool print,
      double eq_threashold, double ineq_threashold)
{
  const double eq_norm = (eqConstraintsMat() * solution() + eqConstraintsVec()).norm();
  typename RtVectorX<max_num_ineq>::d ineq_dist = ineqConstraintsMat() * solution() + ineqConstraintsVec();
  for(int i=0; i<numIneqConstr(); ++i)
    ineq_dist[i] *= ineq_dist[i] <= 0.0 ? 0.0 : 1.0;
  const double ineq_norm = (ineq_dist).norm();
  bool is_sol_valid = eq_norm <= eq_threashold && ineq_norm <= ineq_threashold;

  if(print || (!is_sol_valid && isSolved()))
  {
    std::cout << "qp check (num vars, num equalities, num inequalities, objective value): " <<
        numVariables() << " " << numEqConstr() << " " << numIneqConstr() << " " <<
        solution().transpose() * objectiveQuadPart() * solution() + 2*objectiveLinPart().transpose()*solution()<< std::endl;
    std::cout << "  equality squared dist:   " << eq_norm << std::endl;
    std::cout << "  inequality squared dist: " << ineq_norm << std::endl;
    std::cout << "  isSolved(): ";
    isSolved()? (std::cout <<"yes"):(std::cout <<"no");
    std::cout << std::endl;
  }

  return is_sol_valid;
}

template <int max_dim_qp, int max_num_eq, int max_num_ineq>
void RtQuadraticProgram<max_dim_qp, max_num_eq, max_num_ineq>::logQpProblem()
{
	for(int i=0; i<eQLT_SIZE_; ++i)
	{
		if(!qp_fs_[i]->openNextFile()) assert(false && "could not open file");
	}

	qp_fs_[eQLT_HESS_]->file_stream_ << objectiveQuadPart();
	qp_fs_[eQLT_LIN_PART_]->file_stream_ << objectiveLinPart();
	qp_fs_[eQLT_EQ_MAT_]->file_stream_ << eqConstraintsMat();
	qp_fs_[eQLT_EQ_VEC_]->file_stream_ << eqConstraintsVec();
	qp_fs_[eQLT_INEQ_MAT_]->file_stream_ << ineqConstraintsMat();
	qp_fs_[eQLT_INEQ_VEC_]->file_stream_ << ineqConstraintsVec();
	qp_fs_[eQLT_SOL_]->file_stream_ << solution();
}

} /* namespace floating_base_utilities */
#endif /* RTQUADRATICPROGRAM_H_ */
