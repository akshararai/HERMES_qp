/*********************************************************************
 Autonomous Motion Department
 Max-Planck Intelligent Systems
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         RtQuadraticProgram.hh

 \author       Alexander Herzog
 \date         Jul 13, 2013

 *********************************************************************/

#ifndef RTQUADRATICPROGRAM_HH_
#define RTQUADRATICPROGRAM_HH_

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "RtMatrixX.h"
#include "FileSequence.h"

namespace floating_base_utilities
{

template <int max_dim_qp, int max_num_eq, int max_num_ineq>
class RtQuadraticProgram
{
protected:
//  int dim_qp_, dim_eq_, dim_ineq_;
  typename RtMatrixX<max_dim_qp, max_dim_qp>::d H_;
  typename RtVectorX<max_dim_qp>::d g_;
  typename RtMatrixX<max_num_eq, max_dim_qp>::d Eq_mat_;
  typename RtVectorX<max_num_eq>::d eq_vec_;
  typename RtMatrixX<max_num_ineq, max_dim_qp>::d Ineq_mat_;
  typename RtVectorX<max_num_ineq>::d ineq_vec_;
  typename RtVectorX<max_dim_qp>::d sol_;

public:
  enum QPProperties{
    ePP_MightZeroIneqs = 1 << 0
//      ePP_ObjectivePD       = 1 << 1,
//      ePP_MightZeroIneqs    = 1 << 2
  //      ObjectivePD = 1 << 3,
    };
  int qp_properties_;

  /***!
   * Accessors
   */
  const typename RtMatrixX<max_dim_qp, max_dim_qp>::d& objectiveQuadPart() const;
  typename RtMatrixX<max_dim_qp, max_dim_qp>::d& objectiveQuadPart();
  const typename RtVectorX<max_dim_qp>::d& objectiveLinPart() const;
  typename RtVectorX<max_dim_qp>::d& objectiveLinPart();
  const typename RtMatrixX<max_num_eq, max_dim_qp>::d& eqConstraintsMat() const;
  typename RtMatrixX<max_num_eq, max_dim_qp>::d& eqConstraintsMat();
  const typename RtVectorX<max_num_eq>::d& eqConstraintsVec() const;
  typename RtVectorX<max_num_eq>::d& eqConstraintsVec();
  const typename RtMatrixX<max_num_ineq, max_dim_qp>::d& ineqConstraintsMat() const;
  typename RtMatrixX<max_num_ineq, max_dim_qp>::d& ineqConstraintsMat();
  const typename RtVectorX<max_num_ineq>::d& ineqConstraintsVec() const;
  typename RtVectorX<max_num_ineq>::d& ineqConstraintsVec();
  const typename RtVectorX<max_dim_qp>::d& solution() const;
  typename RtVectorX<max_dim_qp>::d& solution();
  int numVariables() const;
  int numEqConstr() const;
  int numIneqConstr() const;

  RtQuadraticProgram();

  virtual ~RtQuadraticProgram();

  bool virtual checkSolution(bool print = true, double eq_threashold = 0.001, double ineq_threashold = 0.001);

  bool virtual solve() =0;
  void virtual logQpProblem();
  bool virtual isSolved() const =0;

  inline virtual void reset();

  inline virtual void reset(int dim_qp, int num_eq, int num_ineq);

  void printProblemCondition(std::ostream& stream);

  /**!
   * Append equality constraints mat*x + vec = 0
   * @param mat
   * @param vec
   */
  template <typename Derived, typename OtherDerived>
  inline void appendEqualities(const Eigen::MatrixBase<Derived>& mat,
                      const Eigen::MatrixBase<OtherDerived>& vec)
  {
    RtAffineUtils::append(Eq_mat_, eq_vec_, mat, vec);
  };

  template <typename Derived>
  inline void appendEqualities(const Eigen::MatrixBase<Derived>& mat)
  {
    RtAffineUtils::append(Eq_mat_, eq_vec_, mat);
  };

  /**!
   * Append inequality constraints mat*x + vec <= 0
   * @param mat
   * @param vec
   */
  template <typename Derived, typename OtherDerived>
  inline void appendInequalities(const Eigen::MatrixBase<Derived>& mat,
                      const Eigen::MatrixBase<OtherDerived>& vec)
  {
    RtAffineUtils::append(Ineq_mat_, ineq_vec_, mat, vec);
  };

  template <typename Derived>
  inline void appendInequalities(const Eigen::MatrixBase<Derived>& mat)
  {
    RtAffineUtils::append(Ineq_mat_, ineq_vec_, mat);
  };


private:
  enum QpLogTypes {eQLT_HESS_=0, eQLT_LIN_PART_, eQLT_EQ_MAT_, eQLT_EQ_VEC_, eQLT_INEQ_MAT_, eQLT_INEQ_VEC_, eQLT_SOL_, eQLT_SIZE_};


  int rtqp_collect_problems_;
  int rtqp_collect_rate_;	
  std::vector<boost::shared_ptr<FileSequence> > qp_fs_;
};

}  //namespace
#endif /* RTQUADRATICPROGRAM_HH_ */
