/*********************************************************************
 Autonomous Motion Department
 Max-Planck Intelligent Systems
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         RtMinJerkGenerator.hh

 \author       Alexander Herzog
 \date         Jul 13, 2013

 *********************************************************************/

#ifndef RT_MIN_JERK_GENERATOR_HH_
#define RT_MIN_JERK_GENERATOR_HH_

#include <Eigen/Eigen>
#include "RtMatrixX.h"

namespace floating_base_utilities
{

template<int Num_Max_Coeffs>
class RtMinJerkGenerator
{
private:
  static const int num_max_coeffs_upper_bound_ = 3*Num_Max_Coeffs;

public:
  RtMinJerkGenerator();
  virtual ~RtMinJerkGenerator();

//  bool initialize(const Eigen::Vector2d& time, const Eigen::Vector2d& pos,
//                  const Eigen::Vector2d& vel, const Eigen::Vector2d& acc);

  template<typename PosConstMat, typename VelConstMat, typename AccConstMat>
  bool initialize(const Eigen::MatrixBase<PosConstMat>& pos_const,
                  const Eigen::MatrixBase<VelConstMat>& vel_const,
                  const Eigen::MatrixBase<AccConstMat>& acc_const);

  bool query(double time, double& pos, double &vel, double& acc);
  bool isInitialized() const{return initialized_;};

private:
  bool initialized_;

  Eigen::Vector2d time_;
//  Eigen::VectorXd coeffs_;
  typename RtMatrixX<num_max_coeffs_upper_bound_, 2>::d coeffs_;


  int num_coeffs_;
};

} /* namespace floating_base_utilities */
#endif /* RT_MIN_JERK_GENERATOR_HH_ */
