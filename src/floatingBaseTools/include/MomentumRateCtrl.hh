/*********************************************************************
 Autonomous Motion Department
 Max-Planck Intelligent Systems
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         MomentumRateCtrl.hh

 \author       Alexander Herzog
 \date         Jul 18, 2013

 *********************************************************************/

#ifndef MOMENTUMRATECTRL_HH_
#define MOMENTUMRATECTRL_HH_

#include "KinematicsEigen.h"
#include "RtMatrixX.h"
#include "HierarchAffineCost.h"
#include "HierarchInverseDynamics.hh"
#include <SL_user.h>


namespace floating_base_utilities
{


template< int Max_Ineq_Rows, int Max_Eq_Rows, bool use_momentum_rate_mat=false>
class MomentumRateCtrl : public HierarchAffineCost
{
public:
  static const unsigned int num_variables_ = (use_momentum_rate_mat)?N_DOFS+6:floatingBaseToolsRobotInfo::max_contact_force_dim_ * N_ENDEFFS;
  typedef HierarchAffineCost BaseClass;

  MomentumRateCtrl(HierarchInverseDynamics< Max_Ineq_Rows, Max_Eq_Rows>& opt_prob_composer);
  virtual ~MomentumRateCtrl(){};

  void initialize(int rank);

  void addCostToHierarchy(int rank) const;
  void updateAfterSolutionFound();
  int maxRank() const
  {
#ifndef RTEIG_NO_ASSERTS
//    assert(ranks_.minCoeff() >= 0 && "Maximum possible rank has not been set!");
#endif
    return ranks_.maxCoeff();
  };

  void update(const Eigen::Matrix<double, 6, 1>& des_mom_rate);

  const Eigen::Matrix<double, 6, 6>& weightingMat() const{return weight_;};
  Eigen::Matrix<double, 6, 6>& weightingMat() {return weight_;};

  RtVectorX<6>::i ranks_;
  const typename RtMatrixX<6, num_variables_>::d& matrix() const{return mat_;};
  const typename Eigen::Matrix<double, 6, 1>& vector() const{return vec_;};
private:
  HierarchInverseDynamics< Max_Ineq_Rows, Max_Eq_Rows>* opt_prob_composer_;
  const KinematicsEigen* kinematics_;
//#ifdef DONT_USE_MOMENTUM_MAT
  typename RtMatrixX<6, num_variables_>::d mat_;
//#else
  const MomentumComputation* momentum_comp_;
//  typename Eigen::Matrix<double, 6, N_DOFS+6> mat_;
//#endif
  Eigen::Matrix<double, 6, 1> vec_;
  Eigen::Matrix<double, 6, 1> slack_;
  Eigen::Matrix<double, 6, 6> weight_;
  std::string name_;
};

}  //namespace
#endif
