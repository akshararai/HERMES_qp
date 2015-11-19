/*********************************************************************
 Autonomous Motion Department
 Max-Planck Intelligent Systems
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         JointSpaceCtrl.hh

 \author       Alexander Herzog
 \date         Aug 18, 2014

 *********************************************************************/

#pragma once

#include "KinematicsEigen.h"
#include "RtMatrixX.h"
#include "HierarchAffineCost.h"
#include "HierarchInverseDynamics.hh"
#include <SL_user.h>

namespace floating_base_utilities
{

/*!
 * @class JointSpaceCtrl
 * @brief represents a joint space controller controller of the form: weights_* (qdd - qdd_desired)
 */
template< int Max_Ineq_Rows, int Max_Eq_Rows>
class JointSpaceCtrl : public HierarchAffineCost
{
public:
  typedef HierarchAffineCost BaseClass;

  JointSpaceCtrl(HierarchInverseDynamics<Max_Ineq_Rows, Max_Eq_Rows>& opt_prob_composer);
  virtual ~JointSpaceCtrl(){};

  void initialize(int rank);

  void addCostToHierarchy(int rank) const;
  void update(const Eigen::Matrix<double, N_DOFS+6, 1>& des_qdd);
  void updateAfterSolutionFound();
  int maxRank() const
  {
#ifndef RTEIG_NO_ASSERTS
//    assert(rank_ > 0 && "Maximum possible rank has not been set!");
#endif
	return ranks_.maxCoeff();
  };

  const Eigen::Matrix<double, N_DOFS+6, N_DOFS+6>& weightingMat() const{return weight_;};
  Eigen::Matrix<double, N_DOFS+6, N_DOFS+6>& weightingMat() {return weight_;};

  const Eigen::Matrix<double, N_DOFS+6, N_DOFS+6>& matrix() const{return weight_;};
  const Eigen::Matrix<double, N_DOFS+6, 1>& vector() const{return vec_;};

  RtVectorX<N_DOFS+6>::i ranks_;
private:
  HierarchInverseDynamics<Max_Ineq_Rows, Max_Eq_Rows>* opt_prob_composer_;
  KinematicsEigen* kinematics_;
  Eigen::Matrix<double, N_DOFS+6, 1> vec_;
  Eigen::Matrix<double, N_DOFS+6, 1> slack_;
  Eigen::Matrix<double, N_DOFS+6, N_DOFS+6> weight_;
  std::string name_;
};

}  //namespace
