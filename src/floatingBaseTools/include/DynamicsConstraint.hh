/*********************************************************************
 Autonomous Motion Department
 Max-Planck Intelligent Systems
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         DynamicsConstraint.hh

 \author       Alexander Herzog
 \date         Jul 18, 2013

 *********************************************************************/

#pragma once

#include "KinematicsEigen.h"
#include "RtMatrixX.h"
#include "HierarchAffineCost.h"
#include "HierarchInverseDynamics.hh"
#include <SL_user.h>


namespace floating_base_utilities
{


template< int Max_Ineq_Rows, int Max_Eq_Rows, bool use_momentum_rate_mat=false>
class DynamicsConstraint : public HierarchAffineCost
{
public:
  typedef HierarchAffineCost BaseClass;

  DynamicsConstraint(HierarchInverseDynamics< Max_Ineq_Rows, Max_Eq_Rows>& opt_prob_composer);
  virtual ~DynamicsConstraint(){};

  void initialize(int rank = 0);

  void addCostToHierarchy(int rank) const;
  void updateAfterSolutionFound();
  int maxRank() const
  {
    return rank_;
  };

  void update();

  int rank_;
  const Eigen::Matrix<double, 6, N_DOFS+6+6*N_ENDEFFS>& matrix() const{return mat_;};
  const Eigen::Matrix<double, 6, 1>& vector() const{return vec_;};
private:
  HierarchInverseDynamics< Max_Ineq_Rows, Max_Eq_Rows>& opt_prob_composer_;
  KinematicsEigen& kinematics_;

  Eigen::Matrix<double, 6, N_DOFS+6+6*N_ENDEFFS> mat_;
  Eigen::Matrix<double, 6, 1> vec_;

  Eigen::Matrix<double, 6, 1> slack_;
  std::string name_;
};

}  //namespace
