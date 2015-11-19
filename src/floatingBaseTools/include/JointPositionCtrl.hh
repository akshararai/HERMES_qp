/*
 * JointPositionCtrl.hh
 *
 *  Created on: Aug 15, 2014
 *      Author: righetti
 */

#ifndef JOINTPOSITIONCTRL_HH_
#define JOINTPOSITIONCTRL_HH_

#include "KinematicsEigen.h"
#include "RtMatrixX.h"
#include "HierarchAffineCost.h"
#include "HierarchInverseDynamics.hh"
#include <SL_user.h>

namespace floating_base_utilities
{

/**
 * represents a position controller of the form: weights_* (Jacobian * qdd + derived_Jacobian*qd - qdd_desired)
 */
template< int Max_Ineq_Rows, int Max_Eq_Rows, int Num_Joints>
class JointPositionCtrl : public HierarchAffineCost
{
public:
  typedef HierarchAffineCost BaseClass;

  JointPositionCtrl(HierarchInverseDynamics<Max_Ineq_Rows, Max_Eq_Rows>& opt_prob_composer);
  virtual ~JointPositionCtrl(){};

  void initialize(int rank, int starting_joint, std::string ctrl_name=std::string("JPC"));

  void addCostToHierarchy(int rank) const;
  void update(const Eigen::Matrix<double, Num_Joints, 1>& ref_pos,
              const Eigen::Matrix<double, Num_Joints, 1>& ref_vel,
              const Eigen::Matrix<double, Num_Joints, 1>& ref_acc);
  void updateAfterSolutionFound();
  int maxRank() const
  {
#ifndef RTEIG_NO_ASSERTS
//    assert(rank_ > 0 && "Maximum possible rank has not been set!");
#endif
	return ranks_.maxCoeff();
  };

  const Eigen::Matrix<double, Num_Joints, 1>& weightingMat() const{return weight_;};
  Eigen::Matrix<double, Num_Joints, 1>& weightingMat() {return weight_;};

  const typename Eigen::Matrix<double, Num_Joints, Num_Joints>& matrix() const{return mat_;};
  const typename Eigen::Matrix<double, Num_Joints, 1>& vector() const{return vec_;};

  typename RtVectorX<Num_Joints>::i ranks_;

  Eigen::Matrix<double, Num_Joints, 1> P_gains_, D_gains_;
  Eigen::Matrix<double, Num_Joints, Num_Joints> variable_gain_;
  int starting_joint_;

private:
  HierarchInverseDynamics<Max_Ineq_Rows, Max_Eq_Rows>* opt_prob_composer_;
  KinematicsEigen* kinematics_;
  typename Eigen::Matrix<double, Num_Joints, Num_Joints> mat_;
  Eigen::Matrix<double, Num_Joints, 1> vec_;
  Eigen::Matrix<double, Num_Joints, 1> slack_;
  Eigen::Matrix<double, Num_Joints, 1> weight_;
  Eigen::Matrix<double, Num_Joints, 1> des_acc_;
  std::string name_;
};

}  //namespace
#endif /* JointPOSITIONCTRL_HH_ */
