/*********************************************************************
 Autonomous Motion Department
 Max-Planck Intelligent Systems
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         CartesianPositionCtrl.hh

 \author       Alexander Herzog
 \date         Jul 16, 2013

 *********************************************************************/

#ifndef CARTESIANPOSITIONCTRL_HH_
#define CARTESIANPOSITIONCTRL_HH_

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
template< int Max_Ineq_Rows, int Max_Eq_Rows>
class CartesianPositionCtrl : public HierarchAffineCost
{
public:
  typedef HierarchAffineCost BaseClass;

  CartesianPositionCtrl(HierarchInverseDynamics<Max_Ineq_Rows, Max_Eq_Rows>& opt_prob_composer);
  virtual ~CartesianPositionCtrl(){};

  void initialize(int rank, int link_id);

  void addCostToHierarchy(int rank) const;
  void update(const Eigen::Matrix<double, 6, 1>& des_acc);
  void updateAfterSolutionFound();
  int maxRank() const
  {
#ifndef RTEIG_NO_ASSERTS
//    assert(rank_ > 0 && "Maximum possible rank has not been set!");
#endif
	return ranks_.maxCoeff();
  };

  int linkId() const{return link_id_;};
//  int& linkId(){return link_id_;};
  const Eigen::Matrix<double, 6, 6>& weightingMat() const{return weight_;};
  Eigen::Matrix<double, 6, 6>& weightingMat() {return weight_;};

  const typename Eigen::Matrix<double, 6, N_DOFS+6>& matrix() const{return mat_;};
  const typename Eigen::Matrix<double, 6, 1>& vector() const{return vec_;};

  RtVectorX<6>::i ranks_;
private:
  HierarchInverseDynamics<Max_Ineq_Rows, Max_Eq_Rows>* opt_prob_composer_;
  KinematicsEigen* kinematics_;
  typename Eigen::Matrix<double, 6, N_DOFS+6> mat_;
  Eigen::Matrix<double, 6, 1> vec_;
  Eigen::Matrix<double, 6, 1> slack_;
  Eigen::Matrix<double, 6, 6> weight_;
  std::string name_;
  int link_id_;
};

}  //namespace
#endif /* CARTESIANPOSITIONCTRL_HH_ */
