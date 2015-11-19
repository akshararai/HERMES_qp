/*********************************************************************
 Autonomous Motion Department
 Max-Planck Intelligent Systems
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         FloatingBaseImpedanceCtrl.hh

 \author       Alexander Herzog
 \date         Jan 23, 2014

 *********************************************************************/

#ifndef FLOATINGBASEIMPEDANCECTRL_HH_
#define FLOATINGBASEIMPEDANCECTRL_HH_

#include "KinematicsEigen.h"
#include "CartesianPDCmd.h"
#include "RtMatrixX.h"
#include "HierarchAffineCost.h"
#include "HierarchInverseDynamics.hh"
#include <SL_user.h>

namespace floating_base_utilities
{

/**
 * represents an impedance controller
 **/
template< int Max_Ineq_Rows, int Max_Eq_Rows>
class FloatingBaseImpedanceCtrl : public HierarchAffineCost
{
public:
  typedef HierarchAffineCost BaseClass;

  FloatingBaseImpedanceCtrl(HierarchInverseDynamics<Max_Ineq_Rows, Max_Eq_Rows>& opt_prob_composer);
  virtual ~FloatingBaseImpedanceCtrl(){};

  void initialize(int rank);

  void addCostToHierarchy(int rank) const;
  void update(const bool* is_eff_constr);
  void updateAfterSolutionFound();
  int maxRank() const
  {
#ifndef RTEIG_NO_ASSERTS
//    assert(rank_ > 0 && "Maximum possible rank has not been set!");
#endif
    return rank_;
  };

//  int& linkId(){return link_id_;};
  const Eigen::Matrix<double, 6, 6>& weightingMat() const{return weight_;};
  Eigen::Matrix<double, 6, 6>& weightingMat() {return weight_;};

  const Eigen::Matrix<double,6, N_DOFS+6>& matrix() const{return mat_;};

  CartesianPDCmd impedance_cmds_[N_ENDEFFS+1];
private:
  HierarchInverseDynamics<Max_Ineq_Rows, Max_Eq_Rows>* opt_prob_composer_;
  KinematicsEigen* kinematics_;
  Eigen::Matrix<double,6, N_DOFS+6> mat_;
  Eigen::Matrix<double, 6, 1> vec_;
  Eigen::Matrix<double, 6, 1> slack_;
  Eigen::Matrix<double, 6, 6> weight_;
  std::string name_;
  int rank_;
};

}  //namespace
#endif /* FLOATINGBASEIMPEDANCECTRL_HH_ */
