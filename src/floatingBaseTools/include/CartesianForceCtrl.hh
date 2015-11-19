/*********************************************************************
 Autonomous Motion Department
 Max-Planck Intelligent Systems
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         CartesianForceCtrl.hh

 \author       Alexander Herzog
 \date         Jul 16, 2013

 *********************************************************************/

#ifndef CARTESIANFORCECTRL_HH_
#define CARTESIANFORCECTRL_HH_

#include "KinematicsEigen.h"
#include "RtMatrixX.h"
#include "HierarchAffineCost.h"
#include "HierarchInverseDynamics.hh"
#include <SL_user.h>

namespace floating_base_utilities
{

template< int Max_Ineq_Rows, int Max_Eq_Rows>
class CartesianForceCtrl : public HierarchAffineCost
{
public:
  typedef HierarchAffineCost BaseClass;

  enum ControlType
  {
    eCT_TRACK,
    eCT_MIN_FRC,
    eCT_MAX_FRC,
    eCT_FRC_SAT
  };
  CartesianForceCtrl(HierarchInverseDynamics<Max_Ineq_Rows, Max_Eq_Rows>& opt_prob_composer);
  virtual ~CartesianForceCtrl(){};

  /***
   * Initialize controller.
   * @param rank
   * @param endeff_id
   * @param in_local_frame Is the desired force (and weighting) expressed in
   * world frame or in local frame? CAUTION: The local frame is standardized as
   * described in <robot>User/config/floatingBaseTools.cf
   * @param ctrl_type
   */
  void initialize(int rank, int endeff_id, bool in_local_frame, ControlType ctrl_type = eCT_TRACK);

  void addCostToHierarchy(int rank) const;
  int maxRank() const
  {
#ifndef RTEIG_NO_ASSERTS
//    assert(rank_ > 0 && "Maximum possible rank has not been set!");
#endif
    return rank_;
  };

  void update(const Eigen::Matrix<double, 6, 1>& des_frc);
  void updateAfterSolutionFound();

  int endeffId() const{return endeff_id_;};
  const Eigen::Matrix<double, 6, 6>& weightingMat() const{return weight_;};
  Eigen::Matrix<double, 6, 6>& weightingMat() {return weight_;};
  const Eigen::Matrix<double, 6, 1>& saturation() const{return saturation_;};
  Eigen::Matrix<double, 6, 1>& saturation() {return saturation_;};

private:
  HierarchInverseDynamics<Max_Ineq_Rows, Max_Eq_Rows>* opt_prob_composer_;
  const KinematicsEigen* kinematics_;
  const ContactHelper* contact_helper_;
  ControlType control_type_;
  int ctrl_type_mat_sign_;
  int ctrl_type_vec_sign_;
  RtMatrixX<12,6>::d mat_;
  RtVectorX<12>::d vec_;
  Eigen::Matrix<double, 6, 1> slack_;
  Eigen::Matrix<double, 6, 1> saturation_;
  double slack_norm_;
  Eigen::Matrix<double, 6, 6> weight_;
  std::string name_;
  int rank_;
  int endeff_id_;
  bool in_local_frame_;
  Eigen::Matrix<double, 6, 6> world_local_trans_;
};

}  //namespace
#endif /* CARTESIANFORCECTRL_HH_ */
