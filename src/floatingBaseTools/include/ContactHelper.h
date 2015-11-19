/*********************************************************************
 Autonomous Motion Department
 Max-Planck Intelligent Systems
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         ContactHelper.h

 \author       Alexander Herzog
 \date         Aug 23, 2013

 *********************************************************************/

#ifndef CONTACTHELPER_H_
#define CONTACTHELPER_H_

#include "floatingBaseToolsRobotInfo.h"
#include "RtMatrixX.h"
#include "floatingBaseToolsRobotInfo.h"
#include "KinematicsEigen.h"

namespace floating_base_utilities
{

class ContactHelper
{
public:
  static const int max_endeff_corners_=4;
  ContactHelper(std::string config_file);
  virtual ~ContactHelper(){};

  virtual void initialize(const KinematicsEigen* kinematics);
  virtual void update();
  virtual const RtMatrixX<6, floatingBaseToolsRobotInfo::max_contact_force_dim_>::d&
        getContactForceToMomentumRateMap(int endeff_id) const = 0;
//  virtual const Eigen::Matrix4d& stdToRobotEndeffFrame(int endeff_id) const = 0;
//  void standardizedEndeffPose(int id, Eigen::Matrix4d& pose) const;
  /** Get contact points that construct the support polygon. They are in local frame. **/
  virtual bool computeSupportPolygonCornersCCW(int endeff_id, RtMatrixX<3,max_endeff_corners_>::d& corners) const = 0;
  virtual const Eigen::Matrix<double, 6, 1>& getMeasuredForceAtEndeff(int endeff_id) const = 0;
  bool isEndeffValid(int endeff_id) const;
  bool isEndeffPoint(int endeff_id) const;
  bool isEndeffLine(int endeff_id) const;
  bool isEndeffFlat(int endeff_id) const;


  template<typename Mat, typename Vec>
  void constructMomrateMap(Eigen::MatrixBase<Mat>& mat,
      Eigen::MatrixBase<Vec>& vec) const;

  char config_file_[50];

  Eigen::Vector3d support_scale_;
  double friction_coefficient_;

  const floating_base_utilities::KinematicsEigen* kinematics_;
  RtMatrixX<3,max_endeff_corners_>::d support_polygon_corners_[N_ENDEFFS+1];
  Eigen::Vector3d sup_pol_normals_[N_ENDEFFS+1];
  Eigen::Vector3d endeff_pos_to_sup_polyg_projection_[N_ENDEFFS+1];

  bool getNormalDistOfSuppPol(int endeff_id, Eigen::Vector3d& normal, double& dist) const;

  RtMatrixX<3,max_endeff_corners_>::d  projected_sup_pol_corners_[N_ENDEFFS+1];
  double projected_sup_pol_dists_[N_ENDEFFS+1];
  Eigen::Matrix4d projected_foot_frames_[N_ENDEFFS+1];
  bool getGravityProjectedFootPrint(int endeff_id,
      RtMatrixX<3,max_endeff_corners_>::d& corners, Eigen::Matrix4d& projected_foot_frame) const;
  void supportInteriorInequalities(const RtMatrixX<3,max_endeff_corners_>::d& corners,
  		RtMatrixX<max_endeff_corners_,3>::d& normals, RtVectorX<max_endeff_corners_>::d& dists) const;

  void getCoPInequality(int endeff_id, RtMatrixX<max_endeff_corners_,6>::d& mat) const;
  void getFrictionInequality(int endeff_id, Eigen::Matrix<double, 4,6>& mat) const;

  void getCoPInequalityProjected(int endeff_id, RtMatrixX<max_endeff_corners_,6>::d& mat) const;
  void getFrictionInequalityProjected(int endeff_id, Eigen::Matrix<double, 4,6>& mat) const;

  void computeEndeffCoP(int eff_i, const Eigen::Matrix<double, 6, 1>& generalized_forces_world,
        const Eigen::Matrix4d& footprint_to_world_frame, Eigen::Vector3d& cop_local) const;
  void computeEndeffCoP(int eff_i, const Eigen::Matrix<double, 6, 1>& generalized_forces_world,
        Eigen::Vector3d& cop_local) const;

  void computeCOPTorque(int eff_i,
        const Eigen::Matrix<double, 2*N_CART, 1>& generalized_forces_world,
        const Eigen::Matrix4d& footprint_to_world_frame, double& moment) const;
  void computeCOPTorque(int eff_i,
        const Eigen::Matrix<double, 2*N_CART, 1>& generalized_forces_world,
        double& moment) const;

  void computeOverallCoP(const Eigen::Matrix<double, 6, 1>& generalized_forces_world_L,
         const Eigen::Matrix<double, 6, 1>& generalized_forces_world_R, Eigen::Vector3d& cop_local) const;

};


template<typename Mat, typename Vec>
void ContactHelper::constructMomrateMap(
    Eigen::MatrixBase<Mat>& mat, Eigen::MatrixBase<Vec>& vec) const
{
  RtVectorXUtils::setZero(vec, 6);
  vec(2) = -kinematics_->robotMass()*G;

  for (int eff_i = 1; eff_i <= N_ENDEFFS; ++eff_i)
  {
    mat.block(0, floatingBaseToolsRobotInfo::max_contact_force_dim_*(eff_i-1), 6,floatingBaseToolsRobotInfo::max_contact_force_dim_) =
        getContactForceToMomentumRateMap(eff_i);
  }
}

} /* namespace floating_base_utilities */
#endif /* CONTACTHELPER_H_ */
