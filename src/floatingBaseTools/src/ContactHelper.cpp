/*********************************************************************
 Autonomous Motion Department
 Max-Planck Intelligent Systems
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         ContactHelper.cpp

 \author       Alexander Herzog
 \date         Aug 23, 2013

 *********************************************************************/

#include <iostream>
#include <Eigen/Geometry>
#include "ContactHelper.h"
#include "GeometryUtils.h"
#include "SL_common.h"

#define N_ENDEFFS_2 1

namespace floating_base_utilities
{

ContactHelper::ContactHelper(std::string config_file) : kinematics_(NULL)
{
  sprintf(config_file_, "%s", config_file.c_str());
  for(int e=N_ENDEFFS_2; e<=N_ENDEFFS; ++e)
  {
    sup_pol_normals_[e].setZero();
    projected_sup_pol_dists_[e] = 0.0;
  }
}

void ContactHelper::initialize(const KinematicsEigen* kinematics)
{
  kinematics_ = kinematics;

  if(!read_parameter_pool_double(config_file_, "FRICTION_COEFICIENT", &friction_coefficient_))
          assert(false && "reading parameters failed");
  if(!read_parameter_pool_double_array(config_file_, "FOOT_SURFACE_SCALE", support_scale_.size(), &support_scale_.data()[-1]))
  {
    support_scale_.setOnes();
    assert(false && "reading parameters failed");
  }

  for (unsigned int e=N_ENDEFFS_2; e<= N_ENDEFFS; ++e)
  {
    if(computeSupportPolygonCornersCCW(e, support_polygon_corners_[e]))
    {
      Eigen::Vector3d cntr = support_polygon_corners_[e].rowwise().sum();
      cntr /= support_polygon_corners_[e].cols();
      RtMatrixX<3,max_endeff_corners_>::d scaled_offsets = support_polygon_corners_[e].colwise() - cntr;
      scaled_offsets = support_scale_.asDiagonal()*scaled_offsets;
      support_polygon_corners_[e] = scaled_offsets.colwise() + cntr;
    }
    else
    {
      RtMatrixXUtils::resize(support_polygon_corners_[e], 3, 0);
    }


    if(isEndeffPoint(e))
    {
      endeff_pos_to_sup_polyg_projection_[e] = support_polygon_corners_[e].col(0);
    }
    else if(isEndeffLine(e))
    {
      Eigen::Vector3d corner0_ankle = -support_polygon_corners_[e].col(0);
      Eigen::Vector3d corner01_normed = (support_polygon_corners_[e].col(1) - support_polygon_corners_[e].col(0));
      corner01_normed.normalize();
      endeff_pos_to_sup_polyg_projection_[e] = support_polygon_corners_[e].col(0) + corner01_normed * corner01_normed.dot(corner0_ankle);
    }
    else if(isEndeffFlat(e))
    {
      // compute representation of support polygon
      Eigen::Vector3d normal;
      double dist;
      getNormalDistOfSuppPol(e, normal, dist);

      // project ankle on support polygon
      endeff_pos_to_sup_polyg_projection_[e] = normal.dot(support_polygon_corners_[e].col(0)) * normal;
    }
  }
}

void ContactHelper::update()
{
  for(int e=N_ENDEFFS_2; e<=N_ENDEFFS; ++e)
  {
    if(!getNormalDistOfSuppPol(e, sup_pol_normals_[e], projected_sup_pol_dists_[e]))
      assert(false && "Could not get Normal,Distance representation of support polygon");

    getGravityProjectedFootPrint(e, projected_sup_pol_corners_[e],
        projected_foot_frames_[e]);
  }
}

bool ContactHelper::getGravityProjectedFootPrint(int endeff_id,
    RtMatrixX<3,max_endeff_corners_>::d& corners, Eigen::Matrix4d& projected_foot_frame) const
{
  if(!isEndeffFlat(endeff_id))
    return false;

  Eigen::Vector3d up_dir = -kinematics_->standardizedEndeffPose(endeff_id).block<1,3>(2,0).transpose();
  const double ankle_sole_dist = endeff_pos_to_sup_polyg_projection_[endeff_id].norm();
  Eigen::Vector3d new_ankle_proj = ankle_sole_dist*up_dir*(up_dir.dot(endeff_pos_to_sup_polyg_projection_[endeff_id])>= 0?1.0:-1.0);

  // project support polygon corners
  RtMatrixXUtils::resize(corners, support_polygon_corners_[endeff_id].rows(), support_polygon_corners_[endeff_id].cols());
  for(int i=0; i<support_polygon_corners_[endeff_id].cols(); ++i)
  {
    corners.col(i) = support_polygon_corners_[endeff_id].col(i)+up_dir *
        up_dir.dot(new_ankle_proj-support_polygon_corners_[endeff_id].col(i));
  }

  // compute frame of projected footprint
  Eigen::Matrix4d proj_to_std_endeff_frame = Eigen::Matrix4d::Identity();
  proj_to_std_endeff_frame.topRightCorner<3,1>() = endeff_pos_to_sup_polyg_projection_[endeff_id];
  proj_to_std_endeff_frame.block<3,1>(0,1) = up_dir;
  proj_to_std_endeff_frame.block<3,1>(0,0) = Eigen::Vector3d::UnitX() -
      up_dir * up_dir[0];
  proj_to_std_endeff_frame.block<3,1>(0,0).normalize();
  proj_to_std_endeff_frame.block<3,1>(0,2) = Eigen::Vector3d::UnitZ() -
      up_dir * up_dir[2];
  proj_to_std_endeff_frame.block<3,1>(0,2).normalize();

  // now x,z are not orthogonal to each other
  proj_to_std_endeff_frame.block<3,1>(0,0) = Eigen::AngleAxisd(0.25*M_PI, up_dir).matrix() *
		  (proj_to_std_endeff_frame.block<3,1>(0,0)+ proj_to_std_endeff_frame.block<3,1>(0,2));
  proj_to_std_endeff_frame.block<3,1>(0,0).normalize();
  proj_to_std_endeff_frame.block<3,1>(0,2) = proj_to_std_endeff_frame.block<3,1>(0,0).cross(up_dir);

  // transform corners into that frame
  Eigen::Matrix4d proj_to_std_endeff_frame_inv = proj_to_std_endeff_frame;
  GeometryUtils::invertEigenTransform(proj_to_std_endeff_frame_inv);
  corners = proj_to_std_endeff_frame_inv.topLeftCorner<3,3>() *
		  corners;
  corners = corners.colwise() + proj_to_std_endeff_frame_inv.topRightCorner<3,1>();

  //transform through standardized endeff frame
  Eigen::Matrix4d std_endeff_frame;
  kinematics_->standardizedEndeffPose(endeff_id, std_endeff_frame);
  projected_foot_frame = std_endeff_frame*proj_to_std_endeff_frame;

  return sup_pol_normals_[endeff_id].dot(up_dir) < std::cos(M_PI/180*89);
}

bool ContactHelper::getNormalDistOfSuppPol(int endeff_id, Eigen::Vector3d& normal, double& dist) const
{
  if(!isEndeffFlat(endeff_id))
    return false;

  Eigen::Vector3d corner0_ankle = -support_polygon_corners_[endeff_id].col(0);
  Eigen::Vector3d corner01 = (support_polygon_corners_[endeff_id].col(1) - support_polygon_corners_[endeff_id].col(0));
  Eigen::Vector3d corner02 = (support_polygon_corners_[endeff_id].col(2) - support_polygon_corners_[endeff_id].col(0));
  normal = corner01.cross(corner02);
  normal.normalize();
  dist = -normal.dot(support_polygon_corners_[endeff_id].col(0));

  return true;
}

/** generates an affine mapping, s.t. Ax+a<= 0 iff x in support polygon**/
void ContactHelper::supportInteriorInequalities(const RtMatrixX<3,max_endeff_corners_>::d& corners,
		RtMatrixX<max_endeff_corners_,3>::d& normals, RtVectorX<max_endeff_corners_>::d& dists) const
{
	RtMatrixXUtils::resize(normals, corners.cols(), corners.rows());
	RtVectorXUtils::resize(dists, corners.cols());
	for(unsigned int i = 0; i<corners.cols(); ++i)
	{
		Eigen::Vector3d c01 = corners.col((i+1)%corners.cols()) - corners.col(i);
		Eigen::Vector3d c02 = corners.col((i+2)%corners.cols()) - corners.col(i);
		normals.row(i) = (c02).cross( c01 ).cross(c01).transpose();
		normals.row(i).normalize();
                dists[i] = -normals.row(i)*corners.col(i);
//                dists[i] = -normals.row(i)*corners.rowwise().sum();

	}
}

// mat * lambda <= 0 iff the cop is inside of the projected footprint
void ContactHelper::getCoPInequalityProjected(int endeff_id, RtMatrixX<max_endeff_corners_,6>::d& mat) const
{
	Eigen::Vector3d ankle_pos = -endeff_pos_to_sup_polyg_projection_[endeff_id];
	const RtMatrixX<3,max_endeff_corners_>::d& sup_corners = projected_sup_pol_corners_[endeff_id];
	const Eigen::Matrix4d& sup_frame = projected_foot_frames_[endeff_id];
//        Eigen::Vector3d ankle_pos = -endeff_pos_to_sup_polyg_projection_[endeff_id];
//        const RtMatrixX<3,max_endeff_corners_>::d& sup_corners = support_polygon_corners_[endeff_id];
//        Eigen::Matrix4d sup_frame = kinematics_->endeffPose(endeff_id)*stdToRobotEndeffFrame(endeff_id);

	RtMatrixX<max_endeff_corners_,3>::d support_ineq_mat;
	RtVectorX<max_endeff_corners_>::d support_ineq_vec;
	supportInteriorInequalities(sup_corners, support_ineq_mat, support_ineq_vec);
	RtMatrixXUtils::resize(mat,support_ineq_mat.rows(), 6);
	Eigen::Matrix3d selector1;
	selector1 << 0, 0, -1, 0, 0, 0, 1, 0, 0;
	Eigen::Vector3d selector2;
	selector2 << 0, -1, 0;
	mat.leftCols(3) = (support_ineq_vec*selector2.transpose() - support_ineq_mat*
	    selector1*GeometryUtils::vector_to_skew_matrix(ankle_pos))*sup_frame.
	    topLeftCorner<3,3>().transpose();
	mat.rightCols(3) = support_ineq_mat*selector1*sup_frame.
	 topLeftCorner<3,3>().transpose();

}

// mat * lambda <= 0 iff the cop is inside of the projected footprint
void ContactHelper::getCoPInequality(int endeff_id, RtMatrixX<max_endeff_corners_,6>::d& mat) const
{
//  Eigen::Vector3d ankle_pos = -endeff_pos_to_sup_polyg_projection_[endeff_id];
//  const RtMatrixX<3,max_endeff_corners_>::d& sup_corners = projected_sup_pol_corners_[endeff_id];
//  const Eigen::Matrix4d& sup_frame = projected_foot_frames_[endeff_id];
    Eigen::Vector3d ankle_pos = -endeff_pos_to_sup_polyg_projection_[endeff_id];
    const RtMatrixX<3,max_endeff_corners_>::d& sup_corners = support_polygon_corners_[endeff_id];
    Eigen::Matrix4d sup_frame = kinematics_->standardizedEndeffPose(endeff_id);

  RtMatrixX<max_endeff_corners_,3>::d support_ineq_mat;
  RtVectorX<max_endeff_corners_>::d support_ineq_vec;
  supportInteriorInequalities(sup_corners, support_ineq_mat, support_ineq_vec);
  RtMatrixXUtils::resize(mat,support_ineq_mat.rows(), 6);
  Eigen::Matrix3d selector1;
  selector1 << 0, 0, -1, 0, 0, 0, 1, 0, 0;
  Eigen::Vector3d selector2;
  selector2 << 0, -1, 0;
  mat.leftCols(3) = (support_ineq_vec*selector2.transpose() - support_ineq_mat*
      selector1*GeometryUtils::vector_to_skew_matrix(ankle_pos))*sup_frame.
      topLeftCorner<3,3>().transpose();
  mat.rightCols(3) = support_ineq_mat*selector1*sup_frame.
      topLeftCorner<3,3>().transpose();

}

// mat * lambda <= 0 iff forces are inside of the friction pyramide
void ContactHelper::getFrictionInequalityProjected(int endeff_id, Eigen::Matrix<double, 4,6>& mat) const
{
  const Eigen::Matrix4d& sup_frame = projected_foot_frames_[endeff_id];
//  Eigen::Matrix4d sup_frame = kinematics_->endeffPose(endeff_id)*stdToRobotEndeffFrame(endeff_id);

  const double normal_component = 1 / std::sqrt(friction_coefficient_*friction_coefficient_ + 1);
  const double tangntial_component = friction_coefficient_ / normal_component;

  Eigen::Matrix<double, 4, 3> friction_base;

  friction_base << tangntial_component, normal_component,                   0.0,
                                   0.0, normal_component,   tangntial_component,
                  -tangntial_component, normal_component,                   0.0,
                                   0.0, normal_component,  -tangntial_component;

  mat.rightCols(3).setZero();
  mat.leftCols(3) = friction_base * sup_frame.topLeftCorner<3,3>().transpose();
}

// mat * lambda <= 0 iff forces are inside of the friction pyramide
void ContactHelper::getFrictionInequality(int endeff_id, Eigen::Matrix<double, 4,6>& mat) const
{
  Eigen::Matrix4d std_endeff_frame;
  kinematics_->standardizedEndeffPose(endeff_id, std_endeff_frame);
//  Eigen::Matrix4d sup_frame = kinematics_->endeffPose(endeff_id)*stdToRobotEndeffFrame(endeff_id);

  const double normal_component = 1 / std::sqrt(friction_coefficient_*friction_coefficient_ + 1);
  const double tangntial_component = friction_coefficient_ / normal_component;

  Eigen::Matrix<double, 4, 3> friction_base;

  friction_base << tangntial_component, normal_component,                   0.0,
                                   0.0, normal_component,   tangntial_component,
                  -tangntial_component, normal_component,                   0.0,
                                   0.0, normal_component,  -tangntial_component;

  mat.rightCols(3).setZero();
  mat.leftCols(3) = friction_base * std_endeff_frame.topLeftCorner<3,3>().transpose();
}

/** footprint_to_world_frame needs to be standardized (y axis is normal to support polygon)**/
void ContactHelper::computeEndeffCoP(int eff_i, const Eigen::Matrix<double, 2*N_CART, 1>& generalized_forces_world,
                                                   const Eigen::Matrix4d& footprint_to_world_frame, Eigen::Vector3d& cop_local) const
{
  assert(isEndeffFlat(eff_i));
  Eigen::Matrix4d foot_sole_frame_inv = footprint_to_world_frame;
  GeometryUtils::invertEigenTransform(foot_sole_frame_inv);
  Eigen::Matrix<double, 3, 1> ankle_pos_foot = foot_sole_frame_inv.topRows(3) * kinematics_->endeffPose(eff_i).rightCols(1); // ankle position relative to footprint position in footprint frame

  Eigen::Matrix<double, 6, 1> gen_frcs_foot;
  gen_frcs_foot.block<3,1>(0,0) = foot_sole_frame_inv.topLeftCorner<3,3>() * generalized_forces_world.block<3,1>(0,0); // rotate wrench into footprint frame for moment computation below
  gen_frcs_foot.block<3,1>(3,0) = foot_sole_frame_inv.topLeftCorner<3,3>() * generalized_forces_world.block<3,1>(3,0);

  Eigen::Matrix<double, 3, 1> ank_mom = Eigen::Matrix<double, 3, 1>::Zero();
  ank_mom = ankle_pos_foot.cross(gen_frcs_foot.block<3,1>(0,0)); // moment due to ankle forces expressed about origin of footprint frame

  cop_local[0] = (gen_frcs_foot[5] + ank_mom[2])/gen_frcs_foot[1];
  cop_local[1] = 0.0;
  cop_local[2] = -(gen_frcs_foot[3] + ank_mom[0])/gen_frcs_foot[1];

}

void ContactHelper::computeEndeffCoP(int eff_i, const Eigen::Matrix<double, 6, 1>& generalized_forces_world,
        Eigen::Vector3d& cop_local) const
{
  computeEndeffCoP(eff_i, generalized_forces_world,
      kinematics_->standardizedEndeffPose(eff_i), cop_local);
};

void ContactHelper::computeCOPTorque(int eff_i,
      const Eigen::Matrix<double, 2*N_CART, 1>& generalized_forces_world,
      const Eigen::Matrix4d& footprint_to_world_frame, double& moment) const
{
  assert(isEndeffFlat(eff_i));
  Eigen::Matrix4d foot_sole_frame_inv = footprint_to_world_frame;
  GeometryUtils::invertEigenTransform(foot_sole_frame_inv);
  Eigen::Matrix<double, 3, 1> ankle_pos_foot = foot_sole_frame_inv.topRows(3) * kinematics_->endeffPose(eff_i).rightCols(1);

  Eigen::Matrix<double, 3, 3> a_cross_rot_trans = GeometryUtils::vector_to_skew_matrix(ankle_pos_foot);
  a_cross_rot_trans = a_cross_rot_trans*foot_sole_frame_inv.block(0,0,3,3);

  Eigen::Matrix<double, 6, 3> lambda_quad = Eigen::Matrix<double, 6, 3>::Zero();
  lambda_quad.block(3,0,3,3) = Eigen::Matrix<double, 3, 3>::Identity();
  for(unsigned int i=0; i<3; ++i)
  {
    lambda_quad.block(0,0,3,3) += a_cross_rot_trans.block(i,0,1,3).transpose() * foot_sole_frame_inv.block(i,0,1,3);
  }

  moment = (generalized_forces_world.transpose() * lambda_quad *generalized_forces_world.block(0,0,3,1))[0];
  moment /= (foot_sole_frame_inv.block(1,0,1,3) * generalized_forces_world.block(0,0,3,1))[0];
}

void ContactHelper::computeCOPTorque(int eff_i,
      const Eigen::Matrix<double, 2*N_CART, 1>& generalized_forces_world,
      double& moment) const
{
  computeCOPTorque(eff_i, generalized_forces_world,
      kinematics_->standardizedEndeffPose(eff_i), moment);
}

  void ContactHelper::computeOverallCoP(const Eigen::Matrix<double, 2*N_CART, 1>& generalized_forces_world_L, const Eigen::Matrix<double, 2*N_CART, 1>& generalized_forces_world_R, Eigen::Vector3d& cop_local) const
{
  assert(isEndeffFlat(LEFT_FOOT)&&isEndeffFlat(RIGHT_FOOT));

  Eigen::Matrix4d foot_sole_frame_inv_L = kinematics_->standardizedEndeffPose(LEFT_FOOT);
  GeometryUtils::invertEigenTransform(foot_sole_frame_inv_L);

  Eigen::Matrix4d foot_sole_frame_inv_R = kinematics_->standardizedEndeffPose(RIGHT_FOOT);
  GeometryUtils::invertEigenTransform(foot_sole_frame_inv_R);

  Eigen::Vector3d ankle_pos_foot_L = foot_sole_frame_inv_L.topRows(3) * kinematics_->endeffPose(LEFT_FOOT).rightCols(1); // ankle position relative to footprint position in footprint frame
  Eigen::Vector3d ankle_pos_foot_R = foot_sole_frame_inv_R.topRows(3) * kinematics_->endeffPose(RIGHT_FOOT).rightCols(1); // ankle position relative to footprint position in footprint frame

  Eigen::Matrix<double,6,1> gen_frcs_foot_L = Eigen::Matrix<double,6,1>::Zero(); // measured wrench rotated into footprint frame
  gen_frcs_foot_L.block<3,1>(0,0) = foot_sole_frame_inv_L.topLeftCorner<3,3>() * generalized_forces_world_L.block<3,1>(0,0); // rotate wrench into footprint frame for moment computation below
  gen_frcs_foot_L.block<3,1>(3,0) = foot_sole_frame_inv_L.topLeftCorner<3,3>() * generalized_forces_world_L.block<3,1>(3,0);
  Eigen::Matrix<double,6,1> gen_frcs_foot_R = Eigen::Matrix<double,6,1>::Zero(); // measured wrench rotated into footprint frame
  gen_frcs_foot_R.block<3,1>(0,0) = foot_sole_frame_inv_R.topLeftCorner<3,3>() * generalized_forces_world_R.block<3,1>(0,0); // rotate wrench into footprint frame for moment computation below
  gen_frcs_foot_R.block<3,1>(3,0) = foot_sole_frame_inv_R.topLeftCorner<3,3>() * generalized_forces_world_R.block<3,1>(3,0);

  Eigen::Vector3d ank_mom_L = ankle_pos_foot_L.cross(gen_frcs_foot_L.block<3,1>(0,0)); // moment due to ankle forces expressed about origin of footprint frame
  Eigen::Vector3d ank_mom_R = ankle_pos_foot_R.cross(gen_frcs_foot_R.block<3,1>(0,0)); // moment due to ankle forces expressed about origin of footprint frame

  Eigen::Matrix4d frame_R_L = foot_sole_frame_inv_L*kinematics_->standardizedEndeffPose(RIGHT_FOOT); // transformation from right foot sole frame to left foot sole frame

  Eigen::Matrix<double,6,1> gen_frcs_foot_R_L = Eigen::Matrix<double,6,1>::Zero();
  gen_frcs_foot_R_L.block<3,1>(0,0) = frame_R_L.block<3,3>(0,0)*gen_frcs_foot_R.block<3,1>(0,0);
  gen_frcs_foot_R_L.block<3,1>(3,0) = frame_R_L.block<3,3>(0,0)*gen_frcs_foot_R.block<3,1>(3,0);
  Eigen::Vector3d total_mom_L = gen_frcs_foot_L.block<3,1>(3,0) + gen_frcs_foot_R_L.block<3,1>(3,0) + ank_mom_L + frame_R_L.block<3,3>(0,0)*ank_mom_R
    + (frame_R_L.block<3,1>(0,3)).cross(gen_frcs_foot_R_L.block<3,1>(0,0));

  // Balance moments on foot (now that all moments are expressed in footprint frame) about CoP location and solve for local CoP
  cop_local[0] = total_mom_L[2] / (gen_frcs_foot_L[1] + gen_frcs_foot_R_L[1]);
  cop_local[1] = 0.0;
  cop_local[2] = -total_mom_L[0] / (gen_frcs_foot_L[1] + gen_frcs_foot_R_L[1]);

}

bool ContactHelper::isEndeffValid(int endeff_id) const {return support_polygon_corners_[endeff_id].cols() > 0;};
bool ContactHelper::isEndeffPoint(int endeff_id) const {return support_polygon_corners_[endeff_id].cols() == 1;};
bool ContactHelper::isEndeffLine(int endeff_id) const {return support_polygon_corners_[endeff_id].cols() == 2;};
bool ContactHelper::isEndeffFlat(int endeff_id) const {return support_polygon_corners_[endeff_id].cols() >= 3;};
//void ContactHelper::standardizedEndeffPose(int id, Eigen::Matrix4d& pose) const
//{
//  pose = kinematics_->endeffPose(id)*stdToRobotEndeffFrame(id);
//}

} /* namespace momentum_balance_control */
