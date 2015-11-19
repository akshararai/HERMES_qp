/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         FootContactHandlerHermes.cpp

 \author       Alexander Herzog
 \date         August 8, 2012

 *********************************************************************/

#include <iostream>
#include <Eigen/Eigen>

#include <FootContactHandlerHermes.h>
#include "GeometryUtils.h"

#include <SL.h>
#include <SL_user.h>
#include <SL_common.h>
#include <utility.h>
#include <utility_macros.h>
#include <SL_kinematics.h>
#include <SL_task_servo.h>




using namespace std;
using namespace floating_base_utilities;
namespace momentum_balance_control
{

FootContactHandlerHermes::FootContactHandlerHermes(std::string config_file) : BaseClass(config_file)
{}

FootContactHandlerHermes::~FootContactHandlerHermes()
{}

//bool FootContactHandlerHermes::computeSupportPolygonCornersCCW(int endeff_id, RtMatrixX<3,BaseClass::max_endeff_corners_>::d& corners) const
//{
//  const int num_corners = BaseClass::max_endeff_corners_;
//  unsigned int foot_corner_id[num_corners];
//  switch(endeff_id)
//  {
//  case LEFT_FOOT:
//    foot_corner_id[0] = L_IN_METATARSAL;
//    foot_corner_id[1] = L_OUT_HEEL;
//  break;
//  case RIGHT_FOOT:
//    foot_corner_id[0] = R_IN_METATARSAL;
//    foot_corner_id[1] = R_OUT_HEEL;
//    break;
//  default:
//    return false;
//  }
//
//  RtMatrixXUtils::setZero(corners, 3,num_corners);
//  for(int i=0; i<2; ++i)
//  {
//    corners.col(i) = BaseClass::kinematics_->linkPosition(foot_corner_id[i]);
//  }
//
//  Eigen::Matrix4d world_to_std_foot_frame;
//  BaseClass::kinematics_->standardizedEndeffPose(endeff_id, world_to_std_foot_frame);
//  GeometryUtils::invertEigenTransform(world_to_std_foot_frame);
//  corners = world_to_std_foot_frame.topLeftCorner<3,3>() * corners;
//  corners.leftCols(2).colwise() += world_to_std_foot_frame.topRightCorner<3,1>();
//
//  Eigen::Vector3d cntr = corners.rowwise().sum() / 2.0;
//  Eigen::Matrix3d rot;
//  rot = Eigen::AngleAxisd(-2.0*M_PI/num_corners, Eigen::Vector3d::UnitY());
//  for(int i=1; i<num_corners; ++i)
//  {
//    corners.col(i) = cntr + rot*(corners.col(i-1)-cntr);
//  }
//
//  for(int i=0; i<num_corners; ++i)
//  {
//    std::cout << "[";
//    for(int j=0; j<3; ++j) std::cout << corners(j,i) << ",";
//    std::cout << "],"<<std::endl;
//  }
//
//  return true;
//
//}

bool FootContactHandlerHermes::computeSupportPolygonCornersCCW(int endeff_id, RtMatrixX<3,BaseClass::max_endeff_corners_>::d& corners) const
{
  const int num_corners = 4;
  unsigned int foot_corner_id[num_corners];
  switch(endeff_id)
  {
  case LEFT_FOOT:
    foot_corner_id[0] = L_IN_METATARSAL; //L_IN_TOE;
    foot_corner_id[1] = L_OUT_METATARSAL; //L_OUT_TOE;
    foot_corner_id[2] = L_OUT_HEEL;
    foot_corner_id[3] = L_IN_HEEL;
  break;
  case RIGHT_FOOT:
    foot_corner_id[0] = R_IN_METATARSAL; //R_IN_TOE;
    foot_corner_id[1] = R_IN_HEEL;
    foot_corner_id[2] = R_OUT_HEEL;
    foot_corner_id[3] = R_OUT_METATARSAL; //R_OUT_TOE;
    break;
  default:
    return false;
  }

  RtMatrixXUtils::resize(corners, 3,num_corners);
  for(int i=0; i<num_corners; ++i)
  {
    corners.col(i) = BaseClass::kinematics_->linkPosition(foot_corner_id[i]);
  }

  const double elast_scale = 0.145/0.205;
  switch(endeff_id)
  {
  case LEFT_FOOT:
    corners.col(0) = corners.col(3) + elast_scale*(corners.col(0) - corners.col(3));
    corners.col(1) = corners.col(2) + elast_scale*(corners.col(1) - corners.col(2));
    break;
  case RIGHT_FOOT:
    corners.col(0) = corners.col(1) + elast_scale*(corners.col(0) - corners.col(1));
    corners.col(3) = corners.col(2) + elast_scale*(corners.col(3) - corners.col(2));
    break;
  default:
    return false;
  }

  Eigen::Matrix4d world_to_std_foot_frame;
  BaseClass::kinematics_->standardizedEndeffPose(endeff_id, world_to_std_foot_frame);
  GeometryUtils::invertEigenTransform(world_to_std_foot_frame);
  corners = world_to_std_foot_frame.topLeftCorner<3,3>() * corners;
  corners.colwise() += world_to_std_foot_frame.topRightCorner<3,1>();

  return true;

}

void FootContactHandlerHermes::initialize(const KinematicsEigen* kinematics){
  BaseClass::initialize(kinematics);

  for(int eff=1; eff<=N_ENDEFFS;++eff){
    Eigen::Matrix4d wrld_to_eff = kinematics_->endeffPose(eff);
    GeometryUtils::invertEigenTransform(wrld_to_eff);
    Eigen::Vector3d aaa = kinematics_->cartesianJointPosition(eff==RIGHT_FOOT?R_AAA:L_AAA);
    eff_to_aaa_[eff] = wrld_to_eff.topLeftCorner<3,3>()*aaa + wrld_to_eff.topRightCorner<3,1>();
  }
}

void FootContactHandlerHermes::update()
{
  BaseClass::update();
  for(int eff_i=1; eff_i<=N_ENDEFFS; ++eff_i)
  {
    if(BaseClass::kinematics_->endeffectors()[eff_i].c[1])
    {
#ifndef RTEIG_NO_ASSERTS
      for(int i=1; i<=6; ++i)
        assert(BaseClass::kinematics_->endeffectors()[eff_i].c[i] == true &&
              "The endeffector of a biped must be either fully constrained or fully unconstrained.");
#endif
      RtMatrixXUtils::setIdentity(contact_frc_2_momrate_mats_[eff_i]);
      contact_frc_2_momrate_mats_[eff_i].block(3, 0, 3, 3) = GeometryUtils::vector_to_skew_matrix(
            BaseClass::kinematics_->endeffPosition(eff_i) - BaseClass::kinematics_->cog());
    }else{
      RtMatrixXUtils::setZero(contact_frc_2_momrate_mats_[eff_i]);
    }
  }

  measured_contact_frcs_[LEFT_FOOT] = Eigen::Map<Eigen::Matrix<double, 6, 1> >(&(misc_sensor[L_CFx]));
  Eigen::Vector3d left_sens_pos(FTA_Z_OFF, FTA_X_OFF, -FTA_Y_OFF);
  left_sens_pos += eff_to_aaa_[LEFT_FOOT];
  measured_contact_frcs_[LEFT_FOOT].tail<3>() += left_sens_pos.cross(measured_contact_frcs_[LEFT_FOOT].head<3>());

  measured_contact_frcs_[RIGHT_FOOT] = Eigen::Map<Eigen::Matrix<double, 6, 1> >(&(misc_sensor[R_CFx]));
  Eigen::Vector3d right_sens_pos(FTA_Z_OFF, FTA_X_OFF, FTA_Y_OFF);
  right_sens_pos += eff_to_aaa_[RIGHT_FOOT];
  measured_contact_frcs_[RIGHT_FOOT].tail<3>() += right_sens_pos.cross(measured_contact_frcs_[RIGHT_FOOT].head<3>());
}

}  // namespace
