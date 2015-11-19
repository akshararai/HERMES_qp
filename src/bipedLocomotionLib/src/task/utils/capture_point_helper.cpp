/*
 * capture_point_helper.cpp
 *
 *  Created on: Apr 27, 2013
 *      Author: righetti
 */

#include "capture_point_helper.h"

CapturePointHelper::CapturePointHelper():initialized_(false)
{
}

CapturePointHelper::~CapturePointHelper()
{
}

bool CapturePointHelper::initialize(double z_com, double z_floor)
{
  initialized_ = true;
  z_com_ = z_com;
  z_floor_ = z_floor;
  omega0_ = sqrt(G/fabs(z_com_));

  return initialized_;
}

bool CapturePointHelper::getInstantaneousCapturePoint(const Eigen::Vector3d& com, const Eigen::Vector3d& dcom,
                                                      Eigen::Vector3d& icp)
{
  icp = com + dcom/omega0_;
  icp(2) = z_floor_;

  return true;
}

bool CapturePointHelper::getPredictedCapturePoint(double deltaT, const Eigen::Vector3d& icp,
                                                  const Eigen::Vector3d& cop, Eigen::Vector3d& pred_cp)
{
  pred_cp = (icp - cop)*exp(omega0_*deltaT) + cop;
  pred_cp(2) = z_floor_;

  return true;
}
