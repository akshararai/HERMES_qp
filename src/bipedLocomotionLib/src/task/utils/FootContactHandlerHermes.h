/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         FootContactHandlerHermes.h

 \author       Alexander Herzog
 \date         August 8, 2012

 *********************************************************************/


#ifndef MBC_FOOT_CONTACT_HANDLER_H_
#define MBC_FOOT_CONTACT_HANDLER_H_


#include <vector>
#include <Eigen/Eigen>

#include "KinematicsEigen.h"
#include "floatingBaseToolsRobotInfo.h"
#include "ContactHelper.h"

#include <SL.h>
#include <SL_user.h>
#include <SL_common.h>
#include <utility.h>
#include <utility_macros.h>
#include <SL_kinematics.h>

namespace momentum_balance_control
{

class FootContactHandlerHermes : public floating_base_utilities::ContactHelper
{
  typedef floating_base_utilities::ContactHelper BaseClass;
public:
  FootContactHandlerHermes(std::string config_file = std::string(""));
  virtual ~FootContactHandlerHermes();

  void initialize(const floating_base_utilities::KinematicsEigen* kinematics);
  void update();

  virtual bool computeSupportPolygonCornersCCW(int endeff_id, floating_base_utilities::RtMatrixX<3,BaseClass::max_endeff_corners_>::d& corners) const;

  const floating_base_utilities::RtMatrixX<6, floating_base_utilities::
        floatingBaseToolsRobotInfo::max_contact_force_dim_>::d&
        getContactForceToMomentumRateMap(int endeff_id) const{return contact_frc_2_momrate_mats_[endeff_id];};

  const Eigen::Matrix<double, 6, 1>& getMeasuredForceAtEndeff(int endeff_id) const{return measured_contact_frcs_[endeff_id];};

  Eigen::Matrix<double, 6, 1> measured_contact_frcs_[N_ENDEFFS+1];

private:
	floating_base_utilities::RtMatrixX<6, floating_base_utilities::
	      floatingBaseToolsRobotInfo::max_contact_force_dim_>::d
	      contact_frc_2_momrate_mats_[N_ENDEFFS+1];
	Eigen::Vector3d eff_to_aaa_[N_ENDEFFS+1];

};
}  //namespace
#endif /* MBC_FOOT_CONTACT_HANDLER_H_ */
