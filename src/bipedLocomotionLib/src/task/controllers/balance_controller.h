/*
 * balance_controller.h
 *
 *  Created on: Aug 28, 2013
 *      Author: righetti
 *
 *      This balance controller implements a simple controller to keep the CoM in between the feet
 *      using a simple PD control law for the desired CoM motion and damping for the angular momentum
 */

#ifndef BALANCE_CONTROLLER_H_
#define BALANCE_CONTROLLER_H_

#include <Eigen/Eigen>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include "RtMinJerkGenerator.hh"
#include "HierarchInverseDynamics.hh"
#include "BipedalTaskComposer.h"
#include "capture_point_helper.h"
#include "KinematicsEigen.h"
#include "MomentumComputation.h"
#include "FloatingBaseKinematics.h"
#include "OptimalInverseDynamicsEigen.h"
#include "RtMinJerkGenerator.hh"
#include "FootContactHandlerHermes.h"

#include "biped_controller.h"

#include "SL.h"
#include "SL_user.h"
#include "SL_tasks.h"
#include "SL_task_servo.h"
#include "SL_collect_data.h"
#include "SL_objects.h"
#include "utility_macros.h"

class BalanceController : public BipedController
{
public:
  BalanceController();
  virtual ~BalanceController();

  bool initialize();
  bool run();

private:
  double cycle_duration_;

  enum BalancingState {BALANCE, INIT};
  BalancingState current_state_;

  Eigen::Vector3d com_centered_bias_;

  double com_sine_frequency_, init_duration_;
  Eigen::Vector3d com_sine_amplitude_;
  double timer_;

  floating_base_utilities::RtMinJerkGenerator<3> com_trajectory_[3];

  char config_file_[30];

  double push_force_, push_dur_;
  Eigen::Matrix<double, 40, 1> dummy_variables_;
};

#endif /* balance_CONTROLLER_H_ */
