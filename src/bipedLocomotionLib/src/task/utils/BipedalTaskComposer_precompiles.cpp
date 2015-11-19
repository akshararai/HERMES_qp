/*********************************************************************
 Autonomous Motion Department
 Max-Planck Intelligent Systems
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         BipedalTaskComposer_precompiles.cpp

 \author       Alexander Herzog
 \date         Jan 25, 2014

 *********************************************************************/
#include <boost/thread.hpp>
#include "BipedalTaskComposer.h"
#include "HierarchInverseDynamics.h"
#include "CartesianPositionCtrl.h"
#include "CartesianForceCtrl.h"
#include "MomentumRateCtrl.h"
#include "FloatingBaseImpedanceCtrl.h"
#include "JointPositionCtrl.h"
#include "GeometryUtils.h"
#include "quaternions.h"


template class floating_base_utilities::JointPositionCtrl< momentum_balance_control::BipedalTaskComposer::max_ineq_constraints_, momentum_balance_control::BipedalTaskComposer::max_eq_constraints_,N_DOFS>;
template class floating_base_utilities::JointPositionCtrl< momentum_balance_control::BipedalTaskComposer::max_ineq_constraints_, momentum_balance_control::BipedalTaskComposer::max_eq_constraints_,7>;
template class floating_base_utilities::CartesianPositionCtrl< momentum_balance_control::BipedalTaskComposer::max_ineq_constraints_, momentum_balance_control::BipedalTaskComposer::max_eq_constraints_>;
template class floating_base_utilities::FloatingBaseImpedanceCtrl< momentum_balance_control::BipedalTaskComposer::max_ineq_constraints_, momentum_balance_control::BipedalTaskComposer::max_eq_constraints_>;
template class floating_base_utilities::CartesianForceCtrl< momentum_balance_control::BipedalTaskComposer::max_ineq_constraints_, momentum_balance_control::BipedalTaskComposer::max_eq_constraints_>;
template class floating_base_utilities::MomentumRateCtrl< momentum_balance_control::BipedalTaskComposer::max_ineq_constraints_, momentum_balance_control::BipedalTaskComposer::max_eq_constraints_>;
template class floating_base_utilities::MomentumRateCtrl< momentum_balance_control::BipedalTaskComposer::max_ineq_constraints_, momentum_balance_control::BipedalTaskComposer::max_eq_constraints_, true>;
template class floating_base_utilities::HierarchInverseDynamics< momentum_balance_control::BipedalTaskComposer::max_ineq_constraints_, momentum_balance_control::BipedalTaskComposer::max_eq_constraints_>;
