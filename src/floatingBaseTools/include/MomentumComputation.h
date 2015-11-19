/*
 * MomentumComputation.h
 *
 *  Created on: Apr 2, 2013
 *      Author: herzog
 */

#ifndef MOMENTUMCOMPUTATION_H_
#define MOMENTUMCOMPUTATION_H_


#include <Eigen/Eigen>
#include <vector>
#include <iostream>

#include "KinematicsEigen.h"
#include "GeometryUtils.h"

#include <SL.h>
#include <SL_common.h>
#include <utility.h>
#include <utility_macros.h>
#include <SL_kinematics.h>
#include <SL_user.h>


namespace floating_base_utilities
{

class MomentumComputation
{
public:
  MomentumComputation();
  virtual ~MomentumComputation(){};

  bool initialize();


  /*!
   * given the new kinematics state computes various dynamics quantities -> momentum and centroidal momentum matrix
   * @param kinematics_eigen
   * @return
   */
  bool update(const KinematicsEigen& kinematics_eigen);

  const Eigen::Matrix<double, 2*N_CART, 1>& getMomentum() const{return momentum_;}
  const Eigen::Matrix<double, N_CART, 1>& getdCog() const{return dcog_;}

  const Eigen::Matrix<double, N_CART, 1>& getddCog() const{return ddcog_;}
  const Eigen::Matrix<double, 2*N_CART, N_DOFS+2*N_CART>& getCentroidalMomentumMatrix() const{return centroidal_momentum_matrix_;}
  const Eigen::Matrix<double, 2*N_CART, N_DOFS+2*N_CART>& getdCentroidalMomentumMatrix() const{return dcentroidal_momentum_matrix_;}


  /*
   * computes the moment acting at the COP due to the gen. ground reaction forces
   */
  void computeCOPTorque(const Eigen::Vector3d& ankle_pos, const Eigen::Matrix4d& foot_sole_frame,
                        const Eigen::Matrix<double, 2*N_CART, 1>& generalized_forces_world,
                        double& moment) const;

  /*!
   * @param [in] ankle_pos ankle position in world frame
   * @param [in]  foot_sole_frame frame that has origin on foot sole and is s.t. y-axis is normal to foot surface (you
   *                    might need to transform the foot transform that you get from linkInformation() before passing it
   *                    to this function)
   * @param [in]  generalized_forces_world the generalized forces applied at the ankle, represented in world frame
   * @return [out]  cop computed cop, represented in foot frame
   */
  bool computeFootCoP(const Eigen::Vector3d& ankle_pos, const Eigen::Matrix4d& foot_sole_frame,
                      Eigen::Matrix<double, 2*N_CART, 1>& generalized_forces_world,
                      Eigen::Vector3d& cop);


private:
  Eigen::Matrix<double, 2*N_CART, N_DOFS+2*N_CART> centroidal_momentum_matrix_;
  Eigen::Matrix<double, 2*N_CART, N_DOFS+2*N_CART> dcentroidal_momentum_matrix_;
  Eigen::Matrix<double, 2*N_CART, 1> momentum_;
  bool num_deriv_ok_;

  Eigen::Matrix<double, N_CART, 1> dcog_,ddcog_;
};




} /* namespace floating_base_utilities */
#endif /* MOMENTUMCOMPUTATION_H_ */
