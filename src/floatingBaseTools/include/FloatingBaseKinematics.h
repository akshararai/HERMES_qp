/*
 * FloatingBaseKinematics.h
 *
 *  Created on: Mar 5, 2012
 *      Author: righetti
 */

#ifndef FLOATINGBASEKINEMATICS_HPP_
#define FLOATINGBASEKINEMATICS_HPP_

//using Eigen3
#include <Eigen/Eigen>
#include <iostream>

#include <SL.h>
#include <utility.h>
#include <utility_macros.h>
#include <SL_kinematics.h>
#include <SL_dynamics.h>
#include <SL_task_servo.h>
#include <SL_user.h>

namespace floating_base_utilities
{

class FloatingBaseKinematics
{
public:

  FloatingBaseKinematics();
  virtual ~FloatingBaseKinematics();

  void computeJacobians(SL_Jstate* j_state, SL_Cstate& base_state, SL_quat& base_orient, SL_endeff* end_effectors, int& num_constraints);

  void computeQRDecomposition(bool update_projectors = false);

  int getNumConstraints() const{return num_constraints_;};

  /*!
   * returns the constrained Jacobian and friends (aka nullspace projector, pseudo inverse and dJc/dt)
   * in SL Matrix representation form. the Jacobian is originally computed in prepareInverseDynamics()
   * @param Jc
   * @param dJc
   * @param Jc_pinv
   * @param null_Jc
   */
  void getConstraintJacobian(Matrix Jc, Matrix dJc, Matrix Jc_pinv, Matrix null_Jc);
  void getConstraintJacobian(Eigen::Matrix<double, 6*N_ENDEFFS, N_DOFS+6>& Jc,
                             Eigen::Matrix<double, 6*N_ENDEFFS, N_DOFS+6>& dJc,
                             Eigen::Matrix<double, N_DOFS+6, 6*N_ENDEFFS>& Jc_pinv,
                             Eigen::Matrix<double, N_DOFS+6, N_DOFS+6>& null_Jc);

  void getNullspaceProjector(Eigen::Matrix<double, N_DOFS+6, N_DOFS+6>& null_Jc);

  void getConstrainedJacobian(Eigen::Matrix<double, 6*N_ENDEFFS, N_DOFS+6>& constrained_jacobian);
  void getUnconstrainedJacobian(Eigen::Matrix<double, 6*N_ENDEFFS, N_DOFS+6>& unconstrained_jacobian);

  const Eigen::Matrix<double, N_DOFS+6, N_DOFS+6>& Q() {return jc_svd_.matrixV();};
  const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 0, 6*N_ENDEFFS, 6*N_ENDEFFS>& R_inverse(){return R_inverse_;};


  /* some getters for const access of member variables */
  const Eigen::Matrix<double, 6*N_ENDEFFS, N_DOFS+6>& getConstraintJacobian() const{return constrained_jacobian_;};
  const Eigen::Matrix<double, 6*N_ENDEFFS, N_DOFS+6>& getUnconstrainedJacobian() const{return unconstrained_jacobian_;};
  const Eigen::Matrix<double, N_DOFS+6, N_DOFS+6>& getNullspaceProjector() const{return null_Jc_;};
  const Eigen::Matrix<double, N_DOFS+6, 6*N_ENDEFFS>& getConstrJacobianPsuedoInverse() const{return Jc_pinv_;};
  const Eigen::Matrix<double, 6*N_ENDEFFS, N_DOFS+6>& getConstrJacobianDerivative() const{return dJc_;};
  const Eigen::JacobiSVD<Eigen::Matrix<double, 6*N_ENDEFFS, N_DOFS+6> >& getJacobianSVD() const {return jc_svd_;};
  const Eigen::Matrix<double, 6*N_ENDEFFS, N_DOFS+6>& getUnconstrJacobianDerivative() const{return dJunconstr_;};
  const Matrix& getJointAxes() const { return Xaxis_; }
  const Matrix& getJointOrigins() const { return Xorigin_; }
  const Matrix& getLinkPositions() const { return Xlink_; }
  const Matrix* getLinkHomogenousMatrices() const { return Ahmat_; }
  const Matrix& getEndeffectorJointJacobians() const { return endeff_joint_jacobians_; }
  const Matrix& getEndeffectorBaseJacobians() const { return endeff_base_jacobians_; }
   

  //we want everyone to be able to access those (TODO make nicer to forbid write... read only access)
  /* deprecated */ SL_endeff end_effectors_[N_ENDEFFS+1];
  const SL_endeff* getEndEffectors() const{return end_effectors_;};

private:

  int num_constraints_;
  int num_unconstrained_dimensions_;

  Eigen::Matrix<double, 6*N_ENDEFFS, N_DOFS+6> constrained_jacobian_;
  Eigen::Matrix<double, 6*N_ENDEFFS, N_DOFS+6> unconstrained_jacobian_;


  Eigen::Matrix<double, 6*N_ENDEFFS, N_DOFS+6> previous_Jc_;
  Eigen::Matrix<double, 6*N_ENDEFFS, N_DOFS+6> previous_Junconstr_;
  Eigen::Matrix<double, N_DOFS+6, 6*N_ENDEFFS> Jc_pinv_;
  Eigen::Matrix<double, 6*N_ENDEFFS, N_DOFS+6> dJc_;
  Eigen::Matrix<double, 6*N_ENDEFFS, N_DOFS+6> dJunconstr_;
  Eigen::Matrix<double, N_DOFS+6, N_DOFS+6> null_Jc_;

  bool valid_jacobian_;
  Eigen::Matrix<int, N_ENDEFFS, 6> previous_constraints_;


  //we fix the size of the matrix to guarantee we don't have any memory allocation (Xenomai safe)
  Eigen::JacobiSVD<Eigen::Matrix<double, 6*N_ENDEFFS, N_DOFS+6> > jc_svd_;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 0, 6*N_ENDEFFS, 6*N_ENDEFFS> R_inverse_;

  // SL kinematics variables (all 1-indexed, SL-style)
  Matrix Xmcog_;                    // N_DOFS x 3
  Matrix Xaxis_;                    // N_DOFS x 3
  Matrix Xorigin_;                  // N_DOFS x 3
  Matrix Xlink_;                    // N_LINKS  x 3
  Matrix Ahmat_[N_LINKS+1];         // 4 x 4
  Matrix Ahmat_dof_[N_DOFS+1];      // 4 x 4
  Matrix endeff_joint_jacobians_;   // (2 * N_CART * N_ENDEFFS) x N_DOFS
  Matrix endeff_base_jacobians_;    // (2 * N_CART * N_ENDEFFS) x (2 * N_CART)

};


}


#endif /* FLOATINGBASEKINEMATICS_HPP_ */
