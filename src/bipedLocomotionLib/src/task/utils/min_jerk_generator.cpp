/*!=============================================================================
 ==============================================================================

 \file    min_jerk_generator.cpp

 \author  righetti
 \date    Apr 17, 2013

 ==============================================================================
 \remarks


 ============================================================================*/

#include <cstdio>
#include "min_jerk_generator.h"

namespace inverse_kinematics
{

MinJerkGenerator::MinJerkGenerator():initialized_(false)
{
}

MinJerkGenerator::~MinJerkGenerator()
{
}

bool MinJerkGenerator::initialize(const Eigen::MatrixXd& pos_const, const Eigen::MatrixXd& vel_const, const Eigen::MatrixXd& acc_const)
{
  //count the number of constraints
  num_coeffs_ = pos_const.rows() + vel_const.rows() + acc_const.rows();

  Eigen::MatrixXd regress_mat = Eigen::MatrixXd::Zero(num_coeffs_, num_coeffs_);
  Eigen::VectorXd constraints = Eigen::VectorXd::Zero(num_coeffs_);

  time_.setZero();

  int index = 0;

  //add pos constraints
  for(int i=0; i<pos_const.rows(); ++i)
  {
    double t = pos_const(i,0);

    if(time_(0)>t)
      time_(0) = t;
    if(time_(1)<t)
      time_(1) = t;

    constraints(index) = pos_const(i,1);

    //fill the row of the regress matrix
    for(int j=0; j<num_coeffs_; ++j)
    {
      regress_mat(index, j) = pow(t, double(j));
    }
    index++;
  }

  //add vel constraints
  for(int i=0; i<vel_const.rows(); ++i)
  {
    double t = vel_const(i,0);

    constraints(index) = vel_const(i,1);

    //fill the row of the regress matrix
    for(int j=1; j<num_coeffs_; ++j)
    {
      regress_mat(index, j) = j*pow(t, double(j-1));
    }
    index++;
  }

  //add acc constraints
  for(int i=0; i<acc_const.rows(); ++i)
  {
    double t = acc_const(i,0);

    constraints(index) = acc_const(i,1);

    //fill the row of the regress matrix
    for(int j=2; j<num_coeffs_; ++j)
    {
      regress_mat(index, j) = j*(j-1)*pow(t, double(j-2));
    }
    index++;
  }

  coeffs_ = regress_mat.householderQr().solve(constraints);

  initialized_ = true;
  return initialized_;

}
bool MinJerkGenerator::initialize(const Eigen::Vector2d& time, const Eigen::Vector2d& pos,
                                  const Eigen::Vector2d& vel, const Eigen::Vector2d& acc)
{
  initialized_ = true;

  //set the time boundaries
  time_ = time;
  num_coeffs_ = 6;

  //compute the coefficients
  Eigen::Matrix<double, 6, 6> regress_mat;
  regress_mat <<
      1, time(0), time(0)*time(0), time(0)*time(0)*time(0), time(0)*time(0)*time(0)*time(0), time(0)*time(0)*time(0)*time(0)*time(0),
      1, time(1), time(1)*time(1), time(1)*time(1)*time(1), time(1)*time(1)*time(1)*time(1), time(1)*time(1)*time(1)*time(1)*time(1),
      0, 1, 2*time(0), 3*time(0)*time(0), 4*time(0)*time(0)*time(0), 5*time(0)*time(0)*time(0)*time(0),
      0, 1, 2*time(1), 3*time(1)*time(1), 4*time(1)*time(1)*time(1), 5*time(1)*time(1)*time(1)*time(1),
      0, 0, 2, 6*time(0), 12*time(0)*time(0), 20*time(0)*time(0)*time(0),
      0, 0, 2, 6*time(1), 12*time(1)*time(1), 20*time(1)*time(1)*time(1);

  Eigen::Matrix<double, 6, 1> way_points;
  way_points << pos(0), pos(1),vel(0),vel(1),acc(0),acc(1);

  coeffs_ = regress_mat.householderQr().solve(way_points);


  return initialized_;
}

bool MinJerkGenerator::query(double time, double& pos, double& vel, double& acc)
{
  assert(initialized_);
  if(time<time_(0) || time>time_(1))
  {
    printf("ERROR: time query needs to be within bounds.\n");
    return false;
  }

  pos = 0.0;
  for(int i=0; i<num_coeffs_; ++i)
    pos += coeffs_(i)*pow(time, double(i));

  vel=0.0;
  for(int i=1; i<num_coeffs_; ++i)
    vel += coeffs_(i)*double(i)*pow(time, double(i-1));

  acc=0.0;
  for(int i=2; i<num_coeffs_; ++i)
    acc += coeffs_(i)*double(i)*double(i-1)*pow(time, double(i-2));

  return true;
}

} /* namespace SL */
