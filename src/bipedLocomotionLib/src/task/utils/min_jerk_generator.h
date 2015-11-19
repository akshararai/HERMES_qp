/*!=============================================================================
 ==============================================================================

 \file    min_jerk_generator.h

 \author  righetti
 \date    Apr 17, 2013

 ==============================================================================
 \remarks
 
 
 ============================================================================*/

#ifndef MIN_JERK_GENERATOR_H_
#define MIN_JERK_GENERATOR_H_

#include <eigen3/Eigen/Eigen>

namespace inverse_kinematics
{

class MinJerkGenerator
{
public:
  MinJerkGenerator();
  virtual ~MinJerkGenerator();

  bool initialize(const Eigen::Vector2d& time, const Eigen::Vector2d& pos,
                  const Eigen::Vector2d& vel, const Eigen::Vector2d& acc);

  bool initialize(const Eigen::MatrixXd& pos_const, const Eigen::MatrixXd& vel_const,
                  const Eigen::MatrixXd& acc_const);

  bool query(double time, double& pos, double &vel, double& acc);
  bool isInitialized() const{return initialized_;};

private:
  bool initialized_;

  Eigen::Vector2d time_;
  Eigen::VectorXd coeffs_;

  int num_coeffs_;
};

} /* namespace SL */
#endif /* MIN_JERK_GENERATOR_H_ */
