/*!=============================================================================
  ==============================================================================

  \file    geometry_utils.h

  \author  righetti
  \date    Oct 17, 2012

  ==============================================================================
  \remarks
  
  
  ============================================================================*/

#ifndef GEOMETRY_UTILS_H_
#define GEOMETRY_UTILS_H_

#include <Eigen/Eigen>

#include <SL.h>
#include <utility.h>
#include <utility_macros.h>
#include <SL_common.h>

namespace floating_base_utilities
{

class GeometryUtils
{
public:

//computes the logarithm map on SE(3)
static Eigen::Matrix4d log_map_se3(const Eigen::Matrix4d& T);

//computes the logarithm map on SO(3)
static Eigen::Matrix3d log_map_so3(const Eigen::Matrix3d& R);



/* convenient functions that do all compute so(3) -> SO(3)*/

//axis needs to have unit length
static Eigen::Matrix3d exp_map_SO3(const Eigen::Vector3d& axis, double angle);
//w has length of rotation per second
static Eigen::Matrix3d exp_map_SO3(const Eigen::Vector3d& w);
//Axis is skew matrix from unit length axis
static Eigen::Matrix3d exp_map_SO3(const Eigen::Matrix3d& Axis, double angle);
//W is skew matrix from angular velocity
static Eigen::Matrix3d exp_map_SO3(const Eigen::Matrix3d& W);


/* some geometrical transformations for convenience */
static double get_angle_from_so3(const Eigen::Matrix3d& W);
static Eigen::Matrix3d vector_to_skew_matrix(const Eigen::Vector3d& v);
static Eigen::Vector3d skew_matrix_to_vector(const Eigen::Matrix3d& s);
static void invertEigenTransform(Eigen::Matrix4d& T);

static void SLQuaternionToEigen(const Vector sl_quat, Eigen::Quaterniond& eig_quat);
static void EigQuaternionToSL(const Eigen::Quaterniond& eig_quat, Vector sl_quat);

template<typename RotMat>
static void rotationMatrixToQuaternion(const Eigen::MatrixBase<RotMat>& des_rot,
                                     Eigen::Quaterniond& quaternion);
template<typename RotMat>
static void rotationMatrixToQuaternion(const Eigen::MatrixBase<RotMat>& des_rot,
                                     SL_quat& sl_quaternion);

/** be carefull: if you use this for orientation control then you want to use -err_vec
 * as a command: u = - GAIN * err_vec
 * @param des_rot
 * @param cur_rot
 * @param err_vec
 */
template<typename DesRotDer, typename CurRotDer, typename errVecDer>
static void computeOrientationError(const Eigen::MatrixBase<DesRotDer>& des_rot,
      const Eigen::MatrixBase<CurRotDer>& cur_rot, Eigen::MatrixBase<errVecDer>& err_vec);

template<typename errVecDer>
static void computeOrientationError(const Eigen::Quaterniond& des_rot,
      const Eigen::Quaterniond& cur_rot, Eigen::MatrixBase<errVecDer>& err_vec);

private:
  GeometryUtils(){};
 virtual ~GeometryUtils(){};
};

template<typename DesRotDer, typename CurRotDer, typename errVecDer>
void GeometryUtils::computeOrientationError(const Eigen::MatrixBase<DesRotDer>& des_rot,
      const Eigen::MatrixBase<CurRotDer>& cur_rot, Eigen::MatrixBase<errVecDer>& err_vec)
{
  Eigen::Quaterniond des_rot_quat(des_rot);
  Eigen::Quaterniond cur_rot_quat(cur_rot);

  computeOrientationError(des_rot_quat, cur_rot_quat, err_vec);
}


template<typename errVecDer>
void GeometryUtils::computeOrientationError(const Eigen::Quaterniond& des_rot,
      const Eigen::Quaterniond& cur_rot, Eigen::MatrixBase<errVecDer>& err_vec)
{
  SL_quat des_rot_sl;
  SL_quat cur_rot_sl;
    MY_VECTOR(err_vec_sl, 1, 3);
    EigQuaternionToSL(des_rot, des_rot_sl.q);
    EigQuaternionToSL(cur_rot, cur_rot_sl.q);

    // fix sign
    {
      double dot=0.0;
      for (int j=1; j<=N_QUAT; j++)
      {
        dot+=cur_rot_sl.q[j]*des_rot_sl.q[j];
      }
      if (dot<0)
      {
        for (int j=1; j<=N_QUAT; j++)
        {
          des_rot_sl.q[j]=-des_rot_sl.q[j];
        }
      }
    }
    quatErrorVector(des_rot_sl.q, cur_rot_sl.q, err_vec_sl);

    err_vec = Eigen::Vector3d(&(err_vec_sl[1]));
}


template<typename RotMat>
void GeometryUtils::rotationMatrixToQuaternion(const Eigen::MatrixBase<RotMat>& des_rot,
                                    Eigen::Quaterniond& quaternion)
{
  MY_MATRIX(sl_mat, 1, 3, 1, 3);
  SL_quat sl_quaternion;
  for(int r=1; r<=3; ++r)
    for(int c=1; c<=3; ++c)
      sl_mat[r][c] = des_rot(r-1, c-1);
  linkQuat(sl_mat, &sl_quaternion);
  quaternion.w() = sl_quaternion.q[_QW_];
  quaternion.x() = sl_quaternion.q[_QX_];
  quaternion.y() = sl_quaternion.q[_QY_];
  quaternion.z() = sl_quaternion.q[_QZ_];
}

template<typename RotMat>
void GeometryUtils::rotationMatrixToQuaternion(const Eigen::MatrixBase<RotMat>& des_rot,
                                     SL_quat& sl_quaternion)
{
  MY_MATRIX(sl_mat, 1, 3, 1, 3);
  for(int r=1; r<=3; ++r)
    for(int c=1; c<=3; ++c)
      sl_mat[r][c] = des_rot(r-1, c-1);
  linkQuat(sl_mat, &sl_quaternion);
}

}

#endif /* GEOMETRY_UTILS_H_ */
