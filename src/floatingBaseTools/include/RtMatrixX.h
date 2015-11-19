/*********************************************************************
 Autonomous Motion Department
 Max-Planck Intelligent Systems
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         RtMatrixX.h

 \author       Alexander Herzog
 \date         Jul 13, 2013

 *********************************************************************/

#ifndef RTMATRIXX_H_
#define RTMATRIXX_H_

#include <Eigen/Dense>

namespace floating_base_utilities
{

template<int _MaxRows, int _MaxCols>
struct RtMatrixX
{
#ifdef RTEIG_DYN_ALLOCS
          typedef Eigen::MatrixXd d;
#else
          static const int min_possible_elements_ = 2;
          typedef Eigen::Matrix<double, Eigen::Dynamic,
                  Eigen::Dynamic, Eigen::AutoAlign,
                  (_MaxRows>=min_possible_elements_)?_MaxRows:min_possible_elements_,
                  (_MaxCols>=min_possible_elements_)?_MaxCols:min_possible_elements_> d;
          typedef Eigen::Matrix<float, Eigen::Dynamic,
                  Eigen::Dynamic, Eigen::AutoAlign,
                  (_MaxRows>=min_possible_elements_)?_MaxRows:min_possible_elements_,
                  (_MaxCols>=min_possible_elements_)?_MaxCols:min_possible_elements_> f;
          typedef Eigen::Matrix<int, Eigen::Dynamic,
              Eigen::Dynamic, Eigen::AutoAlign,
              (_MaxRows>=min_possible_elements_)?_MaxRows:min_possible_elements_,
              (_MaxCols>=min_possible_elements_)?_MaxCols:min_possible_elements_> i;
#endif
};

template<int _MaxRows, int _MaxCols>
const int RtMatrixX<_MaxRows, _MaxCols>::min_possible_elements_;

template<int _MaxRows>
struct RtVectorX
{

#ifdef RTEIG_DYN_ALLOCS
         typedef Eigen::VectorXd d;
#else
         static const int min_possible_elements_ = 2;
         typedef Eigen::Matrix<double, Eigen::Dynamic,
                 1, Eigen::AutoAlign,
                 (_MaxRows>=min_possible_elements_)?_MaxRows:min_possible_elements_, 1> d;
         typedef Eigen::Matrix<float, Eigen::Dynamic,
                 1, Eigen::AutoAlign,
                 (_MaxRows>=min_possible_elements_)?_MaxRows:min_possible_elements_, 1> f;
         typedef Eigen::Matrix<int, Eigen::Dynamic,
             1, Eigen::AutoAlign,
             (_MaxRows>=min_possible_elements_)?_MaxRows:min_possible_elements_, 1> i;
         typedef Eigen::Matrix<bool, Eigen::Dynamic,
             1, Eigen::AutoAlign,
             (_MaxRows>=min_possible_elements_)?_MaxRows:min_possible_elements_, 1> b;
#endif

};

template<int _MaxRows>
const int RtVectorX<_MaxRows>::min_possible_elements_;

class RtMatrixXUtils
{
private:
  RtMatrixXUtils(){};
  virtual ~RtMatrixXUtils(){};
public:


  template<int MaxRows, int MaxCols>
  static inline void createSelectorMat(typename RtMatrixX<MaxRows, MaxCols>::d& selector_mat,
        const typename RtVectorX<MaxRows>::d& row_selector, int num_valids, double threashold)
  {
    RtMatrixXUtils::setZero(selector_mat,num_valids, row_selector.size());
    int row = 0;
    for(int col=0; col<row_selector.size(); ++col)
    {
      if(row_selector[col] > threashold)
      {
        selector_mat(row,col) = 1.0;
        ++row;
      }
    }
  }

  template<int MaxRows, int MaxCols>
  static inline void removeZeroRows(typename RtMatrixX<MaxRows, MaxCols>::d& mat,
        double threashold = remove_zero_threash_)
  {
    if(mat.rows() == 0)
      return;
    typename RtVectorX<MaxRows>::d squared_norms;
    squared_norms = mat.rowwise().squaredNorm();
    const double normed_threash = squared_norms.size() * threashold * threashold;
    int num_valids = 0;
    for(int i=1; i<squared_norms.size(); ++i)
      if(squared_norms[i] > normed_threash)
        ++num_valids;

    if(num_valids == mat.rows())
    {
      return;
    }
    else if(num_valids == 0)
    {
      RtMatrixXUtils::resize(mat, 0, mat.cols());
    }
    else
    {
      typename RtMatrixX<MaxRows, MaxCols>::d selector_mat;
      createSelectorMat(selector_mat, squared_norms, num_valids, normed_threash);
      mat = selector_mat * mat;
    }
  };

  /**!
   * Resize matrix by keeping data untouched without memory allocation.
   * Asserts that maximum size is not exceeded.
   * @param mat
   * @param rows
   * @param cols
   */
  template <typename Derived>
  static inline void conservativeResize(Eigen::MatrixBase<Derived>& mat,
        int rows, int cols)
  {
#ifndef RTEIG_NO_ASSERTS
    assert(rows <= mat.MaxRowsAtCompileTime
          && cols <= mat.MaxColsAtCompileTime);
#endif
    mat.derived().conservativeResize(rows, cols);
  };

  /**!
   * Resize matrix without memory allocation. Asserts that maximum size is not exceeded.
   * @param mat
   * @param rows
   * @param cols
   */
  template <typename Derived>
  static inline void resize(Eigen::MatrixBase<Derived>& mat,
        int rows, int cols)
  {
#ifndef RTEIG_NO_ASSERTS
    assert(rows <= mat.MaxRowsAtCompileTime
          && cols <= mat.MaxColsAtCompileTime);
#endif
    mat.derived().resize(rows, cols);
  };

  template <typename Derived>
    static inline void setZero(Eigen::MatrixBase<Derived>& mat)
  {
#ifdef RTEIG_DYN_ALLOCS
            resize(mat,1, 1);
#else
            resize(mat, mat.MaxRowsAtCompileTime, mat.MaxColsAtCompileTime);
#endif
    mat.derived().setZero();
  }

  template <typename Derived>
    static inline void setZero(Eigen::MatrixBase<Derived>& mat, int rows, int cols)
  {
    resize(mat, rows, cols);
    mat.derived().setZero();
  }

  template <typename Derived>
    static inline void setOnes(Eigen::MatrixBase<Derived>& mat)
  {
#ifdef RTEIG_DYN_ALLOCS
            resize(mat,1, 1);
#else
            resize(mat, mat.MaxRowsAtCompileTime, mat.MaxColsAtCompileTime);
#endif
    mat.derived().setOnes();
  }

  template <typename Derived>
    static inline void setOnes(Eigen::MatrixBase<Derived>& mat, int rows, int cols)
  {
    resize(mat, rows, cols);
    mat.derived().setOnes();
  }

  template <typename Derived>
    static inline void setConstant(Eigen::MatrixBase<Derived>& mat, double scalar)
  {
#ifdef RTEIG_DYN_ALLOCS
            resize(mat,1, 1);
#else
            resize(mat, mat.MaxRowsAtCompileTime, mat.MaxColsAtCompileTime);
#endif
    mat.derived().setConstant(scalar);
  }

  template <typename Derived>
    static inline void setConstant(Eigen::MatrixBase<Derived>& mat, int rows,
        int cols, double scalar)
  {
    resize(mat, rows, cols);
    mat.derived().setConstant(scalar);
  }

  template <typename Derived>
    static inline void setIdentity(Eigen::MatrixBase<Derived>& mat)
  {
#ifndef RTEIG_NO_ASSERTS
    assert(mat.MaxRowsAtCompileTime == mat.MaxColsAtCompileTime);
#endif
#ifdef RTEIG_DYN_ALLOCS
            resize(mat,1, 1);
#else
            resize(mat, mat.MaxRowsAtCompileTime, mat.MaxRowsAtCompileTime);
#endif
    mat.derived().setIdentity();
  }

  template <typename Derived>
    static inline void setIdentity(Eigen::MatrixBase<Derived>& mat, int rows)
  {
    resize(mat, rows, rows);
    mat.derived().setIdentity();
  }

  /***
   * Append a matrix to another one
   * @param aff_mat
   * @param subst_mat
   */
  template <typename AffMatDerived, typename SubstMatDerived>
  static inline void append(Eigen::MatrixBase<AffMatDerived>& aff_mat,
      const Eigen::MatrixBase<SubstMatDerived>& subst_mat)
  {
#ifndef RTEIG_NO_ASSERTS
    assert(subst_mat.cols() == aff_mat.cols()
          && aff_mat.rows() + subst_mat.rows() <= aff_mat.MaxRowsAtCompileTime);
#endif
    const int last_row = aff_mat.rows();
    if(last_row == 0)
    {
      aff_mat = subst_mat;
    }
    else
    {
      RtMatrixXUtils::conservativeResize(aff_mat, aff_mat.rows() + subst_mat.rows(), aff_mat.cols());
      aff_mat.block(last_row, 0, subst_mat.rows(), subst_mat.cols()) = subst_mat;
    }
  }

  /**!
   * Append a matrix at the end of another one starting from a certein column.
   * AffMat and SubstMat are not required to have the same number of columns.
   * @param aff_mat
   * @param subst_mat
   */
  template <typename AffMatDerived, typename SubstMatDerived>
  static inline void appendAtColumn(Eigen::MatrixBase<AffMatDerived>& aff_mat,
      const Eigen::MatrixBase<SubstMatDerived>& subst_mat, int starting_column)
  {
#ifndef RTEIG_NO_ASSERTS
    assert(subst_mat.cols() + starting_column <= aff_mat.cols()
          && aff_mat.rows() + subst_mat.rows() <= aff_mat.MaxRowsAtCompileTime);
#endif
    const int last_row = aff_mat.rows();
    if(last_row == 0)
    {
      RtMatrixXUtils::resize(aff_mat, aff_mat.rows() + subst_mat.rows(), aff_mat.cols());
    }
    else
    {
      RtMatrixXUtils::conservativeResize(aff_mat, aff_mat.rows() + subst_mat.rows(), aff_mat.cols());
    }
    aff_mat.block(last_row, 0, subst_mat.rows(), aff_mat.cols()).setZero();
    aff_mat.block(last_row, starting_column, subst_mat.rows(), subst_mat.cols()) = subst_mat;
  }

//  template<int _mat_rows, int _mat_cols, int _mat_align, int max_mat_rows, int max_mat_cols,
//    typename invDer, typename nullDer>
//  static int computePseudoInv(
//          const Eigen::Matrix<double, _mat_rows, _mat_cols, _mat_align, max_mat_rows, max_mat_cols>& mat,
//          Eigen::MatrixBase<invDer>& inv, Eigen::MatrixBase<nullDer>& null_proj);

  /***
   * Computes the pseudo inverse and a map into the nullspace with highest possible rank.
   * @param mat
   * @param inv
   * @param null_map
   * @return
   */
//  template<int _mat_rows, int _mat_cols, int _mat_align, int max_mat_rows, int max_mat_cols,
//    typename invDer, typename nullDer>
//  static int computePseudoInvWithNullspaceMap(
//          const Eigen::Matrix<double, _mat_rows, _mat_cols, _mat_align, max_mat_rows, max_mat_cols>& mat,
//          Eigen::MatrixBase<invDer>& inv, Eigen::MatrixBase<nullDer>& null_map);
  template<typename sigmaPinvDer, typename svdMatDer>
  static void computeSigmaPinv(Eigen::MatrixBase<sigmaPinvDer>& sigma_pinv_vec,
        const Eigen::JacobiSVD<svdMatDer>& mat_svd);
  template<int _mat_rows, int _mat_cols, int _mat_align, int max_mat_rows,
        int max_mat_cols, typename nullDer>
  static int computeNullspaceMap(
      const Eigen::Matrix<double, _mat_rows, _mat_cols, _mat_align, max_mat_rows, max_mat_cols>& mat,
      Eigen::MatrixBase<nullDer>& null_map,
      const Eigen::JacobiSVD<typename RtMatrixX<max_mat_rows, max_mat_cols>::d >& mat_svd, const double mat_cond_threas = mat_condition_threash_);
  template<typename Mat>
  static inline double conditionNumber(const Mat& mat)
  {
    Eigen::JacobiSVD<Mat> mat_svd;
    mat_svd.compute(mat);
    return conditionNumberFromSvd(mat_svd);
  }
  template<typename Mat, int QRPreconditioner>
  static inline double conditionNumberFromSvd(const Eigen::JacobiSVD<Mat, QRPreconditioner>& mat_svd)
  {
    return mat_svd.singularValues()[0]/mat_svd.singularValues()[mat_svd.singularValues().size()-1];
  }

  static const double remove_zero_threash_;
  static const double mat_condition_threash_;
private:
};


//template<int _mat_rows, int _mat_cols, int _mat_align, int max_mat_rows, int max_mat_cols,
//  typename invDer, typename nullDer>
//int RtMatrixXUtils::computePseudoInv(
//        const Eigen::Matrix<double, _mat_rows, _mat_cols, _mat_align, max_mat_rows, max_mat_cols>& mat,
//        Eigen::MatrixBase<invDer>& inv, Eigen::MatrixBase<nullDer>& null_proj)
//{
//  Eigen::JacobiSVD<typename RtMatrixX<max_mat_rows, max_mat_cols>::d > mat_svd;
//  int dim_range = 0;
//  RtMatrixXUtils::resize(inv, mat.cols(), mat.rows());
//  RtMatrixXUtils::setIdentity(null_proj, mat.cols());
//
//  if(mat.rows() <= mat.cols())
//  {
//    mat_svd.compute(mat, Eigen::ComputeFullU);  //same as ThinU
//    typename RtMatrixX<max_mat_rows, max_mat_rows>::d diag_inv;
//    RtMatrixXUtils::setZero(diag_inv, mat.rows(), mat.rows());
//    const double first_sv = mat_svd.singularValues()[0];
//    for(dim_range=0; dim_range<mat_svd.singularValues().size(); ++dim_range)
//    {
//      if(mat_svd.singularValues()[dim_range] > 0.0 &&
//          first_sv / mat_svd.singularValues()[dim_range] < mat_condition_threash_)
//        diag_inv(dim_range,dim_range) = 1/mat_svd.singularValues()[dim_range]/mat_svd.singularValues()[dim_range];
//      else
//        break;
//    }
//    inv = mat.transpose() * mat_svd.matrixU() * diag_inv * mat_svd.matrixU().transpose();
//  }
//  else
//  {
//    mat_svd.compute(mat, Eigen::ComputeFullV);  //same as ThinV
//    typename RtMatrixX<max_mat_cols, max_mat_cols>::d diag_inv;
//    RtMatrixXUtils::setZero(diag_inv, mat.cols(), mat.cols());
//    const double first_sv = mat_svd.singularValues()[0];
//    for(dim_range=0; dim_range<mat_svd.singularValues().size(); ++dim_range)
//    {
//      if(mat_svd.singularValues()[dim_range] > 0.0 &&
//          first_sv / mat_svd.singularValues()[dim_range] < mat_condition_threash_)
//        diag_inv(dim_range,dim_range) = 1/mat_svd.singularValues()[dim_range]/mat_svd.singularValues()[dim_range];
//      else
//        break;
//    }
//    inv = mat_svd.matrixV() * diag_inv * mat_svd.matrixV().transpose() * mat.transpose();
//  }
//
//  null_proj -= inv*mat;
//
//#ifndef RTEIG_NO_ASSERTS
//  assert((mat * null_proj).norm() < 0.0001 && "Nullspace projector is not correctly computed");
//  assert((mat*inv*mat - mat).norm() < 0.0001 && "Pseudo Inverse is not correctly computed");
//#endif
//
//  return dim_range;
//}

template<typename sigmaPinvDer, typename svdMatDer>
void RtMatrixXUtils::computeSigmaPinv(Eigen::MatrixBase<sigmaPinvDer>& sigma_pinv_vec,
      const Eigen::JacobiSVD<svdMatDer>& mat_svd)
{
#ifndef RTEIG_NO_ASSERTS
  assert(sigma_pinv_vec.rows() == mat_svd.singularValues().size() &&
         sigma_pinv_vec.cols() == 1 && "sigma_pinv_vec wrongly sized");
#endif

  const double min_sing_val = mat_svd.singularValues()[0]/mat_condition_threash_;
//  sigma_pinv_vec = mat_svd.singularValues();
  int i;
  for(i=0;i<mat_svd.singularValues().size();++i)
    if(mat_svd.singularValues()[i]>min_sing_val)
      sigma_pinv_vec[i] = 1.0/mat_svd.singularValues()[i];
    else
    {
      sigma_pinv_vec[i] = 0.0;
      break;
    }
  sigma_pinv_vec.segment(i+1, sigma_pinv_vec.size()-i);
}

template<int _mat_rows, int _mat_cols, int _mat_align, int max_mat_rows, int max_mat_cols, typename nullDer>
int RtMatrixXUtils::computeNullspaceMap(
    const Eigen::Matrix<double, _mat_rows, _mat_cols, _mat_align, max_mat_rows, max_mat_cols>& mat,
    Eigen::MatrixBase<nullDer>& null_map, const Eigen::JacobiSVD<typename RtMatrixX<max_mat_rows, max_mat_cols>::d >& mat_svd,
    const double mat_cond_threas)
{
  int dim_range = 0;

  const double first_sv = mat_svd.singularValues()[0];
  for(dim_range=0; dim_range<mat_svd.singularValues().size(); ++dim_range)
  {
    if(mat_svd.singularValues()[dim_range] <= 0.0 ||
        first_sv / mat_svd.singularValues()[dim_range] > mat_cond_threas)
      break;
  }

  const int dim_null = mat.cols() - dim_range;
  RtMatrixXUtils::resize(null_map, mat.cols(), dim_null);  //asserts that null_map has enough space

  if(null_map.cols() > 0)
    null_map = mat_svd.matrixV().rightCols(dim_null);

#ifndef RTEIG_NO_ASSERTS
  assert((mat * null_map).norm() < 0.0001 && "Nullspace map is not correctly computed");
  typename RtMatrixX<max_mat_cols, max_mat_cols>::d kernel_id;
  RtMatrixXUtils::setIdentity(kernel_id, dim_null);
  assert((null_map.transpose() * null_map - kernel_id).norm() < 0.0001 && "Nullspace map is not correctly computed");
#endif

  return dim_range;
}

//template<int _mat_rows, int _mat_cols, int _mat_align, int max_mat_rows, int max_mat_cols,
//  typename invDer, typename nullDer>
//int RtMatrixXUtils::computePseudoInvWithNullspaceMap(
//        const Eigen::Matrix<double, _mat_rows, _mat_cols, _mat_align, max_mat_rows, max_mat_cols>& mat,
//        Eigen::MatrixBase<invDer>& inv, Eigen::MatrixBase<nullDer>& null_map)
//{
//  Eigen::JacobiSVD<typename RtMatrixX<max_mat_rows, max_mat_cols>::d > mat_svd;
//  int dim_range = 0;
//  RtMatrixXUtils::resize(inv, mat.cols(), mat.rows());
////  RtMatrixXUtils::setIdentity(null_proj, mat.cols());
//
//  mat_svd.compute(mat, Eigen::ComputeFullV);
//  typename RtMatrixX<max_mat_cols, max_mat_cols>::d diag_inv;
//  RtMatrixXUtils::setZero(diag_inv, mat.cols(), mat.cols());
//  const double first_sv = mat_svd.singularValues()[0];
//  for(dim_range=0; dim_range<mat_svd.singularValues().size(); ++dim_range)
//  {
//    if(mat_svd.singularValues()[dim_range] > 0.0 &&
//        first_sv / mat_svd.singularValues()[dim_range] < mat_condition_threash_)
//      diag_inv(dim_range,dim_range) = 1/mat_svd.singularValues()[dim_range]/mat_svd.singularValues()[dim_range];
//    else
//      break;
//  }
//  inv = mat_svd.matrixV() * diag_inv * mat_svd.matrixV().transpose() * mat.transpose();
//
//  const int dim_null = mat.cols() - dim_range;
//  RtMatrixXUtils::resize(null_map, mat.cols(), dim_null);  //asserts that null_map has enough space
//
//  if(null_map.cols() > 0)
//    null_map = mat_svd.matrixV().rightCols(dim_null);
//
//#ifndef RTEIG_NO_ASSERTS
//  assert((mat * null_map).norm() < 0.0001 && "Nullspace map is not correctly computed");
//  typename RtMatrixX<max_mat_cols, max_mat_cols>::d kernel_id;
//  RtMatrixXUtils::setIdentity(kernel_id, dim_null);
//  assert((null_map.transpose() * null_map - kernel_id).norm() < 0.0001 && "Nullspace map is not correctly computed");
//  assert((mat*inv*mat - mat).norm() < 0.0001 && "Pseudo Inverse is not correctly computed");
//#endif
//
//  return dim_range;
//}

class RtVectorXUtils
{
private:
  RtVectorXUtils(){};
  virtual ~RtVectorXUtils(){};
public:

  template <typename Derived>
  static inline void conservativeResize(Eigen::MatrixBase<Derived>& vec,
        int rows)
  {
#ifndef RTEIG_NO_ASSERTS
    assert(rows <= vec.MaxSizeAtCompileTime);
#endif
    vec.derived().conservativeResize(rows);
  };

  template <typename Derived>
  static inline void resize(Eigen::MatrixBase<Derived>& vec,
        int rows)
  {
#ifndef RTEIG_NO_ASSERTS
    assert(rows <= vec.MaxSizeAtCompileTime);
#endif
    vec.derived().resize(rows);
  };

  template <typename Derived>
    static inline void setZero(Eigen::MatrixBase<Derived>& vec)
  {
#ifdef RTEIG_DYN_ALLOCS
            resize(vec, 1);
#else
            resize(vec, vec.MaxSizeAtCompileTime);
#endif

    vec.derived().setZero();
  }

  template <typename Derived>
    static inline void setZero(Eigen::MatrixBase<Derived>& vec, int size)
  {
    resize(vec, size);
    vec.derived().setZero();
  }

  template <typename Derived>
    static inline void setOnes(Eigen::MatrixBase<Derived>& vec)
  {
#ifdef RTEIG_DYN_ALLOCS
            resize(vec, 1);
#else
            resize(vec, vec.MaxSizeAtCompileTime);
#endif
    vec.derived().setOnes();
  }

  template <typename Derived>
    static inline void setOnes(Eigen::MatrixBase<Derived>& vec, int size)
  {
    resize(vec, size);
    vec.derived().setOnes();
  }

  template <typename Derived>
    static inline void setConstant(Eigen::MatrixBase<Derived>& vec, double scalar)
  {
#ifdef RTEIG_DYN_ALLOCS
            resize(vec, 1);
#else
            resize(vec, vec.MaxSizeAtCompileTime);
#endif
    vec.derived().setConstant(scalar);
  }

  template <typename Derived>
    static inline void setConstant(Eigen::MatrixBase<Derived>& vec,
        int size, double scalar)
  {
    resize(vec, size);
    vec.derived().setConstant(scalar);
  }

  /***
   * Append a vector to another one
   * @param aff_mat
   * @param subst_mat
   */
  template <typename AffVecDerived, typename SubstVecDerived>
  static inline void append(Eigen::MatrixBase<AffVecDerived>& aff_vec,
      const Eigen::MatrixBase<SubstVecDerived>& subst_vec)
  {
#ifndef RTEIG_NO_ASSERTS
    assert(aff_vec.rows() + subst_vec.rows() <= aff_vec.MaxSizeAtCompileTime);
#endif
    const int last_row = aff_vec.rows();
    RtVectorXUtils::conservativeResize(aff_vec, aff_vec.rows() + subst_vec.rows());
    aff_vec.block(last_row, 0, subst_vec.rows(), 1) = subst_vec;
  }
};

class RtAffineUtils
{
private:
  RtAffineUtils(){};
  virtual ~ RtAffineUtils(){};

public:

  /**!
   * Append an affine mapping at the end of another one.
   * @param aff_mat
   * @param subst_mat
   */

  template <typename AffMatDerived, typename AffVecDerived,
        typename SubstMatDerived>
  static inline void append(Eigen::MatrixBase<AffMatDerived>& aff_mat,
        Eigen::MatrixBase<AffVecDerived>& aff_vec,
        const Eigen::MatrixBase<SubstMatDerived>& subst_mat)
  {
#ifndef RTEIG_NO_ASSERTS
    assertMatchingDimensions(aff_mat, aff_vec);
    assert(aff_vec.rows() + subst_mat.rows() <= aff_vec.MaxSizeAtCompileTime);
#endif
    const int last_row = aff_vec.rows();
    RtVectorXUtils::conservativeResize(aff_vec, aff_vec.rows() + subst_mat.rows());
    RtMatrixXUtils::append(aff_mat, subst_mat);
    aff_vec.block(last_row, 0, subst_mat.rows(),1).setZero();
  }

  template <typename AffMatDerived, typename AffVecDerived,
        typename SubstMatDerived, typename SubstVecDerived>
  static inline void append(Eigen::MatrixBase<AffMatDerived>& aff_mat,
        Eigen::MatrixBase<AffVecDerived>& aff_vec,
        const Eigen::MatrixBase<SubstMatDerived>& subst_mat,
        const Eigen::MatrixBase<SubstVecDerived>& subst_vec)
  {
#ifndef RTEIG_NO_ASSERTS
    assertMatchingDimensions(aff_mat, aff_vec);
    assertMatchingDimensions(subst_mat, subst_vec);
#endif

    const int last_row = aff_vec.rows();
    append(aff_mat, aff_vec, subst_mat);
    aff_vec.block(last_row, 0, subst_mat.rows(),1) = subst_vec;
  }

  /**!
   * Append an affine mapping at the end of another one starting from a certein column.
   * AffMat and SubstMat are not required to have the same number of columns.
   * @param aff_mat
   * @param subst_mat
   */
  template <typename AffMatDerived, typename AffVecDerived,
        typename SubstMatDerived>
  static inline void appendAtColumn(Eigen::MatrixBase<AffMatDerived>& aff_mat,
        Eigen::MatrixBase<AffVecDerived>& aff_vec,
        const Eigen::MatrixBase<SubstMatDerived>& subst_mat, int starting_column)
  {
#ifndef RTEIG_NO_ASSERTS
    assertMatchingDimensions(aff_mat, aff_vec);
    assert(aff_vec.rows() + subst_mat.rows() <= aff_vec.MaxSizeAtCompileTime);
#endif
    const int last_row = aff_vec.rows();
    RtVectorXUtils::conservativeResize(aff_vec, aff_vec.rows() + subst_mat.rows());
    RtMatrixXUtils::appendAtColumn(aff_mat, subst_mat, starting_column);
    aff_vec.block(last_row, 0, subst_mat.rows(),1).setZero();
  }

  template <typename AffMatDerived, typename AffVecDerived,
        typename SubstMatDerived, typename SubstVecDerived>
  static inline void appendAtColumn(Eigen::MatrixBase<AffMatDerived>& aff_mat,
        Eigen::MatrixBase<AffVecDerived>& aff_vec,
        const Eigen::MatrixBase<SubstMatDerived>& subst_mat,
        const Eigen::MatrixBase<SubstVecDerived>& subst_vec, int starting_column)
  {
#ifndef RTEIG_NO_ASSERTS
    assertMatchingDimensions(aff_mat, aff_vec);
    assertMatchingDimensions(subst_mat, subst_vec);
#endif

    const int last_row = aff_vec.rows();
    appendAtColumn(aff_mat, aff_vec, subst_mat, starting_column);
    aff_vec.block(last_row, 0, subst_mat.rows(),1) = subst_vec;
  }

  template<int MaxRows, int MaxCols>
  static inline void removeZeroRows(typename RtMatrixX<MaxRows, MaxCols>::d& mat,
        typename RtVectorX<MaxRows>::d& vec,
        double threashold = RtMatrixXUtils::remove_zero_threash_)
  {
    if(mat.rows() == 0)
      return;
    typename RtVectorX<MaxRows>::d squared_norms;
    squared_norms = mat.rowwise().squaredNorm();
    const double normed_threash = std::sqrt(double(squared_norms.size())) * threashold * threashold;
    int num_valids = 0;
    for(int i=0; i<squared_norms.size(); ++i)
      if(squared_norms[i] > normed_threash)
        ++num_valids;

    if(num_valids == 0)
    {
      RtMatrixXUtils::resize(mat, 0, mat.cols());
      RtVectorXUtils::resize(vec, 0);
    }
    else
    {
      typename RtMatrixX<MaxRows, MaxCols>::d selector_mat;
      RtMatrixXUtils::createSelectorMat<MaxRows, MaxCols>(selector_mat, squared_norms, num_valids, normed_threash);
      mat = selector_mat * mat;
      vec = selector_mat * vec;
    }
  };

  /**!
   * Given aff_mat*x + aff_vec, substitute x = subst_mat * y + subst_vec into x.
   * @param aff_mat
   * @param aff_vec
   * @param subst_mat
   * @param subst_vec
   */
  template <typename AffMatDerived, typename AffVecDerived,
        typename SubstMatDerived, typename SubstVecDerived>
  static inline void substitute(Eigen::MatrixBase<AffMatDerived>& aff_mat,
        Eigen::MatrixBase<AffVecDerived>& aff_vec,
        const Eigen::MatrixBase<SubstMatDerived>& subst_mat,
        const Eigen::MatrixBase<SubstVecDerived>& subst_vec)
  {
#ifndef RTEIG_NO_ASSERTS
    assertMatchingDimensions(aff_mat, aff_vec);
    assertMatchingDimensions(subst_mat, subst_vec);
#endif
    aff_vec += aff_mat*subst_vec;
    substitute(aff_mat, subst_mat);
  };

  template <typename AffMatDerived, typename SubstMatDerived>
  static inline void substitute(Eigen::MatrixBase<AffMatDerived>& aff_mat,
        const Eigen::MatrixBase<SubstMatDerived>& subst_mat)
  {
#ifndef RTEIG_NO_ASSERTS
    assert(aff_mat.rows()*subst_mat.cols() <= aff_mat.MaxSizeAtCompileTime);
#endif
    aff_mat = aff_mat*subst_mat;
  };


#ifndef RTEIG_NO_ASSERTS
  template <typename AffMatDerived, typename AffVecDerived>
  static inline void assertMatchingDimensions(const Eigen::MatrixBase<AffMatDerived>& aff_mat,
        const Eigen::MatrixBase<AffVecDerived>& aff_vec)
  {
    assert(aff_mat.rows() == aff_vec.rows());
    assert(aff_vec.cols() == 1);
  };
#endif
};

/**!
 * We ignore constant terms!
 */
class RtQuadraticUtils
{
private:
  RtQuadraticUtils(){};
  virtual ~ RtQuadraticUtils(){};

public:


  // BE CAREFUL with const correctness: quad_mat is NOT const! const will be casted away!
  template <typename QuadMatDerived, typename AffMatDerived>
  static inline void addFromNorm(const Eigen::MatrixBase<QuadMatDerived>& quad_mat,
                                 const Eigen::MatrixBase<AffMatDerived>& aff_mat)
  {
    const_cast<Eigen::MatrixBase<QuadMatDerived>&>(quad_mat) += aff_mat.transpose() * aff_mat;
  }

  // BE CAREFUL with const correctness: quad_mat is NOT const! const will be casted away!
  template <typename QuadMatDerived, typename QuadVecDerived,
        typename AffMatDerived, typename AffVecDerived>
  static inline void addFromNorm(const Eigen::MatrixBase<QuadMatDerived>& quad_mat,
                                 const Eigen::MatrixBase<QuadVecDerived>& quad_vec,
                                 const Eigen::MatrixBase<AffMatDerived>& aff_mat,
                                 const Eigen::MatrixBase<AffVecDerived>& aff_vec)
  {
#ifndef RTEIG_NO_ASSERTS
    assertMatchingDimensions(quad_mat, quad_vec);
    RtAffineUtils::assertMatchingDimensions(aff_mat, aff_vec);
#endif
    addFromNorm(quad_mat, aff_mat);
    const_cast<Eigen::MatrixBase<QuadVecDerived>&>(quad_vec) += aff_mat.transpose() * aff_vec;
  }

  template <typename QuadMatDerived, typename AffMatDerived, typename WeightMatDerived>
  static inline void addFromWeightedNorm(Eigen::MatrixBase<QuadMatDerived>& quad_mat,
                                 const Eigen::MatrixBase<AffMatDerived>& aff_mat,
                                 const Eigen::MatrixBase<WeightMatDerived>& weight_mat)
  {
    quad_mat += aff_mat.transpose() * weight_mat * aff_mat;
  }

  template <typename QuadMatDerived, typename QuadVecDerived,
        typename AffMatDerived, typename AffVecDerived, typename WeightMatDerived>
  static inline void addFromWeightedNorm(Eigen::MatrixBase<QuadMatDerived>& quad_mat,
                                 Eigen::MatrixBase<QuadVecDerived>& quad_vec,
                                 const Eigen::MatrixBase<AffMatDerived>& aff_mat,
                                 const Eigen::MatrixBase<AffVecDerived>& aff_vec,
                                 const Eigen::MatrixBase<WeightMatDerived>& weight_mat)
  {
#ifndef RTEIG_NO_ASSERTS
    assertMatchingDimensions(quad_mat, quad_vec);
    RtAffineUtils::assertMatchingDimensions(aff_mat, aff_vec);
#endif
    addFromWeightedNorm(quad_mat, aff_mat, weight_mat);
    quad_vec += aff_mat.transpose() * weight_mat * aff_vec;
  }

  template <typename QuadMatDerived, typename SubstMatDerived>
  static inline void substitute(Eigen::MatrixBase<QuadMatDerived>& quad_mat,
                                const Eigen::MatrixBase<SubstMatDerived>& subst_mat)
  {
#ifndef RTEIG_NO_ASSERTS
    assert(subst_mat.cols()*subst_mat.cols() <= quad_mat.MaxSizeAtCompileTime);
#endif
    quad_mat = subst_mat.transpose() * quad_mat *subst_mat;
  }

  template <typename QuadMatDerived, typename QuadVecDerived,
  typename SubstMatDerived, typename SubstVecDerived>
  static inline void substitute(Eigen::MatrixBase<QuadMatDerived>& quad_mat,
                                Eigen::MatrixBase<QuadVecDerived>& quad_vec,
                                const Eigen::MatrixBase<SubstMatDerived>& subst_mat,
                                const Eigen::MatrixBase<SubstVecDerived>& subst_vec)
  {
#ifndef RTEIG_NO_ASSERTS
    assertMatchingDimensions(quad_mat, quad_vec);
    RtAffineUtils::assertMatchingDimensions(subst_mat, subst_vec);
    assert(subst_mat.cols() <= quad_vec.MaxSizeAtCompileTime);
#endif
    quad_vec += quad_mat*subst_vec;
    quad_vec = subst_mat.transpose() * quad_vec;
    substitute(quad_mat, subst_mat);

  }

#ifndef RTEIG_NO_ASSERTS
  template <typename QuadMatDerived, typename QuadVecDerived>
  static inline void assertMatchingDimensions(const Eigen::MatrixBase<QuadMatDerived>& quad_mat,
        const Eigen::MatrixBase<QuadVecDerived>& quad_vec)
  {
    assert(quad_mat.rows() == quad_mat.cols() && quad_mat.cols() == quad_vec.rows());
    assert(quad_vec.cols() == 1);
  };
#endif

};

}  //namespace
#endif /* RTMATRIXX_H_ */
