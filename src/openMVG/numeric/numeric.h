
// Copyright (c) 2007, 2008 libmv authors.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.

// Copyright (c) 2012, 2013 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef OPENMVG_NUMERIC_NUMERIC_H
#define OPENMVG_NUMERIC_NUMERIC_H

//--
// Eigen
// http://eigen.tuxfamily.org/dox-devel/QuickRefPage.html
//--
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/QR>
#include <Eigen/SparseCore>
#include <Eigen/SVD>
#include <Eigen/StdVector>

#include <cmath>
#include <numeric>
#include <string>
#include <iostream>
#include <vector>

namespace openMVG {

// Check MSVC
#if _WIN32 || _WIN64
#if _WIN64
#define ENV64BIT
#else
#define ENV32BIT
#endif
#endif

// Check GCC
#if __GNUC__
#if __x86_64__ || __ppc64__ || _LP64
#define ENV64BIT
#else
#define ENV32BIT
#endif
#endif

using Eigen::Map;

typedef Eigen::NumTraits<double> EigenDoubleTraits;

typedef Eigen::Vector3d Vec3;
typedef Eigen::Vector2i Vec2i;
typedef Eigen::Vector2f Vec2f;
typedef Eigen::Vector3f Vec3f;
typedef Eigen::Matrix<double, 9, 1> Vec9;

typedef Eigen::Quaternion<double> Quaternion;

typedef Eigen::Matrix<double, 3, 3> Mat3;

#if defined(ENV32BIT)
typedef Eigen::Matrix<double, 3, 4, Eigen::DontAlign> Mat34;
typedef Eigen::Matrix<double, 2, 1, Eigen::DontAlign> Vec2;
typedef Eigen::Matrix<double, 4, 1, Eigen::DontAlign> Vec4;
typedef Eigen::Matrix<double, 6, 1, Eigen::DontAlign> Vec6;
#else // 64 bits compiler
typedef Eigen::Matrix<double, 3, 4> Mat34;
typedef Eigen::Vector2d Vec2;
typedef Eigen::Vector4d Vec4;
typedef Eigen::Matrix<double, 6, 1> Vec6;
#endif


typedef Eigen::Matrix<double, 4, 4> Mat4;
typedef Eigen::Matrix<unsigned int, Eigen::Dynamic, Eigen::Dynamic> Matu;

typedef Eigen::Matrix<double, 3, 3, Eigen::RowMajor> RMat3;

//-- General purpose Matrix and Vector
typedef Eigen::MatrixXd Mat;
typedef Eigen::VectorXd Vec;
typedef Eigen::Matrix<unsigned int, Eigen::Dynamic, 1> Vecu;
typedef Eigen::MatrixXf Matf;
typedef Eigen::VectorXf Vecf;

typedef Eigen::Matrix<double, 2, Eigen::Dynamic> Mat2X;
typedef Eigen::Matrix<double, 3, Eigen::Dynamic> Mat3X;
typedef Eigen::Matrix<double, 4, Eigen::Dynamic> Mat4X;

typedef Eigen::Matrix<double, Eigen::Dynamic, 9> MatX9;

//-- Sparse Matrix (Column major, and row major)
typedef Eigen::SparseMatrix<double> sMat;
typedef Eigen::SparseMatrix<double, Eigen::RowMajor> sRMat;

//--------------
//-- Function --
//--------------

/// Return the square of a number.

template<typename T>
inline T Square(T x)
{
  return x * x;
}

/// Clamp return the number if inside range, else min or max range.

template<typename T>
inline T clamp(const T & val, const T& min, const T & max)
{
  return std::max(min, std::min(val, max));
  //(val < min) ? val : ((val>max) ? val : max);
}

Mat3 CrossProductMatrix(const Vec3 &x);

// Create a rotation matrix around axis X with the provided radian angle
Mat3 RotationAroundX(double angle);

// Create a rotation matrix around axis Y with the provided radian angle
Mat3 RotationAroundY(double angle);

// Create a rotation matrix around axis Z with the provided radian angle
Mat3 RotationAroundZ(double angle);

Mat3 rotationXYZ(double angleX, double angleY, double angleZ);

// Degree to Radian (suppose input in [0;360])

inline double D2R(double degree)
{
  return degree * M_PI / 180.0;
}

// Radian to degree

inline double R2D(double radian)
{
  return radian / M_PI * 180.0;
}

/// Return in radian the mean rotation amplitude of the given rotation matrix
/// Computed as the mean of matrix column dot products to an Identity matrix
double getRotationMagnitude(const Mat3 & R2);

/**
 * @brief Compute the angle between two rotation matrices.
 * @param[in] R1 The first rotation matrix.
 * @param[in] R2 The second rotation matrix.
 * @return The angle between the two rotations as the angle of the rotation 
 * matrix R1*R2.transpose().
 */
double rotationDifference(const Mat3 & R1, const Mat3 & R2);

inline double SIGN(double x)
{
  return x < 0.0 ? -1.0 : 1.0;
}

// L1 norm = Sum (|x0| + |x1| + |xn|)

template<typename TVec>
inline double NormL1(const TVec &x)
{
  return x.array().abs().sum();
}

// L2 norm = Sqrt (Sum (x0^2 + x1^2 + xn^2))

template<typename TVec>
inline double NormL2(const TVec &x)
{
  return x.norm();
}

// LInfinity norm = max (|x0|, |x1|, ..., |xn|)

template<typename TVec>
inline double NormLInfinity(const TVec &x)
{
  return x.array().abs().maxCoeff();
}

template<typename TVec>
inline double DistanceL1(const TVec &x, const TVec &y)
{
  return (x - y).array().abs().sum();
}

template<typename TVec>
inline double DistanceL2(const TVec &x, const TVec &y)
{
  return (x - y).norm();
}

template<typename TVec>
inline double DistanceLInfinity(const TVec &x, const TVec &y)
{
  return NormLInfinity(x - y);
}

template<typename TVec>
inline bool AreVecNearEqual(const TVec& x, const TVec& y, const double epsilon)
{
  assert(x.cols() == y.cols());
  for(typename TVec::Index i = 0; i < x.cols(); ++i)
  {
    if((y(i) - epsilon > x(i)) 
      || (x(i) > y(i) + epsilon))
      return false;
  }
  return true;
}

template<typename TMat>
inline bool AreMatNearEqual(const TMat& X, const TMat& Y, const double epsilon)
{
  assert(X.cols() == Y.cols());
  assert(X.rows() == Y.rows());
  for(typename TMat::Index i = 0; i < X.rows(); ++i)
  {
    for(typename TMat::Index j = 0; j < X.cols(); ++j)
    {
      if((Y(i,j) - epsilon > X(i,j)) 
        || (X(i,j) > Y(i,j) + epsilon))
        return false;    
    }
  }
  return true;
}

// Solve the linear system Ax = 0 via SVD. Store the solution in x, such that
// ||x|| = 1.0. Return the singular value corresponding to the solution.
// Destroys A and resizes x if necessary.

template <typename TMat, typename TVec>
double Nullspace(TMat *A, TVec *nullspace)
{
  if(A->rows() >= A->cols())
  {
    Eigen::JacobiSVD<TMat> svd(*A, Eigen::ComputeFullV);
    (*nullspace) = svd.matrixV().col(A->cols() - 1);
    return svd.singularValues()(A->cols() - 1);
  }
  // Extend A with rows of zeros to make it square. It's a hack, but is
  // necessary until Eigen supports SVD with more columns than rows.
  TMat A_extended(A->cols(), A->cols());
  A_extended.block(A->rows(), 0, A->cols() - A->rows(), A->cols()).setZero();
  A_extended.block(0, 0, A->rows(), A->cols()) = (*A);
  return Nullspace(&A_extended, nullspace);
}

/// Solve the linear system Ax = 0 via SVD. Finds two solutions, x1 and x2, such
/// that x1 is the best solution and x2 is the next best solution (in the L2
/// norm sense). Store the solution in x1 and x2, such that ||x|| = 1.0. Return
/// the singular value corresponding to the solution x1. Destroys A and resizes
/// x if necessary.

template <typename TMat, typename TVec1, typename TVec2>
inline double Nullspace2(TMat *A, TVec1 *x1, TVec2 *x2)
{
  if(A->rows() >= A->cols())
  {
    Eigen::JacobiSVD<TMat> svd(*A, Eigen::ComputeFullV);
    TMat V = svd.matrixV();
    *x1 = V.col(A->cols() - 1);
    *x2 = V.col(A->cols() - 2);
    return svd.singularValues()(A->cols() - 1);
  }
  // Extend A with rows of zeros to make it square. It's a hack, but is
  // necessary until Eigen supports SVD with more columns than rows.
  TMat A_extended(A->cols(), A->cols());
  A_extended.block(A->rows(), 0, A->cols() - A->rows(), A->cols()).setZero();
  A_extended.block(0, 0, A->rows(), A->cols()) = (*A);
  return Nullspace2(&A_extended, x1, x2);
}

// Make a rotation matrix such that center becomes the direction of the
// positive z-axis, and y is oriented close to up by default.
Mat3 LookAt(const Vec3 &center, const Vec3 & up = Vec3::UnitY());

Mat3 LookAt2(const Vec3 &eyePosition3D,
             const Vec3 &center3D = Vec3::Zero(),
             const Vec3 &upVector3D = Vec3::UnitY());

#define SUM_OR_DYNAMIC(x,y) (x==Eigen::Dynamic||y==Eigen::Dynamic)?Eigen::Dynamic:(x+y)

template<typename Derived1, typename Derived2>
struct hstack_return
{
  typedef typename Derived1::Scalar Scalar;

  enum
  {
    RowsAtCompileTime = Derived1::RowsAtCompileTime,
    ColsAtCompileTime = SUM_OR_DYNAMIC(Derived1::ColsAtCompileTime, Derived2::ColsAtCompileTime),
    Options = Derived1::Flags & Eigen::RowMajorBit ? Eigen::RowMajor : 0,
    MaxRowsAtCompileTime = Derived1::MaxRowsAtCompileTime,
    MaxColsAtCompileTime = SUM_OR_DYNAMIC(Derived1::MaxColsAtCompileTime, Derived2::MaxColsAtCompileTime)
  };
  typedef Eigen::Matrix<Scalar,
  RowsAtCompileTime,
  ColsAtCompileTime,
  Options,
  MaxRowsAtCompileTime,
  MaxColsAtCompileTime> type;
};

template<typename Derived1, typename Derived2>
typename hstack_return<Derived1, Derived2>::type
HStack(const Eigen::MatrixBase<Derived1>& lhs, const Eigen::MatrixBase<Derived2>& rhs)
{
  typename hstack_return<Derived1, Derived2>::type res;
  res.resize(lhs.rows(), lhs.cols() + rhs.cols());
  res << lhs, rhs;
  return res;
}

template<typename Derived1, typename Derived2>
struct vstack_return
{
  typedef typename Derived1::Scalar Scalar;

  enum
  {
    RowsAtCompileTime = SUM_OR_DYNAMIC(Derived1::RowsAtCompileTime, Derived2::RowsAtCompileTime),
    ColsAtCompileTime = Derived1::ColsAtCompileTime,
    Options = Derived1::Flags & Eigen::RowMajorBit ? Eigen::RowMajor : 0,
    MaxRowsAtCompileTime = SUM_OR_DYNAMIC(Derived1::MaxRowsAtCompileTime, Derived2::MaxRowsAtCompileTime),
    MaxColsAtCompileTime = Derived1::MaxColsAtCompileTime
  };
  typedef Eigen::Matrix<Scalar,
  RowsAtCompileTime,
  ColsAtCompileTime,
  Options,
  MaxRowsAtCompileTime,
  MaxColsAtCompileTime> type;
};

template<typename Derived1, typename Derived2>
typename vstack_return<Derived1, Derived2>::type
VStack(const Eigen::MatrixBase<Derived1>& lhs, const Eigen::MatrixBase<Derived2>& rhs)
{
  typename vstack_return<Derived1, Derived2>::type res;
  res.resize(lhs.rows() + rhs.rows(), lhs.cols());
  res << lhs, rhs;
  return res;
}
#undef SUM_OR_DYNAMIC

template<typename TMat>
inline double FrobeniusNorm(const TMat &A)
{
  return sqrt(A.array().abs2().sum());
}

template<typename TMat>
inline double FrobeniusDistance(const TMat &A, const TMat &B)
{
  return FrobeniusNorm(A - B);
}

template<class TMat>
double CosinusBetweenMatrices(const TMat &a, const TMat &b)
{
  return (a.array() * b.array()).sum() /
          FrobeniusNorm(a) / FrobeniusNorm(b);
}

/**
 * @brief It extracts the columns of given indices from the given matrix
 * 
 * @param[in] A The NxM input matrix
 * @param[in] columns The list of K indices to extract
 * @return A NxK matrix
 */
template <typename TMat, typename TCols>
TMat ExtractColumns(const TMat &A, const TCols &columns)
{
  TMat compressed(A.rows(), columns.size());
  for(std::size_t i = 0; i < static_cast<std::size_t> (columns.size()); ++i)
  {
    // check for indices out of range
    assert(columns[i]<A.cols());
    compressed.col(i) = A.col(columns[i]);
  }
  return compressed;
}

void MeanAndVarianceAlongRows(const Mat &A,
                              Vec *mean_pointer,
                              Vec *variance_pointer);

bool exportMatToTextFile(const Mat & mat, const std::string & filename,
                         const std::string & sPrefix = "A");

inline int is_finite(const double val)
{
#ifdef _MSC_VER
  return _finite(val);
#else
  return std::isfinite(val);
#endif
}

/// Get back the min, mean, median and the max
///  values of an iterable sequence.

template <typename Type, typename DataInputIterator>
void minMaxMeanMedian(DataInputIterator begin, DataInputIterator end,
                      Type & min, Type & max, Type & mean, Type & median)
{
  if(std::distance(begin, end) < 1)
    return;

  std::vector<Type> vec_val(begin, end);
  std::sort(vec_val.begin(), vec_val.end());
  min = vec_val[0];
  max = vec_val[vec_val.size() - 1];
  mean = accumulate(vec_val.begin(), vec_val.end(), Type(0))
          / static_cast<Type> (vec_val.size());
  median = vec_val[vec_val.size() / 2];
}

/// Display to the console the min, mean, median and the max
///  values of an iterable sequence.

template <typename Type, typename DataInputIterator>
void minMaxMeanMedian(DataInputIterator begin, DataInputIterator end)
{
  Type min, max, mean, median;
  minMaxMeanMedian(begin, end, min, max, mean, median);
  std::cout << "\n"
          << "\t min: " << min << "\n"
          << "\t mean: " << mean << "\n"
          << "\t median: " << median << std::endl
          << "\t max: " << max << std::endl;
}

/**
 ** Split a range [ a ; b [ into a set of n ranges :
 [ a ; c1 [ U [ c1 ; c2 [ U ... U [ c(n-1) ; b [
 **
  Output range vector only store [ a , c1 , c2 , ... , b ]

 ** if input range can't be split (range [a;b[ size is less than nb_split, only return [a;b[ range
 **
 ** @param range_start Start of range to split
 ** @param range_end End of range to split
 ** @param nb_split Number of desired split
 ** @param d_range Output splitted range
 **/
template < typename T >
void SplitRange(const T range_start, const T range_end, const int nb_split,
                std::vector< T > & d_range)
{
  const T range_length = range_end - range_start;
  if(range_length < nb_split)
  {
    d_range.push_back(range_start);
    d_range.push_back(range_end);
  }
  else
  {
    const T delta_range = range_length / nb_split;

    d_range.push_back(range_start);
    for(int i = 1; i < nb_split; ++i)
    {
      d_range.push_back(range_start + i * delta_range);
    }
    d_range.push_back(range_end);
  }
}


} // namespace openMVG


#endif  // OPENMVG_NUMERIC_NUMERIC_H
