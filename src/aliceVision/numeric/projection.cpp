// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// Copyright (c) 2010 libmv contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/numeric/projection.hpp>
#include <aliceVision/numeric/numeric.hpp>

#include <cmath>

namespace aliceVision {

/// Compute P = K[R|t]
void P_From_KRt(const Mat3 &K,  const Mat3 &R,  const Vec3 &t, Mat34 *P)
{
  assert(P != nullptr);
  *P = P_From_KRt(K, R, t);
}

Mat34 P_From_KRt(const Mat3 &K, const Mat3 &R, const Vec3 &t)
{
  return K * HStack(R,t);
}

void KRt_From_P(const Mat34 &P, Mat3 *Kp, Mat3 *Rp, Vec3 *tp)
{
  assert(Kp != nullptr);
  assert(Rp != nullptr);
  assert(tp != nullptr);

  // Decompose using the RQ decomposition HZ A4.1.1 pag.579.
  Mat3 K = P.block(0, 0, 3, 3);

  Mat3 Q;
  Q.setIdentity();

  // Set K(2,1) to zero.
  if (K(2,1) != 0)
  {
    double c = -K(2,2);
    double s = K(2,1);
    const double length = std::hypot(c, s);
    c /= length;
    s /= length;
    Mat3 Qx;
    Qx << 1, 0, 0,
          0, c, -s,
          0, s, c;
    K = K * Qx;
    Q = Qx.transpose() * Q;
  }
  // Set K(2,0) to zero.
  if (K(2,0) != 0)
  {
    double c = K(2,2);
    double s = K(2,0);
    const double length = std::hypot(c, s);
    c /= length;
    s /= length;
    Mat3 Qy;
    Qy << c, 0, s,
          0, 1, 0,
         -s, 0, c;
    K = K * Qy;
    Q = Qy.transpose() * Q;
  }
  // Set K(1,0) to zero.
  if (K(1,0) != 0)
  {
    double c = -K(1,1);
    double s = K(1,0);
    const double length = std::hypot(c, s);
    c /= length;
    s /= length;
    Mat3 Qz;
    Qz << c,-s, 0,
          s, c, 0,
          0, 0, 1;
    K = K * Qz;
    Q = Qz.transpose() * Q;
  }

  Mat3 R = Q;

  //Mat3 H = P.block(0, 0, 3, 3);
  // RQ decomposition
  //Eigen::HouseholderQR<Mat3> qr(H);
  //Mat3 K = qr.matrixQR().triangularView<Eigen::Upper>();
  //Mat3 R = qr.householderQ();

  // Ensure that the diagonal is positive and R determinant == 1.
  if (K(2,2) < 0)
  {
    K = -K;
    R = -R;
  }
  if (K(1,1) < 0)
  {
    Mat3 S;
    S << 1, 0, 0,
         0,-1, 0,
         0, 0, 1;
    K = K * S;
    R = S * R;
  }
  if (K(0,0) < 0)
  {
    Mat3 S;
    S << -1, 0, 0,
          0, 1, 0,
          0, 0, 1;
    K = K * S;
    R = S * R;
  }

  // Compute translation.
  Eigen::PartialPivLU<Mat3> lu(K);
  Vec3 t = lu.solve(P.col(3));

  if(R.determinant() < 0)
  {
    R = -R;
    t = -t;
  }

  // scale K so that K(2,2) = 1
  K = K / K(2,2);

  *Kp = K;
  *Rp = R;
  *tp = t;
}

Mat3 F_from_P(const Mat34 & P1, const Mat34 & P2)
{
  Mat3 F12;

  typedef Eigen::Matrix<double, 2, 4> Mat24;
  Mat24 X1 = P1.block<2, 4>(1, 0);
  Mat24 X2;
  X2 << P1.row(2), P1.row(0);
  Mat24 X3 = P1.block<2, 4>(0, 0);
  Mat24 Y1 = P2.block<2, 4>(1, 0);
  Mat24 Y2;
  Y2 << P2.row(2), P2.row(0);
  Mat24 Y3 = P2.block<2, 4>(0, 0);


  Mat4 X1Y1, X2Y1, X3Y1, X1Y2, X2Y2, X3Y2, X1Y3, X2Y3, X3Y3;
  X1Y1 << X1, Y1;
  X2Y1 << X2, Y1;
  X3Y1 << X3, Y1;
  X1Y2 << X1, Y2;
  X2Y2 << X2, Y2;
  X3Y2 << X3, Y2;
  X1Y3 << X1, Y3;
  X2Y3 << X2, Y3;
  X3Y3 << X3, Y3;


  F12 << X1Y1.determinant(), X2Y1.determinant(), X3Y1.determinant(),
          X1Y2.determinant(), X2Y2.determinant(), X3Y2.determinant(),
          X1Y3.determinant(), X2Y3.determinant(), X3Y3.determinant();

  return F12;
}

Vec2 Project(const Mat34 &P, const Vec3 &X)
{
  return Vec3(P * X.homogeneous()).hnormalized();
}

void Project(const Mat34 &P, const Mat3X &X, Mat2X *x)
{
  assert(x != nullptr);
  Project(P, Mat4X(X.colwise().homogeneous()), x);
}

void Project(const Mat34 &P, const Mat4X &X, Mat2X *x)
{
  assert(x != nullptr);
  *x = Project(P, X);
}

Mat2X Project(const Mat34 &P, const Mat3X &X)
{
  return Project(P, Mat4X(X.colwise().homogeneous()));
}

Mat2X Project(const Mat34 &P, const Mat4X &X)
{
  return Mat3X(P * X).colwise().hnormalized();
}

void HomogeneousToEuclidean(const Vec4 &H, Vec3 *X)
{
  assert(X != nullptr);
  *X = H.hnormalized();
}

void EuclideanToHomogeneous(const Mat &X, Mat *H)
{
  assert(H != nullptr);
  *H = X.colwise().homogeneous();
}

double Depth(const Mat3 &R, const Vec3 &t, const Vec3 &X)
{
  return (R*X)[2] + t[2];
}

Vec3 EuclideanToHomogeneous(const Vec2 &x)
{
  return x.homogeneous();
}

void HomogeneousToEuclidean(const Mat &H, Mat *X)
{
  assert(X != nullptr);
  *X = H.colwise().hnormalized();
}

Mat3X EuclideanToHomogeneous(const Mat2X &x)
{
  return x.colwise().homogeneous();
}

void EuclideanToHomogeneous(const Mat2X &x, Mat3X *h)
{
  assert(h != nullptr);
  *h = x.colwise().homogeneous();
}

void HomogeneousToEuclidean(const Mat3X &h, Mat2X *e)
{
  assert(e != nullptr);
  *e = h.colwise().hnormalized();
}

void EuclideanToNormalizedCamera(const Mat2X &x, const Mat3 &K, Mat2X *n)
{
  assert(n != nullptr);
  HomogeneousToNormalizedCamera(x.colwise().homogeneous(), K, n);
}

void HomogeneousToNormalizedCamera(const Mat3X &x, const Mat3 &K, Mat2X *n)
{
  assert(n != nullptr);
  *n = (K.inverse() * x).colwise().hnormalized();
}

/// Estimates the root mean square error (2D)
double reprojectionErrorRMSE(const Mat2X &x_image,
                           const Mat4X &X_world,
                           const Mat34 &P)
{
    const std::size_t num_points = x_image.cols();
    const Mat2X dx = Project(P, X_world) - x_image;
    return std::sqrt(dx.squaredNorm() / num_points);
}

/// Estimates the root mean square error (2D)
double reprojectionErrorRMSE(const Mat2X &x_image,
                           const Mat3X &X_world,
                           const Mat3 &K,
                           const Mat3 &R,
                           const Vec3 &t)
{
    Mat34 P;
    P_From_KRt(K, R, t, &P);
    return reprojectionErrorRMSE(x_image, X_world.colwise().homogeneous(), P);
}

} // namespace aliceVision
