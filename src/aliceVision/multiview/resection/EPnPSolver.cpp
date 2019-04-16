// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "EPnPSolver.hpp"
#include <aliceVision/numeric/projection.hpp>

namespace aliceVision {
namespace multiview {
namespace resection {

/**
 * @brief Selects 4 virtual control points using mean and PCA.
 * @param[in] x3d
 * @param[out] xCentered
 * @param[out] xControlPoints
 */
void selectControlPoints(const Mat3X& x3d, Mat* xCentered, Mat34* xControlPoints)
{
  const std::size_t nbPoints = x3d.cols();

  // the first virtual control point, C0, is the centroid.
  Vec mean, variance;
  MeanAndVarianceAlongRows(x3d, &mean, &variance);
  xControlPoints->col(0) = mean;

  // computes PCA
  xCentered->resize (3, nbPoints);
  for(std::size_t c = 0; c < nbPoints; ++c)
  {
    xCentered->col(c) = x3d.col (c) - mean;
  }

  Mat3 xCenteredSQ = (*xCentered) * xCentered->transpose();
  Eigen::JacobiSVD<Mat3> xCenteredSQsvd(xCenteredSQ, Eigen::ComputeFullU);
  const Vec3 w = xCenteredSQsvd.singularValues();
  const Mat3 u = xCenteredSQsvd.matrixU();

  for(std::size_t c = 0; c < 3; ++c)
  {
    const double k = std::sqrt(w(c) / nbPoints);
    xControlPoints->col(c + 1) = mean + k * u.col(c);
  }
}

/**
 * @brief Computes the barycentric coordinates for all real points
 * @param[in] xWorldCentered
 * @param[in] xControlPoints
 * @param[out] alphas
 */
void computeBarycentricCoordinates(const Mat3X& xWorldCentered, const Mat34& xControlPoints, Mat4X* alphas)
{
  const std::size_t nbPoints = xWorldCentered.cols();
  Mat3 C2;

  for(std::size_t c = 1; c < 4; ++c)
  {
    C2.col(c-1) = xControlPoints.col(c) - xControlPoints.col(0);
  }

  Mat3 C2inv = C2.inverse();
  Mat3X a = C2inv * xWorldCentered;

  alphas->resize(4, nbPoints);
  alphas->setZero();
  alphas->block(1, 0, 3, nbPoints) = a;

  for(std::size_t c = 0; c < nbPoints; ++c)
  {
    (*alphas)(0, c) = 1.0 - alphas->col(c).sum();
  }
}

/**
 * @brief Estimates the coordinates of all real points in the camera coordinate frame
 * @param[in] alphas
 * @param[in] betas
 * @param[in] U
 * @param[out] xCamera
 */
void computePointsCoordinatesInCameraFrame(const Mat4X& alphas, const Vec4& betas, const Eigen::Matrix<double, 12, 12>& U, Mat3X* xCamera)
{
  const std::size_t nbPoints = alphas.cols();

  // estimates the control points in the camera reference frame.
  Mat34 C2b;
  C2b.setZero();
  for(std::size_t cu = 0; cu < 4; ++cu)
  {
    for(std::size_t c = 0; c < 4; ++c)
    {
      C2b.col(c) += betas(cu) * U.block(11 - cu, c * 3, 1, 3).transpose();
    }
  }

  // estimates the 3D points in the camera reference frame
  xCamera->resize(3, nbPoints);
  for(std::size_t c = 0; c < nbPoints; ++c)
  {
    xCamera->col(c) = C2b * alphas.col(c);
  }

  // check the sign of the z coordinate of the points (should be positive)
  std::size_t num_z_neg = 0;
  for(Mat3X::Index i = 0; i < xCamera->cols(); ++i)
  {
    if((*xCamera)(2,i) < 0)
    {
      num_z_neg++;
    }
  }

  // if more than 50% of z are negative, we change the signs
  if(num_z_neg > 0.5 * xCamera->cols())
  {
    C2b = -C2b;
    *xCamera = -(*xCamera);
  }
}

/**
 * @brief absoluteOrientation
 * @param[in] X
 * @param[in] Xp
 * @param[out] R
 * @param[out] t
 */
void absoluteOrientation(const Mat3X& X, const Mat3X& Xp, Mat3* R, Vec3* t)
{
  const int nbPoints = static_cast<int>(X.cols());
  Vec3 C  = X.rowwise().sum() / nbPoints;   // centroid of X.
  Vec3 Cp = Xp.rowwise().sum() / nbPoints;  // centroid of Xp.

  // normalize the two point sets.
  Mat3X Xn(3, nbPoints), Xpn(3, nbPoints);

  for(int i = 0; i < nbPoints; ++i)
  {
    Xn.col(i)  = X.col(i) - C;
    Xpn.col(i) = Xp.col(i) - Cp;
  }

  // construct the N matrix (pg. 635).
  const double Sxx = Xn.row(0).dot(Xpn.row(0));
  const double Syy = Xn.row(1).dot(Xpn.row(1));
  const double Szz = Xn.row(2).dot(Xpn.row(2));
  const double Sxy = Xn.row(0).dot(Xpn.row(1));
  const double Syx = Xn.row(1).dot(Xpn.row(0));
  const double Sxz = Xn.row(0).dot(Xpn.row(2));
  const double Szx = Xn.row(2).dot(Xpn.row(0));
  const double Syz = Xn.row(1).dot(Xpn.row(2));
  const double Szy = Xn.row(2).dot(Xpn.row(1));

  Mat4 N;
  N << Sxx + Syy + Szz, Syz - Szy,        Szx - Sxz,        Sxy - Syx,
       Syz - Szy,       Sxx - Syy - Szz,  Sxy + Syx,        Szx + Sxz,
       Szx - Sxz,       Sxy + Syx,       -Sxx + Syy - Szz,  Syz + Szy,
       Sxy - Syx,       Szx + Sxz,        Syz + Szy,       -Sxx - Syy + Szz;

  // find the unit quaternion q that maximizes qNq. It is the eigenvector
  // corresponding to the largest eigenvalue.
  const Vec4 q = N.jacobiSvd(Eigen::ComputeFullU).matrixU().col(0);

  // retrieve the 3x3 rotation matrix.
  const Vec4 qq = q.array() * q.array();

  const double q0q1 = q(0) * q(1);
  const double q0q2 = q(0) * q(2);
  const double q0q3 = q(0) * q(3);
  const double q1q2 = q(1) * q(2);
  const double q1q3 = q(1) * q(3);
  const double q2q3 = q(2) * q(3);

  (*R) << qq(0) + qq(1) - qq(2) - qq(3),
          2 * (q1q2 - q0q3),
          2 * (q1q3 + q0q2),
          2 * (q1q2+ q0q3),
          qq(0) - qq(1) + qq(2) - qq(3),
          2 * (q2q3 - q0q1),
          2 * (q1q3 - q0q2),
          2 * (q2q3 + q0q1),
          qq(0) - qq(1) - qq(2) + qq(3);

  // fix the handedness of the R matrix.
  if(R->determinant() < 0)
  {
    R->row(2) = -R->row(2);
  }

  // compute the final translation.
  *t = Cp - *R * C;
}

bool EPnPSolver::resection(const Mat2X& x2d, const Mat3X& x3d, Mat3* R, Vec3* t) const
{
  assert(x2d.cols() == x3d.cols());
  assert(x2d.cols() > 3);

  const std::size_t nbPoints = x3d.cols();

  // select the control points.
  Mat34 xControlPoints;
  Mat xCentered;

  selectControlPoints(x3d, &xCentered, &xControlPoints);

  // compute the barycentric coordinates.
  Mat4X alphas(4, nbPoints);
  computeBarycentricCoordinates(xCentered, xControlPoints, &alphas);

  // estimates the M matrix with the barycentric coordinates
  Mat M(2 * nbPoints, 12);

  for(std::size_t c = 0; c < nbPoints; ++c)
  {
    const double a0 = alphas(0, c);
    const double a1 = alphas(1, c);
    const double a2 = alphas(2, c);
    const double a3 = alphas(3, c);
    const double ui = x2d(0, c);
    const double vi = x2d(1, c);

    M.block(2*c, 0, 2, 12) << a0, 0,
                              a0*(-ui), a1, 0,
                              a1*(-ui), a2, 0,
                              a2*(-ui), a3, 0,
                              a3*(-ui), 0,
                              a0, a0*(-vi), 0,
                              a1, a1*(-vi), 0,
                              a2, a2*(-vi), 0,
                              a3, a3*(-vi);
  }

  // @todo: avoid the transpose by rewriting the u2.block() calls.
  Eigen::JacobiSVD<Mat> MtMsvd(M.transpose()*M, Eigen::ComputeFullU);
  Eigen::Matrix<double, 12, 12> u2 = MtMsvd.matrixU().transpose();

  // estimate the L matrix.
  Eigen::Matrix<double, 6, 3> dv1;
  Eigen::Matrix<double, 6, 3> dv2;
  Eigen::Matrix<double, 6, 3> dv3;
  Eigen::Matrix<double, 6, 3> dv4;

  dv1.row(0) = u2.block(11, 0, 1, 3) - u2.block(11, 3, 1, 3);
  dv1.row(1) = u2.block(11, 0, 1, 3) - u2.block(11, 6, 1, 3);
  dv1.row(2) = u2.block(11, 0, 1, 3) - u2.block(11, 9, 1, 3);
  dv1.row(3) = u2.block(11, 3, 1, 3) - u2.block(11, 6, 1, 3);
  dv1.row(4) = u2.block(11, 3, 1, 3) - u2.block(11, 9, 1, 3);
  dv1.row(5) = u2.block(11, 6, 1, 3) - u2.block(11, 9, 1, 3);
  dv2.row(0) = u2.block(10, 0, 1, 3) - u2.block(10, 3, 1, 3);
  dv2.row(1) = u2.block(10, 0, 1, 3) - u2.block(10, 6, 1, 3);
  dv2.row(2) = u2.block(10, 0, 1, 3) - u2.block(10, 9, 1, 3);
  dv2.row(3) = u2.block(10, 3, 1, 3) - u2.block(10, 6, 1, 3);
  dv2.row(4) = u2.block(10, 3, 1, 3) - u2.block(10, 9, 1, 3);
  dv2.row(5) = u2.block(10, 6, 1, 3) - u2.block(10, 9, 1, 3);
  dv3.row(0) = u2.block( 9, 0, 1, 3) - u2.block( 9, 3, 1, 3);
  dv3.row(1) = u2.block( 9, 0, 1, 3) - u2.block( 9, 6, 1, 3);
  dv3.row(2) = u2.block( 9, 0, 1, 3) - u2.block( 9, 9, 1, 3);
  dv3.row(3) = u2.block( 9, 3, 1, 3) - u2.block( 9, 6, 1, 3);
  dv3.row(4) = u2.block( 9, 3, 1, 3) - u2.block( 9, 9, 1, 3);
  dv3.row(5) = u2.block( 9, 6, 1, 3) - u2.block( 9, 9, 1, 3);
  dv4.row(0) = u2.block( 8, 0, 1, 3) - u2.block( 8, 3, 1, 3);
  dv4.row(1) = u2.block( 8, 0, 1, 3) - u2.block( 8, 6, 1, 3);
  dv4.row(2) = u2.block( 8, 0, 1, 3) - u2.block( 8, 9, 1, 3);
  dv4.row(3) = u2.block( 8, 3, 1, 3) - u2.block( 8, 6, 1, 3);
  dv4.row(4) = u2.block( 8, 3, 1, 3) - u2.block( 8, 9, 1, 3);
  dv4.row(5) = u2.block( 8, 6, 1, 3) - u2.block( 8, 9, 1, 3);

  Eigen::Matrix<double, 6, 10> L;
  for(std::size_t r = 0; r < 6; ++r)
  {
    L.row(r) << dv1.row(r).dot(dv1.row(r)),
          2.0 * dv1.row(r).dot(dv2.row(r)),
                dv2.row(r).dot(dv2.row(r)),
          2.0 * dv1.row(r).dot(dv3.row(r)),
          2.0 * dv2.row(r).dot(dv3.row(r)),
                dv3.row(r).dot(dv3.row(r)),
          2.0 * dv1.row(r).dot(dv4.row(r)),
          2.0 * dv2.row(r).dot(dv4.row(r)),
          2.0 * dv3.row(r).dot(dv4.row(r)),
                dv4.row(r).dot(dv4.row(r));
  }

  Vec rho;
  rho.resize(6);
  rho << (xControlPoints.col(0) - xControlPoints.col(1)).squaredNorm(),
         (xControlPoints.col(0) - xControlPoints.col(2)).squaredNorm(),
         (xControlPoints.col(0) - xControlPoints.col(3)).squaredNorm(),
         (xControlPoints.col(1) - xControlPoints.col(2)).squaredNorm(),
         (xControlPoints.col(1) - xControlPoints.col(3)).squaredNorm(),
         (xControlPoints.col(2) - xControlPoints.col(3)).squaredNorm();

  // there are three possible solutions based on the three approximations of L
  // (betas). below, each one is solved for then the best one is chosen.
  Mat3X xCamera;
  Mat3 K;  K.setIdentity();
  std::vector<Mat3> Rs(3);
  std::vector<Vec3> ts(3);
  Vec rmse(3);

  bool bSol = false;

  // find the first possible solution for R, t corresponding to:
  // Betas          = [b00 b01 b11 b02 b12 b22 b03 b13 b23 b33]
  // Betas_approx_1 = [b00 b01     b02         b03]
  Vec4 betas = Vec4::Zero();
  Eigen::Matrix<double, 6, 4> l_6x4;

  for(std::size_t r = 0; r < 6; ++r)
  {
    l_6x4.row(r) << L(r, 0), L(r, 1), L(r, 3), L(r, 6);
  }

  Eigen::JacobiSVD<Mat> svd_of_l4(l_6x4, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Vec4 b4 = svd_of_l4.solve(rho);

  if((l_6x4 * b4).isApprox(rho, 1e-3))
  {
    if(b4(0) < 0)
    {
      b4 = -b4;
    }

    b4(0) =  std::sqrt(b4(0));
    betas << b4(0), b4(1) / b4(0), b4(2) / b4(0), b4(3) / b4(0);
    computePointsCoordinatesInCameraFrame(alphas, betas, u2, &xCamera);
    absoluteOrientation(x3d, xCamera, &Rs[0], &ts[0]);
    rmse(0) = reprojectionErrorRMSE(x2d, x3d, K, Rs[0], ts[0]);
    bSol = true;
  }
  else
  {
    // ALICEVISION_LOG_WARNING("First approximation of beta not good enough.");
    ts[0].setZero();
    rmse(0) = std::numeric_limits<double>::max();
  }

  // find the second possible solution for R, t corresponding to:
  // Betas          = [b00 b01 b11 b02 b12 b22 b03 b13 b23 b33]
  // Betas_approx_2 = [b00 b01 b11]
  betas.setZero();
  Eigen::Matrix<double, 6, 3> l_6x3;
  l_6x3 = L.block(0, 0, 6, 3);
  Eigen::JacobiSVD<Mat> svdOfL3(l_6x3, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Vec3 b3 = svdOfL3.solve(rho);

  if((l_6x3 * b3).isApprox(rho, 1e-3))
  {
    if(b3(0) < 0)
    {
      betas(0) = std::sqrt(-b3(0));
      betas(1) = (b3(2) < 0) ? std::sqrt(-b3(2)) : 0;
    }
    else
    {
      betas(0) = std::sqrt(b3(0));
      betas(1) = (b3(2) > 0) ? std::sqrt(b3(2)) : 0;
    }
    if (b3(1) < 0) {
      betas(0) = -betas(0);
    }

    betas(2) = 0;
    betas(3) = 0;
    computePointsCoordinatesInCameraFrame(alphas, betas, u2, &xCamera);
    absoluteOrientation(x3d, xCamera, &Rs[1], &ts[1]);
    rmse(1) = reprojectionErrorRMSE(x2d, x3d, K, Rs[1], ts[1]);
    bSol = true;
  }
  else
  {
    // ALICEVISION_LOG_WARNING("Second approximation of beta not good enough.");
    ts[1].setZero();
    rmse(1) = std::numeric_limits<double>::max();
  }

  // find the third possible solution for R, t corresponding to:
  // Betas          = [b00 b01 b11 b02 b12 b22 b03 b13 b23 b33]
  // Betas_approx_3 = [b00 b01 b11 b02 b12]
  betas.setZero();
  Eigen::Matrix<double, 6, 5> l_6x5;
  l_6x5 = L.block(0, 0, 6, 5);
  Eigen::JacobiSVD<Mat> svdOfL5(l_6x5, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Vec b5 = svdOfL5.solve(rho);

  if((l_6x5 * b5).isApprox(rho, 1e-3))
  {
    if (b5(0) < 0)
    {
      betas(0) = std::sqrt(-b5(0));

      if(b5(2) < 0)
      {
        betas(1) = std::sqrt(-b5(2));
      }
      else
      {
        b5(2) = 0;
      }
    }
    else
    {
      betas(0) = std::sqrt(b5(0));

      if (b5(2) > 0)
      {
        betas(1) = std::sqrt(b5(2));
      }
      else
      {
        b5(2) = 0;
      }
    }

    if(b5(1) < 0)
    {
      betas(0) = -betas(0);
    }
    betas(2) = b5(3) / betas(0);
    betas(3) = 0;
    computePointsCoordinatesInCameraFrame(alphas, betas, u2, &xCamera);
    absoluteOrientation(x3d, xCamera, &Rs[2], &ts[2]);
    rmse(2) = reprojectionErrorRMSE(x2d, x3d, K, Rs[2], ts[2]);
    bSol = true;
  }
  else
  {
    // ALICEVISION_LOG_WARNING("Third approximation of beta not good enough.");
    ts[2].setZero();
    rmse(2) = std::numeric_limits<double>::max();
  }

  // finally, with all three solutions, select the (R, t) with the best RMSE.
  std::size_t n = 0;
  if (rmse(1) < rmse(0))
  {
    n = 1;
  }

  if(rmse(2) < rmse(n))
  {
    n = 2;
  }

  if(bSol)
  {
    // If at least one solution have been found
    *R = Rs[n];
    *t = ts[n];

    return true;
  }
  return false;
}

void EPnPSolver::solve(const Mat& x2d, const Mat& x3d, std::vector<robustEstimation::Mat34Model>& models) const
{
  Mat3 R;
  Vec3 t;
  Mat34 P;

  if(resection(x2d, x3d, &R, &t))
  {
    P_from_KRt(Mat3::Identity(), R, t, &P); // K = Id
    models.emplace_back(P);
  }
}

}  // namespace resection
}  // namespace multiview
}  // namespace aliceVision
