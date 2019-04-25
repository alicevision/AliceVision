// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// Copyright (c) 2011 Laurent Kneip, ETH Zurich.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "P3PSolver.hpp"
#include <aliceVision/numeric/projection.hpp>

namespace aliceVision {
namespace multiview {
namespace resection {

void solveQuartic(const Vec5& factors, Vec4& realRoots)
{
  const double A = factors[0];
  const double B = factors[1];
  const double C = factors[2];
  const double D = factors[3];
  const double E = factors[4];

  const double A_pw2 = A*A;
  const double B_pw2 = B*B;
  const double A_pw3 = A_pw2*A;
  const double B_pw3 = B_pw2*B;
  const double A_pw4 = A_pw3*A;
  const double B_pw4 = B_pw3*B;

  const double alpha = -3 * B_pw2 / (8 * A_pw2) + C / A;
  const double beta = B_pw3 / (8 * A_pw3) - B * C / (2 * A_pw2) + D / A;
  const double gamma = -3 * B_pw4 / (256 * A_pw4) + B_pw2 * C / (16 * A_pw3) - B * D / (4 * A_pw2) + E / A;

  const double alpha_pw2 = alpha * alpha;
  const double alpha_pw3 = alpha_pw2 * alpha;

  const std::complex<double> P(-alpha_pw2 / 12 - gamma, 0);
  const std::complex<double> Q(-alpha_pw3 / 108 + alpha * gamma / 3 - pow(beta, 2) / 8, 0);
  const std::complex<double> R = -Q / 2.0 + sqrt(pow(Q, 2.0) / 4.0 + pow(P, 3.0) / 27.0);

  const std::complex<double> U = pow(R, (1.0 / 3.0));
  std::complex<double> y;

  if(U.real() == 0)
    y = -5.0 * alpha / 6.0 - pow(Q, (1.0 / 3.0));
  else
    y = -5.0 * alpha / 6.0 - P / (3.0 * U) + U;

  const std::complex<double> w = sqrt(alpha + 2.0 * y);

  std::complex<double> temp;

  temp = -B / (4.0 * A) + 0.5 * (w + sqrt(-(3.0 * alpha + 2.0 * y + 2.0 * beta / w)));
  realRoots[0] = temp.real();
  temp = -B / (4.0 * A) + 0.5 * (w - sqrt(-(3.0 * alpha + 2.0 * y + 2.0 * beta / w)));
  realRoots[1] = temp.real();
  temp = -B / (4.0 * A) + 0.5 * (-w + sqrt(-(3.0 * alpha + 2.0 * y - 2.0 * beta / w)));
  realRoots[2] = temp.real();
  temp = -B / (4.0 * A) + 0.5 * (-w - sqrt(-(3.0 * alpha + 2.0 * y - 2.0 * beta / w)));
  realRoots[3] = temp.real();
}

/**
 * @brief Compute the absolute pose of a camera using three 3D-to-2D correspondences
 *
 * Reference: [1] A Novel Parametrization of the P3P-Problem for a Direct Computation of
 *              Absolute Camera Position and Orientation
 *              Kneip, L.; Scaramuzza, D. ; Siegwart, R.
 *              CVPR 2011
 *
 * @param[in] featureVectors: 3x3 matrix with UNITARY feature vectors (each column is a vector)
 * @param[in] worldPoints: 3x3 matrix with corresponding 3D world points (each column is a point)
 * @param[out] solutions: 3x16 matrix that will contain the solutions
 *                   form: [ C1,R1, C2,R2 ... ]
 *                   the obtained orientation matrices are defined as transforming points from the cam to the world frame
 * @return true if correct execution, false if world points aligned
 * @author: Laurent Kneip, adapted to the project by Pierre Moulon
 */
bool computeP3PPoses(const Mat3& featureVectors, const Mat3& worldPoints, Mat& solutions)
{
  solutions = Mat(3, 4 * 4);

  // extraction of world points

  Vec3 P1 = worldPoints.col(0);
  Vec3 P2 = worldPoints.col(1);
  Vec3 P3 = worldPoints.col(2);

  // verification that world points are not colinear

  if(((P2 - P1).cross(P3 - Vec3(1, 1, 1))).norm() == 0)
    return false;

  // extraction of feature vectors

  Vec3 f1 = featureVectors.col(0);
  Vec3 f2 = featureVectors.col(1);
  Vec3 f3 = featureVectors.col(2);

  // creation of intermediate camera frame

  Vec3 e1 = f1;
  Vec3 e3 = (f1.cross(f2)).normalized();
  Vec3 e2 = e3.cross(e1);

  Mat3 T;
  T.row(0) = e1;
  T.row(1) = e2;
  T.row(2) = e3;

  f3 = T * f3;

  // reinforce that f3[2] > 0 for having theta in [0;pi]

  if(f3[2] > 0)
  {
    f1 = featureVectors.col(1);
    f2 = featureVectors.col(0);
    f3 = featureVectors.col(2);

    e1 = f1;
    e3 = (f1.cross(f2)).normalized();
    e2 = e3.cross(e1);

    T.row(0) = e1;
    T.row(1) = e2;
    T.row(2) = e3;

    f3 = T * f3;

    P1 = worldPoints.col(1);
    P2 = worldPoints.col(0);
    P3 = worldPoints.col(2);
  }

  // creation of intermediate world frame

  const Vec3 n1 = (P2 - P1).normalized();
  const Vec3 n3 = (n1.cross(P3 - P1)).normalized();
  const Vec3 n2 = n3.cross(n1);

  Mat3 N;
  N.row(0) = n1;
  N.row(1) = n2;
  N.row(2) = n3;

  // extraction of known parameters

  P3 = N * (P3 - P1);

  const double d_12 = (P2 - P1).norm();
  const double f_1 = f3[0] / f3[2];
  const double f_2 = f3[1] / f3[2];
  const double p_1 = P3[0];
  const double p_2 = P3[1];

  const double cos_beta = f1.transpose() * f2;

  const double sign = ((cos_beta < 0) ? -1.0 : 1.0) ;
  const double b = sign * std::sqrt(1.0  / (1.0 - pow(cos_beta, 2)) - 1.0);

  // definition of temporary variables for avoiding multiple computation

  const double f_1_pw2 = pow(f_1, 2);
  const double f_2_pw2 = pow(f_2, 2);
  const double p_1_pw2 = pow(p_1, 2);
  const double p_1_pw3 = p_1_pw2 * p_1;
  const double p_1_pw4 = p_1_pw3 * p_1;
  const double p_2_pw2 = pow(p_2, 2);
  const double p_2_pw3 = p_2_pw2 * p_2;
  const double p_2_pw4 = p_2_pw3 * p_2;
  const double d_12_pw2 = pow(d_12, 2);
  const double b_pw2 = pow(b, 2);

  // computation of factors of 4th degree polynomial

  Vec5 factors;
  factors << -f_2_pw2 * p_2_pw4 - p_2_pw4 * f_1_pw2 - p_2_pw4,

          2. * p_2_pw3 * d_12 * b +
          2. * f_2_pw2 * p_2_pw3 * d_12 * b
          - 2. * f_2 * p_2_pw3 * f_1*d_12,

          -f_2_pw2 * p_2_pw2 * p_1_pw2
          - f_2_pw2 * p_2_pw2 * d_12_pw2 * b_pw2
          - f_2_pw2 * p_2_pw2 * d_12_pw2
          + f_2_pw2 * p_2_pw4
          + p_2_pw4 * f_1_pw2
          + 2. * p_1 * p_2_pw2 * d_12
          + 2. * f_1 * f_2 * p_1 * p_2_pw2 * d_12 * b
          - p_2_pw2 * p_1_pw2 * f_1_pw2
          + 2. * p_1 * p_2_pw2 * f_2_pw2 * d_12
          - p_2_pw2 * d_12_pw2 * b_pw2
          - 2. * p_1_pw2*p_2_pw2,

          2. * p_1_pw2 * p_2 * d_12 * b
          + 2. * f_2 * p_2_pw3 * f_1 * d_12
          - 2. * f_2_pw2 * p_2_pw3 * d_12 * b
          - 2. * p_1 * p_2 * d_12_pw2*b,

          -2. * f_2 * p_2_pw2 * f_1 * p_1 * d_12 * b
          + f_2_pw2 * p_2_pw2 * d_12_pw2
          + 2. * p_1_pw3 * d_12
          - p_1_pw2 * d_12_pw2
          + f_2_pw2 * p_2_pw2 * p_1_pw2
          - p_1_pw4
          - 2. * f_2_pw2 * p_2_pw2 * p_1 * d_12
          + p_2_pw2 * f_1_pw2 * p_1_pw2
          + f_2_pw2 * p_2_pw2 * d_12_pw2*b_pw2;

  // computation of roots

  Vec4 realRoots;
  solveQuartic(factors, realRoots);

  // backsubstitution of each solution

  for(int i = 0; i < 4; ++i)
  {
    const double cot_alpha = (-f_1 * p_1 / f_2 - realRoots[i] * p_2 + d_12 * b) / (-f_1 * realRoots[i] * p_2 / f_2 + p_1 - d_12);
    const double cos_theta = realRoots[i];
    double sin_theta = sqrt(1 - pow(realRoots[i], 2));
    const double sin_alpha = sqrt(1. / (pow(cot_alpha, 2) + 1));
    double cos_alpha = sqrt(1. - pow(sin_alpha, 2));

    if(cot_alpha < 0)
      cos_alpha = -cos_alpha;

    if(!is_finite(sin_theta))
      sin_theta = 0;

    Vec3 C(d_12 * cos_alpha * (sin_alpha * b + cos_alpha),
           cos_theta * d_12 * sin_alpha * (sin_alpha * b + cos_alpha),
           sin_theta * d_12 * sin_alpha * (sin_alpha * b + cos_alpha));

    C = P1 + N.transpose() * C;

    Mat3 R;
    R << -cos_alpha, -sin_alpha*cos_theta, -sin_alpha*sin_theta,
            sin_alpha, -cos_alpha*cos_theta, -cos_alpha*sin_theta,
            0, -sin_theta, cos_theta;

    R = N.transpose() * R.transpose() * T;

    solutions.col(i * 4) = C;
    solutions.block<3, 3>(0, i * 4 + 1) = R.transpose();
  }

  return true;
}

void P3PSolver::solve(const Mat& x2d, const Mat& x3d, std::vector<robustEstimation::Mat34Model>& models) const
{
  assert(2 == x2d.rows());
  assert(3 == x3d.rows());
  assert(x2d.cols() == x3d.cols());

  Mat3 R;
  Vec3 t;
  Mat34 P;

  Mat solutions = Mat(3, 16);

  Mat3 pt2D_3x3;
  pt2D_3x3.block<2, 3>(0, 0) = x2d;
  pt2D_3x3.row(2).fill(1);
  pt2D_3x3.col(0).normalize();
  pt2D_3x3.col(1).normalize();
  pt2D_3x3.col(2).normalize();

  Mat3 pt3D_3x3 = x3d;

  if(computeP3PPoses(pt2D_3x3, pt3D_3x3, solutions))
  {
    for(size_t i = 0; i < 4; ++i)
    {
      R = solutions.block<3, 3>(0, i * 4 + 1);
      t = -R * solutions.col(i * 4);
      P_from_KRt(Mat3::Identity(), R, t, &P); // K = Id

      models.emplace_back(P);
    }
  }
}

}  // namespace resection
}  // namespace multiview
}  // namespace aliceVision
