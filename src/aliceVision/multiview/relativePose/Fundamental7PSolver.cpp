// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// Copyright (c) 2010 libmv contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Fundamental7PSolver.hpp"
#include <aliceVision/numeric/polynomial.hpp>
#include <aliceVision/multiview/epipolarEquation.hpp>

namespace aliceVision {
namespace multiview {
namespace relativePose {

void Fundamental7PSolver::solve(const Mat& x1, const Mat& x2, std::vector<robustEstimation::Mat3Model>& models) const
{
  assert(2 == x1.rows());
  assert(7 <= x1.cols());
  assert(x1.rows() == x2.rows());
  assert(x1.cols() == x2.cols());

  Vec9 f1, f2;

  if(x1.cols() == 7)
  {
    // set up the homogeneous system Af = 0 from the equations x'T*F*x = 0.

    // in the minimal solution use fixed sized matrix to let Eigen and the
    // compiler doing the maximum of optimization.
    Mat9 A = Mat::Zero(9,9);
    encodeEpipolarEquation(x1, x2, &A);

    // Eigen::FullPivLU<Mat9> luA(A);
    // ALICEVISION_LOG_DEBUG("rank(A) = " << luA.rank());
    // Eigen::JacobiSVD<Mat9> svdA(A);
    // ALICEVISION_LOG_DEBUG("Its singular values are:\n" << svdA.singularValues());

    // find the two F matrices in the nullspace of A.
    Nullspace2(&A, &f1, &f2);

    // @fixme here there is a potential error, we should check that the size of
    // null(A) is 2. Otherwise we have a family of possible solutions for the
    // fundamental matrix (ie infinite solution). This happens, e.g., when matching
    // the image against itself or in other degenerate configurations of the camera,
    // such as pure rotation or correspondences all on the same plane (cf HZ pg296 table 11.1)
    // This is not critical for just matching images with geometric validation, 
    // it becomes an issue if the estimated F has to be used for retrieving the 
    // motion of the camera.
  }
  else
  {
    // set up the homogeneous system Af = 0 from the equations x'T*F*x = 0.
    Mat A(x1.cols(), 9);
    encodeEpipolarEquation(x1, x2, &A);

    // find the two F matrices in the nullspace of A.
    Nullspace2(&A, &f1, &f2);
  }

  Mat3 F1 = Map<RMat3>(f1.data());
  Mat3 F2 = Map<RMat3>(f2.data());

  // use the condition det(F) = 0 to determine F.
  // in other words, solve: det(F1 + a*F2) = 0 for a.
  const double a = F1(0, 0), j = F2(0, 0),
               b = F1(0, 1), k = F2(0, 1),
               c = F1(0, 2), l = F2(0, 2),
               d = F1(1, 0), m = F2(1, 0),
               e = F1(1, 1), n = F2(1, 1),
               f = F1(1, 2), o = F2(1, 2),
               g = F1(2, 0), p = F2(2, 0),
               h = F1(2, 1), q = F2(2, 1),
               i = F1(2, 2), r = F2(2, 2);

  // run fundamental_7point_coeffs.py to get the below coefficients.
  // the coefficients are in ascending powers of alpha, i.e. P[N]*x^N.
  const double P[4] = {
    a*e*i + b*f*g + c*d*h - a*f*h - b*d*i - c*e*g,
    a*e*r + a*i*n + b*f*p + b*g*o + c*d*q + c*h*m + d*h*l + e*i*j + f*g*k -
    a*f*q - a*h*o - b*d*r - b*i*m - c*e*p - c*g*n - d*i*k - e*g*l - f*h*j,
    a*n*r + b*o*p + c*m*q + d*l*q + e*j*r + f*k*p + g*k*o + h*l*m + i*j*n -
    a*o*q - b*m*r - c*n*p - d*k*r - e*l*p - f*j*q - g*l*n - h*j*o - i*k*m,
    j*n*r + k*o*p + l*m*q - j*o*q - k*m*r - l*n*p };

  // solve for the roots of P[3]*x^3 + P[2]*x^2 + P[1]*x + P[0] = 0.
  double roots[3];
  const int nbRoots = SolveCubicPolynomial(P, roots);

  // build the fundamental matrix for each solution.
  for(int kk = 0; kk < nbRoots; ++kk)
    models.emplace_back(F1 + roots[kk] * F2);
}


void Fundamental7PSphericalSolver::solve(const Mat& x1, const Mat& x2, std::vector<robustEstimation::Mat3Model>& models) const
{
    assert(3 == x1.rows());
    assert(7 <= x1.cols());
    assert(x1.rows() == x2.rows());
    assert(x1.cols() == x2.cols());

    Vec9 f1, f2;
    if(x1.cols() == 7)
    {
        // Set up the homogeneous system Af = 0 from the equations x'T*F*x = 0.
        typedef Eigen::Matrix<double, 9, 9> Mat9;
        // In the minimal solution use fixed sized matrix to let Eigen and the
        //  compiler doing the maximum of optimization.
        Mat9 A = Mat::Zero(9, 9);
        encodeEpipolarSphericalEquation(x1, x2, &A);
        //    Eigen::FullPivLU<Mat9> luA(A);
        //    ALICEVISION_LOG_DEBUG("\n rank(A) = " << luA.rank());
        //    Eigen::JacobiSVD<Mat9> svdA(A);
        //    cout << "Its singular values are:" << endl << svdA.singularValues() << endl;
        // Find the two F matrices in the nullspace of A.
        Nullspace2(&A, &f1, &f2);
        //@fixme here there is a potential error, we should check that the size of
        // null(A) is 2. Otherwise we have a family of possible solutions for the
        // fundamental matrix (ie infinite solution). This happens, e.g., when matching
        // the image against itself or in other degenerate configurations of the camera,
        // such as pure rotation or correspondences all on the same plane (cf HZ pg296 table 11.1)
        // This is not critical for just matching images with geometric validation,
        // it becomes an issue if the estimated F has to be used for retrieving the
        // motion of the camera.
    }
    else
    {
        // Set up the homogeneous system Af = 0 from the equations x'T*F*x = 0.
        Mat A(x1.cols(), 9);
        encodeEpipolarSphericalEquation(x1, x2, &A);
        // Find the two F matrices in the nullspace of A.
        Nullspace2(&A, &f1, &f2);
    }

    Mat3 F1 = Map<RMat3>(f1.data());
    Mat3 F2 = Map<RMat3>(f2.data());

    // Then, use the condition det(F) = 0 to determine F. In other words, solve
    // det(F1 + a*F2) = 0 for a.
    double a = F1(0, 0), j = F2(0, 0), b = F1(0, 1), k = F2(0, 1), c = F1(0, 2), l = F2(0, 2), d = F1(1, 0),
           m = F2(1, 0), e = F1(1, 1), n = F2(1, 1), f = F1(1, 2), o = F2(1, 2), g = F1(2, 0), p = F2(2, 0),
           h = F1(2, 1), q = F2(2, 1), i = F1(2, 2), r = F2(2, 2);

    // Run fundamental_7point_coeffs.py to get the below coefficients.
    // The coefficients are in ascending powers of alpha, i.e. P[N]*x^N.
    double P[4] = {
        a * e * i + b * f * g + c * d * h - a * f * h - b * d * i - c * e * g,
        a * e * r + a * i * n + b * f * p + b * g * o + c * d * q + c * h * m + d * h * l + e * i * j + f * g * k -
            a * f * q - a * h * o - b * d * r - b * i * m - c * e * p - c * g * n - d * i * k - e * g * l - f * h * j,
        a * n * r + b * o * p + c * m * q + d * l * q + e * j * r + f * k * p + g * k * o + h * l * m + i * j * n -
            a * o * q - b * m * r - c * n * p - d * k * r - e * l * p - f * j * q - g * l * n - h * j * o - i * k * m,
        j * n * r + k * o * p + l * m * q - j * o * q - k * m * r - l * n * p};

    // Solve for the roots of P[3]*x^3 + P[2]*x^2 + P[1]*x + P[0] = 0.
    double roots[3];
    int num_roots = SolveCubicPolynomial(P, roots);

    // Build the fundamental matrix for each solution.
    for(int kk = 0; kk < num_roots; ++kk)
    {
        models.emplace_back(F1 + roots[kk] * F2);
    }
}

}  // namespace relativePose
}  // namespace multiview
}  // namespace aliceVision
