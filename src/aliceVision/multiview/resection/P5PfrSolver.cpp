// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "P5PfrSolver.hpp"
#include <aliceVision/numeric/projection.hpp>
#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/system/Logger.hpp>

#include <cmath>
#include <iostream>

namespace aliceVision {
namespace multiview {
namespace resection {

/**
 * @brief Compute the nullspace, choose the algorithm based on input matrix size
 * @param[in] A matrix
 */
Mat nullspace(const Mat &A)
{
  Mat N;
  if(A.rows() < A.cols())
  {
    // LU decomposition
    Eigen::FullPivLU<Mat> lu(A);
    N = lu.kernel();
  }
  else
  {
    // SVD decomposition
    Eigen::JacobiSVD<Mat> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    N = svd.matrixV();
  }
  return N;
}

Mat P5PfrModel::divisionToPolynomialModelDistortion(const Mat& x2d /*,double maxRadius*/) const
{
  Vec k(3);

  // make k of length 3 if shorter
  for(Vec::Index i = 0; i < 3; ++i)
  {
    k(i) = (i < _r.rows()) ? _r(i) : 0;
  }

  Vec di(x2d.cols());
  Vec o(x2d.cols());

  for(Vec::Index i = 0; i < x2d.cols(); ++i)
  {
    di(i) = x2d(0, i);
    o(i) = 1;
  }

  const Vec h1 = o + k(0) * di.array().pow(2).matrix() + k(1) * di.array().pow(4).matrix() + k(2) * di.array().pow(6).matrix();
  const Vec ri = h1.transpose().cwiseInverse().asDiagonal() * di;
  const double Sr04 = ri.array().pow(4).sum();
  const double Sr06 = ri.array().pow(6).sum();
  const double Sr08 = ri.array().pow(8).sum();
  const double Sr10 = ri.array().pow(10).sum();
  const double Sr12 = ri.array().pow(12).sum();
  const double Sr14 = ri.array().pow(14).sum();
  const double Sr3d = (di.asDiagonal() * ri.array().pow(3).matrix()).sum();
  const double Sr5d = (di.asDiagonal() * ri.array().pow(5).matrix()).sum();
  const double Sr7d = (di.asDiagonal() * ri.array().pow(7).matrix()).sum();

  Mat A = Mat(3, 3);
  A << Sr06, Sr08, Sr10, Sr08, Sr10, Sr12, Sr10, Sr12, Sr14;

  Vec b = Vec(3);
  b << Sr3d - Sr04, Sr5d - Sr06, Sr7d - Sr08;

  return A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
}

bool computePosesRD(const Mat& featureVectors, const Mat& worldPoints, int numOfRadialCoeff, std::vector<P5PfrModel>* solutions)
{
  // Eliminate all linear stuff
  Mat A = Mat(5, 8);
  for(Vec::Index i = 0; i < 5; ++i)
  {
    Mat X = Mat(1, 8);
    X << -featureVectors(1, i) * worldPoints(0, i),
         -featureVectors(1, i) * worldPoints(1, i),
         -featureVectors(1, i) * worldPoints(2, i),
         -featureVectors(1, i),
          featureVectors(0, i) * worldPoints(0, i),
          featureVectors(0, i) * worldPoints(1, i),
          featureVectors(0, i) * worldPoints(2, i),
          featureVectors(0, i);

    A.block(i, 0, 1, 8) = X;
  }

  // 3D Nullspace    
  const Mat N = nullspace(A);

  // Construct the matrix C
  Mat C = Mat(2, 6);
  C << N.block(0, 0, 3, 1).transpose() * N.block(4, 0, 3, 1),
       N.block(0, 0, 3, 1).transpose() * N.block(4, 1, 3, 1) + N.block(0, 1, 3, 1).transpose() * N.block(4, 0, 3, 1),
       N.block(0, 0, 3, 1).transpose() * N.block(4, 2, 3, 1) + N.block(0, 2, 3, 1).transpose() * N.block(4, 0, 3, 1),
       N.block(0, 1, 3, 1).transpose() * N.block(4, 1, 3, 1),
       N.block(0, 1, 3, 1).transpose() * N.block(4, 2, 3, 1) + N.block(0, 2, 3, 1).transpose() * N.block(4, 1, 3, 1),
       N.block(0, 2, 3, 1).transpose() * N.block(4, 2, 3, 1),
       N.block(0, 0, 3, 1).transpose() * N.block(0, 0, 3, 1) - N.block(4, 0, 3, 1).transpose() * N.block(4, 0, 3, 1),
       N.block(0, 0, 3, 1).transpose() * N.block(0, 1, 3, 1) + N.block(0, 1, 3, 1).transpose() * N.block(0, 0, 3, 1) - (N.block(4, 0, 3, 1).transpose() * N.block(4, 1, 3, 1) + N.block(4, 1, 3, 1).transpose() * N.block(4, 0, 3, 1)),
       N.block(0, 0, 3, 1).transpose() * N.block(0, 2, 3, 1) + N.block(0, 2, 3, 1).transpose() * N.block(0, 0, 3, 1) - (N.block(4, 0, 3, 1).transpose() * N.block(4, 2, 3, 1) + N.block(4, 2, 3, 1).transpose() * N.block(4, 0, 3, 1)),
       N.block(0, 1, 3, 1).transpose() * N.block(0, 1, 3, 1) - N.block(4, 1, 3, 1).transpose() * N.block(4, 1, 3, 1),
       N.block(0, 1, 3, 1).transpose() * N.block(0, 2, 3, 1) + N.block(0, 2, 3, 1).transpose() * N.block(0, 1, 3, 1) - (N.block(4, 1, 3, 1).transpose() * N.block(4, 2, 3, 1) + N.block(4, 2, 3, 1).transpose() * N.block(4, 1, 3, 1)),
       N.block(0, 2, 3, 1).transpose() * N.block(0, 2, 3, 1) - N.block(4, 2, 3, 1).transpose() * N.block(4, 2, 3, 1);

  // Normalize C to get reasonable numbers when computing d
  const Vec &c1 = C.row(0);
  const Vec &c2 = C.row(1);
  Mat sC = Mat(2, 2);
  sC << (6 / c1.norm()), 0, 0, (6 / c2.norm());
  C = sC * C;

  // Determinant coefficients
  Mat d = Mat(5, 1);
  d << C(0, 0) * C(0, 0) * C(1, 3) * C(1, 3) - C(0, 0) * C(0, 1) * C(1, 1) * C(1, 3) - 2 * C(0, 0) * C(0, 3) * C(1, 0) * C(1, 3) + C(0, 0) * C(0, 3) * C(1, 1) * C(1, 1) + C(0, 1) * C(0, 1) * C(1, 0) * C(1, 3) - C(0, 1) * C(0, 3) * C(1, 0) * C(1, 1) + C(0, 3) * C(0, 3) * C(1, 0) * C(1, 0),
      -C(0, 0) * C(0, 1) * C(1, 3) * C(1, 4) + 2 * C(0, 0) * C(0, 2) * C(1, 3) * C(1, 3) + 2 * C(0, 0) * C(0, 3) * C(1, 1) * C(1, 4) - 2 * C(0, 0) * C(0, 3) * C(1, 2) * C(1, 3) - C(0, 0) * C(0, 4) * C(1, 1) * C(1, 3) + C(0, 1) * C(0, 1) * C(1, 2) * C(1, 3) - C(0, 1) * C(0, 2) * C(1, 1) * C(1, 3) - C(0, 1) * C(0, 3) * C(1, 0) * C(1, 4) - C(0, 1) * C(0, 3) * C(1, 1) * C(1, 2) + 2 * C(0, 1) * C(0, 4) * C(1, 0) * C(1, 3) - 2 * C(0, 2) * C(0, 3) * C(1, 0) * C(1, 3) + C(0, 2) * C(0, 3) * C(1, 1) * C(1, 1) + 2 * C(0, 3) * C(0, 3) * C(1, 0) * C(1, 2) - C(0, 3) * C(0, 4) * C(1, 0) * C(1, 1),
      -2 * C(0, 0) * C(0, 3) * C(1, 3) * C(1, 5) + C(0, 0) * C(0, 3) * C(1, 4) * C(1, 4) - C(0, 0) * C(0, 4) * C(1, 3) * C(1, 4) + 2 * C(0, 0) * C(0, 5) * C(1, 3) * C(1, 3) + C(0, 1) * C(0, 1) * C(1, 3) * C(1, 5) - C(0, 1) * C(0, 2) * C(1, 3) * C(1, 4) - C(0, 1) * C(0, 3) * C(1, 1) * C(1, 5) - C(0, 1) * C(0, 3) * C(1, 2) * C(1, 4) + 2 * C(0, 1) * C(0, 4) * C(1, 2) * C(1, 3) - C(0, 1) * C(0, 5) * C(1, 1) * C(1, 3) + C(0, 2) * C(0, 2) * C(1, 3) * C(1, 3) + 2 * C(0, 2) * C(0, 3) * C(1, 1) * C(1, 4) - 2 * C(0, 2) * C(0, 3) * C(1, 2) * C(1, 3) - C(0, 2) * C(0, 4) * C(1, 1) * C(1, 3) + 2 * C(0, 3) * C(0, 3) * C(1, 0) * C(1, 5) + C(0, 3) * C(0, 3) * C(1, 2) * C(1, 2) - C(0, 3) * C(0, 4) * C(1, 0) * C(1, 4) - C(0, 3) * C(0, 4) * C(1, 1) * C(1, 2) - 2 * C(0, 3) * C(0, 5) * C(1, 0) * C(1, 3) + C(0, 3) * C(0, 5) * C(1, 1) * C(1, 1) + C(0, 4) * C(0, 4) * C(1, 0) * C(1, 3),
      -C(0, 1) * C(0, 3) * C(1, 4) * C(1, 5) + 2 * C(0, 1) * C(0, 4) * C(1, 3) * C(1, 5) - C(0, 1) * C(0, 5) * C(1, 3) * C(1, 4) - 2 * C(0, 2) * C(0, 3) * C(1, 3) * C(1, 5) + C(0, 2) * C(0, 3) * C(1, 4) * C(1, 4) - C(0, 2) * C(0, 4) * C(1, 3) * C(1, 4) + 2 * C(0, 2) * C(0, 5) * C(1, 3) * C(1, 3) + 2 * C(0, 3) * C(0, 3) * C(1, 2) * C(1, 5) - C(0, 3) * C(0, 4) * C(1, 1) * C(1, 5) - C(0, 3) * C(0, 4) * C(1, 2) * C(1, 4) + 2 * C(0, 3) * C(0, 5) * C(1, 1) * C(1, 4) - 2 * C(0, 3) * C(0, 5) * C(1, 2) * C(1, 3) + C(0, 4) * C(0, 4) * C(1, 2) * C(1, 3) - C(0, 4) * C(0, 5) * C(1, 1) * C(1, 3),
       C(0, 3) * C(0, 3) * C(1, 5) * C(1, 5) - C(0, 3) * C(0, 4) * C(1, 4) * C(1, 5) - 2 * C(0, 3) * C(0, 5) * C(1, 3) * C(1, 5) + C(0, 3) * C(0, 5) * C(1, 4) * C(1, 4) + C(0, 4) * C(0, 4) * C(1, 3) * C(1, 5) - C(0, 4) * C(0, 5) * C(1, 3) * C(1, 4) + C(0, 5) * C(0, 5) * C(1, 3) * C(1, 3);

  // Companion matrix
  d = d * (1.0 / d(0, 0));
  Mat M = Mat(4, 4);
  M << 0, 0, 0, -d(4, 0),
       1, 0, 0, -d(3, 0),
       0, 1, 0, -d(2, 0),
       0, 0, 1, -d(1, 0);

  // solve it
  Eigen::EigenSolver<Mat> es(M);
  Mat g1_im = es.eigenvalues().imag();
  Mat g1_re = es.eigenvalues().real();

  // separate real solutions
  const double eps = 2.2204e-16;
  std::vector<double> vec_g1_real;
  for(Mat::Index i = 0; i < 4; ++i)
  {
    if(std::abs(g1_im(i, 0)) < eps)
      vec_g1_real.push_back(g1_re(i, 0));
  }
  if(vec_g1_real.empty())
    return false;
  Vec g1 = Map<Vec>(vec_g1_real.data(), vec_g1_real.size());

  //get g2 : Sg1 * <g2 ^ 3, g2 ^ 2, g2, 1 >= 0
  //   SG1 : = << C14 | C12*g1 + C15  | C11*g1 ^ 2 + C13*g1 + C16 | 0              >,
  //             <  0 | C14      | C12*g1 + C15        | C11*g1 ^ 2 + C13*g1 + C16  >,
  //             <C24 | C22*g1 + C25  | C21*g1 ^ 2 + C23*g1 + C26 | 0              >,
  //             <  0 | C24      | C22*g1 + C25        | C21*g1 ^ 2 + C23*g1 + C26 >> ;
  Mat g2 = Mat(g1.rows(), g1.cols());
  for(Mat::Index i = 0; i < g1.rows(); ++i)
  {
    Mat M2G = Mat(4, 4);
    M2G <<  C(0, 3),
            C(0, 1) * g1(i) + C(0, 4),
            C(0, 0) * g1(i) * g1(i) + C(0, 2) * g1(i) + C(0, 5),
            0, 0,
            C(0, 3),
            C(0, 1) * g1(i) + C(0, 4),
            C(0, 0) * g1(i) * g1(i) + C(0, 2) * g1(i) + C(0, 5),
            C(1, 3),
            C(1, 1) * g1(i) + C(1, 4),
            C(1, 0) * g1(i) * g1(i) + C(1, 2) * g1(i) + C(1, 5),
            0, 0,
            C(1, 3),
            C(1, 1) * g1(i) + C(1, 4),
            C(1, 0) * g1(i) * g1(i) + C(1, 2) * g1(i) + C(1, 5);

    Mat NM2G = nullspace(M2G);

    g2(i) = NM2G(2, NM2G.cols() - 1) / NM2G(3, NM2G.cols() - 1);
  }

  // Get P for all pairs of solutions[g1, g2]
  for(int i = 0; i < g1.rows(); ++i)
  {
    // The first two rows of P(P : = zip((g1, g2)->N[1] * g1 + N[2] * g2 + N[3], G1, G2) : )
    const Vec4 p1 = N.block(0, 0, 4, 1) * g1(i) + N.block(0, 1, 4, 1) * g2(i) + N.block(0, 2, 4, 1);
    const Vec4 p2 = N.block(4, 0, 4, 1) * g1(i) + N.block(4, 1, 4, 1) * g2(i) + N.block(4, 2, 4, 1);

    Mat34 P;
    P.row(0) = ((1 / p1.block(0, 0, 3, 1).norm()) * p1).transpose();
    P.row(1) = ((1 / p2.block(0, 0, 3, 1).norm()) * p2).transpose();
    P.row(2) << P(0, 1) * P(1, 2) - P(0, 2) * P(1, 1), -P(0, 0) * P(1, 2) + P(0, 2) * P(1, 0), P(0, 0) * P(1, 1) - P(0, 1) * P(1, 0), 0;

    // Form equations on k p34 and t = 1 / f: B <p34, t, k1, k2 ^ 2, k3 ^ 3, 1> = 0
    Mat B = Mat(5, 6);
    for(Mat::Index j = 0; j < 5; ++j)
    { // for all point pairs[u, X]
      const double r2 = featureVectors(0, j) * featureVectors(0, j) + featureVectors(1, j) * featureVectors(1, j); // temporary vals
      const double ee11 = (P.block(0, 0, 1, 3) * worldPoints.col(j))(0, 0) + P(0, 3);
      const double ee21 = (P.block(1, 0, 1, 3) * worldPoints.col(j))(0, 0) + P(1, 3);
      const double ee31 = (P.block(2, 0, 1, 3) * worldPoints.col(j))(0, 0);
      const double ee32 = featureVectors(1, j) * ee31;
      const double ee33 = -featureVectors(0, j) * ee31;

      if(abs(featureVectors(1, j)) > abs(featureVectors(0, j)))
      {
        B.row(j) << featureVectors(1, j), ee32, -ee21*r2, -ee21 * r2*r2, -ee21 * r2 * r2*r2, -ee21;
      }
      else
      {
        B.row(j) << -featureVectors(0, j), ee33, ee11*r2, ee11 * r2*r2, ee11 * r2 * r2*r2, ee11;
      }
    }

    // select columns
    Mat U;
    switch(numOfRadialCoeff)
    {
    case 1:
      U = Mat(6, 4);
      U << 1, 0, 0, 0,
           0, 1, 0, 0,
           0, 0, 1, 0,
           0, 0, 0, 0,
           0, 0, 0, 0,
           0, 0, 0, 1;
      break;

    case 2:
      U = Mat(6, 5);
      U << 1, 0, 0, 0, 0,
           0, 1, 0, 0, 0,
           0, 0, 1, 0, 0,
           0, 0, 0, 1, 0,
           0, 0, 0, 0, 0,
           0, 0, 0, 0, 1;
      break;

    case 3:
      U = Mat::Identity(6, 6);
      break;

      default:
        std::cerr << "\nError: the number of radial parameters must be between 1 to 3!\n";
      return false;
    }
    B = B * U;

    // find the right 1D null space
    const Mat NBfull = nullspace(B);
    const Mat NB = NBfull.col(NBfull.cols() - 1);
    const Mat V = NB.col(NB.cols() - 1);
    Mat tk = V * (1 / V(V.rows() - 1, V.cols() - 1));

    // make f positive
    if(tk(1, 0) < 0)
    {
      tk.block(0, 0, 2, 1) = -tk.block(0, 0, 2, 1);
      P.block(0, 0, 2, 4) = -P.block(0, 0, 2, 4);
    }

    P(2, 3) = tk(0, 0) / tk(1, 0);
    Mat K = Mat(3, 3);
    K << 1.0 / tk(1, 0), 0, 0, 0, 1.0 / tk(1, 0), 0, 0, 0, 1;
    Mat R = P.block(0, 0, 3, 3);

    //Mat C = -R.transpose() * P.block(0, 3, 3, 1);
    Vec r = Vec(numOfRadialCoeff);
    r << tk.block(tk.rows() - numOfRadialCoeff - 1, 0, numOfRadialCoeff, 1);

    // In[1] we have
    // [u - u0][f 0 0]
    // [v - v0] = [0 f 0][R | -R*C][X]
    // [1 + r*((u - u0) ^ 2 + (v - v0) ^ 2)][0 0 1][1]
    // but we want
    // [(u - u0) / f]
    // [(v - v0) / f] = [R | -R*C][X]
    // [1 + (r*f ^ 2)*((u - u0) ^ 2 + (v - v0) ^ 2) / f ^ 2][1]

    // instead not deal with f dependent r
    for(Mat::Index j = 0; j < numOfRadialCoeff; ++j) // f^2, f^4, f^6
      r(j) *= pow(K(0, 0), 2 * (j + 1));

    // output
    const Vec3 t = P.block(0, 3, 3, 1);
    const double f = (1.0 / tk(1, 0));
    solutions->emplace_back(R, t, r, f);
  }
  return true;
}

bool computePosesRP(const Mat& featureVectors, const Mat& worldPoints, int numOfRadialCoeff, std::vector<P5PfrModel>* solutions)
{
  if(computePosesRD(featureVectors, worldPoints, numOfRadialCoeff, solutions))
  {
    const Mat p2dRadius = featureVectors.colwise().norm();
    for(std::size_t i = 0; i < solutions->size(); ++i)
    {
      P5PfrModel &m = solutions->at(i);
      m._r = m.divisionToPolynomialModelDistortion((1 / m._f) * p2dRadius /*,p2dRadius.maxCoeff(),*/);
    }
    return true;
  }
  return false;
}

double reprojectionErrorRD(const P5PfrModel& model, const Vec2& p2d, const Vec3& p3d)
{
  if(model._r.rows() > 1)
  {
     ALICEVISION_CERR("Projection function is not implemented for the radial division undistortion model for more than one parameter." << std::endl);
     throw std::invalid_argument("Projection function is not implemented for the radial division undistortion model for more than one parameter.");
  }

  Vec3 v = model._R * p3d + model._t; // from delta to epsilon
  v *= 1.0 / v(2); // normalize to have v(3, :) = 1

  // undistorted squared radius
  const double ru2 = v(0) * v(0) + v(1) * v(1);

  // works for fish-eye, i.e. when distort image gets smaller on the image plane
  const double h1 = sqrt(-4 * model._r(0) * ru2 + 1);
  const double h2 = 0.5 * ((-2 * model._r(0) * ru2 + 1) - h1) * (1 / (model._r(0) * model._r(0)));
  const double rd = sqrt(h2 * (1 / ru2));

  // distort in epsilon
  const double h3 = rd / sqrt(ru2);
  Vec2 u;
  u << v(0) * h3, v(1) * h3;

  // to alpha
  u = model._f * u;
  return (p2d - u).norm();
}

double reprojectionErrorRP(const P5PfrModel& model, const Vec2& p2d, const Vec3& p3d)
{
  Vec3 v = model._R * p3d + model._t; // from delta to epsilon
  v *= 1.0 / v(2); // normalize to have v(3, :) = 1

  double t = 1; // the final radius parameter
  const double r = std::hypot(v(0), v(1));
  for(Mat::Index i = 0; i < model._r.rows(); ++i)
    t += model._r(i) * pow(r, 2 * (i + 1));

  Vec2 u;
  u << v(0) * t, v(1) * t;

  // to alpha
  u = model._f * u;
  return (p2d - u).norm();
}

} // namespace resection
} // namespace multiview
} // namespace aliceVision
