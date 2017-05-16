/*
 * Copyright (c) 2011, Laurent Kneip, ETH Zurich
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of ETH Zurich nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ETH ZURICH BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// Copyright (c) 2012, 2013 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef OPENMVG_MULTIVIEW_RESECTION_P5PFR_CPP
#define OPENMVG_MULTIVIEW_RESECTION_P5PFR_CPP

#include "openMVG/multiview/projection.hpp"
#include "openMVG/numeric/numeric.h"
#include "openMVG/multiview/solver_resection_p5pfr.hpp"

#include <cmath>
#include <iostream>

namespace openMVG {
namespace resection {

// Compute the nullspace, choose the algorithm based on input matrix size

Mat nulls(Mat &A)
{
  Mat N;
  if(A.rows() < A.cols())
  { // LU decomposition
    Eigen::FullPivLU<Mat> lu(A);
    N = lu.kernel();
  }
  else
  { // SVD decomposition 
    Eigen::JacobiSVD<Mat> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    N = svd.matrixV();
  }
  return N;
}

// Power of each element of a vector

Vec cwisePow(Vec a, int k)
{
  Vec b = Vec(a.rows());
  for(int i = 0; i < a.rows(); ++i)
    b(i) = pow(a(i), k);
  return b;
}

// Inversion of the radial division undistortion to Brown polynomial distortion model conversion

void rddiv2pol(M *model, double dmax, Mat dpts2D)
{
  Vec r = model->_r;
  Vec k = Vec(3);
  for(int i = 0; i < 3; ++i) // make k of length 3 if shorter
    k(i) = (i < r.rows()) ? r(i) : 0;

  Vec di = Vec(dpts2D.cols());
  Vec o = Vec(dpts2D.cols());
  for(int i = 0; i < dpts2D.cols(); ++i)
  {
    di(i) = dpts2D(0, i);
    o(i) = 1;
  }

  Vec h1 = o + k(0) * cwisePow(di, 2) + k(1) * cwisePow(di, 4) + k(2) * cwisePow(di, 6);
  Vec ri = h1.transpose().cwiseInverse().asDiagonal() * di;
  double Sr04 = cwisePow(ri, 4).sum();
  double Sr06 = cwisePow(ri, 6).sum();
  double Sr08 = cwisePow(ri, 8).sum();
  double Sr10 = cwisePow(ri, 10).sum();
  double Sr12 = cwisePow(ri, 12).sum();
  double Sr14 = cwisePow(ri, 14).sum();
  double Sr3d = (di.asDiagonal() * cwisePow(ri, 3)).sum();
  double Sr5d = (di.asDiagonal() * cwisePow(ri, 5)).sum();
  double Sr7d = (di.asDiagonal() * cwisePow(ri, 7)).sum();

  Mat A = Mat(3, 3);
  A << Sr06, Sr08, Sr10, Sr08, Sr10, Sr12, Sr10, Sr12, Sr14;
  Vec b = Vec(3);
  b << Sr3d - Sr04, Sr5d - Sr06, Sr7d - Sr08;

  model->_r = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
}

// Compute P5Pfr algorithm with radial division undistortion model

bool compute_P5Pfr_Poses_RD(const Mat & featureVectors, const Mat & worldPoints, const int num_r, std::vector<M> *solutions)
{
  Mat pt2D(featureVectors);
  Mat pt3D(worldPoints);

  // Eliminate all linear stuff
  Mat A = Mat(5, 8);
  for(int i = 0; i < 5; ++i)
  {
    Mat X = Mat(1, 8);
    X << -pt2D(1, i) * pt3D(0, i), -pt2D(1, i) * pt3D(1, i), -pt2D(1, i) * pt3D(2, i), -pt2D(1, i), pt2D(0, i) * pt3D(0, i), pt2D(0, i) * pt3D(1, i), pt2D(0, i) * pt3D(2, i), pt2D(0, i);
    A.block(i, 0, 1, 8) = X;
  }

  // 3D Nullspace    
  Mat N = nulls(A);

  // Construct the matrix C
  Mat C = Mat(2, 6);
  C << N.block(0, 0, 3, 1).transpose() * N.block(4, 0, 3, 1), N.block(0, 0, 3, 1).transpose() * N.block(4, 1, 3, 1) + N.block(0, 1, 3, 1).transpose() * N.block(4, 0, 3, 1), N.block(0, 0, 3, 1).transpose() * N.block(4, 2, 3, 1) + N.block(0, 2, 3, 1).transpose() * N.block(4, 0, 3, 1), N.block(0, 1, 3, 1).transpose() * N.block(4, 1, 3, 1), N.block(0, 1, 3, 1).transpose() * N.block(4, 2, 3, 1) + N.block(0, 2, 3, 1).transpose() * N.block(4, 1, 3, 1), N.block(0, 2, 3, 1).transpose() * N.block(4, 2, 3, 1), N.block(0, 0, 3, 1).transpose() * N.block(0, 0, 3, 1) - N.block(4, 0, 3, 1).transpose() * N.block(4, 0, 3, 1), N.block(0, 0, 3, 1).transpose() * N.block(0, 1, 3, 1) + N.block(0, 1, 3, 1).transpose() * N.block(0, 0, 3, 1) - (N.block(4, 0, 3, 1).transpose() * N.block(4, 1, 3, 1) + N.block(4, 1, 3, 1).transpose() * N.block(4, 0, 3, 1)), N.block(0, 0, 3, 1).transpose() * N.block(0, 2, 3, 1) + N.block(0, 2, 3, 1).transpose() * N.block(0, 0, 3, 1) - (N.block(4, 0, 3, 1).transpose() * N.block(4, 2, 3, 1) + N.block(4, 2, 3, 1).transpose() * N.block(4, 0, 3, 1)), N.block(0, 1, 3, 1).transpose() * N.block(0, 1, 3, 1) - N.block(4, 1, 3, 1).transpose() * N.block(4, 1, 3, 1), N.block(0, 1, 3, 1).transpose() * N.block(0, 2, 3, 1) + N.block(0, 2, 3, 1).transpose() * N.block(0, 1, 3, 1) - (N.block(4, 1, 3, 1).transpose() * N.block(4, 2, 3, 1) + N.block(4, 2, 3, 1).transpose() * N.block(4, 1, 3, 1)), N.block(0, 2, 3, 1).transpose() * N.block(0, 2, 3, 1) - N.block(4, 2, 3, 1).transpose() * N.block(4, 2, 3, 1);

  // Normalize C to get reasonable numbers when computing d
  Vec c1(C.row(0));
  Vec c2(C.row(1));
  Mat sC = Mat(2, 2);
  sC << (6 / c1.norm()), 0, 0, (6 / c2.norm());
  C = sC * C;

  // Determinant coefficients
  Mat d = Mat(5, 1);
  d << C(0, 0) * C(0, 0) * C(1, 3) * C(1, 3) - C(0, 0) * C(0, 1) * C(1, 1) * C(1, 3) - 2 * C(0, 0) * C(0, 3) * C(1, 0) * C(1, 3) + C(0, 0) * C(0, 3) * C(1, 1) * C(1, 1) + C(0, 1) * C(0, 1) * C(1, 0) * C(1, 3) - C(0, 1) * C(0, 3) * C(1, 0) * C(1, 1) + C(0, 3) * C(0, 3) * C(1, 0) * C(1, 0), -C(0, 0) * C(0, 1) * C(1, 3) * C(1, 4) + 2 * C(0, 0) * C(0, 2) * C(1, 3) * C(1, 3) + 2 * C(0, 0) * C(0, 3) * C(1, 1) * C(1, 4) - 2 * C(0, 0) * C(0, 3) * C(1, 2) * C(1, 3) - C(0, 0) * C(0, 4) * C(1, 1) * C(1, 3) + C(0, 1) * C(0, 1) * C(1, 2) * C(1, 3) - C(0, 1) * C(0, 2) * C(1, 1) * C(1, 3) - C(0, 1) * C(0, 3) * C(1, 0) * C(1, 4) - C(0, 1) * C(0, 3) * C(1, 1) * C(1, 2) + 2 * C(0, 1) * C(0, 4) * C(1, 0) * C(1, 3) - 2 * C(0, 2) * C(0, 3) * C(1, 0) * C(1, 3) + C(0, 2) * C(0, 3) * C(1, 1) * C(1, 1) + 2 * C(0, 3) * C(0, 3) * C(1, 0) * C(1, 2) - C(0, 3) * C(0, 4) * C(1, 0) * C(1, 1), -2 * C(0, 0) * C(0, 3) * C(1, 3) * C(1, 5) + C(0, 0) * C(0, 3) * C(1, 4) * C(1, 4) - C(0, 0) * C(0, 4) * C(1, 3) * C(1, 4) + 2 * C(0, 0) * C(0, 5) * C(1, 3) * C(1, 3) + C(0, 1) * C(0, 1) * C(1, 3) * C(1, 5) - C(0, 1) * C(0, 2) * C(1, 3) * C(1, 4) - C(0, 1) * C(0, 3) * C(1, 1) * C(1, 5) - C(0, 1) * C(0, 3) * C(1, 2) * C(1, 4) + 2 * C(0, 1) * C(0, 4) * C(1, 2) * C(1, 3) - C(0, 1) * C(0, 5) * C(1, 1) * C(1, 3) + C(0, 2) * C(0, 2) * C(1, 3) * C(1, 3) + 2 * C(0, 2) * C(0, 3) * C(1, 1) * C(1, 4) - 2 * C(0, 2) * C(0, 3) * C(1, 2) * C(1, 3) - C(0, 2) * C(0, 4) * C(1, 1) * C(1, 3) + 2 * C(0, 3) * C(0, 3) * C(1, 0) * C(1, 5) + C(0, 3) * C(0, 3) * C(1, 2) * C(1, 2) - C(0, 3) * C(0, 4) * C(1, 0) * C(1, 4) - C(0, 3) * C(0, 4) * C(1, 1) * C(1, 2) - 2 * C(0, 3) * C(0, 5) * C(1, 0) * C(1, 3) + C(0, 3) * C(0, 5) * C(1, 1) * C(1, 1) + C(0, 4) * C(0, 4) * C(1, 0) * C(1, 3), -C(0, 1) * C(0, 3) * C(1, 4) * C(1, 5) + 2 * C(0, 1) * C(0, 4) * C(1, 3) * C(1, 5) - C(0, 1) * C(0, 5) * C(1, 3) * C(1, 4) - 2 * C(0, 2) * C(0, 3) * C(1, 3) * C(1, 5) + C(0, 2) * C(0, 3) * C(1, 4) * C(1, 4) - C(0, 2) * C(0, 4) * C(1, 3) * C(1, 4) + 2 * C(0, 2) * C(0, 5) * C(1, 3) * C(1, 3) + 2 * C(0, 3) * C(0, 3) * C(1, 2) * C(1, 5) - C(0, 3) * C(0, 4) * C(1, 1) * C(1, 5) - C(0, 3) * C(0, 4) * C(1, 2) * C(1, 4) + 2 * C(0, 3) * C(0, 5) * C(1, 1) * C(1, 4) - 2 * C(0, 3) * C(0, 5) * C(1, 2) * C(1, 3) + C(0, 4) * C(0, 4) * C(1, 2) * C(1, 3) - C(0, 4) * C(0, 5) * C(1, 1) * C(1, 3), C(0, 3) * C(0, 3) * C(1, 5) * C(1, 5) - C(0, 3) * C(0, 4) * C(1, 4) * C(1, 5) - 2 * C(0, 3) * C(0, 5) * C(1, 3) * C(1, 5) + C(0, 3) * C(0, 5) * C(1, 4) * C(1, 4) + C(0, 4) * C(0, 4) * C(1, 3) * C(1, 5) - C(0, 4) * C(0, 5) * C(1, 3) * C(1, 4) + C(0, 5) * C(0, 5) * C(1, 3) * C(1, 3);

  // Companion matrix
  d = d * (1.0 / d(0, 0));
  Mat M = Mat(4, 4);
  M << 0, 0, 0, -d(4, 0), 1, 0, 0, -d(3, 0), 0, 1, 0, -d(2, 0), 0, 0, 1, -d(1, 0);

  // solve it
  Eigen::EigenSolver<Mat> es(M);
  Mat g1_im = es.eigenvalues().imag();
  Mat g1_re = es.eigenvalues().real();

  // separate real solutions
  double eps = 2.2204e-16;
  std::vector<double> vec_g1_real;
  for(int i = 0; i < 4; ++i)
  {
    if(std::abs(g1_im(i, 0)) < eps)
      vec_g1_real.push_back(g1_re(i, 0));
  }
  if(vec_g1_real.size() == 0)
    return false;
  Vec g1 = Map<Vec>(vec_g1_real.data(), vec_g1_real.size());

  //get g2 : Sg1 * <g2 ^ 3, g2 ^ 2, g2, 1 >= 0
  //   SG1 : = << C14 | C12*g1 + C15	| C11*g1 ^ 2 + C13*g1 + C16 | 0							>,
  //	           <  0 | C14			| C12*g1 + C15				| C11*g1 ^ 2 + C13*g1 + C16	>,
  //	           <C24 | C22*g1 + C25	| C21*g1 ^ 2 + C23*g1 + C26 | 0							>,
  //	           <  0 | C24			| C22*g1 + C25				| C21*g1 ^ 2 + C23*g1 + C26 >> ;
  Mat g2 = Mat(g1.rows(), g1.cols());
  for(int i = 0; i < g1.rows(); ++i)
  {
    Mat M2G = Mat(4, 4);
    M2G << C(0, 3), C(0, 1) * g1(i) + C(0, 4), C(0, 0) * g1(i) * g1(i) + C(0, 2) * g1(i) + C(0, 5), 0, 0, C(0, 3), C(0, 1) * g1(i) + C(0, 4), C(0, 0) * g1(i) * g1(i) + C(0, 2) * g1(i) + C(0, 5), C(1, 3), C(1, 1) * g1(i) + C(1, 4), C(1, 0) * g1(i) * g1(i) + C(1, 2) * g1(i) + C(1, 5), 0, 0, C(1, 3), C(1, 1) * g1(i) + C(1, 4), C(1, 0) * g1(i) * g1(i) + C(1, 2) * g1(i) + C(1, 5);
    Mat NM2G = nulls(M2G);
    g2(i) = NM2G(2, NM2G.cols() - 1) / NM2G(3, NM2G.cols() - 1);
  }

  // Get P for all pairs of solutions[g1, g2]
  for(int i = 0; i < g1.rows(); ++i)
  {
    // The first two rows of P(P : = zip((g1, g2)->N[1] * g1 + N[2] * g2 + N[3], G1, G2) : )
    Vec4 p1 = N.block(0, 0, 4, 1) * g1(i) + N.block(0, 1, 4, 1) * g2(i) + N.block(0, 2, 4, 1);
    Vec4 p2 = N.block(4, 0, 4, 1) * g1(i) + N.block(4, 1, 4, 1) * g2(i) + N.block(4, 2, 4, 1);

    Mat34 P;
    P.row(0) = ((1 / p1.block(0, 0, 3, 1).norm()) * p1).transpose();
    P.row(1) = ((1 / p2.block(0, 0, 3, 1).norm()) * p2).transpose();
    P.row(2) << P(0, 1) * P(1, 2) - P(0, 2) * P(1, 1), -P(0, 0) * P(1, 2) + P(0, 2) * P(1, 0), P(0, 0) * P(1, 1) - P(0, 1) * P(1, 0), 0;

    // Form equations on k p34 and t = 1 / f: B <p34, t, k1, k2 ^ 2, k3 ^ 3, 1> = 0
    Mat B = Mat(5, 6);
    for(int j = 0; j < 5; ++j)
    { // for all point pairs[u, X]
      double r2 = pt2D(0, j) * pt2D(0, j) + pt2D(1, j) * pt2D(1, j); // temporary vals
      double ee11 = (P.block(0, 0, 1, 3) * pt3D.col(j))(0, 0) + P(0, 3);
      double ee21 = (P.block(1, 0, 1, 3) * pt3D.col(j))(0, 0) + P(1, 3);
      double ee31 = (P.block(2, 0, 1, 3) * pt3D.col(j))(0, 0);
      double ee32 = pt2D(1, j) * ee31;
      double ee33 = -pt2D(0, j) * ee31;

      if(abs(pt2D(1, j)) > abs(pt2D(0, j)))
      {
        B.row(j) << pt2D(1, j), ee32, -ee21*r2, -ee21 * r2*r2, -ee21 * r2 * r2*r2, -ee21;
      }
      else
      {
        B.row(j) << -pt2D(0, j), ee33, ee11*r2, ee11 * r2*r2, ee11 * r2 * r2*r2, ee11;
      }
    }

    // select columns
    Mat U;
    switch(num_r)
    {
    case 1:
      U = Mat(6, 4);
      U << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;
      break;
    case 2:
      U = Mat(6, 5);
      U << 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;
      break;
    case 3:
      U = Mat::Identity(6, 6);
      break;
otherwise:
      std::cerr << "\nError: the number of radial parameters must be between 1 to 3!\n";
      return false;
    }
    B = B * U;

    // find the right 1D null space		
    Mat NBfull = nulls(B);
    Mat NB = NBfull.col(NBfull.cols() - 1);
    Mat V = NB.col(NB.cols() - 1);
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

    Mat C = -R.transpose() * P.block(0, 3, 3, 1);
    Vec r = Vec(num_r);
    r << tk.block(tk.rows() - num_r - 1, 0, num_r, 1);

    // In[1] we have
    // [u - u0][f 0 0]
    // [v - v0] = [0 f 0][R | -R*C][X]
    // [1 + r*((u - u0) ^ 2 + (v - v0) ^ 2)][0 0 1][1]
    // but we want
    // [(u - u0) / f]
    // [(v - v0) / f] = [R | -R*C][X]
    // [1 + (r*f ^ 2)*((u - u0) ^ 2 + (v - v0) ^ 2) / f ^ 2][1]

    // instead not deal with f dependent r
    for(int j = 0; j < num_r; ++j) // f^2, f^4, f^6
      r(j) *= pow(K(0, 0), 2 * (j + 1));

    // output
    Vec3 t = P.block(0, 3, 3, 1);
    double f = (1.0 / tk(1, 0));
    resection::M model(R, t, r, f);
    solutions->push_back(model);
  }
  return true;
}

// Compute compute_P5Pfr_Poses_RD and transform the radial division undistortion to Brown polynomial distortion model

bool compute_P5Pfr_Poses_RP(const Mat & featureVectors, const Mat & worldPoints, const int num_r, std::vector<M> *solutions)
{
  if(compute_P5Pfr_Poses_RD(featureVectors, worldPoints, num_r, solutions))
  {
    Mat pt2D_radius = featureVectors.colwise().norm();
    for(int i = 0; i < solutions->size(); ++i)
    {
      M *m = &(solutions->at(i));
      rddiv2pol(m, pt2D_radius.maxCoeff(), (1 / m->_f) * pt2D_radius);
    }
    return true;
  }
  return false;
}

// Compute the reprojection error for the radial division undistortion model

double reproj_error_RD(const M & m, const Vec2 & pt2D, const Vec3 & pt3D)
{
  if(m._r.rows() > 1)
    std::cerr << "Projection function is not implemented for the radial division undistortion model for more than one parameter.\n";

  Vec3 v = m._R * pt3D + m._t; // from delta to epsilon
  v *= 1.0 / v(2); // normalize to have v(3, :) = 1
  double ru2 = v(0) * v(0) + v(1) * v(1); // undistorted squared radius

  // works for fish - eye, i.e.when distorte image gets smaller on the image plane
  double h1 = sqrt(-4 * m._r(0) * ru2 + 1);
  double h2 = 0.5 * ((-2 * m._r(0) * ru2 + 1) - h1) * (1 / (m._r(0) * m._r(0)));
  double rd = sqrt(h2 * (1 / ru2));

  // distort in epsilon
  double h3 = rd / sqrt(ru2);
  Vec2 u;
  u << v(0) * h3, v(1) * h3;

  // to alpha
  u = m._f * u;
  return (pt2D - u).norm();
}

// Compute the reprojection error for Brown polynomial distortion model

double reproj_error_RP(const M & m, const Vec2 & pt2D, const Vec3 & pt3D)
{
  Vec3 v = m._R * pt3D + m._t; // from delta to epsilon
  v *= 1.0 / v(2); // normalize to have v(3, :) = 1

  double t = 1; // the final radius parameter
  double r = sqrt(v(0) * v(0) + v(1) * v(1));
  for(int i = 0; i < m._r.rows(); ++i)
    t = t + m._r(i) * pow(r, 2 * (i + 1));

  Vec2 u;
  u << v(0) * t, v(1) * t;

  // to alpha
  u = m._f * u;
  return (pt2D - u).norm();
}


// Solve the resection problem with unknown R,t,r,f

void P5PfrSolver::Solve(const Mat &pt2Dx, const Mat &pt3Dx, const int num_r, std::vector<M> *models)
{
  assert(2 == pt2Dx.rows());
  assert(3 == pt3Dx.rows());
  assert(5 == pt3Dx.cols());
  assert(5 == pt2Dx.cols());

  // The radial distorision is represented by: the radial division undistortion
  if(!compute_P5Pfr_Poses_RD(pt2Dx, pt3Dx, num_r, models))
    models->clear();

  // The radial distorision is represented by: Brown polynomial distortion model
  /*if (!compute_P5Pfr_Poses_RP(pt2Dx, pt3Dx, num_r, models))
          models->clear();*/
}

// Compute the residual of the projection distance(pt2D, Project(M,pt3D))

double P5PfrSolver::Error(const M & m, const Vec2 & pt2D, const Vec3 & pt3D)
{
  return reproj_error_RD(m, pt2D, pt3D);
  //return reproj_error_RP( m, pt2D, pt3D);
}

} // namespace resection
} // namespace openMVG

#endif // OPENMVG_MULTIVIEW_RESECTION_P5PFR_CPP