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

#ifndef OPENMVG_MULTIVIEW_RESECTION_P4PF_H_
#define OPENMVG_MULTIVIEW_RESECTION_P4PF_H_

#include <iostream>
#include "openMVG/numeric/numeric.h"

namespace openMVG {
namespace resection {

double dabs(double a);

void GJ(double *A, int rcnt, int ccnt, double tol);

// prepare polynomial coefficients
void CalcCoefs(double const *src1, double const *src2, double const *src3, double const *src4,
               double const *src5, double *dst1);

// [glab, glac, glad, glbc, glbd, glcd], [a1; a2], [b1; b2], [c1; c2], [d1;d2]
//  glXY - ||X-Y||^2 - quadratic distances between 3D points X and Y
//  a1 (a2) = x (resp y) measurement of the first 2D point
//  b1 (b2) = x (resp y) measurement of the second 2D point
//  c1 (c2) = x (resp y) measurement of the third 2D point
//  d1 (d2) = x (resp y) measurement of the fourth 2D point
//
// output *A - 10x10 action matrix
void compute_p4pf_poses(double *glab, double *a1, double *b1, double *c1, double *d1, double *A);

bool isNan(Eigen::MatrixXcd *A);

bool validSol(Eigen::MatrixXcd *sol, Mat *vSol);

void getRigidTransform(Mat *pp1, Mat *pp2, Mat *R, Vec3 *t);

// The structure M contain one output model

struct M
{
  double _f;
  Mat _R;
  Vec3 _t;

  M(Mat R, Vec3 t, double f) : _R(R), _t(t), _f(f) { }

  Mat34 getP() const
  {
    Mat34 P;
    Mat K = Mat(3, 3);
    K << _f, 0, 0,
            0, _f, 0,
            0, 0, 1;
    P.block(0, 0, 3, 3) = K*_R;
    P.block(0, 3, 3, 1) = K*_t;
    return P;
  }
};

/*
 *      Author: Martin Bujnak, adapted to openMVG by Michal Polic
 * Description: Compute the absolute pose and focal length of a camera using three 3D-to-2D correspondences
 *   Reference: [1] A general solution to the p4p
 *              Bujnak, M., Kukelova, Z., and Pajdla T.
 *              CVPR 2008
 *
 *       Input: featureVectors: 2x4 matrix with feature vectors with subtracted principal point (each column is a vector)
 *              worldPoints: 3x4 matrix with corresponding 3D world points (each column is a point)
 *
 *      Output: solutions: M x n vector that will contain the each solution in structure M (rotation matrix M._R,
 *						  translation vector M._t, focal length M._f). Following equation holds for each solution:
 *						  lambda*pt2D = diag([M._f M._f 1])*[M._R M._t] * pt3D
 */
struct P4PfSolver
{

  enum
  {
    MINIMUM_SAMPLES = 4
  };

  enum
  {
    MAX_MODELS = 10
  };

  // Solve the problem of camera pose.
  static void Solve(const Mat &pt2Dx, const Mat &pt3Dx, std::vector<M> *models);

  // Compute the residual of the projection distance(pt2D, Project(P,pt3D))
  static double Error(const M & model, const Vec2 & pt2D, const Vec3 & pt3D);
};

} // namespace resection
} // namespace openMVG

#endif // OPENMVG_MULTIVIEW_RESECTION_P4PF_H_