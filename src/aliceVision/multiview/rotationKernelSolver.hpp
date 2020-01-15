// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// Copyright (c) 2010 libmv contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/multiview/projection.hpp>
#include <aliceVision/multiview/twoViewKernel.hpp>

#include <vector>

namespace aliceVision {
namespace rotation {
namespace kernel {

using namespace std;

struct ThreePointRotationSolver {
  enum { MINIMUM_SAMPLES = 3 };
  enum { MAX_MODELS = 1 };
  /**
   * Computes the rotation that transforms p1 to p2 
   *
   * \param p1  A 3xN matrix of column vectors.
   * \param p2  A 3xN matrix of column vectors.
   * \param Hs A vector into which the computed homography is stored.
   *
   * The estimated rotation should approximately hold the condition p2 = T p1.
   */
  static void Solve(const Mat &p1, const Mat &p2, vector<Mat3> *Rs);
};

struct RotationError {
  static double Error(const Mat &R, const Vec3 &p1, const Vec3 &p2) {
    
    Eigen::Vector3d p1_est = R * p1;

    double sina = (p1_est.cross(p2)).norm();
    double cosa = p1_est.dot(p2);

    double angle = atan2(sina, cosa);
    

    return angle * angle;
  }
};


}  // namespace kernel
}  // namespace rotation
}  // namespace aliceVision
