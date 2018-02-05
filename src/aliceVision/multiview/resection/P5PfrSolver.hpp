// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>

#include <iostream>

namespace aliceVision {
namespace resection {

/**
 * @brief The structure p5pfrModel contain one output model
 */
struct p5pfrModel
{
  p5pfrModel(Mat R, Vec3 t, Vec r, double f)
    : _R(R)
    , _t(t)
    , _r(r)
    , _f(f)
  {}

  Mat _R;
  Vec3 _t;
  Vec _r;
  double _f;
};

/**
 * @brief Compute the absolute pose, focal length and radial distorsion of a camera using three 3D-to-2D correspondences
 * @author Tomas Pajdla, adapted to aliceVision by Michal Polic
 * @ref [1] Time solution to the absolute pose problem with unknown radial distortion and focal length
 *          Kukelova, Z., Bujnak, M., and Pajdla T.
 *          ICCV 2013
 */
struct P5PfrSolver
{
  enum
  {
    MINIMUM_SAMPLES = 5
  };

  enum
  {
    MAX_MODELS = 10
  };

  /**
   * @brief Solve the problem of camera pose.
   *
   * @param pt2Dx featureVectors:
   * 2 x 5 matrix with feature vectors with principal point at [0; 0] (each column is a vector)
   *
   * @param pt3Dx worldPoints:
   * 3 x 5 matrix with corresponding 3D world points (each column is a point)
   *
   * @param num_r numOfRadialCoeff:
   * integer which reperesents how many radial distorsion parameters should be computed [min 1, max 3]
   *
   * @param solutions:
   * M x n vector that will contain the each solution in structure M (p5pfModel._R - rotation matrix,
   * p5pfModel._t - translation vector, p5pfModel._r - the radial division undistortion parameters, p5pfModel._f - focal length).
   */
  static void solve(const Mat &pt2Dx,
                    const Mat &pt3Dx,
                    const int num_r,
                    std::vector<p5pfrModel> *models);

  /**
   * @brief Compute the residual of the projection distance(pt2D, Project(P,pt3D))
   * @param model solution
   * @param pt2D feature vector
   * @param pt3D corresponding 3D world point
   */
  static double error(const p5pfrModel &model,
                      const Vec2 &pt2D,
                      const Vec3 &pt3D);
};

/**
 * @brief divisionToPolynomialModelDistortion
 * inversion of the radial division undistortion to Brown polynomial distortion model conversion
 * @author Tomas Pajdla, adapted to aliceVision by Michal Polic
 * @param divisionModel camera description with radial division undistortion parameters 'KRCrd'
 * @param maxRadius maximal distorted radius, 1 implicit
 * @param points2d  points on which is the difference minimized, dmax//max(C.K([1 5]))*[0 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 0.95 1] implicit
 * @return camera description with polynomial radial distoriton parameters 'KRCp'
 */
Mat divisionToPolynomialModelDistortion(const p5pfrModel &divisionModel,
                                        double maxRadius,
                                        const Mat &points2d);

/**
 * @brief computeP5PfrPosesRD
 * @param featureVectors
 * @param worldPoints
 * @param numOfRadialCoeff
 * @param solutions
 */
bool computeP5PfrPosesRD(const Mat &featureVectors,
                         const Mat &worldPoints,
                         int numOfRadialCoeff,
                         std::vector<p5pfrModel> *solutions);

/**
 * @brief Compute computeP5PfrPosesRD and transform the radial division undistortion to Brown polynomial distortion model
 * @param featureVectors
 * @param worldPoints
 * @param numOfRadialCoeff
 * @param solutions
 */
bool computeP5PfrPosesRP(const Mat &featureVectors,
                         const Mat &worldPoints,
                         int numOfRadialCoeff,
                         std::vector<p5pfrModel> *solutions);

/**
 * @brief Compute the reprojection error for the radial division undistortion model
 * @param m P5Pfr model
 * @param pt2D feature vector
 * @param pt3D corresponding 3D world point
 * @return reprojection error for the radial division undistortion model
 */
double reprojectionErrorRD(const p5pfrModel &m,
                           const Vec2 &pt2D,
                           const Vec3 &pt3D);

/**
 * @brief Compute the reprojection error for Brown polynomial distortion model
 * @param m P5Pfr model
 * @param pt2D feature vector
 * @param pt3D corresponding 3D world point
 * @return reprojection error for Brown polynomial distortion model
 */
double reprojectionErrorRP(const p5pfrModel &m,
                           const Vec2 &pt2D,
                           const Vec3 &pt3D);

} // namespace resection
} // namespace aliceVision
