// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/robustEstimation/ISolver.hpp>
#include <aliceVision/multiview/resection/ISolverErrorResection.hpp>

namespace aliceVision {
namespace multiview {
namespace resection {

/**
 * @brief The structure P5PfrModel contain one output model
 *        camera description with radial division undistortion parameters 'KRCrd'
 */
struct P5PfrModel
{
  P5PfrModel(const Mat& R, const Vec3& t, const Vec& r, double f)
    : _R(R)
    , _t(t)
    , _r(r)
    , _f(f)
  {}

  /**
   * @brief Inversion of the radial division undistortion to Brown polynomial distortion model conversion
   * @param[in] x2d Points on which is the difference minimized, dmax//max(C.K([1 5]))*[0 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 0.95 1] implicit
   * @param[in] maxRadius Maximal distorted radius, 1 implicit
   * @return camera description with polynomial radial distoriton parameters 'KRCp'
   */
  Mat divisionToPolynomialModelDistortion(const Mat& x2d /*,double maxRadius*/) const;

  /// rotation matrix
  Mat _R;
  /// translation vector
  Vec3 _t;
  /// radial division undistortion parameters
  Vec _r;
  /// focal length
  double _f;
};

/**
 * @brief computeP5PfrPosesRD
 * @param[in] featureVectors
 * @param[in] worldPoints
 * @param[in] numOfRadialCoeff
 * @param[out] solutions
 */
bool computePosesRD(const Mat& featureVectors, const Mat& worldPoints, int numOfRadialCoeff, std::vector<P5PfrModel>* solutions);

/**
 * @brief Compute computePosesRD and transform the radial division undistortion to Brown polynomial distortion model
 * @param[in] featureVectors
 * @param[in] worldPoints
 * @param[in] numOfRadialCoeff
 * @param[out] solutions
 */
bool computePosesRP(const Mat& featureVectors, const Mat& worldPoints, int numOfRadialCoeff, std::vector<P5PfrModel>* solutions);

/**
 * @brief Compute the reprojection error for the radial division undistortion model
 * @param[in] P5PfrModel model
 * @param[in] p2d feature vector
 * @param[in] p3d corresponding 3D world point
 * @return reprojection error for the radial division undistortion model
 */
double reprojectionErrorRD(const P5PfrModel& model, const Vec2& p2d, const Vec3& p3d);

/**
 * @brief Compute the reprojection error for Brown polynomial distortion model
 * @param[in] P5PfrModel model
 * @param[in] p2d feature vector
 * @param[in] p3d corresponding 3D world point
 * @return reprojection error for Brown polynomial distortion model
 */
double reprojectionErrorRP(const P5PfrModel& model, const Vec2 &p2d, const Vec3& p3d);

struct P5PfrError : public ISolverErrorResection<P5PfrModel>
{
  /**
   * @brief Compute the residual of the projection distance(p2d, project(P,p3d))
   * @param[in] model solution
   * @param[in] p2d feature vector
   * @param[in] p3d corresponding 3D world point
   */
  inline double error(const P5PfrModel& model, const Vec2& p2d, const Vec3& p3d) const override
  {
    return reprojectionErrorRD(model, p2d, p3d);
  }
};


/**
 * @brief Compute the absolute pose, focal length and radial distortion of a camera using three 3D-to-2D correspondences
 * @author Tomas Pajdla, adapted to aliceVision by Michal Polic
 * @ref [1] Time solution to the absolute pose problem with unknown radial distortion and focal length
 *          Kukelova, Z., Bujnak, M., and Pajdla T.
 *          ICCV 2013
 */
template<int numOfRadialCoeff_>
class P5PfrSolver : public robustEstimation::ISolver<P5PfrModel>
{
public:

  /**
   * @brief Return the minimum number of required samples
   * @return minimum number of required samples
   */
  inline std::size_t getMinimumNbRequiredSamples() const override
  {
    return 5;
  }

  /**
   * @brief Return the maximum number of models
   * @return maximum number of models
   */
  inline std::size_t getMaximumNbModels() const override
  {
    return 10;
  }

  /**
   * @brief Solve the problem of camera pose.
   *
   * @param[in] x2d featureVectors 2 x 5 matrix with feature vectors with principal point at [0; 0] (each column is a vector)
   * @param[in] x3d worldPoints 3 x 5 matrix with corresponding 3D world points (each column is a point)
   * @param[in] numOfRadialCoeff number of radial distortion parameters to be computed [min 1, max 3]
   * @param[out] models M x n vector that will contain the each solution in structure M
   */
  void solve(const Mat& x2d, const Mat& x3d, std::vector<P5PfrModel>& models) const override
  {
    assert(2 == x2d.rows());
    assert(3 == x3d.rows());
    assert(5 == x3d.cols());
    assert(5 == x2d.cols());
    assert(numOfRadialCoeff_ >= 1 && numOfRadialCoeff_ <= 3 && "P5PfrSolver error: the number of radial parameters must be between 1 to 3 !");

    // the radial distortion is represented by: the radial division undistortion
    if(!computePosesRD(x2d, x3d, numOfRadialCoeff_, &models))
      models.clear();

    // the radial distortion is represented by: Brown polynomial distortion model
    //if(!P5PfrSolver::computePosesRP(x2d, x3d, numR, &models))
    //    models.clear();
  }

  /**
   * @brief Solve the problem of camera pose..
   *
   * @param[in]  x2d 2d points in the first image. One per column.
   * @param[in]  x3d Corresponding 3d points in the second image. One per column.
   * @param[out] models A vector into which the computed models are stored.
   * @param[in]  weights.
   */
  void solve(const Mat& x2d, const Mat& x3d, std::vector<P5PfrModel>& models, const std::vector<double>& weights) const override
  {
     throw std::logic_error("P5PfrSolver does not support problem solving with weights.");
  }
};

} // namespace resection
} // namespace multiview
} // namespace aliceVision
