// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// Copyright (c) 2010 libmv contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>

#include <vector>

namespace aliceVision {

/**
 * @brief A N-view metric dataset.
 *        All points are seen by all cameras.
 */
struct NViewDataSet
{
  /// Internal parameters (fx, fy, etc).
  std::vector<Mat3> _K;
  /// Rotation.
  std::vector<Mat3> _R;
  /// Translation.
  std::vector<Vec3> _t;
  /// Camera centers.
  std::vector<Vec3> _C;
  /// 3D points.
  Mat3X _X;
  /// Projected points; may have noise added.
  std::vector<Mat2X> _x;
  /// Indexes of points corresponding to the projections
  std::vector<Vecu>  _x_ids;
  /// Actual number of cameras.
  std::size_t _n;

  /**
   * @brief Return P=K*[R|t] for the Inth camera
   * @param[in] i The Inth camera
   * @return P=K*[R|t] for the Inth camera
   */
  Mat34 P(size_t i) const;

  /**
   * @brief Export in PLY the point structure and camera and camera looking dir.
   * @param[in] outFilename
   */
  void exportToPLY(const std::string& outFilename) const;
};

struct NViewDatasetConfigurator
{
  /// Internal camera parameters (focal, principal point)
  int _fx, _fy, _cx, _cy;

  /// Camera random position parameters
  double _dist;
  double _jitter_amount;

  NViewDatasetConfigurator(int fx = 1000,  int fy = 1000,
                           int cx = 500,   int cy  = 500,
                           double distance = 1.5,
                           double jitter_amount = 0.01 );
};

/**
 * @brief Place cameras on a circle with point in the center
 */
NViewDataSet NRealisticCamerasRing(std::size_t nviews, std::size_t npoints, const NViewDatasetConfigurator& config = NViewDatasetConfigurator());

/**
 * @brief Place cameras on cardiod shape with point in the center
 */
NViewDataSet NRealisticCamerasCardioid(std::size_t nviews, std::size_t npoints, const NViewDatasetConfigurator& config = NViewDatasetConfigurator());

} // namespace aliceVision
