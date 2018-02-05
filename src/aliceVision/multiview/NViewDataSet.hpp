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

// A N-view metric dataset.
// All points are seen by all cameras.
struct NViewDataSet {
  std::vector<Mat3> _K;   // Internal parameters (fx, fy, etc).
  std::vector<Mat3> _R;   // Rotation.
  std::vector<Vec3> _t;   // Translation.
  std::vector<Vec3> _C;   // Camera centers.
  Mat3X _X;          // 3D points.
  std::vector<Mat2X> _x;  // Projected points; may have noise added.
  std::vector<Vecu>  _x_ids;// Indexes of points corresponding to the projections

  std::size_t _n;  // Actual number of cameras.

  //-- Return P=K*[R|t] for the Inth camera
  Mat34 P(size_t i) const;

  /// Export in PLY the point structure and camera and camera looking dir.
  void ExportToPLY(const std::string & out_file_name) const;
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

/// Place cameras on a circle with point in the center
NViewDataSet NRealisticCamerasRing(std::size_t nviews, std::size_t npoints,
                                   const NViewDatasetConfigurator
                                     config = NViewDatasetConfigurator());

/// Place cameras on cardiod shape with point in the center
NViewDataSet NRealisticCamerasCardioid(std::size_t nviews, std::size_t npoints,
                                       const NViewDatasetConfigurator
                                        config = NViewDatasetConfigurator());

} // namespace aliceVision
