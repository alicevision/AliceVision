// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfm/SfMData.hpp>

namespace aliceVision {
namespace sfm {

/// Generic basis struct for triangulation of track data contained
///  in the SfMData scene structure.
struct StructureComputation_basis
{
  bool _bConsoleVerbose;

  StructureComputation_basis(bool bConsoleVerbose = false);

  virtual void triangulate(SfMData & sfm_data) const = 0;
};


/// Triangulation of track data contained in the structure of a SfMData scene.
// Use a blind estimation:
// - Triangulate tracks using all observations
// - Inlier/Outlier classification is done by a cheirality test
struct StructureComputation_blind: public StructureComputation_basis
{
  StructureComputation_blind(bool bConsoleVerbose = false);

  virtual void triangulate(SfMData & sfm_data) const;
};

/// Triangulation of track data contained in the structure of a SfMData scene.
// Use a robust estimation:
// - Triangulate tracks using a RANSAC scheme
// - Check cheirality and a pixel residual error (TODO: make it a parameter)
struct StructureComputation_robust: public StructureComputation_basis
{
  StructureComputation_robust(bool bConsoleVerbose = false);

  virtual void triangulate(SfMData & sfm_data) const;

  /// Robust triangulation of track data contained in the structure
  /// All observations must have View with valid Intrinsic and Pose data
  /// Invalid landmark are removed.
  void robust_triangulation(SfMData & sfm_data) const;

  /// Robustly try to estimate the best 3D point using a ransac Scheme
  /// Return true for a successful triangulation
  bool robust_triangulation(
    const SfMData & sfm_data,
    const Observations & observations,
    Vec3 & X,
    const IndexT min_required_inliers = 3,
    const IndexT min_sample_index = 3) const;

private:
  /// Triangulate a given track from a selection of observations
  Vec3 track_sample_triangulation(
    const SfMData & sfm_data,
    const Observations & observations,
    const std::set<IndexT> & samples) const;
};

} // namespace sfm
} // namespace aliceVision
