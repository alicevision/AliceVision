// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/types.hpp>
#include <aliceVision/geometry/Frustum.hpp>

namespace aliceVision {

namespace sfmData {
class SfMData;
} // namespace sfmData

namespace sfm {

class FrustumFilter
{
public:
  typedef HashMap<IndexT, geometry::Frustum> FrustumsT;
  typedef HashMap<IndexT, std::pair<double, double> > NearFarPlanesT;

  FrustumFilter(const sfmData::SfMData& sfmData, const double zNear = -1., const double zFar = -1.);

  /// init a frustum for each valid views of the SfM scene
  void initFrustum(const sfmData::SfMData& sfmData);

  /// return intersecting View frustum pairs
  PairSet getFrustumIntersectionPairs() const;

  /// export defined frustum in PLY file for viewing
  bool export_Ply(const std::string& filename) const;

private:

  /// Init near and far plane depth from SfMData structure or defined value
  void init_z_near_z_far_depth(const sfmData::SfMData& sfmData, const double zNear = -1., const double zFar = -1.);

  // Data

  /// tell if we use truncated or infinite frustum
  bool _bTruncated;
  /// frustum for the valid view (defined pose+intrinsic)
  FrustumsT frustum_perView;
  /// Near & Far plane distance per view
  NearFarPlanesT z_near_z_far_perView;
};

} // namespace sfm
} // namespace aliceVision
