// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#pragma once

#include "aliceVision/types.hpp"
#include "aliceVision/geometry/Frustum.hpp"

namespace aliceVision {
namespace sfm {

struct SfMData;

class FrustumFilter
{
public:
  typedef HashMap<IndexT, geometry::Frustum> FrustumsT;
  typedef HashMap<IndexT, std::pair<double, double> > NearFarPlanesT;

  // Constructor
  FrustumFilter(const SfMData & sfm_data,
    const double zNear = -1., const double zFar = -1.);

  // Init a frustum for each valid views of the SfM scene
  void initFrustum(const SfMData & sfm_data);

  // Return intersecting View frustum pairs
  PairSet getFrustumIntersectionPairs() const;

  // Export defined frustum in PLY file for viewing
  bool export_Ply(const std::string & filename) const;

private:

  /// Init near and far plane depth from SfMData structure or defined value
  void init_z_near_z_far_depth(const SfMData & sfm_data,
    const double zNear = -1., const double zFar = -1.);

  //--
  // Data
  //--
  bool _bTruncated; // Tell if we use truncated or infinite frustum

  FrustumsT frustum_perView; // Frustum for the valid view (defined pose+intrinsic)

  NearFarPlanesT z_near_z_far_perView; // Near & Far plane distance per view
};

} // namespace sfm
} // namespace aliceVision
