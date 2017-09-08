// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#ifndef OPENMVG_SFM_DATA_FILTERS_FRUSTUM_HPP
#define OPENMVG_SFM_DATA_FILTERS_FRUSTUM_HPP

#include "aliceVision/types.hpp"
#include "aliceVision/geometry/frustum.hpp"

namespace aliceVision {
namespace sfm {

struct SfM_Data;

class Frustum_Filter
{
public:
  typedef Hash_Map<IndexT, geometry::Frustum> FrustumsT;
  typedef Hash_Map<IndexT, std::pair<double, double> > NearFarPlanesT;

  // Constructor
  Frustum_Filter(const SfM_Data & sfm_data,
    const double zNear = -1., const double zFar = -1.);

  // Init a frustum for each valid views of the SfM scene
  void initFrustum(const SfM_Data & sfm_data);

  // Return intersecting View frustum pairs
  Pair_Set getFrustumIntersectionPairs() const;

  // Export defined frustum in PLY file for viewing
  bool export_Ply(const std::string & filename) const;

private:

  /// Init near and far plane depth from SfM_Data structure or defined value
  void init_z_near_z_far_depth(const SfM_Data & sfm_data,
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

#endif // OPENMVG_SFM_DATA_FILTERS_FRUSTUM_HPP
