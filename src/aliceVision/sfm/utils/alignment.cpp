// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include "aliceVision/sfm/utils/alignment.hpp"
#include "aliceVision/geometry/rigid_transformation3D_srt.hpp"

namespace aliceVision {
namespace sfm {

bool computeSimilarity(const SfM_Data & sfmDataA,
                       const SfM_Data & sfmDataB,
                       double * out_S,
                       Mat3 * out_R,
                       Vec3 * out_t)
{
  assert(out_S != nullptr);
  assert(out_R != nullptr);
  assert(out_t != nullptr);
  
  std::vector<IndexT> commonViewIds;
  getCommonViewsWithPoses(sfmDataA, sfmDataB, commonViewIds);
  if(commonViewIds.size() < 2)
  {
    ALICEVISION_LOG_WARNING("Cannot compute similarities. Need at least 2 common views.");
    return false;
  }
  ALICEVISION_LOG_DEBUG("Found " << commonViewIds.size() << " common views.");

  // Move input point in appropriate container
  Mat xA(3, commonViewIds.size());
  Mat xB(3, commonViewIds.size());
  for (size_t i = 0; i  < commonViewIds.size(); ++i)
  {
    IndexT viewId = commonViewIds[i];
    xA.col(i) = sfmDataA.GetPoses().at(sfmDataA.GetViews().at(viewId)->getPoseId()).center();
    xB.col(i) = sfmDataB.GetPoses().at(sfmDataB.GetViews().at(viewId)->getPoseId()).center();
  }

  // Compute rigid transformation p'i = S R pi + t
  double S;
  Vec3 t;
  Mat3 R;
  std::vector<std::size_t> inliers;
  if(!aliceVision::geometry::ACRansac_FindRTS(xA, xB, S, t, R, inliers, true))
    return false;

  ALICEVISION_LOG_DEBUG("There are " << commonViewIds.size() << " common cameras and " << inliers.size() << " were used to compute the similarity transform.");

  *out_S = S;
  *out_R = R;
  *out_t = t;
  return true;
}

} //namespace sfm
} //namespace aliceVision
