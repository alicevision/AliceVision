// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#pragma once

#include "aliceVision/sfm/sfm_data.hpp"
#include "aliceVision/geometry/pose3.hpp"

namespace aliceVision {
namespace sfm {

inline void getCommonViews(const SfM_Data & sfmDataA,
                           const SfM_Data & sfmDataB,
                           std::vector<IndexT>& outIndexes)
{
  for(const auto& viewA: sfmDataA.GetViews())
  {
    if(sfmDataB.GetViews().find(viewA.first) != sfmDataB.GetViews().end())
    {
      outIndexes.push_back(viewA.first);
    }
  }
}

inline void getCommonViewsWithPoses(const SfM_Data & sfmDataA,
                                    const SfM_Data & sfmDataB,
                                    std::vector<IndexT>& outIndexes)
{
  for(const auto& viewA: sfmDataA.GetViews())
  {
    // check there is a view with the same ID and both of them have pose and 
    // intrinsics defined
    if(!sfmDataA.IsPoseAndIntrinsicDefined(viewA.second.get()))
      continue;

    if(sfmDataB.GetViews().find(viewA.first) != sfmDataB.GetViews().end() &&
       sfmDataB.IsPoseAndIntrinsicDefined(viewA.first))
    {
      outIndexes.push_back(viewA.first);
    }
  }
}

/**
 * @brief Compute a 5DOF rigid transform between the two set of cameras.
 *
 * @param[in] sfmDataA
 * @param[in] sfmDataB
 * @param[out] out_S output scale factor
 * @param[out] out_R output rotation 3x3 matrix
 * @param[out] out_t output translation vector
 * @return true if it finds a similarity transformation
 */
bool computeSimilarity(const SfM_Data & sfmDataA,
                       const SfM_Data & sfmDataB,
                       double * out_S,
                       Mat3 * out_R,
                       Vec3 * out_t);


inline void applyTransform(SfM_Data & sfmData,
                           const double S,
                           const Mat3& R,
                           const Vec3& t,
                           bool transformControlPoints = false)
{
  for(auto& viewPair: sfmData.views)
  {
    const View& view = *viewPair.second;
    geometry::Pose3 pose = sfmData.getPose(view);
    pose = pose.transformSRt(S, R, t);
    sfmData.setPose(view, pose);
  }
  
  for(auto& landmark: sfmData.structure)
  {
    landmark.second.X = S * R * landmark.second.X + t;
  }
  
  if(!transformControlPoints)
    return;
  
  for(auto& controlPts: sfmData.control_points)
  {
    controlPts.second.X = S * R * controlPts.second.X + t;
  }
}

}
}
