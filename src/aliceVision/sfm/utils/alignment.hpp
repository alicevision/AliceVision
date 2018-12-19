// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/geometry/Pose3.hpp>

namespace aliceVision {
namespace sfm {

inline void getCommonViews(const sfmData::SfMData& sfmDataA,
                           const sfmData::SfMData& sfmDataB,
                           std::vector<IndexT>& outIndexes)
{
  for(const auto& viewA: sfmDataA.getViews())
  {
    if(sfmDataB.getViews().find(viewA.first) != sfmDataB.getViews().end())
    {
      outIndexes.push_back(viewA.first);
    }
  }
}

inline void getCommonViewsWithPoses(const sfmData::SfMData& sfmDataA,
                                    const sfmData::SfMData& sfmDataB,
                                    std::vector<IndexT>& outIndexes)
{
  for(const auto& viewA: sfmDataA.getViews())
  {
    // check there is a view with the same ID and both of them have pose and 
    // intrinsics defined
    if(!sfmDataA.isPoseAndIntrinsicDefined(viewA.second.get()))
      continue;

    if(sfmDataB.getViews().find(viewA.first) != sfmDataB.getViews().end() &&
       sfmDataB.isPoseAndIntrinsicDefined(viewA.first))
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
bool computeSimilarity(const sfmData::SfMData& sfmDataA,
                       const sfmData::SfMData& sfmDataB,
                       double* out_S,
                       Mat3* out_R,
                       Vec3* out_t);


/**
 * @brief Apply a transformation the given SfMData
 *
 * @param sfmData The goiven SfMData
 * @param S scale
 * @param R rotation
 * @param t translation
 * @param transformControlPoints
 */
inline void applyTransform(sfmData::SfMData& sfmData,
                           const double S,
                           const Mat3& R,
                           const Vec3& t,
                           bool transformControlPoints = false)
{
  for(auto& viewPair: sfmData.views)
  {
    const sfmData::View& view = *viewPair.second;
    if(sfmData.existsPose(view))
    {
      geometry::Pose3 pose = sfmData.getPose(view).getTransform();
      pose = pose.transformSRt(S, R, t);
      sfmData.setPose(view, sfmData::CameraPose(pose));
    }
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

/**
 * @brief Compute the new coordinate system in the given reconstruction so that the mean
 * of the camera centers is the origin of the world coordinate system, a
 * dominant plane P is fitted to the set of the optical centers and the scene
 * aligned so that P roughly define the (x,y) plane, and the scale is set so
 * that the optical centers RMS is "1.0".
 * (Hartley-like normalization, p.180)
 *
 * @param[in] sfmData
 * @param[out] out_S scale
 * @param[out] out_R rotation
 * @param[out] out_t translation
 */
void computeNewCoordinateSystemFromCameras(const sfmData::SfMData& sfmData,
                                           double& out_S,
                                           Mat3& out_R,
                                           Vec3& out_t);

/**
 * @brief Compute the new coordinate system in the given reconstruction so that a landmark (e.g. artificial)
 * defines the origin of the world coordinate system, a dominant plane P is fitted from
 * a subset of landmarks (e.g. artificial ones) and the scene aligned so that P
 * roughly defined the (x,y) plane, and scale is set so that their RMS is "1.0".
 * (Hartley-like normalization, p.180)
 *
 * @param[in] sfmData
 * @param[in] imageDescriberTypes Image describer types used for compute the mean of the point cloud
 * @param[out] out_S scale
 * @param[out] out_R rotation
 * @param[out] out_t translation
 */
void computeNewCoordinateSystemFromLandmarks(const sfmData::SfMData& sfmData,
                                             const std::vector<feature::EImageDescriberType>& imageDescriberTypes,
                                             double& out_S,
                                             Mat3& out_R,
                                             Vec3& out_t);

/**
 * @brief Compute the new coordinate system in the given reconstruction so that the given camera is the
 * origin of the world coordinate system. If this camera (view) does not exist, the transformation
 * remains unchanged.
 *
 * @param[in] sfmData
 * @param[in] cameraName name of the reference picture (e.g. "cam_1.png")
 * @param[out] out_S scale
 * @param[out] out_R rotation
 * @param[out] out_t translation
 */
void computeNewCoordinateSystemFromSingleCamera(const sfmData::SfMData& sfmData,
                                                const std::string& cameraName,
                                                double& out_S,
                                                Mat3& out_R,
                                                Vec3& out_t);

} // namespace sfm
} // namespace aliceVision
