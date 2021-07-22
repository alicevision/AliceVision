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

inline void getCommonPoseId(const sfmData::SfMData& sfmDataA,
                            const sfmData::SfMData& sfmDataB,
                            std::vector<IndexT>& outIndexes)
{
    for (const auto& poseA : sfmDataA.getPoses())
    {
        if (sfmDataB.getPoses().find(poseA.first) != sfmDataB.getPoses().end())
        {
            outIndexes.push_back(poseA.first);
        }
    }
}


void matchViewsByFilePattern(
    const sfmData::SfMData& sfmDataA,
    const sfmData::SfMData& sfmDataB,
    const std::string& filePatternMatching,
    std::vector<std::pair<IndexT, IndexT>>& out_commonViewIds);


void matchViewsByMetadataMatching(
    const sfmData::SfMData& sfmDataA,
    const sfmData::SfMData& sfmDataB,
    const std::vector<std::string>& metadataList,
    std::vector<std::pair<IndexT, IndexT>>& out_commonViewIds);


/**
 * @brief Compute a 5DOF rigid transform between the two set of cameras based on common viewIds.
 *
 * @param[in] sfmDataA
 * @param[in] sfmDataB
 * @param[in] randomNumberGenerator random number generator
 * @param[out] out_S output scale factor
 * @param[out] out_R output rotation 3x3 matrix
 * @param[out] out_t output translation vector
 * @return true if it finds a similarity transformation
 */
bool computeSimilarityFromCommonCameras_viewId(const sfmData::SfMData& sfmDataA,
                       const sfmData::SfMData& sfmDataB,
                      std::mt19937 & randomNumberGenerator,
                       double* out_S,
                       Mat3* out_R,
                       Vec3* out_t);

/**
* @brief Compute a 5DOF rigid transform between the two set of cameras based on common poseIds.
*
* @param[in] sfmDataA
* @param[in] sfmDataB
* @param[in] randomNumberGenerator random number generator
* @param[out] out_S output scale factor
* @param[out] out_R output rotation 3x3 matrix
* @param[out] out_t output translation vector
* @return true if it finds a similarity transformation
*/
bool computeSimilarityFromCommonCameras_poseId(
    const sfmData::SfMData& sfmDataA,
    const sfmData::SfMData& sfmDataB,
    std::mt19937 & randomNumberGenerator,
    double* out_S,
    Mat3* out_R,
    Vec3* out_t);

bool computeSimilarityFromCommonCameras_imageFileMatching(
    const sfmData::SfMData& sfmDataA,
    const sfmData::SfMData& sfmDataB,
    const std::string& filePatternMatching,
    std::mt19937 &randomNumberGenerator,
    double* out_S,
    Mat3* out_R,
    Vec3* out_t);

bool computeSimilarityFromCommonCameras_metadataMatching(
    const sfmData::SfMData& sfmDataA,
    const sfmData::SfMData& sfmDataB,
    const std::vector<std::string>& metadataList,
    std::mt19937 &randomNumberGenerator,
    double* out_S,
    Mat3* out_R,
    Vec3* out_t);


bool computeSimilarityFromCommonMarkers(
    const sfmData::SfMData& sfmDataA,
    const sfmData::SfMData& sfmDataB,
    std::mt19937 &randomNumberGenerator,
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
  for(auto& poseIt: sfmData.getPoses())
  {
    geometry::Pose3 pose = poseIt.second.getTransform();
    pose = pose.transformSRt(S, R, t);
    poseIt.second.setTransform(pose);
  }
  for (auto& rigIt : sfmData.getRigs())
  {
      for (auto& subPose : rigIt.second.getSubPoses())
      {
          subPose.pose.center() *= S;
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
 * @brief Compute a new coordinate system using the GPS data available in the metadata. The transformation will bring the
 * model in the cartesian metric reference system.
 * @param[in] sfmData The sfmdata containing the scene.
 * @param[in,out] randomNumberGenerator The random number generator.
 * @param[out] out_S the scale factor.
 * @param[out] out_R the rotation.
 * @param[out] out_t the translation.
 * @return false if no reliable transformation can be computed or the sfmdata does not contain gps metadata, true otherwise.
 */
bool computeNewCoordinateSystemFromGpsData(const sfmData::SfMData& sfmData, std::mt19937 &randomNumberGenerator,
                                             double& out_S,
                                             Mat3& out_R,
                                             Vec3& out_t);

/**
 * @brief Retrieve the View Id from a string expression (integer with the view id or filename of the input image).
 * @param[in] sfmData: sfm scene
 * @param[in] camName: cameraName name of the reference picture (e.g. "cam_1.png")
 */
IndexT getViewIdFromExpression(const sfmData::SfMData& sfmData, const std::string& camName);

/**
 * @brief Retrieve the id of the camera in the center of the reconstructed cameras.
 */
IndexT getCenterCameraView(const sfmData::SfMData& sfmData);

/**
 * @brief Compute the new coordinate system in the given reconstruction so that the given camera is the
 * origin of the world coordinate system. If this camera (view) does not exist, the transformation
 * remains unchanged.
 *
 * @param[in] sfmData
 * @param[in] viewId
 * @param[out] out_S scale
 * @param[out] out_R rotation
 * @param[out] out_t translation
 */
void computeNewCoordinateSystemFromSingleCamera(const sfmData::SfMData& sfmData,
                                                const IndexT viewId,
                                                double& out_S,
                                                Mat3& out_R,
                                                Vec3& out_t);

struct MarkerWithCoord
{
    int id;
    Vec3 coord;
};

std::istream& operator>>(std::istream& in, MarkerWithCoord& marker);

std::ostream& operator<<(std::ostream& os, const MarkerWithCoord& marker);

/**
* @brief Compute a new coordinate system so that markers are aligned with the target coordinates.
*
* @param[in] sfmData
* @param[in] imageDescriberType
* @param[in] markers: markers id associated to a target 3D coordinate
* @param[in] withScaling
* @param[out] out_S scale
* @param[out] out_R rotation
* @param[out] out_t translation
*/
bool computeNewCoordinateSystemFromSpecificMarkers(const sfmData::SfMData& sfmData,
    const feature::EImageDescriberType& imageDescriberType,
    const std::vector<MarkerWithCoord>& markers,
    bool withScaling,
    double& out_S,
    Mat3& out_R,
    Vec3& out_t);

} // namespace sfm
} // namespace aliceVision
