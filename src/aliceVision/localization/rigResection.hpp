// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once
#include <aliceVision/config.hpp>
#include <aliceVision/types.hpp>
#include <aliceVision/feature/imageDescriberCommon.hpp>
#include <aliceVision/camera/PinholeRadial.hpp>
#include <aliceVision/geometry/Pose3.hpp>
#include <aliceVision/numeric/numeric.hpp>

#include <vector>

namespace aliceVision {
namespace localization{

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OPENGV)

/**
 * @brief It computes the pose of a camera rig given the 2d-3d associations of 
 * each camera along with the internal calibration of each camera and the external 
 * calibration of the cameras wrt the main one.
 * 
 * @param[in] vec_pts2d A vector of the same size as the number of the camera in 
 * the rig, each element of the vector contains the 2d points of the associations 
 * for each camera.
 * @param[in] vec_pts3d A vector of the same size as the number of the camera in 
 * the rig, each element of the vector contains the 3d points of the associations 
 * for each camera. A 2d-3d association is represented by (vec_pts2d[i].col(j), vec_pts3d[i].col(j)).
 * @param[in] vec_queryIntrinsics A vector containing the intrinsics for each 
 * camera of the rig.
 * @param[in] vec_subPoses A vector containing the subposes of the cameras wrt 
 * the main one, ie the camera 0. This vector has numCameras-1 elements.
 * @param[in] descTypesPerCamera optional vector of describer types per camera.
 * It is used in the weighting stategy to decide if the resection is strongly supported.
 * @param[out] rigPose The rig pose referred to the position of the main camera.
 * @param[out] inliers A vector of the same size as the number of cameras c
 * ontaining the indices of inliers.
 * @param[in] threshold The threshold in radians to use in the ransac process. It
 * represents the maximum angular error between the direction of the 3D point in
 * space and the bearing vector of the feature (i.e. the direction of the 
 * re-projection ray).
 * @param[in] maxIterations Maximum number of iteration for the ransac process.
 * @param[in] verbosity Mute/unmute the debugging messages.
 * @return true if the ransac has success.
 */
EstimationStatus rigResection(const std::vector<Mat> &vec_pts2d,
                  const std::vector<Mat> &vec_pts3d,
                  const std::vector<camera::PinholeRadialK3 > &vec_queryIntrinsics,
                  const std::vector<geometry::Pose3 > &vec_subPoses,
                  const std::vector< std::vector<feature::EImageDescriberType> > * descTypesPerCamera,
                  geometry::Pose3 &rigPose,
                  std::vector<std::vector<std::size_t> > &inliers,
                  double threshold = degreeToRadian(0.1),
                  std::size_t maxIterations = 100,
                  bool verbosity = true);

#endif

}
}
