// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/camera/Pinhole.hpp>
#include <pxr/usd/usd/stage.h>

namespace aliceVision {
namespace sfmDataIO {

/**
 * @brief Export SfM data to USD (Universal Scene Description) format.
 * 
 * The UsdExporter class provides functionality to export camera poses and intrinsics
 * from AliceVision's SfM data to Pixar's USD format, enabling integration with
 * USD-based pipelines and DCC applications.
 */
class UsdExporter
{
public:
    /**
     * @brief Construct a new USD Exporter.
     * 
     * @param filename The output USD file path
     * @param frameRate The frame rate for the animation timeline (e.g., 24.0 for 24 fps)
     */
    UsdExporter(const std::string & filename, double frameRate);

    /**
     * @brief Create a new camera in the USD stage.
     * 
     * Creates a USD camera primitive with the specified name. This camera
     * can then be animated using addFrame().
     * 
     * @param cameraName The unique name for the camera in the USD stage
     */
    void createNewCamera(const std::string & cameraName);
    
    /**
     * @brief Add a frame of camera data (pose and intrinsics) to an existing camera.
     * 
     * Records the camera's pose and intrinsic parameters at a specific frame,
     * creating keyframe animation data in the USD file.
     * 
     * @param cameraName The name of the camera to add the frame to
     * @param pose The camera pose (position and orientation) for this frame
     * @param camera The pinhole camera model containing intrinsic parameters
     * @param frameId The frame index/time code for this keyframe
     */
    void addFrame(const std::string & cameraName, const sfmData::CameraPose & pose, const camera::Pinhole & camera, IndexT frameId);

    /**
     * @brief Add a frame of camera data with uncertainty ellipsoid.
     *
     * Like addFrame(), but also animates a UsdGeomSphere ellipsoid (sibling of the camera,
     * in world space) representing the 1-sigma position uncertainty from the 6x6 covariance.
     *
     * @param cameraName   The name of the camera to add the frame to
     * @param pose         The camera pose for this frame
     * @param intrinsic    The pinhole camera model containing intrinsic parameters
     * @param uncertainty  The 6x6 pose covariance matrix (DOF: [angleAxis, center])
     * @param frameId      The frame index/time code for this keyframe
     */
    void addFrameWithUncertainty(const std::string & cameraName,
                                 const sfmData::CameraPose & pose,
                                 const camera::Pinhole & intrinsic,
                                 const sfmData::PoseUncertainty & uncertainty,
                                 IndexT frameId);

    /**
     * @brief Finalize and save the USD file.
     * 
     * Completes the USD export process and writes the data to disk.
     * Should be called after all cameras and frames have been added.
     */
    void terminate();

private:
    pxr::UsdStageRefPtr _stage;       ///< USD stage containing the scene hierarchy
    IndexT _startTimeCode;             ///< First frame index in the animation timeline
    IndexT _endTimeCode;               ///< Last frame index in the animation timeline
};

}  // namespace sfmDataIO
}  // namespace aliceVision
