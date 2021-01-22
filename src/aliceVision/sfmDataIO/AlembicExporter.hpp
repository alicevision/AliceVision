// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/geometry/Pose3.hpp>
#include <aliceVision/camera/Pinhole.hpp>
#include <aliceVision/types.hpp>

#include <memory>
#include <string>

namespace aliceVision {
namespace sfmDataIO {

class AlembicExporter
{
public:

  explicit AlembicExporter(const std::string& filename);

  ~AlembicExporter();

  /**
   * @brief Return the filename associated to the alembic file.
   * @return the filename associated to the alembic file.
   */
  std::string getFilename() const;

  /**
   * @brief Add SfM Data
   * @param[in] sfmdata SfMData container
   * @param[in] flagsPart filter the elements to add
   */
  void addSfM(const sfmData::SfMData& sfmdata, ESfMData flagsPart = ESfMData::ALL);

  /**
   * @brief Add a SfM single camera
   * @param sfmData The corresponding view scene
   * @param view The corresponding view
   */
  void addSfMSingleCamera(const sfmData::SfMData& sfmData, const sfmData::View& view, ESfMData flagsPart);

  /**
   * @brief Add a SfM camera rig
   * @param sfmData The corresponding rig scene
   * @param rigId The rig Id in the scene
   * @param viewIds The sub-pose view ids in the scene
   */
  void addSfMCameraRig(const sfmData::SfMData& sfmData,
                       IndexT rigId,
                       const std::vector<IndexT>& viewIds,
                       ESfMData flagsPart);

  /**
   * @brief Add a set of 3d points
   * @param[in] points The 3D points to add
   */
  void addLandmarks(const sfmData::Landmarks& points,
                    const sfmData::LandmarksUncertainty& landmarksUncertainty = sfmData::LandmarksUncertainty(),
                    bool withVisibility = true,
                    bool withFeatures = true);

  /**
   * @brief Add a camera
   * @param[in] name The camera identifier
   * @param[in] view The corresponding view
   * @param[in] pose The camera pose (nullptr if undefined)
   * @param[in] intrinsic The camera intrinsic (nullptr if undefined)
   * @param[in] uncertainty The camera uncertainty values (nullptr if undefined)
   * @param[in,out] parent The Alembic parent node
   */
  void addCamera(const std::string& name,
                 const sfmData::View& view,
                 const sfmData::CameraPose* pose = nullptr,
                 std::shared_ptr<camera::IntrinsicBase> intrinsic = nullptr,
                 const Vec6* uncertainty = nullptr);

  /**
   * @brief Add a keyframe to the animated camera
   * @param[in] pose The camera pose
   * @param[in] cam The camera intrinsics parameters
   * @param[in] imagePath The localized image path
   * @param[in] viewId View id
   * @param[in] intrinsicId Intrinsic id
   * @param[in] sensorWidthMM Width of the sensor in millimeters
   */
  void addCameraKeyframe(const geometry::Pose3& pose,
                         const camera::Pinhole* cam,
                         const std::string& imagePath,
                         IndexT viewId,
                         IndexT intrinsicId,
                         float sensorWidthMM=36.0);

  /**
   * @brief Initiate an animated camera
   * @param[in] name The camera identifier
   */
  void initAnimatedCamera(const std::string& name, std::size_t startFrame = 1);
  


  /**
   * @brief Register keyframe on the previous values
   * @param[in] imagePath The Path to the image
   */
  void jumpKeyframe(const std::string& imagePath = std::string());

private:
  struct DataImpl;
  std::unique_ptr<DataImpl> _dataImpl;
};

} // namespace sfmDataIO
} // namespace aliceVision
