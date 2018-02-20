// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfm/SfMData.hpp>
#include <aliceVision/sfm/sfmDataIO.hpp>
#include <aliceVision/geometry/Pose3.hpp>
#include <aliceVision/camera/Pinhole.hpp>
#include <aliceVision/types.hpp>

#include <memory>
#include <string>

namespace aliceVision {
namespace sfm {

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
  void addSfM(const sfm::SfMData& sfmdata, ESfMData flagsPart = ESfMData::ALL);

  /**
   * @brief Add a SfM single camera
   * @param sfmData The corresponding view scene
   * @param view The corresponding view
   */
  void addSfMSingleCamera(const SfMData& sfmData, const View& view);

  /**
   * @brief Add a SfM camera rig
   * @param sfmData The corresponding rig scene
   * @param rigId The rig Id in the scene
   * @param viewIds The sub-pose view ids in the scene
   */
  void addSfMCameraRig(const SfMData& sfmData,
                    IndexT rigId,
                    const std::vector<IndexT>& viewIds);

  /**
   * @brief Add a set of 3d points
   * @param[in] points The 3D points to add
   */
  void addLandmarks(const Landmarks& points, bool withVisibility = true);

  /**
   * @brief Add a camera
   * @param[in] name The camera identifier
   * @param[in] view The corresponding view
   * @param[in] pose The camera pose (nullptr if undefined)
   * @param[in] intrinsic The camera intrinsic (nullptr if undefined)
   * @param[in,out] parent The Alembic parent node
   */
  void addCamera(const std::string& name,
                 const View& view,
                 const geometry::Pose3* pose = nullptr,
                 const camera::IntrinsicBase* intrinsic = nullptr);

  /**
   * @brief Add a keyframe to the animated camera
   * @param[in] pose The camera pose
   * @param[in] cam The camera intrinsics parameters
   * @param[in] imagePath The localized image path
   * @param[in] id_view View id
   * @param[in] id_intrinsic Intrinsic id
   * @param[in] sensorWidth_mm Width of the sensor in millimeters
   */
  void addCameraKeyframe(const geometry::Pose3 &pose,
                         const camera::Pinhole *cam,
                         const std::string &imagePath,
                         const IndexT id_view,
                         const IndexT id_intrinsic,
                         const float sensorWidth_mm=36.0);

  /**
   * @brief Initiate an animated camera
   * @param[in] name The camera identifier
   */
  void initAnimatedCamera(const std::string &name);
  


  /**
   * @brief Register keyframe on the previous values
   * @param[in] imagePath The Path to the image
   */
  void jumpKeyframe(const std::string& imagePath = std::string());

private:
  struct DataImpl;
  std::unique_ptr<DataImpl> _dataImpl;
};

} // namespace sfm
} // namespace aliceVision
