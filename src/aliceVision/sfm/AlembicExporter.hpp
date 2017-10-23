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

#include <Alembic/Abc/OObject.h>

#include <memory>
#include <string>

namespace aliceVision {
namespace sfm {

class AlembicExporter
{
public:

  AlembicExporter(const std::string &filename);

  /**
   * @brief Add a set of 3D points from a SFM scene
   * @param[in] points The 3D points to add
   */
  void addPoints(const sfm::Landmarks &points,
                 bool withVisibility=true);


  /**
   * @brief Add a camera rig
   *
   * @param[in] rigId The rig Id
   * @param[in] rigPoseId The rig pose Id
   * @param[in] views rig views
   * @param[in] viewsImagePaths The Paths to the images
   * @param[in] intrinsics The cameras intrinsics
   * @param[in] rigPose The rig pose
   * @param[in] subPoses The sub-poses
   */
  void appendCameraRig(IndexT rigId,
                       IndexT rigPoseId,
                       const std::vector<View>& views,
                       const std::vector<std::string>& viewsImagePaths,
                       const std::vector<camera::Pinhole*>& intrinsics,
                       const geometry::Pose3& rigPose,
                       const std::vector<RigSubPose>& subPoses);


  /**
   * @brief Add a single camera
   *
   * @param[in] cameraName The camera name
   * @param[in] view The corresponding view
   * @param[in] intrinsic The camera intrinsic
   * @param[in] pose The camera pose
   * @param[in] viewImagePath The Path to the image
   */
  void appendCamera(const std::string& cameraName,
                    const View& view,
                    const std::string& viewImagePath,
                    const camera::Pinhole* intrinsic,
                    const geometry::Pose3& pose);
  /**
   * @brief Add a single camera
   *
   * @param[in] cameraName The camera name
   * @param[in] view The corresponding view
   * @param[in] intrinsic The camera intrinsics
   * @param[in] pose The camera pose
   * @param[in] viewImagePath The Path to the image
   * @param[in,out] parent The Alembic parent node
   */
  void appendCamera(const std::string& cameraName,
                    const View& view,
                    const std::string& viewImagePath,
                    const camera::Pinhole* intrinsic,
                    const geometry::Pose3& pose,
                    Alembic::Abc::OObject& parent);

  /**
   * @brief Initiate an animated camera
   * 
   * @param[in] cameraName An identifier for the camera
   */
  void initAnimatedCamera(const std::string &cameraName);
  
  /**
   * @brief Add a keyframe to the animated camera
   * 
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
   * @brief Register keyframe on the previous values
   */
  void jumpKeyframe(const std::string &imagePath = std::string());
  
  /**
   * @brief Add SfM Data
   * 
   * @param[in] sfmdata SfMData container
   * @param[in] flags_part filter the elements to add
   */
  void add(const sfm::SfMData &sfmdata, sfm::ESfMData flags_part = sfm::ESfMData::ALL);

  /**
   * @brief Return the filename associated to the alembic file.
   * @return the filename associated to the alembivc file.
   */
  std::string getFilename();

  virtual ~AlembicExporter();

private:
  
  struct DataImpl;
  std::unique_ptr<DataImpl> _data;

};

} // namespace sfm
} // namespace aliceVision

