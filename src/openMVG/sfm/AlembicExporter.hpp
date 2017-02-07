#ifdef HAVE_ALEMBIC

#pragma once

#include <openMVG/sfm/sfm_data.hpp>
#include <openMVG/sfm/sfm_data_io.hpp>
#include <openMVG/geometry/pose3.hpp>
#include <openMVG/cameras/Camera_Pinhole.hpp>
#include <openMVG/types.hpp>

#include <memory>
#include <string>

namespace openMVG {
namespace sfm {

class AlembicExporter
{
public:

  AlembicExporter(const std::string &filename);

  /**
   * @brief Add a set of 3D points from a SFM scene
   * @param points The 3D points to add
   */
  void addPoints(const sfm::Landmarks &points,
                 bool withVisibility=true);

  /**
   * @brief Add a single camera
   * 
   * @param[in] name An identifier for the camera
   * @param[in] pose The camera pose
   * @param[in] cam The camera intrinsics
   * @param[in] imagePath Path to the image
   * @param[in] id_view View id
   * @param[in] id_intrinsic Intrinsic id
   * @param[in] sensorWidth_mm Width of the sensor in millimeters
   * @param[in] id_pose Pose id
   */
  void appendCamera(const std::string &name, 
                    const geometry::Pose3 &pose, 
                    const cameras::Pinhole_Intrinsic *cam,
                    const std::string &imagePath,
                    const IndexT id_view,
                    const IndexT id_intrinsic,
                    const float sensorWidth_mm=36.0,
                    const IndexT id_pose=UndefinedIndexT);
  
  /**
   * @brief Initiate an animated camera
   * 
   * @param cameraName An identifier for the camera
   */
  void initAnimatedCamera(const std::string &cameraName);
  
  /**
   * @brief Add a keyframe to the animated camera
   * 
   * @param pose The camera pose
   * @param cam The camera intrinsics parameters
   * @param imagePath The localized image path
   * @param id_view View id
   * @param id_intrinsic Intrinsic id
   * @param sensorWidth_mm Width of the sensor in millimeters
   */
  void addCameraKeyframe(const geometry::Pose3 &pose,
                           const cameras::Pinhole_Intrinsic *cam,
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
   * @param sfmdata SfM_Data container
   * @param flags_part filter the elements to add
   */
  void add(const sfm::SfM_Data &sfmdata, sfm::ESfM_Data flags_part = sfm::ESfM_Data::ALL);

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
} // namespace openMVG

#endif // HAVE_ALEMBIC

