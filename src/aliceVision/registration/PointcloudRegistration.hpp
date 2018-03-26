#pragma once

#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>

namespace aliceVision {
namespace registration {
/**
 * @brief This class allowed to register a source (moving) cloud  to a target (fixed) cloud using an
 * approach based on the Generalized Iterative Closest Point algorithm.
 * To use it, you have to:
 * 1. Load source and target cloud.
 * 2. (optional) Choose a rescaling strategy (None, Manual, Auto) setting up directly a scale ratio 
 * (\c setScaleRatio()) or a measurement made on the source (\c setSourceMeasurments()) and the 
 * target pointclouds (\c setTargetMeasurments()).
 * 3. Run the \c align() function.
 * 4. Pick up the final transformation (\c getFinalTransformation()) and use it as you wish.
 */
class PointcloudRegistration
{
public:
  
  PointcloudRegistration();
  
  ~PointcloudRegistration(){;}
  
  enum ERescaleMode {None, Manual, Auto};
  
  /**
   * @brief Load the source (moving) cloud.
   * @param[in] file: The file name of the cloud. Accepted formats: .ply or .pcd.
   * @return EXIT_FAILURE if something wrong happens, else EXIT_SUCCESS.
   */
  int loadSourceCloud(const std::string & file);
  
  /**
   * @brief Load the target (fixed) cloud.
   * @param[in] file: The file name of the cloud. Accepted formats: .ply or .pcd.
   * @return EXIT_FAILURE if something wrong happens, else EXIT_SUCCESS.
   */
  int loadTargetCloud(const std::string & file);
  
  /**
   * @brief Save the \c cloud according to the \c file path. Accepted format: .ply.
   * @param[in] file: The file name of the cloud.
   * @param[in] cloud: The cloud to save.
   * @return EXIT_FAILURE if something wrong happens, else EXIT_SUCCESS.
   */
  static int saveCloud(const std::string & file, const pcl::PointCloud<pcl::PointXYZ> & cloud);
  
  /**
   * @brief To set a cloud as the source of the alignment (: moving cloud).
   * @param[in] cloud The cloud.
   */
  inline void setSourceCloud(const pcl::PointCloud<pcl::PointXYZ> & cloud) {sourceCloud = cloud;}
  
  /**
   * @brief To set a cloud as the target of the alignment (: fixed cloud).
   * @param[in] cloud The cloud.
   */
  inline void setTargetCloud(const pcl::PointCloud<pcl::PointXYZ> & cloud) {targetCloud = cloud;}
  
  /**
   * @brief To indicate a measurement made on the source point cloud.
   * The same measure must be done on the target model (whatever the unit).
   * It is used to evaluate manually the scale ratio between source and target clouds.
   * @param[in] measurement The distance on the source cloud.
   */
  void setSourceMeasurements(const float measurement);
  
  /**
   * @brief To indicate a measurement made on the target point cloud.
   * The same measure must be done on the source model (whatever the unit).
   * It is used to evaluate manually the scale ratio between source and target clouds.
   * @param[in] measurement The distance on the target cloud.
   */
  void setTargetMeasurements(const float measurement);
  
  /**
   * @brief To indicate directly the scale ratio between source and target clouds.
   * The scale ratio represent targetSize / sourceSize.
   * @param[in] ratio The scale ratio (= targetSize / sourceSize).
   * @return EXIT_FAILURE if val <= 0, else EXIT_SUCCESS.
   */
  inline void setScaleRatio(const float ratio);
  
  /**
   * @brief To indicate the whished size of the voxel grid apply on the clouds during processing.
   * The default value is 0.1. 
   * You can increase the value for gain speed, but a too hight may concluded to a bad registration.
   * @param[in] val The voxel grid size.
   */
  inline void setVoxelSize(const float val) {voxelSize = val;}
  
  /**
   * @brief Allowes you to choose your scaling modes: 
   * - None: no rescale is apply on the source cloud.
   * - Manually: the source cloud is rescaled according to the scale ratio
   * (let check: \c setSourceMeasurment() \c setTargetMeasurment() and \c setScaleRatio()) 
   * - Auto: *[TODO] not yet implemented*
   * @param[in] mode The rescale mode.
   */
  inline void setRescaleMode(const ERescaleMode mode) {rescaleMode = mode;}
  
  /**
   * @brief To modify the number of close points used to compute normals.
   * The default value is 10.
   * @param[in] k The number points.
   */
  inline void setKSeachNormals(const int k) {kSearchNormals = k;}
  
  /**
   * @brief To display or not the 3D visulisation of the different step of the registering.
   * Set to \c false by default.
   * @param[in] show A boolean.
   */
  inline void setShowPipeline(const bool show) {showPipeline = show;}
  
  /**
   * @brief Return the transformation matrix obtains at the end of the alignment, such as: 
   * T * sourceCloud = targetCloud
   * @return A 4x4 matrix ([R|t]).
   */
  inline Eigen::Matrix4f getFinalTransformation() const {return finalTransformation;}
  
  /**
   * @brief Perform the alignment of the source cloud on the target cloud and gives
   * you the registered source cloud.
   * The transformation matrix is available using: \c getFinalTransformation().
   * @param[out] registeredSourceCloud The result of the alignment applied on the source cloud.
   * @return EXIT_FAILURE if something wrong happens, else EXIT_SUCCESS.
   */
  int align(pcl::PointCloud<pcl::PointXYZ> & registeredSourceCloud);
  
  /**
   * @brief Perform the alignment of the source cloud on the target cloud and gives
   * you the registered source cloud.
   * The transformation matrix is available using: \c getFinalTransformation().
   * @return EXIT_FAILURE if something wrong happens, else EXIT_SUCCESS.
   */
  int align();
  
private:
  
  /**
   * @brief Move a pointcloud to the origin of the world coordinate.
   * @param[in,out] cloud The cloud to move.
   * @return The 4x4 transformation matrix associated to this translation.
   */
  static Eigen::Matrix4f moveToOrigin(pcl::PointCloud<pcl::PointXYZ> & cloud);
  
  /**
   * @brief Apply the PCL's 'Generalized Iterative Closest Point' algorithm refining the source cloud
   * according to the target cloud position.
   * @param source_cloud[in] The source cloud (moving).
   * @param target_cloud[in] The target cloud (fixed)
   * @param source_normals[in] The normals associated to the source cloud.
   * @param target_normals[in] The normals associated to the target cloud.
   * @param registered_source_cloud[out]
   * @return 
   */
  static Eigen::Matrix4f applyGeneralizedICP(const pcl::PointCloud<pcl::PointXYZ> & source_cloud,
                                             const pcl::PointCloud<pcl::PointXYZ> & target_cloud,
                                             const pcl::PointCloud<pcl::Normal> & source_normals,
                                             const pcl::PointCloud<pcl::Normal> & target_normals,
                                             pcl::PointCloud<pcl::PointXYZ> & registered_source_cloud);
  
  
  /**
   * @brief Estimate the resolution of a pointcloud.
   * Source: https://github.com/PointCloudLibrary/pcl/blob/master/doc/tutorials/content/sources/correspondence_grouping/correspondence_grouping.cpp
   * @param[in] cloud The cloud to evaluate.
   * @return The computed resolution.
   */
  static double computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud);
  
  /**
   * @brief Compute the normal for each point of the pointcloud, using its 'k' closest neighbours.
   * @param[in] cloud The cloud we want to know normals.
   * @param[out] normals The computed normals.
   * @param[out] ksearch The number of neighbours used to compute normals.
   */
  static void estimateNormals(const pcl::PointCloud<pcl::PointXYZ> & cloud,
                              pcl::PointCloud<pcl::Normal> & normals, 
                              int ksearch);
  
  /**
   * @brief To downsample a pointcloud using a voxelgrid approach. 
   * @param[in,out] cloud The downsampled pointcloud.
   * @param[in] voxelSize The size of the voxelgrid.
   */
  static void applyVoxelGrid(pcl::PointCloud<pcl::PointXYZ> & cloud, const float voxelSize);
  
  /**
   * @brief Return the 4x4 transformation matrix associated to a scale value.
   * T = [s 0 0 0]
   *     [0 s 0 0]
   *     [0 0 s 0]
   *     [0 0 0 1]
   * @param[in] scale The scale s.
   * @return The 4x4 transformation matrix T.
   */
  Eigen::Matrix4f getPureScaleTransformation(const float scale);
  
  /**
   * @brief [TODO] Compute the scale ratio between the source and the target clouds using SVD (1st eigen value).
   * @return The 4x4 transformation matrix T associated to the estimated scale.
   */
  Eigen::Matrix4f rescaleAuto() { return Eigen::Matrix4f(Eigen::Matrix4f::Identity()); } 
  
  /**
   * @brief Load a cloud from a file name.
   * @param[in] file The file name.
   * @param[out] cloud The loaded cloud.
   * @return EXIT_FAILURE if something wrong happens, else EXIT_SUCCESS.
   */
  int loadCloud(const std::string & file, pcl::PointCloud<pcl::PointXYZ> & cloud);
  
  /**
   * @brief Move the \c sourceCloud to the \c targetCloud position.
   * @return The 4x4 transformation matrix associated to the translation.
   */
  Eigen::Matrix4f moveSourceToTargetPosition();  
  
  /**
   * @brief 3D visualizer for 2 pointclouds. Source is green, target is red.
   * @param[in] windowName The name of the window.
   * @param[in] source The source (green) cloud.
   * @param[in] target The target (red) cloud.
   */
  void draw(
      const std::string & windowName,
      const pcl::PointCloud<pcl::PointXYZ> & source,
      const pcl::PointCloud<pcl::PointXYZ> & target); 
  
  /**
   * @brief 3D visualizer for 2 subsampled pointclouds. Source is green, target is red.
   * @param[in] windowName The name of the window.
   * @param[in] source The source (green) cloud.
   * @param[in] target The target (red) cloud.
   * @param[in] voxelSize The size of the applied voxel grid.
   */
  void draw(
      const std::string & windowName,
      const pcl::PointCloud<pcl::PointXYZ> & source,
      const pcl::PointCloud<pcl::PointXYZ> & target, 
      const float voxelSize);
  
  /**
   * @brief 3D visualizer for 2 sourceCloud  coulds. Source is green, target is red.
   * @param[in] windowName The name of the window.
   * @param[in] source The source (green) cloud.
   * @param[in] target The target (red) cloud.
   * @param[in] source The source normals.
   * @param[in] target The target normals.
   * @param[in] voxelSize The size of the applied voxel grid.
   */
  void draw(
      const std::string & windowName, 
      const pcl::PointCloud<pcl::PointXYZ> & source,
      const pcl::PointCloud<pcl::PointXYZ> & target,
      const pcl::PointCloud<pcl::Normal> & sourceNormals, 
      const pcl::PointCloud<pcl::Normal> & targetNormals);
  
  /**
   * @brief 3D visualizer for 2 pointclouds moved to the origin. Source is green, target is red.
   * @param[in] windowName The name of the window.
   * @param[in] source The source (green) cloud.
   * @param[in] target The target (red) cloud.
   */
  void drawCentered(
      const std::string & windowName,
      const pcl::PointCloud<pcl::PointXYZ> & source,
      const pcl::PointCloud<pcl::PointXYZ> & target);
  
  /**
   * @brief 3D visualizer for 2 subsampled pointclouds moved to the origin. Source is green, target is red.
   * @param[in] windowName The name of the window.
   * @param[in] source The source (green) cloud.
   * @param[in] target The target (red) cloud.
   * @param[in] voxelSize The size of the applied voxel grid.
   */
  void drawCentered(
      const std::string & windowName,
      const pcl::PointCloud<pcl::PointXYZ> & source,
      const pcl::PointCloud<pcl::PointXYZ> & target,
      const float voxelSize);
  
  /**
   * @brief To activate the visualizers.
   */
  void goDraw() const; 
  
  // -- Data member
  
  pcl::PointCloud<pcl::PointXYZ> sourceCloud; /**< The source cloud: mobile could.*/
  
  pcl::PointCloud<pcl::PointXYZ> targetCloud; /**< The target cloud: fixed cloud.*/
  
  ERescaleMode rescaleMode; /**< The mode used to resize the source cloud */
  
  float sourceMeasurements; /**< A distance in the source cloud. Must have the same unit as \c targetMeasurements. */
  
  float targetMeasurements; /**< A distance in the target cloud. Must have the same unit as \c sourceMeasurements. */
  
  float scaleRatio; /**< The scale ratio between the source and the target clouds. Is the \c targetMeasurements / \c sourceMeasurements ratio. */
  
  float voxelSize; /**< The size of the voxel grid applied on source and target clouds. */
  
  int kSearchNormals; /**< The number of closest neighbours used to compute normals. */
  
  bool showPipeline; /**< Vizualise (or not) each step of the alignement. */
  
  Eigen::Matrix4f finalTransformation; /**< Is the computed transformation such as: T * sourceCloud = targetCloud/ */
};

} // namespace registration
} // namespace aliceVision

// #endif // POINTCLOUD_ALIGNMENT_H_
