#pragma once

#include <openMVG/types.hpp>
#include <openMVG/sfm/sfm_view.hpp>
#include <openMVG/sfm/sfm_view_metadata.hpp>
#include <openMVG/sfm/Rig.hpp>
#include <openMVG/sfm/sfm_landmark.hpp>
#include <openMVG/geometry/pose3.hpp>
#include <openMVG/cameras/cameras.hpp>

namespace openMVG {
namespace sfm {

/// Define a collection of View
using Views = Hash_Map<IndexT, std::shared_ptr<View> >;

/// Define a collection of Pose (indexed by View::id_pose)
using Poses = Hash_Map<IndexT, geometry::Pose3>;

/// Define a collection of IntrinsicParameter (indexed by View::id_intrinsic)
using Intrinsics = Hash_Map<IndexT, std::shared_ptr<cameras::IntrinsicBase> >;

/// Define a collection of landmarks are indexed by their TrackId
using Landmarks = Hash_Map<IndexT, Landmark>;

/// Define a collection of Rig
using Rigs = std::map<IndexT, Rig>;

/// Generic SfM data container
/// Store structure and camera properties:
struct SfM_Data
{
  /// Considered views
  Views views;
  /// Considered camera intrinsics (indexed by view.id_intrinsic)
  Intrinsics intrinsics;
  /// Considered poses (indexed by view.id_pose)
  Poses poses;
  /// Considered rigs
  Rigs rigs;
  /// Structure (3D points with their 2D observations)
  Landmarks structure;
  /// Controls points (stored as Landmarks (id_feat has no meaning here))
  Landmarks control_points;

  /// Root Views path
  std::string s_root_path;
  std::string _featureFolder;
  std::string _matchingFolder;

  bool operator==(const SfM_Data& other) const;

  //--
  // Accessors
  //--
  const Views& GetViews() const {return views;}
  const Poses& GetPoses() const {return poses;}
  const Intrinsics& GetIntrinsics() const {return intrinsics;}
  const Landmarks& GetLandmarks() const {return structure;}
  const Landmarks& GetControl_Points() const {return control_points;}
  const std::string& getFeatureFolder() const {return _featureFolder;}
  const std::string& getMatchingFolder() const {return _matchingFolder;}

  /**
   * @brief Return a pointer to an intrinsic if available or nullptr otherwise.
   * @param intrinsicId
   */
  const cameras::IntrinsicBase * GetIntrinsicPtr(IndexT intrinsicId) const
  {
    if(intrinsics.count(intrinsicId))
      return intrinsics.at(intrinsicId).get();
    return nullptr;
  }

  /**
   * @brief Return a pointer to an intrinsic if available or nullptr otherwise.
   * @param intrinsicId
   */
  cameras::IntrinsicBase * GetIntrinsicPtr(IndexT intrinsicId)
  {
    if(intrinsics.count(intrinsicId))
      return intrinsics.at(intrinsicId).get();
    return nullptr;
  }

  /**
   * @brief Return a shared pointer to an intrinsic if available or nullptr otherwise.
   * @param intrinsicId
   */
  std::shared_ptr<cameras::IntrinsicBase> GetIntrinsicSharedPtr(IndexT intrinsicId)
  {
    if(intrinsics.count(intrinsicId))
      return intrinsics.at(intrinsicId);
    return nullptr;
  }

  std::set<IndexT> GetViewsKeys() const
  {
    std::set<IndexT> viewKeys;
    for(auto v: views)
        viewKeys.insert(v.first);
    return viewKeys;
  }

  /// Check if the View have defined intrinsic and pose
  bool IsPoseAndIntrinsicDefined(const View * view) const
  {
    if (view == nullptr) return false;
    return (
      view->id_intrinsic != UndefinedIndexT &&
      view->id_pose != UndefinedIndexT &&
      intrinsics.find(view->id_intrinsic) != intrinsics.end() &&
      poses.find(view->id_pose) != poses.end());
  }
  
  bool IsPoseAndIntrinsicDefined(IndexT viewID) const
  { 
    return IsPoseAndIntrinsicDefined(views.at(viewID).get());
  }

  /**
   * @brief Gives the pose of the input view. If this view is part of a RIG, it returns rigPose + rigSubPose.
   * @param[in] view The given view
   */
  const geometry::Pose3 getPose(const View& view) const
  {
    // check the view has valid pose / rig etc
    if(!view.isPartOfRig())
    {
      return poses.at(view.id_pose);
    }

    // get the pose of the rig and the subpose of the camera
    const geometry::Pose3& rigPose = getRigPose(view);
    const geometry::Pose3& subPose = getRigSubPose(view).pose;

    // multiply rig pose by camera subpose
    return rigPose * subPose;   //TODO: check the order
  }

  /// Get the pose associated to a view
  const geometry::Pose3 GetPoseOrDie(const View * view) const
  {
    return poses.at(view->id_pose);
  }

  void setFeatureFolder(const std::string& featureFolder)
  {
    _featureFolder = featureFolder;
  }
  void setMatchingFolder(const std::string& matchingFolder)
  {
    _matchingFolder = matchingFolder;
  }

private:

  /**
   * @brief Get Rig pose of a given camera view
   * @param view The given view
   * @return Rig pose of the given camera view
   */
  const geometry::Pose3& getRigPose(const View& view) const
  {
    return poses.at(view.id_pose);
  }

  /**
   * @brief Get Rig subPose of a given camera view
   * @param view The given view
   * @return Rig subPose of the given camera view
   */
  const RigSubPose& getRigSubPose(const View& view) const
  {
    const Rig& rig = rigs.at(view.getRigId());
    return rig.getSubPose(view.getSubPoseId());
  }
};

/**
 * @brief ColorizeTracks Add the associated color to each 3D point of
 * the sfm_data, using the track to determine the best view from which
 * to get the color.
 * @param sfm_data The container of the data
 * @return true if everything went well
 */
bool ColorizeTracks( SfM_Data & sfm_data );

} // namespace sfm
} // namespace openMVG
