#pragma once

#include <openMVG/types.hpp>

#include <third_party/stlplus3/filesystemSimplified/file_system.hpp>

#include <cereal/cereal.hpp>

#include <string>
#include <utility>

namespace openMVG {
namespace sfm {

/**
 * @brief A view define an image by a string and unique indexes for
 * the view, the camera intrinsic, the pose and the subpose if the camera is part of a rig
 */
class View
{
public:

  /// Constructor (use unique index for the view_id)
  View(const std::string & sImgPath = "",
       IndexT view_id = UndefinedIndexT,
       IndexT intrinsic_id = UndefinedIndexT,
       IndexT pose_id = UndefinedIndexT,
       IndexT width = UndefinedIndexT,
       IndexT height = UndefinedIndexT)
    : s_Img_path(sImgPath)
    , id_view(view_id)
    , id_intrinsic(intrinsic_id)
    , id_pose(pose_id)
    , ui_width(width)
    , ui_height(height)
  {}

  virtual ~View() {}

  bool operator==(const View& other) const {
    // Image paths can be different
    return id_view == other.id_view &&
            id_intrinsic == other.id_intrinsic &&
            id_pose == other.id_pose &&
            ui_width == other.ui_width &&
            ui_height == other.ui_height &&
            _rigId == other._rigId &&
            _subPoseId == other._subPoseId;
  }

  /**
   * @brief Return if true or false the view is part of a rig
   * @return true if the view is part of a rig
   */
  bool isPartOfRig() const
  {
    return _rigId != UndefinedIndexT;
  }

  /**
   * @brief Get the Rig Id
   * @return Rig Id or UndefinedIndexT
   */
  IndexT getRigId() const
  {
    return _rigId;
  }

  /**
   * @brief Get the SubPose Id
   * @return SubPose Id or UndefinedIndexT
   */
  IndexT getSubPoseId() const
  {
    return _subPoseId;
  }

  /**
   * @brief Set rig Id and subPose Id
   * @param rigId
   * @param subPoseId
   */
  void setRigSubPose(IndexT rigId, IndexT subPoseId)
  {
    _rigId = rigId;
    _subPoseId = subPoseId;
  }

  /**
   * @brief cereal save method
   * @param ar The archive
   */
  template<class Archive>
  void save(Archive& ar) const
  {
    //Define a view with two string (base_path & basename)
    const std::string localPath = stlplus::folder_append_separator(stlplus::folder_part(s_Img_path));
    const std::string filename = stlplus::filename_part(s_Img_path);

    ar(cereal::make_nvp("local_path", localPath),
       cereal::make_nvp("filename", filename),
       cereal::make_nvp("width", ui_width),
       cereal::make_nvp("height", ui_height),
       cereal::make_nvp("id_view", id_view),
       cereal::make_nvp("id_intrinsic", id_intrinsic),
       cereal::make_nvp("id_pose", id_pose),
       cereal::make_nvp("id_rig", _rigId),
       cereal::make_nvp("id_subpose", _subPoseId));
  }

  /**
   * @brief cereal load method
   * @param ar The archive
   */
  template<class Archive>
  void load(Archive& ar)
  {
    std::string localPath;
    std::string filename;

    ar(cereal::make_nvp("local_path", localPath),
       cereal::make_nvp("filename", filename),
       cereal::make_nvp("width", ui_width),
       cereal::make_nvp("height", ui_height),
       cereal::make_nvp("id_view", id_view),
       cereal::make_nvp("id_intrinsic", id_intrinsic),
       cereal::make_nvp("id_pose", id_pose));

    // try to load from file id_rig_subpose
    try
    {
      ar(cereal::make_nvp("id_rig", _rigId),
         cereal::make_nvp("id_subpose", _subPoseId));
    }
    catch(cereal::Exception const &)
    {
      _rigId = UndefinedIndexT;
      _subPoseId = UndefinedIndexT;
    }

    s_Img_path = stlplus::create_filespec(localPath, filename);
  }

  /// image path on disk
  std::string s_Img_path;
  /// id of the view
  IndexT id_view;
  /// index of intrinsics
  IndexT id_intrinsic;
  /// either the pose of the rig or the pose of the camera if there's no rig
  IndexT id_pose;
  /// image size
  IndexT ui_width, ui_height;

private:

  /// corresponding rig Id or undefined
  IndexT _rigId = UndefinedIndexT;
  /// corresponding subPose Id or undefined
  IndexT _subPoseId = UndefinedIndexT;
};

} // namespace sfm
} // namespace openMVG
