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
       IndexT viewId = UndefinedIndexT,
       IndexT intrinsicId = UndefinedIndexT,
       IndexT poseId = UndefinedIndexT,
       std::size_t width = 0,
       std::size_t height = 0,
       IndexT rigId = UndefinedIndexT,
       IndexT subPoseId = UndefinedIndexT)
    : _imagePath(sImgPath)
    , _viewId(viewId)
    , _intrinsicId(intrinsicId)
    , _poseId(poseId)
    , _width(width)
    , _height(height)
    , _rigId(rigId)
    , _subPoseId(subPoseId)
  {}

  virtual ~View() {}

  bool operator==(const View& other) const {
    // Image paths can be different
    return _viewId == other._viewId &&
            _intrinsicId == other._intrinsicId &&
            _poseId == other._poseId &&
            _width == other._width &&
            _height == other._height &&
            _rigId == other._rigId &&
            _subPoseId == other._subPoseId;
  }


  /**
   * @brief Get view image path
   * @return image path
   */
  const std::string& getImagePath() const
  {
    return _imagePath;
  }

  /**
   * @brief Get view image width
   * @return width
   */
  const std::size_t getWidth() const
  {
    return _width;
  }

  /**
   * @brief Get view image height
   * @return height
   */
  const std::size_t getHeight() const
  {
    return _height;
  }

  /**
   * @brief Get the viewId
   * @return viewId
   */
  IndexT getViewId() const
  {
    return _viewId;
  }

  /**
   * @brief Get the intrinsicId
   * @return intrinsicId
   */
  IndexT getIntrinsicId() const
  {
    return _intrinsicId;
  }

  /**
   * @brief Get the poseId
   * @return poseId
   */
  IndexT getPoseId() const
  {
    return _poseId;
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
   * @brief Return if true or false the view is part of a rig
   * @return true if the view is part of a rig
   */
  bool isPartOfRig() const
  {
    return _rigId != UndefinedIndexT;
  }

  /**
   * @brief Set the viewId
   * @return viewId
   */
  void setViewId(IndexT viewId)
  {
    _viewId = viewId;
  }

  /**
   * @brief Set the intrinsicId
   * @return intrinsicId
   */
  void setIntrinsicId(IndexT intrinsicId)
  {
    _intrinsicId = intrinsicId;
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
    const std::string localPath = stlplus::folder_append_separator(stlplus::folder_part(_imagePath));
    const std::string filename = stlplus::filename_part(_imagePath);

    ar(cereal::make_nvp("local_path", localPath),
       cereal::make_nvp("filename", filename),
       cereal::make_nvp("width", _width),
       cereal::make_nvp("height", _height),
       cereal::make_nvp("id_view", _viewId),
       cereal::make_nvp("id_intrinsic", _intrinsicId),
       cereal::make_nvp("id_pose", _poseId),
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
       cereal::make_nvp("width", _width),
       cereal::make_nvp("height", _height),
       cereal::make_nvp("id_view", _viewId),
       cereal::make_nvp("id_intrinsic", _intrinsicId),
       cereal::make_nvp("id_pose", _poseId));

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

    _imagePath = stlplus::create_filespec(localPath, filename);
  }

private:

  /// image path on disk
  std::string _imagePath;
  /// image width
  std::size_t _width;
  /// image height
  std::size_t _height;
  /// id of the view
  IndexT _viewId;
  /// index of intrinsics
  IndexT _intrinsicId;
  /// either the pose of the rig or the pose of the camera if there's no rig
  IndexT _poseId;
  /// corresponding rig Id or undefined
  IndexT _rigId;
  /// corresponding subPose Id or undefined
  IndexT _subPoseId;
};

} // namespace sfm
} // namespace openMVG
