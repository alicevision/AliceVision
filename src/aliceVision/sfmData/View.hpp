// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/types.hpp>
#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/image/dcp.hpp>
#include <aliceVision/sensorDB/Datasheet.hpp>
#include <aliceVision/camera/IntrinsicInitMode.hpp>
#include <aliceVision/lensCorrectionProfile/lcp.hpp>

#include <aliceVision/sfmData/imageInfo.hpp>
#include <aliceVision/sfmData/exif.hpp>
#include <aliceVision/sfmData/exposureSetting.hpp>

#include <regex>
#include <string>
#include <utility>

namespace aliceVision {
namespace sfmData {

/**
 * @brief A view define an image by a string and unique indexes for
 * the view, the camera intrinsic, the pose and the subpose if the camera is part of a rig
 */
class View
{
  public:
    using sptr = std::shared_ptr<View>;
    using ptr = View*;

  public:
    /**
     * @brief View Constructor
     * @param[in] imagePath The image path on disk
     * @param[in] viewId The view id (use unique index)
     * @param[in] intrinsicId The intrinsic id
     * @param[in] poseId The pose id (or the rig pose id)
     * @param[in] width The image width
     * @param[in] height The image height
     * @param[in] rigId The rig id (or undefined)
     * @param[in] subPoseId The sub-pose id (or undefined)
     * @param[in] metadata The image metadata
     */
    View(const std::string& imagePath = "",
         IndexT viewId = UndefinedIndexT,
         IndexT intrinsicId = UndefinedIndexT,
         IndexT poseId = UndefinedIndexT,
         std::size_t width = 0,
         std::size_t height = 0,
         IndexT rigId = UndefinedIndexT,
         IndexT subPoseId = UndefinedIndexT,
         const std::map<std::string, std::string>& metadata = std::map<std::string, std::string>())
      : _viewId(viewId),
        _intrinsicId(intrinsicId),
        _poseId(poseId),
        _rigId(rigId),
        _subPoseId(subPoseId),
        _image(new ImageInfo(imagePath, width, height, metadata))
    {}

    /**
     * @brief Shallow View Copy
     * return a pointer to a new View
     */
    View* shallowClone() { return new View(*this); }

    /**
     * @brief Deep View Copy
     * return a pointer to a new View
     */
    View* clone()
    {
        View* v = new View(*this);
        v->_image = std::make_shared<ImageInfo>(*this->_image);

        for (size_t i = 0; i < v->getAncestorImages().size(); i++)
        {
            (v->getAncestorImages())[i] = std::make_shared<ImageInfo>(*((this->getAncestorImages())[i]));
        }

        return v;
    }

    bool operator==(const View& other) const
    {
        // image paths can be different
        return _viewId == other._viewId && _intrinsicId == other._intrinsicId && _poseId == other._poseId && _rigId == other._rigId &&
               _subPoseId == other._subPoseId;
    }

    inline bool operator!=(const View& other) const { return !(*this == other); }

    /**
     * @brief Get Image info object
     * @return an object
     */
    const ImageInfo& getImage() const { return *_image; }

    /**
     * @brief Get Image info object
     * @return an object
     */
    ImageInfo& getImage() { return *_image; }

    /**
     * @brief Get Image info pointer
     * @return a shared pointer
     */
    std::shared_ptr<ImageInfo> getImageInfo() { return _image; }

    /**
     * @brief Get the view id
     * @return view id
     */
    IndexT getViewId() const { return _viewId; }

    /**
     * @brief Get the intrinsic id
     * @return intrinsic id
     */
    IndexT getIntrinsicId() const { return _intrinsicId; }

    /**
     * @brief Get the pose id
     * @return pose id
     */
    IndexT getPoseId() const { return _poseId; }

    /**
     * @brief Get the rig id
     * @return rig id or undefined
     */
    IndexT getRigId() const { return _rigId; }

    /**
     * @brief Get the sub-pose id
     * @return sup-pose id or undefined
     */
    IndexT getSubPoseId() const { return _subPoseId; }

    /**
     * @brief Get the frame id
     * @return frame id
     */
    IndexT getFrameId() const { return _frameId; }

    /**
     * @brief Get the resection id
     * @return resection id
     */
    IndexT getResectionId() const { return _resectionId; }

    /**
     * @brief Return if true or false the view is part of a rig
     * @return true if the view is part of a rig
     */
    bool isPartOfRig() const { return _rigId != UndefinedIndexT; }

    /**
     * @brief If the view is part of a camera rig, the camera can be a sub-pose of the rig pose but can also be temporarily solved independently.
     * @return true if the view is not part of a rig.
     *         true if the view is part of a rig and the camera is solved separately.
     *         false if the view is part of a rig and the camera is solved as a sub-pose of the rig pose.
     */
    bool isPoseIndependant() const { return (!isPartOfRig() || _isPoseIndependent); }

    /**
     * @brief Set the given view id
     * @param[in] viewId The given view id
     */
    void setViewId(IndexT viewId) { _viewId = viewId; }

    /**
     * @brief Set the given intrinsic id
     * @param[in] intrinsicId The given intrinsic id
     */
    void setIntrinsicId(IndexT intrinsicId) { _intrinsicId = intrinsicId; }

    /**
     * @brief Set the given pose id
     * @param[in] poseId The given pose id
     */
    void setPoseId(IndexT poseId) { _poseId = poseId; }

    /**
     * @brief setIndependantPose
     * @param independant
     */
    void setIndependantPose(bool independent) { _isPoseIndependent = independent; }

    /**
     * @brief Set the given rig id and the given sub-pose id
     * @param[in] rigId The given rig id
     * @param[in] subPoseId The given sub-pose id
     */
    void setRigAndSubPoseId(IndexT rigId, IndexT subPoseId)
    {
        _rigId = rigId;
        _subPoseId = subPoseId;
    }

    /**
     * @brief Set the given frame id
     * @param[in] frame The given frame id
     */
    void setFrameId(IndexT frameId) { _frameId = frameId; }

    /**
     * @brief Get the list of viewID referencing the source views called "Ancestors"
     * If an image is generated from multiple input images, "Ancestors" allows to keep track of the viewIDs of the original inputs views.
     * For instance, the generated view can come from the fusion of multiple LDR images into one HDR image, the fusion from multi-focus
     * stacking to get a fully focused image, fusion of images with multiple lighting to get a more diffuse lighting, etc.
     * @return list of viewID of the ancestors
     * @param[in] viewId the view ancestor id
     */
    void addAncestor(IndexT viewId) { _ancestors.push_back(viewId); }

    /**
     * @Brief get all ancestors for this view
     * @return ancestors
     */
    const std::vector<IndexT>& getAncestors() const { return _ancestors; }

    /**
     * @brief Add an ancestor images to the existing ones.
     * If an image is generated from multiple input images, "Ancestor Images" allows to keep track of all features
     * of the original images. For instance, the generated view can come from the fusion of multiple LDR images into
     * one HDR image, the fusion from multi-focus stacking to get a fully focused image, fusion of images with multiple
     * lighting to get a more diffuse lighting, etc.
     * @param[in] new ancestor image
     */
    void addAncestorImage(std::shared_ptr<ImageInfo> image)
    {
        if (std::find(_ancestorImages.begin(), _ancestorImages.end(), image) == _ancestorImages.end())
        {
            _ancestorImages.push_back(image);
        }
    }

    /**
     * @Brief get all ancestor images for this view
     * @return ancestor images
     */
    const std::vector<std::shared_ptr<ImageInfo>>& getAncestorImages() const { return _ancestorImages; }

    /**
     * @Brief get all ancestor images for this view
     * @return ancestor images
     */
    std::vector<std::shared_ptr<ImageInfo>>& getAncestorImages() { return _ancestorImages; }

    /**
     * @brief Set the given resection id
     * @param[in] resectionId The given resection id
     */
    void setResectionId(IndexT resectionId) { _resectionId = resectionId; }

  private:
    View(const View& v) = default;

  private:
    /// view id
    IndexT _viewId;
    /// intrinsics id
    IndexT _intrinsicId;
    /// either the pose of the rig or the pose of the camera if there's no rig
    IndexT _poseId;
    /// corresponding rig id or undefined
    IndexT _rigId;
    /// corresponding sub-pose id or undefined
    IndexT _subPoseId;
    /// corresponding frame id for synchronized views
    IndexT _frameId = UndefinedIndexT;
    /// resection id
    IndexT _resectionId = UndefinedIndexT;
    /// pose independent of other view(s)
    bool _isPoseIndependent = true;
    /// list of ancestors
    std::vector<IndexT> _ancestors;
    /// Link  to imageinfo
    std::shared_ptr<ImageInfo> _image;
    /// Link to ancestor images info
    std::vector<std::shared_ptr<ImageInfo>> _ancestorImages;
};

}  // namespace sfmData
}  // namespace aliceVision
