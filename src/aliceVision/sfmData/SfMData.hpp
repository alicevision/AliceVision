// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfmData/CameraPose.hpp>
#include <aliceVision/sfmData/Landmark.hpp>
#include <aliceVision/sfmData/Constraint2D.hpp>
#include <aliceVision/sfmData/RotationPrior.hpp>
#include <aliceVision/sfmData/View.hpp>
#include <aliceVision/sfmData/Rig.hpp>
#include <aliceVision/camera/camera.hpp>
#include <aliceVision/sfmData/HashMapPtr.hpp>
#include <aliceVision/types.hpp>

#include <stdexcept>
#include <cassert>
#include <random>

namespace aliceVision {
namespace sfmData {

/// Define a collection of View
using Views = HashMapPtr<View>;

/// Define a collection of Pose (indexed by view.getPoseId())
using Poses = HashMap<IndexT, CameraPose>;

/// Define a collection of IntrinsicParameter (indexed by view.getIntrinsicId())
using Intrinsics = HashMapPtr<camera::IntrinsicBase>;

/// Define a collection of landmarks are indexed by their TrackId
using Landmarks = HashMap<IndexT, Landmark>;

/// Define a collection of Rig
using Rigs = std::map<IndexT, Rig>;

/// Define uncertainty per pose
using PosesUncertainty = HashMap<IndexT, Vec6>;

/// Define uncertainty per landmark
using LandmarksUncertainty = HashMap<IndexT, Vec3>;

///Define a collection of constraints
using Constraints2D = std::vector<Constraint2D>;

///Define a collection of rotation priors
using RotationPriors = std::vector<RotationPrior>;

/**
 * @brief SfMData container
 * Store structure and camera properties
 */
class SfMData
{
public:
    /// Uncertainty per pose
    PosesUncertainty _posesUncertainty;
    /// Uncertainty per landmark
    LandmarksUncertainty _landmarksUncertainty;
    /// 2D Constraints
    Constraints2D constraints2d;
    /// Rotation priors
    RotationPriors rotationpriors;

    SfMData();
    ~SfMData();

    // Operators

    bool operator==(const SfMData& other) const;

    inline bool operator!=(const SfMData& other) const { return !(*this == other); }

    // Accessors

    /**
     * @brief Get views
     * @return views
     */
    const Views& getViews() const {return _views;}
    Views& getViews() {return _views;}

    /**
     * @brief Get poses
     * @return poses
    */
    const Poses& getPoses() const {return _poses;}
    Poses& getPoses() {return _poses;}

    /**
     * @brief Get rigs
     * @return rigs
     */
    const Rigs& getRigs() const {return _rigs;}
    Rigs& getRigs() {return _rigs;}

    /**
     * @brief Get intrinsics
     * @return intrinsics
     */
    const Intrinsics& getIntrinsics() const {return _intrinsics;}
    Intrinsics& getIntrinsics() {return _intrinsics;}

    /**
     * @brief Get landmarks
     * @return landmarks
     */
    const Landmarks& getLandmarks() const {return _structure;}
    Landmarks& getLandmarks() {return _structure;}

    /**
     * @brief Get Constraints2D
     * @return Constraints2D
     */
    const Constraints2D& getConstraints2D() const {return constraints2d;}
    Constraints2D& getConstraints2D() {return constraints2d;}

    /**
     * @brief Get RotationPriors
     * @return RotationPriors
     */
    const RotationPriors& getRotationPriors() const {return rotationpriors;}
    RotationPriors& getRotationPriors() {return rotationpriors;}

    /**
     * @brief Get relative features folder paths
     * @return features folders paths
     */
    const std::vector<std::string>& getRelativeFeaturesFolders() const
    {
        return _featuresFolders;
    }

    /**
     * @brief Get relative matches folder paths
     * @return matches folder paths
     */
    const std::vector<std::string>& getRelativeMatchesFolders() const
    {
        return _matchesFolders;
    }

    /**
     * @brief Get absolute features folder paths
     * @return features folders paths
     */
    std::vector<std::string> getFeaturesFolders() const;

    /**
     * @brief Get absolute matches folder paths
     * @return matches folder paths
     */
    std::vector<std::string> getMatchesFolders() const;

    /**
     * @brief List the view indexes that have valid camera intrinsic and pose.
     * @return view indexes list
     */
    std::set<IndexT> getValidViews() const;

    /**
     * @brief List the intrinsic indexes that have valid camera intrinsic and pose.
     * @return intrinsic indexes list
     */
    std::set<IndexT> getReconstructedIntrinsics() const;

    /**
     * @brief Return a pointer to an intrinsic if available or nullptr otherwise.
     * @param[in] intrinsicId
     */
    const camera::IntrinsicBase* getIntrinsicPtr(IndexT intrinsicId) const
    {
        if (_intrinsics.count(intrinsicId))
            return _intrinsics.at(intrinsicId).get();
        return nullptr;
    }

    /**
     * @brief Return a pointer to an intrinsic if available or nullptr otherwise.
     * @param[in] intrinsicId
     */
    camera::IntrinsicBase* getIntrinsicPtr(IndexT intrinsicId)
    {
        if(_intrinsics.count(intrinsicId))
            return _intrinsics.at(intrinsicId).get();
        return nullptr;
    }

    /**
     * @brief Return a shared pointer to an intrinsic if available or nullptr otherwise.
     * @param[in] intrinsicId
     */
    std::shared_ptr<camera::IntrinsicBase> getIntrinsicsharedPtr(IndexT intrinsicId)
    {
        if(_intrinsics.count(intrinsicId))
            return _intrinsics.at(intrinsicId);
        return nullptr;
    }

    /**
     * @brief Return a shared pointer to an intrinsic if available or nullptr otherwise.
     * @param[in] intrinsicId
     */
    const std::shared_ptr<camera::IntrinsicBase> getIntrinsicsharedPtr(IndexT intrinsicId) const
    {
        if(_intrinsics.count(intrinsicId))
            return _intrinsics.at(intrinsicId);
        return nullptr;
    }

    /**
     * @brief Get a set of views keys
     * @return set of views keys
     */
    std::set<IndexT> getViewsKeys() const
    {
        std::set<IndexT> viewKeys;
        for (auto v: _views)
            viewKeys.insert(v.first);
        return viewKeys;
    }

    /**
     * @brief Check if the given view have defined intrinsic and pose
     * @param[in] view The given view
     * @return true if intrinsic and pose defined
     */
    bool isPoseAndIntrinsicDefined(const View* view) const
    {
        if (view == nullptr)
            return false;
        return (
            view->getIntrinsicId() != UndefinedIndexT &&
            view->getPoseId() != UndefinedIndexT &&
            (!view->isPartOfRig() || view->isPoseIndependant() || getRigSubPose(*view).status != ERigSubPoseStatus::UNINITIALIZED) &&
            _intrinsics.find(view->getIntrinsicId()) != _intrinsics.end() &&
            _poses.find(view->getPoseId()) != _poses.end()
        );
    }

    /**
     * @brief Check if the given view have defined intrinsic and pose
     * @param[in] viewID The given viewID
     * @return true if intrinsic and pose defined
     */
    bool isPoseAndIntrinsicDefined(IndexT viewId) const
    { 
        return isPoseAndIntrinsicDefined(_views.at(viewId).get());
    }

    /**
     * @brief Check if the given view has an existing pose
     * @param[in] view The given view
     * @return true if the pose exists
     */
    bool existsPose(const View& view) const
    {
        return (_poses.find(view.getPoseId()) != _poses.end());
    }

    /**
     * @brief Gives the view of the input view id.
     * @param[in] viewId The given view id
     * @return the corresponding view reference
     */
    View& getView(IndexT viewId)
    {
        return *(_views.at(viewId));
    }

    /**
     * @brief Gives the view of the input view id.
     * @param[in] viewId The given view id
     * @return the corresponding view ptr
     */
    View::ptr getViewPtr(IndexT viewId)
    {
        return _views.at(viewId).get();
    }

    /**
     * @brief Gives the view of the input view id.
     * @param[in] viewId The given view id
     * @return the corresponding view ptr
     */
    View::sptr getViewSharedPtr(IndexT viewId)
    {
        return _views.at(viewId);
    }

    /**
     * @brief Gives the view of the input view id.
     * @param[in] viewId The given view id
     * @return the corresponding view reference
     */
    const View& getView(IndexT viewId) const
    {
        return *(_views.at(viewId));
    }

    /**
     * @brief Gives the pose of the input view. If this view is part of a rig, it returns rigPose + rigSubPose.
     * @param[in] view The given view
     *
     * @warning: This function returns a CameraPose (a temporary object and not a reference),
     *           because in the RIG context, this pose is the composition of the rig pose and the sub-pose.
     */
    CameraPose getPose(const View& view) const
    {
        // check the view has valid pose / rig etc
        if (!view.isPartOfRig() || view.isPoseIndependant())
        {
            return _poses.at(view.getPoseId());
        }

        // get the pose of the rig
        CameraPose pose = getRigPose(view);

        // multiply rig pose by camera subpose
        pose.setTransform(getRigSubPose(view).pose * pose.getTransform());

        return pose;
    }

    /**
     * @brief  Gives the pose with the given pose id.
     * @param[in] poseId The given pose id
     */
    const CameraPose& getAbsolutePose(IndexT poseId) const
    {
        return _poses.at(poseId);
    }

    /**
     * @brief Get the rig of the given view
     * @param[in] view The given view
     * @return rig of the given view
     */
    const Rig& getRig(const View& view) const
    {
        assert(view.isPartOfRig());
        return _rigs.at(view.getRigId());
    }

    std::set<feature::EImageDescriberType> getLandmarkDescTypes() const
    {
        std::set<feature::EImageDescriberType> output;
        for (auto s : getLandmarks())
        {
            output.insert(s.second.descType);
        }
        return output;
    }

    std::map<feature::EImageDescriberType, int> getLandmarkDescTypesUsages() const
    {
        std::map<feature::EImageDescriberType, int> output;
        for (auto s : getLandmarks())
        {
            if (output.find(s.second.descType) == output.end())
            {
                output[s.second.descType] = 1;
            }
            else
            {
                ++output[s.second.descType];
            }
        }
        return output;
    }

    /**
     * @brief Get the median Camera Exposure Setting
     * @return
     */
    ExposureSetting getMedianCameraExposureSetting() const
    {
        std::vector<ExposureSetting> cameraExposureList;
        cameraExposureList.reserve(_views.size());

        for(const auto& view : _views)
        {
            const ExposureSetting ce = view.second->getImage().getCameraExposureSetting();
            if (ce.isPartiallyDefined())
            {
                auto find = std::find(std::begin(cameraExposureList), std::end(cameraExposureList), ce);
                if (find == std::end(cameraExposureList))
                    cameraExposureList.emplace_back(ce);
            }
        }

        std::nth_element(cameraExposureList.begin(), cameraExposureList.begin() + cameraExposureList.size()/2, cameraExposureList.end());
        const ExposureSetting& ceMedian = cameraExposureList[cameraExposureList.size()/2];

        return ceMedian;
    }

    /**
     * @brief Add the given \p folder to features folders.
     * @note If SfmData's absolutePath has been set, 
     *       an absolute path will be converted to a relative one.
     * @param[in] folder path to a folder containing features
     */
    inline void addFeaturesFolder(const std::string& folder)
    {
        addFeaturesFolders({folder});
    }

    /**
     * @brief Add the given \p folders to features folders.
     * @note If SfmData's absolutePath has been set, 
     *       absolute paths will be converted to relative ones.
     * @param[in] folders paths to folders containing features
     */
    void addFeaturesFolders(const std::vector<std::string>& folders);

    /**
     * @brief Add the given \p folder to matches folders.
     * @note If SfmData's absolutePath has been set, 
     *       an absolute path will be converted to a relative one.
     * @param[in] folder path to a folder containing matches
     */
    inline void addMatchesFolder(const std::string& folder)
    {
        addMatchesFolders({folder});
    }

    /**
     * @brief Add the given \p folders to matches folders.
     * @note If SfmData's absolutePath has been set, 
     *       absolute paths will be converted to relative ones.
     * @param[in] folders paths to folders containing matches
     */
    void addMatchesFolders(const std::vector<std::string>& folders);

    /**
     * @brief Replace the current features folders by the given ones.
     * @note If SfmData's absolutePath has been set, 
     *       absolute paths will be converted to relative ones.
     * @param[in] folders paths to folders containing features
     */
    inline void setFeaturesFolders(const std::vector<std::string>& folders)
    {
        _featuresFolders.clear();
        addFeaturesFolders(folders);
    }

    /**
     * @brief Replace the current matches folders by the given ones.
     * @note If SfmData's absolutePath has been set, 
     *       absolute paths will be converted to relative ones.
     * @param[in] folders paths to folders containing matches
     */
    inline void setMatchesFolders(const std::vector<std::string>& folders)
    {
        _matchesFolders.clear();
        addMatchesFolders(folders);
    }

    /**
     * @brief Set the SfMData file absolute path.
     * @note Internal relative features/matches folders will be remapped 
     *       to be relative to the new absolute \p path.
     * @param[in] path The absolute path to the SfMData file folder
     */
    void setAbsolutePath(const std::string& path);

    /**
     * @brief Set the given pose for the given view
     * if the view is part of a rig, this method update rig pose/sub-pose
     * @param[in] view The given view
     * @param[in] pose The given pose
     */
    void setPose(const View& view, const CameraPose& pose);


    /**
     * @brief Set the given pose for the given poseId
     * @param[in] poseId The given poseId
     * @param[in] pose The given pose
     */
    void setAbsolutePose(IndexT poseId, const CameraPose& pose)
    {
        _poses[poseId] = pose;
    }

    /**
     * @brief Erase yhe pose for the given poseId
     * @param[in] poseId The given poseId
     * @param[in] noThrow If false, throw exception if no pose found
     */
    void erasePose(IndexT poseId, bool noThrow = false)
    {
        auto it =_poses.find(poseId);
        if (it != _poses.end())
            _poses.erase(it);
        else if (!noThrow)
            throw std::out_of_range(std::string("Can't erase unfind pose ") + std::to_string(poseId));
    }

    /**
     * @brief Reset rigs sub-poses parameters
     */
    void resetRigs()
    {
        for (auto rigIt : _rigs)
            rigIt.second.reset();
    }

    /**
     * @brief Insert data from the given sfmData if possible.
     * note: This operation doesn't override existing data.
     * @param[in] sfmData A given SfMData
     */
    void combine(const SfMData& sfmData);

    void clear();

private:
    /// Structure (3D points with their 2D observations)
    Landmarks _structure;
    /// Considered camera intrinsics (indexed by view.getIntrinsicId())
    Intrinsics _intrinsics;
    /// Considered views
    Views _views;
    /// Absolute path to the SfMData file (should not be saved)
    std::string _absolutePath;
    /// Features folders path
    std::vector<std::string> _featuresFolders;
    /// Matches folders path
    std::vector<std::string> _matchesFolders;
    /// Considered poses (indexed by view.getPoseId())
    Poses _poses;
    /// Considered rigs
    Rigs _rigs;

    /**
     * @brief Get Rig pose of a given camera view
     * @param[in] view The given view
     * @return Rig pose of the given camera view
     */
    const CameraPose& getRigPose(const View& view) const
    {
        return _poses.at(view.getPoseId());
    }

    /**
     * @brief Get Rig subPose of a given camera view
     * @param[in] view The given view
     * @return Rig subPose of the given camera view
     */
    const RigSubPose& getRigSubPose(const View& view) const
    {
        assert(view.isPartOfRig());
        const Rig& rig = _rigs.at(view.getRigId());
        return rig.getSubPose(view.getSubPoseId());
    }

    /**
     * @brief Get Rig pose of a given camera view
     * @param[in] view The given view
     * @return Rig pose of the given camera view
     */
    CameraPose& getRigPose(const View& view)
    {
        return _poses.at(view.getPoseId());
    }

    /**
     * @brief Get Rig subPose of a given camera view
     * @param[in] view The given view
     * @return Rig subPose of the given camera view
     */
    RigSubPose& getRigSubPose(const View& view)
    {
        assert(view.isPartOfRig());
        Rig& rig = _rigs.at(view.getRigId());
        return rig.getSubPose(view.getSubPoseId());
    }
};

using LandmarkIdSet = std::vector<std::size_t>;
using LandmarksPerView = stl::flat_map<std::size_t, LandmarkIdSet>;

LandmarksPerView getLandmarksPerViews(const SfMData& sfmData);

} // namespace sfmData
} // namespace aliceVision
