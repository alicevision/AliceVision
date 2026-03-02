// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "SfMData.hpp"

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/utils/filesIO.hpp>

#include <filesystem>

namespace aliceVision {
namespace sfmData {

using namespace aliceVision::geometry;
using namespace aliceVision::camera;
using namespace aliceVision::image;

namespace fs = std::filesystem;

SfMData::SfMData(const SfMData & other, bool unused)
{
    //First copy all the non pointers
    _landmarks = other._landmarks;
    _constraints2d = other._constraints2d;
    _constraintsPoint = other._constraintsPoint;
    _rotationpriors = other._rotationpriors;
    _absolutePath = other._absolutePath;
    _featuresFolders = other._featuresFolders;
    _matchesFolders = other._matchesFolders;
    _poses = other._poses;
    _rigs = other._rigs;
    _posesUncertainty = other._posesUncertainty;
    _landmarksUncertainty = other._landmarksUncertainty;
    _views = other._views;
    _intrinsics = other._intrinsics;
}

SfMData::SfMData(const SfMData & other, const Eigen::Vector3d & bbMin, const Eigen::Vector3d & bbMax)
{
    //First copy all the non pointers
    _constraints2d = other._constraints2d;
    _constraintsPoint = other._constraintsPoint;
    _rotationpriors = other._rotationpriors;
    _absolutePath = other._absolutePath;
    _featuresFolders = other._featuresFolders;
    _matchesFolders = other._matchesFolders;
    _poses = other._poses;
    _rigs = other._rigs;
    _posesUncertainty = other._posesUncertainty;
    _landmarksUncertainty = other._landmarksUncertainty;
    _views = other._views;
    _intrinsics = other._intrinsics;

    for (const auto & pl : other._landmarks)
    {
        const auto & pt = pl.second.getX();

        if (pt.x() < bbMin.x()) continue;
        if (pt.y() < bbMin.y()) continue;
        if (pt.z() < bbMin.z()) continue;
        if (pt.x() > bbMax.x()) continue;
        if (pt.y() > bbMax.y()) continue;
        if (pt.z() > bbMax.z()) continue;

        _landmarks.insert(pl);
    }
}

bool SfMData::operator==(const SfMData& other) const
{
    // Views
    if (_views != other._views)
    {
        return false;
    }

    // Ancestors
    if (_ancestors != other._ancestors)
    {
        return false;
    }

    // Poses
    if ((_poses != other._poses))
    {
        return false;
    }

    // Rigs
    if (_rigs != other._rigs)
    {
        return false;
    }

    // Intrinsics
    if (_intrinsics != other._intrinsics)
    {
        return false;
    }

    // Points IDs are not preserved
    if (_landmarks.size() != other._landmarks.size())
    {
        return false;
    }

    Landmarks::const_iterator landMarkIt = _landmarks.begin();
    Landmarks::const_iterator otherLandmarkIt = other._landmarks.begin();
    for (; landMarkIt != _landmarks.end() && otherLandmarkIt != other._landmarks.end(); ++landMarkIt, ++otherLandmarkIt)
    {
        // Points IDs are not preserved
        // Landmark
        const Landmark& landmark1 = landMarkIt->second;
        const Landmark& landmark2 = otherLandmarkIt->second;
        if (landmark1 != landmark2)
        {
            return false;
        }
    }

    if (_constraints2d.size() != other._constraints2d.size())
    {
        return false;
    }

    Constraints2D::const_iterator constraint2dIt = _constraints2d.begin();
    Constraints2D::const_iterator otherconstraint2dIt = other._constraints2d.begin();
    for (; constraint2dIt != _constraints2d.end() && otherconstraint2dIt != other._constraints2d.end(); ++constraint2dIt, ++otherconstraint2dIt)
    {
        if (*constraint2dIt != *otherconstraint2dIt)
        {
            return false;
        }
    }

    if (_constraintsPoint.size() != other._constraintsPoint.size())
    {
        return false;
    }

    ConstraintsPoint::const_iterator constraintPointIt = _constraintsPoint.begin();
    ConstraintsPoint::const_iterator otherconstraintPointIt = other._constraintsPoint.begin();
    for (; constraintPointIt != _constraintsPoint.end() && otherconstraintPointIt != other._constraintsPoint.end(); ++constraintPointIt, ++otherconstraintPointIt)
    {
        if (*constraintPointIt != *otherconstraintPointIt)
        {
            return false;
        }
    }

    // Root path can be reset during exports
    return true;
}

/**
 * @brief Convert paths in \p folders to absolute paths using \p absolutePath parent folder as base.
 * @param[in] folders list of paths to convert
 * @param[in] absolutePath filepath which parent folder should be used as base for absolute path conversion
 * @return the list of converted absolute paths or input folder if absolutePath is empty
 */
std::vector<std::string> toAbsoluteFolders(const std::vector<std::string>& folders, const std::string& absolutePath)
{
    // If absolute path is not set, return input folders
    if (absolutePath.empty())
        return folders;
    // project folder from project filepath
    const fs::path projectFolder = fs::path(absolutePath).parent_path();
    // Else, convert relative paths to absolute paths
    std::vector<std::string> absolutePaths;
    absolutePaths.reserve(folders.size());
    for (const auto& folder : folders)
    {
        fs::path f(folder);
        if(f.is_relative())
        {
            // convert to absolute path
            f = projectFolder / folder;
        }
        if (fs::exists(f))
        {
            // simplify the path to avoid things like "../.."
            // fs::canonical can only be used if the path exists
            f = fs::canonical(f);
        }
        absolutePaths.push_back(f.string());
    }
    return absolutePaths;
}

/**
 * @brief Add paths contained in \p folders to \p dst as relative paths to \p absolutePath.
 *        Paths already present in \p dst are omitted.
 * @param[in] dst list in which paths should be added
 * @param[in] folders paths to add to \p dst as relative folders
 * @param[in] absolutePath filepath which parent folder should be used as base for relative path conversions
 */
void addAsRelativeFolders(std::vector<std::string>& dst, const std::vector<std::string>& folders, const std::string& absolutePath)
{
    for (auto folderPath : folders)
    {
        // If absolutePath is set, convert to relative path
        if (!absolutePath.empty() && fs::path(folderPath).is_absolute())
        {
            folderPath = fs::relative(folderPath, fs::path(absolutePath).parent_path()).string();
        }
        // Add path only if not already in dst
        if (std::find(dst.begin(), dst.end(), folderPath) == dst.end())
        {
            dst.emplace_back(folderPath);
        }
    }
}

std::vector<std::string> SfMData::getFeaturesFolders() const { return toAbsoluteFolders(_featuresFolders, _absolutePath); }

std::vector<std::string> SfMData::getMatchesFolders() const { return toAbsoluteFolders(_matchesFolders, _absolutePath); }

void SfMData::addFeaturesFolders(const std::vector<std::string>& folders) { addAsRelativeFolders(_featuresFolders, folders, _absolutePath); }

void SfMData::addMatchesFolders(const std::vector<std::string>& folders) { addAsRelativeFolders(_matchesFolders, folders, _absolutePath); }

void SfMData::setAbsolutePath(const std::string& path)
{
    // Get absolute path to features/matches folders
    const std::vector<std::string> featuresFolders = getFeaturesFolders();
    const std::vector<std::string> matchesFolders = getMatchesFolders();
    // Change internal absolute path
    _absolutePath = path;
    // Re-set features/matches folders
    // They will be converted back to relative paths based on updated _absolutePath
    setFeaturesFolders(featuresFolders);
    setMatchesFolders(matchesFolders);
}

std::set<IndexT> SfMData::getValidViews() const
{
    std::set<IndexT> valid_idx;
    for (Views::const_iterator it = _views.begin(); it != _views.end(); ++it)
    {
        const View* v = it->second.get();
        if (isPoseAndIntrinsicDefined(v))
        {
            valid_idx.insert(v->getViewId());
        }
    }
    return valid_idx;
}

std::set<IndexT> SfMData::getReconstructedIntrinsics() const
{
    std::set<IndexT> valid_idx;
    for (Views::const_iterator it = _views.begin(); it != _views.end(); ++it)
    {
        const View* v = it->second.get();
        if (isPoseAndIntrinsicDefined(v))
        {
            valid_idx.insert(v->getIntrinsicId());
        }
    }
    return valid_idx;
}

void SfMData::setPose(const View& view, const CameraPose& absolutePose)
{

    // Pose dedicated for this view (independent from rig, even if it is potentially part of a rig)
    if (view.isPoseIndependant())
    {
        _poses.assign(view.getPoseId(), absolutePose);
        return;
    }

    // Initialized rig
    if (view.getRigId() != UndefinedIndexT)
    {
        const Rig& rig = _rigs.at(view.getRigId());
        RigSubPose& subPose = getRigSubPose(view);

        CameraPose viewPose;
        viewPose.setTransform(subPose.pose.inverse() * absolutePose.getTransform());

        if (absolutePose.isLocked())
        {
            viewPose.lock();
        }

        viewPose.setState(absolutePose.getState());

        _poses.assign(view.getPoseId(), viewPose);

        return;
    }

    throw std::runtime_error("SfMData::setPose: dependent view pose not part of an initialized rig.");
}

void SfMData::combine(const SfMData& sfmData)
{
    if (!_rigs.empty() && !sfmData._rigs.empty())
        throw std::runtime_error("Can't combine two SfMData with rigs");

    // feature folder
    addFeaturesFolders(sfmData.getFeaturesFolders());

    // matching folder
    addMatchesFolders(sfmData.getMatchesFolders());

    // views
    _views.insert(sfmData._views.begin(), sfmData._views.end());

    // intrinsics
    _intrinsics.insert(sfmData._intrinsics.begin(), sfmData._intrinsics.end());

    // poses
    _poses.insert(sfmData._poses.begin(), sfmData._poses.end());

    // rigs
    _rigs.insert(sfmData._rigs.begin(), sfmData._rigs.end());

    // structure
    _landmarks.insert(sfmData._landmarks.begin(), sfmData._landmarks.end());

    // constraints
    _constraints2d.insert(_constraints2d.end(), sfmData._constraints2d.begin(), sfmData._constraints2d.end());

    // constraints
    _constraintsPoint.insert(sfmData._constraintsPoint.begin(), sfmData._constraintsPoint.end());
}

void SfMData::clear()
{
    _views.clear();
    _intrinsics.clear();
    _landmarks.clear();
    _posesUncertainty.clear();
    _landmarksUncertainty.clear();
    _constraints2d.clear();
    _constraintsPoint.clear();
    _rotationpriors.clear();
    _absolutePath.clear();
    _featuresFolders.clear();
    _matchesFolders.clear();
    _poses.clear();
    _rigs.clear();
}

void SfMData::resetParameterStates()
{

    for (auto & [_, pose]: _poses.valueRange())
    {
        pose.initializeState();
    }

    for (auto& pl : _landmarks)
    {
        pl.second.setState(EEstimatorParameterState::REFINED);
    }

    for (auto& pi : _intrinsics)
    {
        pi.second->initializeState();
    }
}

void SfMData::getBoundingBox(Eigen::Vector3d & bbMin, Eigen::Vector3d & bbMax)
{
    bbMin.fill(std::numeric_limits<double>::max());
    bbMax.fill(std::numeric_limits<double>::lowest());

    for (const auto & pl : _landmarks)
    {
        const auto & pt = pl.second.getX();

        bbMin.x() = std::min(bbMin.x(), pt.x());
        bbMin.y() = std::min(bbMin.y(), pt.y());
        bbMin.z() = std::min(bbMin.z(), pt.z());
        bbMax.x() = std::max(bbMax.x(), pt.x());
        bbMax.y() = std::max(bbMax.y(), pt.y());
        bbMax.z() = std::max(bbMax.z(), pt.z());
    }
}

IndexT SfMData::findView(const std::string & imageName) const
{
    IndexT out_viewId = UndefinedIndexT;

    // list views uid / filenames and find the one that correspond to the user ones
    for (const auto& viewPair : getViews())
    {
        const auto & v = viewPair.second;

        if (imageName == std::to_string(v->getViewId()) || 
            imageName == fs::path(v->getImage().getImagePath()).filename().string() ||
            imageName == v->getImage().getImagePath())
        {
            out_viewId = v->getViewId();
            break;
        }
    }

    return out_viewId;
}

void SfMData::removeUnusedIntrinsics()
{
    std::set<IndexT> usedIds;

    for (const auto [_, view] : getViews().valueRange())
    {
        usedIds.insert(view.getIntrinsicId());
    }

    std::erase_if(_intrinsics, [usedIds](const auto & item)
    {
        //If current intrinsicId not found in usedIds
        return (usedIds.find(item.first) == usedIds.end());
    });
}

void SfMData::removeUnusedCameraPoses()
{
    std::set<IndexT> usedIds;

    for (const auto [_, view] : getViews().valueRange())
    {
        usedIds.insert(view.getPoseId());
    }

    std::erase_if(_poses, [usedIds](const auto & item)
    {
        //If current poseId not found in usedIds
        return (usedIds.find(item.first) == usedIds.end());
    });
}

void SfMData::removeInvalidObservations()
{
    const std::set<IndexT> validViews = getValidViews();

    // Erase all observations whose key (viewId) is not in the valid set
    for (auto & [lid, landmark] : getLandmarks())
    {
        // observations is a flat_map, can't use erase_if
        auto & observations = landmark.getObservations();

        sfmData::Observations keptObservations;
        for (const auto [idView, observation] : observations)
        {
            if (validViews.find(idView) != validViews.end())
            {
                keptObservations.emplace(idView, observation);
            }
        }

        observations = std::move(keptObservations);
    }
}

void SfMData::removeUnusedLandmarks()
{
    // Erase all landmarks with no observations
    std::erase_if(getLandmarks(), [](const auto & pair) {
        return pair.second.getObservations().empty();
    });
}

void SfMData::repair()
{
    removeUnusedIntrinsics();
    removeUnusedCameraPoses();
    removeInvalidObservations();
    removeUnusedLandmarks();
}

bool SfMData::isFullyReconstructed() const
{
    for (const auto & [_, view] : getViews().valueRange())
    {
        if (!isPoseAndIntrinsicDefined(view))
        {
            return false;
        }
    }

    return true;
}

LandmarksPerView getLandmarksPerViews(const SfMData& sfmData)
{
    LandmarksPerView landmarksPerView;
    for (const auto& landIt : sfmData.getLandmarks())
    {
        for (const auto& obsIt : landIt.second.getObservations())
        {
            IndexT viewId = obsIt.first;
            LandmarkIdSet& landmarksSet = landmarksPerView[viewId];
            landmarksSet.push_back(landIt.first);
        }
    }

// Sort landmark Ids in each view
#pragma omp parallel for
    for (int i = 0; i < landmarksPerView.size(); ++i)
    {
        LandmarksPerView::iterator it = landmarksPerView.begin();
        std::advance(it, i);
        std::sort(it->second.begin(), it->second.end());
    }

    return landmarksPerView;
}

}  // namespace sfmData
}  // namespace aliceVision
