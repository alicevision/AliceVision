// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ExpansionProcess.hpp"

#include <aliceVision/system/Logger.hpp>

namespace aliceVision {
namespace sfm {

bool ExpansionProcess::process(sfmData::SfMData & sfmData, track::TracksHandler & tracksHandler)
{
    ALICEVISION_LOG_INFO("ExpansionProcess start");

    if (!_iterationHandler)
    {
        return false;
    }

    if (!_historyHandler)
    {
        return false;
    }

    if (!_historyHandler->initialize(sfmData))
    {
        return false;
    }

    //Prepare existing data
    if (!prepareExisting(sfmData, tracksHandler))
    {
        ALICEVISION_LOG_ERROR("Failed to prepare process");
        return false;
    }

    EExpansionMode expansionMode = EExpansionMode::EXPANSION_STRICT;

    int nbPoses = 0;
    
    while (true)
    {
        nbPoses = sfmData.getPoses().size();

        if (!_iterationHandler->process(sfmData, tracksHandler, expansionMode))
        {
            return false;
        }

        if (_postProcessHandler)
        {
            if (_postProcessHandler->process(sfmData, tracksHandler))
            {                
                if (_iterationHandler->getChunkHandler() == nullptr)
                {
                    return false;
                }

                // Perform a new estimation step on modified views
                if (!_iterationHandler->getChunkHandler()->process(sfmData, tracksHandler, _postProcessHandler->getUpdatedViews(), expansionMode))
                {
                    return false;
                }
            }
        }

        ALICEVISION_LOG_INFO("ExpansionProcess poses count : " << sfmData.getPoses().size());
        
        // Everything is already computed, exit
        if (sfmData.isFullyReconstructed())
        {
            break;
        }

        // Nothing changed in the last iteration
        if (sfmData.getPoses().size() == nbPoses)
        {
            //Before exiting, try again with less constraints
            if (expansionMode == EExpansionMode::EXPANSION_PERMISSIVE)
            {
                break;
            }

            expansionMode = EExpansionMode::EXPANSION_PERMISSIVE;
        }
    }
    

    ALICEVISION_LOG_INFO("ExpansionProcess end");

    _historyHandler->debrief(sfmData);

    return true;
}

bool ExpansionProcess::prepareExisting(sfmData::SfMData & sfmData, const track::TracksHandler & tracksHandler)
{
    ALICEVISION_LOG_INFO("ExpansionProcess prepareExisting");

    // Remove points without point fetcher
    std::erase_if(sfmData.getLandmarks(), [](const auto & item) {
        const auto & [key, value] = item;
        if (value.getReferenceViewIndex() != UndefinedIndexT)
        {
            if (value.getPointFetcher() == nullptr)
            {
                return true;
            }
        }

        return false;
    });

    //Prepare existing data
    remapExistingLandmarks(sfmData, tracksHandler);

    // If there are some poses existing
    // We want to make sure everything is on par with requirements
    if (!sfmData.getPoses().empty())
    {   
        if (_iterationHandler->getChunkHandler() == nullptr)
        {
            return false;
        }

        // Process everything in existing sfmData
        if (!_iterationHandler->getChunkHandler()->process(sfmData, tracksHandler, sfmData.getValidViews(), EExpansionMode::EXPANSION_STRICT))
        {
            return false;
        }
    }

    return true;
}

void ExpansionProcess::remapExistingLandmarks(sfmData::SfMData & sfmData, const track::TracksHandler & tracksHandler)
{
    ALICEVISION_LOG_INFO("remapExistingLandmarks");

    // get unmap landmarks
    sfmData::Landmarks landmarks;

    // clear sfmData structure and store them locally
    std::swap(landmarks, sfmData.getLandmarks());

    // builds landmarks temporary comparison structure
    // ObsKey <ViewId, FeatId, decType>
    // ObsToLandmark <ObsKey, LandmarkId>
    using ObsKey = std::tuple<IndexT, IndexT, feature::EImageDescriberType>;
    using ObsToLandmark = std::map<ObsKey, IndexT>;

    ObsToLandmark obsToLandmark;
    for (const auto& landmarkPair : landmarks)
    {
        const IndexT landmarkId = landmarkPair.first;
        if (landmarkPair.second.getObservations().size() == 0)
        {
            continue;
        }

        const IndexT firstViewId = landmarkPair.second.getObservations().begin()->first;
        const IndexT firstFeatureId = landmarkPair.second.getObservations().begin()->second.getFeatureId();
        const feature::EImageDescriberType descType = landmarkPair.second.getDescType();

        obsToLandmark.emplace(ObsKey(firstViewId, firstFeatureId, descType), landmarkId);
    }


    // For each track
    for (const auto & trackPair : tracksHandler.getAllTracks())
    {
        const IndexT trackId = trackPair.first;
        const track::Track& track = trackPair.second;

        //For each feature in track
        for (const auto& featView : track.featPerView)
        {
            ObsKey key(featView.first, featView.second.featureId, track.descType);

            //We assume one feature is associated to only one track
            const ObsToLandmark::const_iterator it = obsToLandmark.find(key);

            if (it == obsToLandmark.end())
            {
                continue;
            }

            auto landmarkPair = landmarks.find(it->second);

            // This should not happen if each feature is associated to only one track
            if (landmarkPair == landmarks.end())
            {
                continue;
            }

            sfmData::Landmark l = landmarkPair->second;
            landmarks.erase(landmarkPair->first);

            // re-insert the landmark with the new id
            sfmData.getLandmarks().emplace(trackId, l);
        }
    }

    if (landmarks.size() > 0)
    {
        ALICEVISION_LOG_INFO("Not all existing landmarks have been remapped");
    }

    ALICEVISION_LOG_INFO("Landmarks count after remapping : " << sfmData.getLandmarks().size());
}

} // namespace sfm
} // namespace aliceVision

