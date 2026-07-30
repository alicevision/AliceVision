// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfm/pipeline/expanding/TrackWeighting.hpp>

#include <algorithm>
#include <vector>
#include <stack>
#include <cmath>
#include <aliceVision/stl/mapUtils.hpp>
#include <aliceVision/system/Logger.hpp>

namespace aliceVision {
namespace sfm {

bool weightObservationsAlongTrack(sfmData::SfMData& sfmData, int fadingSize)
{
    // This weighting function is disabled if fadingSize is equal to 0
    // (and invalid if fadingSize is lower than 0)
    if (fadingSize <= 0)
    {
        ALICEVISION_LOG_DEBUG("Observations will NOT be weighted based on their temporal position within the track.");
        return (fadingSize == 0);
    }

    std::map<IndexT, std::vector<IndexT>> imageSeqSortedViewIDs;
    for (const auto& [imageGroupID, imageGroup] : sfmData.getImageGroups().valueRange())
    {
        if (imageGroup.getType() != sfmData::ImageGroup::Type::ImageSequence)
        {
            continue;
        }

        const sfmData::Views& sfmViews = sfmData.getViews();

        auto isPartOfImageGroup = [&imageGroupID, &sfmData](const auto& viewPair)
                 { return viewPair.second->getImageGroupId() == imageGroupID && sfmData.existsPose(*viewPair.second); };

        const int viewCount = std::count_if(sfmViews.begin(), sfmViews.end(), isPartOfImageGroup);
        if (viewCount == 0)
        {
            continue;
        }

        std::vector<std::pair<IndexT, std::shared_ptr<sfmData::View>>> viewsVec;
        std::copy_if(sfmViews.begin(), sfmViews.end(), std::back_inserter(viewsVec), isPartOfImageGroup);

        std::sort(viewsVec.begin(), viewsVec.end(),
                  [](const auto& viewP1, const auto& viewP2)
                  { return viewP1.second->getFrameId() < viewP2.second->getFrameId(); });

        std::vector<IndexT> viewIDs;
        std::transform(viewsVec.begin(), viewsVec.end(), std::back_inserter(viewIDs), stl::RetrieveKey());

        imageSeqSortedViewIDs.emplace(imageGroupID, viewIDs);
    }

    for (auto& [landmarkId, landmark]: sfmData.getLandmarks())
    {
        auto& observations = landmark.getObservations();

        // Observations that are not part of an imageSequence are discarded (never found in imageSeqSortedViewIDs),
        // It is equivalent to explicitly giving them a weight of 1

        for (const auto& [imageGroupID, sortedViewIDs] : imageSeqSortedViewIDs)
        {
            // The first frames of the imageSequence are NOT considered as the first frames for an observation sequence
            // and must be handled differently (i.e. no fade in)
            bool firstFrames = true;

            // We search for continuous observation sequences.
            // Once an observation sequence ends, we compute the weights, and then look for another observation sequence
            std::stack<sfmData::Observation*> obsSequence;
            const auto lastViewId = sortedViewIDs.back();
            for (const auto& viewID : sortedViewIDs)
            {
                auto it = observations.find(viewID);
                if (it != observations.end())
                {
                    obsSequence.push(&it->second);
                    if (viewID != lastViewId)
                    {
                        continue;
                    }
                }
                if (!obsSequence.empty())
                {
                    // positionWeight is initialized to do a fade out, except for an observation sequence
                    // that includes the last frame of the imageSequence
                    int positionWeight = (viewID == lastViewId && it != observations.end()) ? fadingSize : 0;
                    while (!obsSequence.empty())
                    {
                        // obsSequence.size() handles the fade in
                        // But no fade in for an observation sequence that includes the first frame of the imageSequence
                        int maxPosWeight = firstFrames ? fadingSize : std::min(fadingSize, int(obsSequence.size()));
                        // (positionWeight + 1) handles the fade out
                        positionWeight = std::min(maxPosWeight, positionWeight + 1);
                        sfmData::Observation* obs = obsSequence.top();
                        obsSequence.pop();
                        double obsWeight = obs->getWeight();
                        obsWeight = (obsWeight * positionWeight) / double(fadingSize);
                        obs->setWeight(obsWeight);
                    }
                }
                firstFrames = false;
            }
            if (!obsSequence.empty())
            {
                throw std::out_of_range("obsSequence not empty ! " + std::to_string(obsSequence.size()));
            }
        }
    }
    return true;
}


bool weightTracks(sfmData::SfMData& sfmData, double trackLengthMaxWeight, int trackLengthLowerThreshold, int trackLengthUpperThreshold)
{
    // This weighting function is disabled if trackLengthMaxWeight is equal to 1
    // (and invalid if trackLengthMaxWeight is lower than 1 or if the upper threshold is lower than the lower threshold)
    if (trackLengthMaxWeight <= 1. || trackLengthUpperThreshold <= trackLengthLowerThreshold)
    {
        ALICEVISION_LOG_DEBUG("Observations will NOT be weighted based on the number of observations of the corresponding landmark.");
        return (trackLengthMaxWeight == 1.);
    }

    // The weights are normalized so that the longer tracks get a weight of 1.
    const double invTrackLengthMaxWeight = 1. / trackLengthMaxWeight;
    const double trackLengthWeightDelta = (1. - invTrackLengthMaxWeight) / double(trackLengthUpperThreshold - trackLengthLowerThreshold);

    for (auto& [landmarkId, landmark]: sfmData.getLandmarks())
    {
        auto& observations = landmark.getObservations();

        int clampedTrackLength = std::clamp(observations.size(), size_t(trackLengthLowerThreshold), size_t(trackLengthUpperThreshold));

        const double landmarkWeight = invTrackLengthMaxWeight + double(clampedTrackLength - trackLengthLowerThreshold) * trackLengthWeightDelta;

        // Update the observation weights
        for (auto& [viewId, observation]: observations)
        {
            double obsWeight = observation.getWeight();
            obsWeight *= landmarkWeight;
            observation.setWeight(obsWeight);
        }
    }
    return true;
}


bool resetObservationWeights(sfmData::SfMData& sfmData)
{
    for (auto& [_, landmark] : sfmData.getLandmarks())
    {
        for (auto& [_, obs] : landmark.getObservations())
        {
            obs.setWeight(1.0);
        }
    }
    return true;
}

}
}
