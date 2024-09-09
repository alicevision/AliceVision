// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "preprocess.hpp"

namespace aliceVision {
namespace sfm {

void remapLandmarkIdsToTrackIds(sfmData::SfMData& sfmData, const track::TracksMap & tracks)
{
    using namespace track;

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

    ALICEVISION_LOG_DEBUG("Builds landmarks temporary comparison structure");

    for (const auto& landmarkPair : landmarks)
    {
        const IndexT landmarkId = landmarkPair.first;
        const IndexT firstViewId = landmarkPair.second.getObservations().begin()->first;
        const IndexT firstFeatureId = landmarkPair.second.getObservations().begin()->second.getFeatureId();
        const feature::EImageDescriberType descType = landmarkPair.second.descType;

        obsToLandmark.emplace(ObsKey(firstViewId, firstFeatureId, descType), landmarkId);
    }

    ALICEVISION_LOG_DEBUG("Find corresponding landmark id per track id");

    // find corresponding landmark id per track id
    for (const auto& trackPair : tracks)
    {
        const IndexT trackId = trackPair.first;
        const Track& track = trackPair.second;

        for (const auto& featView : track.featPerView)
        {
            const ObsToLandmark::const_iterator it = obsToLandmark.find(ObsKey(featView.first, featView.second.featureId, track.descType));

            if (it != obsToLandmark.end())
            {
                // re-insert the landmark with the new id
                sfmData.getLandmarks().emplace(trackId, landmarks.find(it->second)->second);
                break;  // one landmark per track
            }
        }
    }

    ALICEVISION_LOG_INFO("Landmark ids to track ids remapping: " << std::endl
                                                                 << "\t- # tracks: " << tracks.size() << std::endl
                                                                 << "\t- # input landmarks: " << landmarks.size() << std::endl
                                                                 << "\t- # output landmarks: " << sfmData.getLandmarks().size());
}

}
}