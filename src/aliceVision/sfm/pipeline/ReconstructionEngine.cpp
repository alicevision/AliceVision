// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ReconstructionEngine.hpp"

#include <aliceVision/feature/RegionsPerView.hpp>
#include <aliceVision/sfm/pipeline/regionsIO.hpp>


namespace aliceVision {
namespace sfm {


void retrieveMarkersId(sfmData::SfMData& sfmData)
{
    std::set<feature::EImageDescriberType> allMarkerDescTypes;
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CCTAG)
    allMarkerDescTypes.insert(feature::EImageDescriberType::CCTAG3);
    allMarkerDescTypes.insert(feature::EImageDescriberType::CCTAG4);
#endif
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_APRILTAG)
    allMarkerDescTypes.insert(feature::EImageDescriberType::APRILTAG16H5);
#endif
    if (allMarkerDescTypes.empty())
        return;

    std::set<feature::EImageDescriberType> usedDescTypes = sfmData.getLandmarkDescTypes();

    std::vector<feature::EImageDescriberType> markerDescTypes;
    std::set_intersection(allMarkerDescTypes.begin(), allMarkerDescTypes.end(),
        usedDescTypes.begin(), usedDescTypes.end(),
        std::back_inserter(markerDescTypes));

    std::set<feature::EImageDescriberType> markerDescTypes_set(markerDescTypes.begin(), markerDescTypes.end());

    if(markerDescTypes.empty())
        return;

    // load the corresponding view regions
    feature::RegionsPerView regionPerView;
    std::set<IndexT> filter;
    // It could be optimized by loading only the minimal number of desc files,
    // but as we are only retrieving them for markers, the performance impact is limited.
    if (!sfm::loadRegionsPerView(regionPerView, sfmData, sfmData.getFeaturesFolders(), markerDescTypes, filter))
    {
        ALICEVISION_THROW_ERROR("Error while loading markers regions.");
    }
    for (auto& landmarkIt : sfmData.getLandmarks())
    {
        auto& landmark = landmarkIt.second;
        if (landmark.observations.empty())
            continue;
        if (markerDescTypes_set.find(landmark.descType) == markerDescTypes_set.end())
            continue;
        landmark.rgb = image::BLACK;

        const auto obs = landmark.observations.begin();
        const feature::Regions& regions = regionPerView.getRegions(obs->first, landmark.descType);
        const feature::CCTAG_Regions* cctagRegions = dynamic_cast<const feature::CCTAG_Regions*>(&regions);
        const feature::APRILTAG_Regions* apriltagRegions = dynamic_cast<const feature::APRILTAG_Regions*>(&regions);
        if (cctagRegions) {
            const auto& d = cctagRegions->Descriptors()[obs->second.id_feat];
            for (int i = 0; i < d.size(); ++i)
            {
                if (d[i] == 255)
                {
                    ALICEVISION_LOG_TRACE("Found cctag marker: " << i << " (landmarkId: " << landmarkIt.first << ").");
                    landmark.rgb.r() = i;
                    break;
                }
            }
        } else if (apriltagRegions) {
            const auto& d = apriltagRegions->Descriptors()[obs->second.id_feat];
            for (int i = 0; i < d.size(); ++i)
            {
                if (d[i] == 255)
                {
                    ALICEVISION_LOG_TRACE("Found apriltag marker: " << i << " (landmarkId: " << landmarkIt.first << ").");
                    landmark.rgb.r() = i;
                    break;
                }
            }
        }
    }
}


} // namespace sfm
} // namespace aliceVision
