// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfm/pipeline/bootstrapping/TracksDepths.hpp>

namespace aliceVision {
namespace sfm {

bool buildSfmDataFromDepthMap(sfmData::SfMData & output,
                            const sfmData::SfMData & sfmData, 
                            const track::TracksMap& tracksMap, 
                            const track::TracksPerView & tracksPerView, 
                            IndexT viewId)
{
    output.clear();

    const sfmData::View & view = sfmData.getView(viewId);
    const camera::IntrinsicBase & intrinsic = sfmData.getIntrinsic(view.getIntrinsicId());

    if (tracksPerView.find(viewId) == tracksPerView.end())
    {
        return false;
    }

    sfmData::Landmarks & landmarks = output.getLandmarks();

    //Copy all intrinsics, because it's light.
    for (const auto & [intrinsicId, intrinsic] : sfmData.getIntrinsics())
    {
        output.getIntrinsics().insert(
            std::make_pair(intrinsicId, 
                camera::IntrinsicBase::sptr(intrinsic->clone()))
            );
    }

    std::set<IndexT> usedViewIds;
    const auto & trackIds = tracksPerView.at(viewId);
    for (const auto & trackId : trackIds)
    {
        const auto & track = tracksMap.at(trackId);
        const auto & feat = track.featPerView.at(viewId);
        const double & Z = feat.depth;

        if (Z <= 0.0)
        {
            continue;
        }
        
        const Vec2 meters = intrinsic.removeDistortion(intrinsic.ima2cam(feat.coords.cast<double>()));
        
        sfmData::Landmark & landmark = landmarks[trackId];
        landmark.getX().x() = meters.x() * Z;
        landmark.getX().y() = meters.y() * Z;
        landmark.getX().z() = Z;

        landmark.setDescType(track.descType);

        for (const auto & [otherViewId, otherFeat] : track.featPerView)
        {
            usedViewIds.insert(otherViewId);
        }
    }

    // Copy only used views
    for (const auto & usedViewId : usedViewIds)
    {
        const auto & iview = sfmData.getViewSharedPtr(usedViewId);

        output.getViews().insert(
            std::make_pair(usedViewId, 
                sfmData::View::sptr(iview->clone()))
            );
    }

    return true;
}

}
}