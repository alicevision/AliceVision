// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ExpansionHistory.hpp"


#include <aliceVision/stl/mapUtils.hpp>

namespace aliceVision {
namespace sfm {

ExpansionHistory::ExpansionHistory()
{
}

bool ExpansionHistory::initialize(const sfmData::SfMData & sfmData)
{
    // Update epoch id 
    // We want to have a starting epoch larger than the one found in the existing views
    _epoch = 0;
    for (const auto & pv : sfmData.getViews())
    {
        _epoch = std::max(_epoch, static_cast<size_t>(pv.second->getResectionId()));
    }

    _epoch++;

    return true;
}

bool ExpansionHistory::beginEpoch(const sfmData::SfMData & sfmData)
{
    return true;
}

void ExpansionHistory::endEpoch(sfmData::SfMData & sfmData, const std::set<IndexT> & selectedViews)
{
    // Get a list of valid views
    std::set<IndexT> validViewIds;
    for (IndexT viewId : selectedViews)
    {
        if (sfmData.isPoseAndIntrinsicDefined(viewId))
        {
            validViewIds.insert(viewId);
        }
    }

    // Store epoch
    for (IndexT viewId : validViewIds)
    {
        sfmData::View & v = sfmData.getView(viewId);
        v.setResectionId(_epoch);
    }

    // Make sure next call will use a different epoch
    _epoch++;
}


void ExpansionHistory::saveState(const sfmData::SfMData & sfmData)
{
    for (const auto & pIntrinsic : sfmData.getIntrinsics())
    {
        size_t usage = 0;
        std::shared_ptr<camera::IntrinsicScaleOffset> iso = std::dynamic_pointer_cast<camera::IntrinsicScaleOffset>(pIntrinsic.second);

        for (const auto & pView : sfmData.getViews())
        {
            IndexT viewIntrinsicId = pView.second->getIntrinsicId();
            if (!sfmData.isPoseAndIntrinsicDefined(pView.second.get()))
            {
                continue;
            }

            if (pIntrinsic.first == viewIntrinsicId)
            {
                usage++;
            }
        }

        //Store usage counter
        _focalHistory[pIntrinsic.first].push_back(std::make_pair(usage, iso->getScale().x()));
    }

    
}

} // namespace sfm
} // namespace aliceVision

