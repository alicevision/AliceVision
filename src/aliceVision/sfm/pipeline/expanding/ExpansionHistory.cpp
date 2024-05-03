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

    for (auto & pfh : _focalHistory)
    {
        const auto & vec = pfh.second;

        size_t lastGood = std::numeric_limits<size_t>::max();
        std::vector<std::pair<size_t, double>> filtered;

        for (int id = vec.size() - 1; id >= 0; id--)
        {
            //Make sure the usage decrease
            if (vec[id].first < lastGood)
            {
                lastGood = vec[id].first;
                filtered.push_back(vec[id]);
            }
        }

        std::vector<double> cropped;
        std::vector<double> focals;
        int largestCount = filtered.front().first;
        bool nomore = false;
        for (int id = 0; id < filtered.size(); id++)
        {
            if (!nomore)
            {
                cropped.push_back(filtered[id].second);
            }

            if (largestCount - filtered[id].first > 25)
            {
                nomore = true;
            }

            focals.push_back(filtered[id].second);
        }
        

        /*const double mean = std::accumulate(cropped.begin(), cropped.end(), 0.0) / cropped.size();
        std::vector<double> diff(cropped.size());
        std::transform(cropped.begin(), cropped.end(), diff.begin(), [mean](double x) { return x - mean; });
        const double sqSum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
        double stddev = std::sqrt(sqSum / cropped.size());

        double minVal = *std::min_element(focals.begin(), focals.end());
        double maxVal = *std::max_element(focals.begin(), focals.end());
        double normStdev = stddev / (maxVal - minVal);*/
    }   
}

} // namespace sfm
} // namespace aliceVision

