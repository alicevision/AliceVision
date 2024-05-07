// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfm/pipeline/expanding/LbaPolicyConnexity.hpp>

namespace aliceVision {
namespace sfm {

bool LbaPolicyConnexity::build(sfmData::SfMData & sfmData, const track::TracksHandler & tracksHandler, const std::set<IndexT> & viewIds) 
{
    if (_historyHandler)
    {
        setupIntrinsics(sfmData);
    }

    ConnexityGraph graph;
    if (!graph.build(sfmData, viewIds))
    {
        return false;
    }

    upgradeSfmData(sfmData, graph);

    return true;
}

void LbaPolicyConnexity::upgradeSfmData(sfmData::SfMData & sfmData, const ConnexityGraph & graph)
{
    /**
     * Propagate states to views in sfmData
    */
    for (auto & pp : sfmData.getPoses())
    {
        int d = graph.getDistance(pp.first);
        if (d <= _distanceLimit)
        {
            if (pp.second.isLocked())
            {
                pp.second.setState(EEstimatorParameterState::CONSTANT);
            }
            else
            {
                pp.second.setState(EEstimatorParameterState::REFINED);
            }
        }
        else if (d == (_distanceLimit + 1))
        {
            pp.second.setState(EEstimatorParameterState::CONSTANT);
        }
        else 
        {
            pp.second.setState(EEstimatorParameterState::IGNORED);
        }
    }

    /*Landmarks*/
    for (auto & pl : sfmData.getLandmarks())
    {
        EEstimatorParameterState state = EEstimatorParameterState::REFINED;

        // By default, the landmark is refined, except if at least one observation comes from an ignored camera

        for (const auto & po : pl.second.getObservations())
        {
            IndexT viewId = po.first;
            IndexT poseId = sfmData.getView(viewId).getPoseId();
            
            if (poseId == UndefinedIndexT)
            {
                continue;
            }

            if (sfmData.getAbsolutePose(poseId).getState() == EEstimatorParameterState::IGNORED)
            {
                state = EEstimatorParameterState::IGNORED;
            }
        }

        pl.second.state = state;
    }
}

void LbaPolicyConnexity::setupIntrinsics(sfmData::SfMData & sfmData)
{
    for (auto & pi : sfmData.getIntrinsics())
    {
        const auto & vec = _historyHandler->getFocalHistory(pi.first);

        size_t lastGood = std::numeric_limits<size_t>::max();
        std::vector<std::pair<size_t, double>> filtered;

        for (int id = vec.size() - 1; id >= 0; id--)
        {
            //Make sure the usage decrease,
            //Remove the steps where the usage increase (we are going to the past)
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
        

        const double mean = std::accumulate(cropped.begin(), cropped.end(), 0.0) / cropped.size();
        std::vector<double> diff(cropped.size());
        std::transform(cropped.begin(), cropped.end(), diff.begin(), [mean](double x) { return x - mean; });
        const double sqSum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
        double stddev = std::sqrt(sqSum / cropped.size());

        double minVal = *std::min_element(focals.begin(), focals.end());
        double maxVal = *std::max_element(focals.begin(), focals.end());
        double normStdev = stddev / (maxVal - minVal);

        if (normStdev < 0.01 || pi.second->isLocked())
        {
            pi.second->setState(EEstimatorParameterState::CONSTANT);
        }
        else
        {
            pi.second->setState(EEstimatorParameterState::REFINED);
        }
    }
}

}
}