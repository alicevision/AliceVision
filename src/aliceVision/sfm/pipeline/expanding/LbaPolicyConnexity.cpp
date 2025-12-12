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

    
    if (!_graph.build(sfmData, tracksHandler.getTracksPerView(), viewIds))
    {
        return false;
    }

    upgradeSfmData(sfmData);

    return true;
}

void LbaPolicyConnexity::upgradeSfmData(sfmData::SfMData & sfmData)
{
    /**
     * Propagate states to views in sfmData
    */
    for (auto & [idPose, pose] : sfmData.getPoses().valueRange())
    {
        int d = _graph.getDistance(idPose);
        if (d <= _distanceLimit)
        {
            if (pose.isLocked())
            {
                pose.setState(EEstimatorParameterState::CONSTANT);
            }
            else
            {
                pose.setState(EEstimatorParameterState::REFINED);
            }
        }
        else if (d == (_distanceLimit + 1))
        {
            pose.setState(EEstimatorParameterState::CONSTANT);
        }
        else 
        {
            pose.setState(EEstimatorParameterState::IGNORED);
        }
    }

    /*Landmarks*/
    for (auto & [idLandmark, landmark] : sfmData.getLandmarks())
    {
        EEstimatorParameterState state = EEstimatorParameterState::REFINED;

        bool hasIgnored = false;
        bool hasEstimated = false;

        // By default, the landmark is refined, except if at least one observation comes from an ignored camera
        for (const auto & [viewId, _] : landmark.getObservations())
        {
            IndexT poseId = sfmData.getView(viewId).getPoseId();
            
            if (poseId == UndefinedIndexT)
            {
                continue;
            }

            if (sfmData.getAbsolutePose(poseId).getState() == EEstimatorParameterState::IGNORED)
            {
                hasIgnored = true;
            }

            if (sfmData.getAbsolutePose(poseId).getState() == EEstimatorParameterState::REFINED)
            {
                hasEstimated = true;
            }
        }

        //If no associated view which has to be computed, 
        //then obviously we want to ignore it (should be implicit in the bundle though)
        if ((!hasEstimated) || hasIgnored)
        {
            landmark.setState(EEstimatorParameterState::IGNORED);
        }
    }
}

void LbaPolicyConnexity::setupIntrinsics(sfmData::SfMData & sfmData)
{
    const int minUsage = 25;

    for (auto & [idIntrinsic, intrinsic] : sfmData.getIntrinsics())
    {   
        //If the intrinsic is locked, we're sure we don't need to look at it
        if (intrinsic->isLocked())
        {
            intrinsic->setState(EEstimatorParameterState::CONSTANT);
            continue;
        }

        //Retrieve focal history (vector of [usageCount, focalvalue])
        const auto & vec = _historyHandler->getFocalHistory(idIntrinsic);
        if (vec.size() == 0)
        {
            intrinsic->setState(EEstimatorParameterState::REFINED);
            continue;
        }

        size_t lastGood = std::numeric_limits<size_t>::max();
        std::vector<std::pair<size_t, double>> filtered;

        //Sort by decreasing id
        for (int id = vec.size() - 1; id >= 0; id--)
        {
            //Make sure the usage decrease.
            if (vec[id].first < lastGood)
            {
                lastGood = vec[id].first;
                filtered.push_back(vec[id]);
            }
        }

        std::vector<double> cropped;
        std::vector<double> focals;
        int largestCount = filtered.front().first;

        //If the largest count is under a threshold, we refine.
        if (largestCount < minUsage)
        {
            intrinsic->setState(EEstimatorParameterState::REFINED);
            continue;
        }

        //Build an history of focal values to estimate its variance
        bool nomore = false;
        for (int id = 0; id < filtered.size(); id++)
        {
            if (!nomore)
            {
                cropped.push_back(filtered[id].second);
            }

            if (largestCount - filtered[id].first > minUsage)
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

        //If the focal stay fixed on a value, then consider it as constant
        if (normStdev < 0.01)
        {
            intrinsic->setState(EEstimatorParameterState::CONSTANT);
        }
        else
        {
            intrinsic->setState(EEstimatorParameterState::REFINED);
        }
    }
}

}
}