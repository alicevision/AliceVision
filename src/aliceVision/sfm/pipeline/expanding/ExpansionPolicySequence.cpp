// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/sfm/pipeline/expanding/ExpansionPolicySequence.hpp>
#include <aliceVision/stl/mapUtils.hpp>

#include <limits>

namespace aliceVision {
namespace sfm {

bool ExpansionPolicySequence::initialize(const sfmData::SfMData & sfmData)
{
    _availableViewsIds.clear();

    const std::set<IndexT> allViewsIds = sfmData.getViewsKeys();
    const std::set<IndexT> allReconstructedViewsIds = sfmData.getValidViews();

    // Initial list is the set of non-localised views in the sfmData
    std::set_difference(allViewsIds.begin(), allViewsIds.end(),
                        allReconstructedViewsIds.begin(), allReconstructedViewsIds.end(),
                        std::inserter(_availableViewsIds, _availableViewsIds.begin()));

    return true;
}

bool ExpansionPolicySequence::process(const sfmData::SfMData & sfmData, const track::TracksHandler & tracksHandler)
{
    _selectedViews.clear();

    if (_availableViewsIds.empty())
    {
        return false;
    }

    // Build the set of frame IDs that are already reconstructed
    // UndefinedIndexT frames are skipped (they carry no sequence information)
    std::set<IndexT> reconstructedFrameIds;
    for (const IndexT viewId : sfmData.getValidViews())
    {
        const IndexT fid = sfmData.getView(viewId).getFrameId();
        if (fid != UndefinedIndexT)
        {
            reconstructedFrameIds.insert(fid);
        }
    }

    // Collect reconstructed track IDs for overlap counting
    std::set<size_t> reconstructedTracksIds;
    std::transform(sfmData.getLandmarks().begin(),
                   sfmData.getLandmarks().end(),
                   std::inserter(reconstructedTracksIds, reconstructedTracksIds.begin()),
                   stl::RetrieveKey());

    struct ViewScoring
    {
        IndexT id;
        size_t count;       // number of shared reconstructed tracks
        IndexT frameIdDist; // minimum frame-ID distance to any reconstructed view
    };

    std::vector<ViewScoring> vscoring;

    std::vector<IndexT> vectorAvailableViewsIds(_availableViewsIds.begin(), _availableViewsIds.end());

    #pragma omp parallel for schedule(dynamic)
    for (int pos = 0; pos < static_cast<int>(vectorAvailableViewsIds.size()); pos++)
    {
        const IndexT cViewId = vectorAvailableViewsIds[pos];
        const sfmData::View & v = sfmData.getView(cViewId);

        // Skip views without intrinsics — resection is impossible
        if (!sfmData.isIntrinsicDefined(v))
        {
            continue;
        }

        // Get tracks visible in this view
        const auto tracksPerViewIt = tracksHandler.getTracksPerView().find(cViewId);
        if (tracksPerViewIt == tracksHandler.getTracksPerView().end() || tracksPerViewIt->second.empty())
        {
            continue;
        }
        const aliceVision::track::TrackIdSet & tracksIds = tracksPerViewIt->second;

        // Count reconstructed tracks shared with this view
        std::vector<std::size_t> viewReconstructedTracksIds;
        std::set_intersection(tracksIds.begin(), tracksIds.end(),
                              reconstructedTracksIds.begin(), reconstructedTracksIds.end(),
                              std::back_inserter(viewReconstructedTracksIds));

        if (viewReconstructedTracksIds.empty())
        {
            continue;
        }

        // Compute minimum frame-ID distance to any already-reconstructed view.
        // Views whose frameId is UndefinedIndexT are given maximum distance so
        // that sequence-aware views are always preferred, but they remain
        // eligible once all sequence-aware candidates are exhausted.
        IndexT minDist = std::numeric_limits<IndexT>::max();
        const IndexT fid = v.getFrameId();
        if (fid != UndefinedIndexT && !reconstructedFrameIds.empty())
        {
            // std::set is ordered — find the nearest frame ID with lower_bound
            auto it = reconstructedFrameIds.lower_bound(fid);
            if (it != reconstructedFrameIds.end())
            {
                const IndexT d = *it - fid; // *it >= fid, so no underflow
                if (d < minDist) minDist = d;
            }
            if (it != reconstructedFrameIds.begin())
            {
                --it;
                const IndexT d = fid - *it; // fid > *it, so no underflow
                if (d < minDist) minDist = d;
            }
        }

        ViewScoring scoring;
        scoring.id = cViewId;
        scoring.count = viewReconstructedTracksIds.size();
        scoring.frameIdDist = minDist;

        #pragma omp critical
        {
            vscoring.push_back(scoring);
        }
    }

    if (vscoring.empty())
    {
        return false;
    }

    // Sort: primary key = frame-ID distance (ascending), secondary = track count (descending)
    std::sort(vscoring.begin(), vscoring.end(),
              [](const ViewScoring & a, const ViewScoring & b)
              {
                  if (a.frameIdDist != b.frameIdDist)
                      return a.frameIdDist < b.frameIdDist;
                  return a.count > b.count;
              });

    // Always add the best candidate, whatever its characteristics
    _selectedViews.insert(vscoring[0].id);

    int maxSetSize = static_cast<int>(_maxViewsPerGroup);
    if (maxSetSize == 0)
    {
        maxSetSize = std::numeric_limits<int>::max();
    }

    // During the unstable bootstrapping phase restrict to one view at a time
    if (sfmData.getValidViews().size() < _nbFirstUnstableViews)
    {
        maxSetSize = 1;
    }

    // Add further views that meet the minimum-points threshold
    for (int i = 1; i < static_cast<int>(vscoring.size()) && static_cast<int>(_selectedViews.size()) < maxSetSize; i++)
    {
        if (vscoring[i].count < _minPointsThreshold)
        {
            continue;
        }

        _selectedViews.insert(vscoring[i].id);
    }

    for (const IndexT id : _selectedViews)
    {
        _availableViewsIds.erase(id);
    }

    return true;
}

std::set<IndexT> ExpansionPolicySequence::getNextViews()
{
    return _selectedViews;
}

void ExpansionPolicySequence::rollback(const std::set<IndexT> & viewsSet)
{
    for (const auto & item : viewsSet)
    {
        ALICEVISION_LOG_INFO("Rollback view : " << item);
        _availableViewsIds.insert(item);
    }
}

}
}
