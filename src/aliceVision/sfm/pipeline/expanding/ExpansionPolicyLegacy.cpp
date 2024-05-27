// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfm/pipeline/expanding/ExpansionPolicyLegacy.hpp>
#include <aliceVision/stl/mapUtils.hpp>

namespace aliceVision {
namespace sfm {

bool ExpansionPolicyLegacy::initialize(const sfmData::SfMData & sfmData)
{
    _availableViewsIds.clear();

    std::set<IndexT> allViewsIds = sfmData.getViewsKeys();
    std::set<IndexT> allReconstructedViewsIds = sfmData.getValidViews();

    //Initial list is the set of non localized views in the sfmData
    std::set_difference(allViewsIds.begin(), allViewsIds.end(), 
                        allReconstructedViewsIds.begin(), allReconstructedViewsIds.end(), 
                        std::inserter(_availableViewsIds, _availableViewsIds.begin()));

    return true;
}

bool ExpansionPolicyLegacy::process(const sfmData::SfMData & sfmData, const track::TracksHandler & tracksHandler)
{
    _selectedViews.clear();

    if (_availableViewsIds.size() == 0)
    {
        return false;
    }

    // Collect tracksIds
    std::set<size_t> reconstructedTracksIds;
    std::transform(sfmData.getLandmarks().begin(),
                   sfmData.getLandmarks().end(),
                   std::inserter(reconstructedTracksIds, reconstructedTracksIds.begin()),
                   stl::RetrieveKey());


    struct ViewScoring
    {
        IndexT id;
        size_t count;
        double score;
    };

    std::vector<ViewScoring> vscoring;

    //Loop over possible views
    for (IndexT cViewId : _availableViewsIds)
    {
        const sfmData::View & v = sfmData.getView(cViewId);

        //If this view has no intrinsic,
        //Then we have now immediate way to do resection
        //Actually, it should be an error given the pipeline
        if (!sfmData.isIntrinsicDefined(v))
        {
            continue;
        }

        // Get all tracks which are visible in the view of interest
        const aliceVision::track::TrackIdSet & tracksIds = tracksHandler.getTracksPerView().at(cViewId);
        if (tracksIds.empty())
        {
            continue;
        }


        // Intersection of tracks for the view and reconstructed tracks
        // This will give the list of tracks for this view which are already reconstructed and have
        // an associated landmark
        std::vector<std::size_t> viewReconstructedTracksIds;
        std::set_intersection(tracksIds.begin(),
                              tracksIds.end(),
                              reconstructedTracksIds.begin(),
                              reconstructedTracksIds.end(),
                              std::back_inserter(viewReconstructedTracksIds));

        
        if (viewReconstructedTracksIds.empty())
        {
            continue;
        }
        
        int maxDim = std::max(v.getImage().getWidth(), v.getImage().getHeight());

        //Compute a score for this view
        ViewScoring scoring;
        scoring.id = cViewId;
        scoring.score = ExpansionPolicyLegacy::computeScore(tracksHandler.getAllTracks(), viewReconstructedTracksIds, cViewId, maxDim, _countPyramidLevels);
        scoring.count = viewReconstructedTracksIds.size();
        vscoring.push_back(scoring);
    }

    if (vscoring.size() == 0)
    {
        return false;
    }

    //Sort views by decreasing scores
    std::sort(vscoring.begin(), vscoring.end(), 
                [](const ViewScoring & v1, const ViewScoring & v2) 
                {
                    return v1.score > v2.score;
                });

    //Always add at least the best score, whatever it is
    _selectedViews.insert(vscoring[0].id);

    int maxSetSize = _maxViewsPerGroup;
    if (sfmData.getValidViews().size() < _nbFirstUnstableViews)
    {
        maxSetSize = 1;
    }

    //Add all views which have valid characteristics
    for (int i = 1; i < vscoring.size() && _selectedViews.size() < maxSetSize; i++)
    {
        if (vscoring[i].count < _minPointsThreshold)
        {
            continue;
        }

        _selectedViews.insert(vscoring[i].id);
    }

    for (IndexT id : _selectedViews)
    {
        _availableViewsIds.erase(id);
    }

    return true;
}
    
std::set<IndexT> ExpansionPolicyLegacy::getNextViews() 
{
    return _selectedViews;
}

double ExpansionPolicyLegacy::computeScore(const track::TracksMap & tracksMap, 
                                    const std::vector<std::size_t> & usedTracks, 
                                    const IndexT viewId, 
                                    const size_t maxSize,
                                    const size_t countLevels)
{
    //Compute min and max level such that max(width, height) <= 2 on maxLevel 
    int maxLevel = std::ceil(std::log2(maxSize));
    int minLevel = std::max(1, maxLevel - int(countLevels));
    int realCountLevels = maxLevel - minLevel + 1;

    std::vector<std::set<std::pair<unsigned int, unsigned int>>> uniques(realCountLevels);


    //Coordinates are considered as integers
    //Power of two divide  == right binary shift on integers

    for (auto trackId : usedTracks)
    {
        auto & track = tracksMap.at(trackId);
        const Vec2 pt = track.featPerView.at(viewId).coords;

        unsigned int ptx = (unsigned int)(pt.x());
        unsigned int pty = (unsigned int)(pt.y());

        for (unsigned int shiftLevel = 0; shiftLevel < realCountLevels; shiftLevel++)
        {
            unsigned int level = minLevel + shiftLevel;
            unsigned int lptx = ptx >> level;
            unsigned int lpty = pty >> level;

            uniques[shiftLevel].insert(std::make_pair(lptx, lpty));
        }
    } 

    double sum = 0.0;
    for (unsigned int shiftLevel = 0; shiftLevel < realCountLevels; shiftLevel++)
    {
        int size = uniques[shiftLevel].size();
        if (size <= 1)
        {
            continue;
        }

        //The higher the level, the higher the weight per cell
        double w = pow(2.0, shiftLevel);
        sum += w * double(size);
    }

    return sum;
}

}
}