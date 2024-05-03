// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.


#pragma once

#include <aliceVision/sfm/pipeline/expanding/ExpansionPolicy.hpp>

namespace aliceVision {
namespace sfm {

class ExpansionPolicyLegacy : public ExpansionPolicy
{
public:

    /**
     * @brief Initialize policy for an iteration
     * @param sfmData the scene to process
     * @return true if the init succeeded
    */
    virtual bool initialize(const sfmData::SfMData & sfmData);

    /**
     * @brief compute policy for an iteration
     * @param sfmData the scene to process
     * @param tracksHandler the tracks for this scene
     * @return true if the policy succeeded
    */
    virtual bool process(const sfmData::SfMData & sfmData, const track::TracksHandler & tracksHandler);
    
    /**
     * @brief Retrieve the selected next views
     * @return a set of views to process
    */
    virtual std::set<IndexT> getNextViews();

    /**
     * @brief Compute score of a view
     * @param tracksMap the scene set of tracks
     * @param usedTracks the list of tracks to consider
     * @param viewId the view of interest 
     * @param maxSize the largest dimension of the view's image
     * @param countLevels the number of levels we want to use (starting from a single pixel level)
     * @return a score
    */
    static double computeScore(const track::TracksMap & tracksMap, 
                                const std::vector<std::size_t> & usedTracks, 
                                const IndexT viewId, 
                                const size_t maxSize,
                                const size_t countLevels);

private:

    // vector of  selected views for this iteration
    std::set<IndexT> _selectedViews;

    // List of available view indices
    std::set<IndexT> _availableViewsIds;

private:
    // Minimal number of points viewed in a view 
    // AND reconstructed in the sfm
    std::size_t _minPointsThreshold = 30;

    // Number of levels in the pyramid starting 
    // from the level with a grid size of 2x2
    // Level 0 = 2x2, Level 1 = 4x4, etc.
    std::size_t _countPyramidLevels = 5;

    // Number of cameras in scene under which the set is considered as unstable 
    size_t _nbFirstUnstableCameras = 30;

    // Maximal number of images in a chunk
    size_t _maxImagesPerGroup = 30;
};

}
}