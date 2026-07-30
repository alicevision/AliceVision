// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfm/pipeline/expanding/ExpansionPolicy.hpp>

namespace aliceVision {
namespace sfm {

/**
 * @brief Expansion policy that selects views by frame ID proximity.
 *
 * At each iteration the available view whose frame ID is closest (in absolute
 * distance) to any already-reconstructed view is selected first, effectively
 * filling holes in the frame-ID sequence before jumping to distant frames.
 */
class ExpansionPolicySequence : public ExpansionPolicy
{
public:
    using uptr = std::unique_ptr<ExpansionPolicySequence>;

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
     * @brief rollback some processed views inside the available views
     * @param viewsSet the set of views that we want to be able to select again.
     */
    virtual void rollback(const std::set<IndexT> & viewsSet);

    /**
     * @brief Set the minimum number of reconstructed tracks required for a view to be selected
     * @param count minimum track count
     */
    void setMinPointsThreshold(std::size_t count)
    {
        _minPointsThreshold = count;
    }

    /**
     * @brief Set the number of views considered as unstable at the start
     * @param count a number of views
     */
    void setNbFirstUnstableViews(size_t count)
    {
        _nbFirstUnstableViews = count;
    }

    /**
     * @brief Set the maximum number of views selected per group
     * @param count a number of views
     */
    void setMaxViewsPerGroup(size_t count)
    {
        _maxViewsPerGroup = count;
    }

private:
    // Views selected for the current iteration
    std::set<IndexT> _selectedViews;

    // Views still available for reconstruction
    std::set<IndexT> _availableViewsIds;

private:
    // Minimum number of reconstructed tracks a view must share with the scene
    std::size_t _minPointsThreshold = 30;

    // Number of cameras in scene under which the set is considered as unstable
    size_t _nbFirstUnstableViews = 30;

    // Maximum number of images in a chunk (0 = unlimited)
    size_t _maxViewsPerGroup = 30;
};

}
}
