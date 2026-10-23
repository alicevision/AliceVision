// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/types.hpp>
#include <aliceVision/sfmData/SfMData.hpp>

#include <array>
#include <map>
#include <vector>

#include <ceres/ordered_groups.h>

namespace ceres {
class Problem;
}

namespace aliceVision {
namespace sfm {

/**
 * @brief Storage for the landmarks parameter blocks given to ceres.
 *
 * A landmark is stored either as a 3D point in world coordinates, or, when it has a reference view,
 * in the reference view geometric frame: a 3D point, or a bearing vector (x/z, y/z) when the landmark
 * has a point fetcher.
 * Blocks are stored in a std::map so their addresses stay valid while ceres uses them.
 *
 * Typical usage: fill(), addToProblem(), solve, then updateSfmData().
 */
class LandmarksContainer
{
  public:
    /// landmark parameters as seen by ceres
    using Block = std::array<double, 3>;

    /**
     * @brief A landmark parameter block with its lock
     */
    struct Entry
    {
        /// landmark parameters, only the first size values are used
        Block block;
        /// number of parameters used by ceres: 3 for a point, 2 for a bearing vector
        int size = 3;
        /// pose id of the reference view, UndefinedIndexT if the block is in world coordinates
        IndexT referencePoseId = UndefinedIndexT;
        /// if true, the whole landmark block is kept constant
        bool lock = true;
    };

    /**
     * @brief Replace the content with all the landmarks of sfmData, except the ignored ones.
     * Locked or constant landmarks are locked, as are all landmarks if the structure is not refined.
     * Clears the container first, which invalidates all previously returned block pointers.
     * @param[in] sfmData the scene providing the landmarks
     * @param[in] refineStructure true if the landmarks should be refined
     */
    void fill(const sfmData::SfMData& sfmData, bool refineStructure);

    /**
     * @brief Add all entries as parameter blocks of the problem, applying their locks
     * @param[in,out] problem the ceres problem
     */
    void addToProblem(ceres::Problem& problem);

    /**
     * @brief Add all landmark blocks to a group of the linear solver ordering
     * @param[in,out] ordering the ceres parameter blocks ordering
     * @param[in] group the elimination group (lower groups are eliminated first)
     */
    void addToOrdering(ceres::ParameterBlockOrdering& ordering, int group);

    /**
     * @brief Copy the solved blocks back into the refined landmarks of sfmData.
     * Blocks expressed in a reference view frame are converted using the sfmData poses,
     * so the poses must be updated first.
     * @param[in,out] sfmData the scene to update
     */
    void updateSfmData(sfmData::SfMData& sfmData) const;

    /**
     * @brief Get the blocks of all entries
     * @return the block pointers, ordered by landmark id
     */
    std::vector<double*> getParameterBlocks();

    /**
     * @brief Check if a landmark has an entry
     * @param[in] landmarkId the landmark id
     * @return true if the landmark is in the container
     */
    bool contains(IndexT landmarkId) const;

    /**
     * @brief Get the data block pointer of a landmark, to give to ceres
     * @param[in] landmarkId the landmark id, must be in the container
     * @return pointer to the landmark block
     */
    double* getBlock(IndexT landmarkId);

    /**
     * @brief Get the entry of a landmark
     * @param[in] landmarkId the landmark id, must be in the container
     * @return the entry
     */
    const Entry& at(IndexT landmarkId) const;

    /**
     * @brief Remove all entries, invalidating all block pointers
     */
    void clear();

  private:
    /// entries indexed by landmark id
    std::map<IndexT, Entry> _entries;
};

}  // namespace sfm
}  // namespace aliceVision
