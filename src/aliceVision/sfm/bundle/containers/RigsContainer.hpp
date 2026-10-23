// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/types.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfm/bundle/containers/PosesContainer.hpp>

#include <map>
#include <vector>

#include <ceres/ordered_groups.h>

namespace ceres {
class Problem;
}

namespace aliceVision {
namespace sfm {

/**
 * @brief Storage for the rig sub-poses parameter blocks given to ceres.
 *
 * Each sub-pose is stored as a block of 6 doubles: ceres angleAxis(3) + center(3),
 * indexed by rig id then sub-pose id, along with its rotation and translation locks.
 * Blocks are stored in std::map so their addresses stay valid while ceres uses them.
 *
 * Typical usage: fill(), addToProblem(), solve, then updateSfmData().
 */
class RigsContainer
{
  public:
    /// a sub-pose block with its locks, same layout as a pose entry
    using Entry = PosesContainer::Entry;

    /**
     * @brief Replace the content with all the initialized sub-poses of sfmData.
     * Constant sub-poses are fully locked, otherwise locks follow the refine flags.
     * Clears the container first, which invalidates all previously returned block pointers.
     * @param[in] sfmData the scene providing the rigs
     * @param[in] refineRotation true if the rotations should be refined
     * @param[in] refineTranslation true if the translations should be refined
     */
    void fill(const sfmData::SfMData& sfmData, bool refineRotation, bool refineTranslation);

    /**
     * @brief Add all entries as parameter blocks of the problem, applying their locks
     * @param[in,out] problem the ceres problem
     */
    void addToProblem(ceres::Problem& problem);

    /**
     * @brief Add all sub-pose blocks to a group of the linear solver ordering
     * @param[in,out] ordering the ceres parameter blocks ordering
     * @param[in] group the elimination group (lower groups are eliminated first)
     */
    void addToOrdering(ceres::ParameterBlockOrdering& ordering, int group);

    /**
     * @brief Copy the solved blocks back into the sub-poses of sfmData
     * @param[in,out] sfmData the scene to update
     */
    void updateSfmData(sfmData::SfMData& sfmData) const;

    /**
     * @brief Get the blocks of all entries
     * @return the block pointers, ordered by rig id then sub-pose id
     */
    std::vector<double*> getParameterBlocks();

    /**
     * @brief Check if a sub-pose has an entry
     * @param[in] rigId the rig id
     * @param[in] subPoseId the sub-pose index in the rig
     * @return true if the sub-pose is in the container
     */
    bool contains(IndexT rigId, IndexT subPoseId) const;

    /**
     * @brief Get the data block pointer of a sub-pose, to give to ceres
     * @param[in] rigId the rig id
     * @param[in] subPoseId the sub-pose index in the rig, must be in the container
     * @return pointer to the 6 doubles sub-pose block
     */
    double* get(IndexT rigId, IndexT subPoseId);

    /**
     * @brief Get the entry of a sub-pose
     * @param[in] rigId the rig id
     * @param[in] subPoseId the sub-pose index in the rig, must be in the container
     * @return the entry
     */
    const Entry& at(IndexT rigId, IndexT subPoseId) const;

    /**
     * @brief Remove all entries, invalidating all block pointers
     */
    void clear();

  private:
    /// entries indexed by rig id then sub-pose id
    std::map<IndexT, std::map<IndexT, Entry>> _entries;
};

}  // namespace sfm
}  // namespace aliceVision
