// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/types.hpp>
#include <aliceVision/sfmData/CameraPose.hpp>
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
 * @brief Storage for the camera poses parameter blocks given to ceres.
 *
 * Each pose is stored as a block of 6 doubles: ceres angleAxis(3) + center(3),
 * along with locks telling which part of the block must stay constant during the solve.
 * Blocks are stored in a std::map so their addresses stay valid while ceres uses them.
 *
 * Typical usage: fill(), addToProblem(), solve, then updateSfmData().
 */
class PosesContainer
{
  public:
    /// pose parameters as seen by ceres: angleAxis(3) + center(3)
    using Block = std::array<double, 6>;

    /**
     * @brief A pose parameter block with its locks
     */
    struct Entry
    {
        /// pose parameters: angleAxis(3) + center(3)
        Block block;
        /// if true, the rotation part (angleAxis) is kept constant
        bool lockRotation = false;
        /// if true, the translation part (center) is kept constant
        bool lockTranslation = false;
    };

    /**
     * @brief Create (or overwrite) the entry of a pose from its current value
     * @param[in] poseId the pose id
     * @param[in] cameraPose the pose used to initialize the block
     * @param[in] lockRotation keep the rotation constant
     * @param[in] lockTranslation keep the translation constant
     * @return the entry, whose block address is stable until clear()
     */
    const Entry& add(IndexT poseId, const sfmData::CameraPose& cameraPose, bool lockRotation, bool lockTranslation);

    /**
     * @brief Replace the content with all the poses of sfmData, except the ignored ones.
     * Locked or constant poses are fully locked, otherwise locks follow the refine flags.
     * Clears the container first, which invalidates all previously returned block pointers.
     * @param[in] sfmData the scene providing the poses
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
     * @brief Add all pose blocks to a group of the linear solver ordering
     * @param[in,out] ordering the ceres parameter blocks ordering
     * @param[in] group the elimination group (lower groups are eliminated first)
     */
    void addToOrdering(ceres::ParameterBlockOrdering& ordering, int group);

    /**
     * @brief Copy the solved blocks back into the refined poses of sfmData
     * @param[in,out] sfmData the scene to update
     */
    void updateSfmData(sfmData::SfMData& sfmData) const;

    /**
     * @brief Get the blocks of all entries
     * @return the block pointers, ordered by pose id
     */
    std::vector<double*> getParameterBlocks();

    /**
     * @brief Add a pose block to the problem: constant if both parts are locked,
     * otherwise the locked part is kept constant with a subset manifold
     * @param[in,out] problem the ceres problem
     * @param[in] block pointer to a 6 doubles pose block
     * @param[in] lockRotation keep the rotation constant
     * @param[in] lockTranslation keep the translation constant
     */
    static void addBlockToProblem(ceres::Problem& problem, double* block, bool lockRotation, bool lockTranslation);

    /**
     * @brief Write a pose into a block as ceres angleAxis(3) + center(3)
     * @param[in] cameraPose the pose to convert
     * @param[out] block the destination block
     */
    static void fillBlock(const sfmData::CameraPose& cameraPose, Block& block);

    /**
     * @brief Check if a pose has an entry
     * @param[in] poseId the pose id
     * @return true if the pose is in the container
     */
    bool contains(IndexT poseId) const;

    /**
     * @brief Get the data block pointer of a pose, to give to ceres
     * @param[in] poseId the pose id, must be in the container
     * @return pointer to the 6 doubles pose block
     */
    double* get(IndexT poseId);

    /**
     * @brief Get the entry of a pose
     * @param[in] poseId the pose id, must be in the container
     * @return the entry
     */
    const Entry& at(IndexT poseId) const;

    /**
     * @brief Remove all entries, invalidating all block pointers
     */
    void clear();

  private:
    /// entries indexed by pose id
    std::map<IndexT, Entry> _entries;
};

}  // namespace sfm
}  // namespace aliceVision
