// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/types.hpp>
#include <aliceVision/camera/IntrinsicBase.hpp>
#include <aliceVision/sfmData/SfMData.hpp>

#include <map>
#include <memory>
#include <vector>

#include <ceres/ordered_groups.h>

namespace ceres {
class Problem;
}

namespace aliceVision {
namespace sfm {

/**
 * @brief Storage for the intrinsics (and distortions) parameter blocks given to ceres.
 *
 * Each intrinsic is stored with a copy of its object (used by the cost functions during the solve),
 * its parameters block, its optional distortion block and the locks telling what must stay constant.
 * Blocks are stored in a std::map so their addresses stay valid while ceres uses them.
 *
 * The container also owns a constant fake distortion block, used for intrinsics without distortion
 * because cost functions always expect a distortion parameter block.
 *
 * Typical usage: fill(), addToProblem(), updateObjects() on each new evaluation point,
 * then updateSfmData() after the solve.
 */
class IntrinsicsContainer
{
  public:
    /**
     * @brief What should be refined, used by fill() to compute the locks
     */
    struct Options
    {
        /// refine the focal length
        bool refineFocal = false;
        /// refine the distortion parameters
        bool refineDistortion = false;
        /// always refine the optical center
        bool refineOpticalCenterAlways = false;
        /// refine the optical center only if the intrinsic is used by enough views
        bool refineOpticalCenterIfEnoughData = false;
        /// minimum number of reconstructed views to refine the optical center with refineOpticalCenterIfEnoughData
        int minNbImagesToRefineOpticalCenter = 3;
    };

    /**
     * @brief The parameter blocks of an intrinsic with their locks
     */
    struct Entry
    {
        /// copy of the sfmData intrinsic, updated during the solve
        std::shared_ptr<camera::IntrinsicBase> object;
        /// intrinsic parameters, as returned by IntrinsicBase::getParameters()
        std::vector<double> intrinsicBlock;
        /// distortion parameters, empty if the intrinsic has no distortion
        std::vector<double> distortionBlock;

        /// if true, the whole intrinsic block is kept constant
        bool lockIntrinsic = true;
        /// if true, the focal length is kept constant
        bool lockFocal = true;
        /// if true, the focal ratio (fx/fy) is kept constant
        bool lockRatio = true;
        /// if true, the optical center is kept constant
        bool lockCenter = true;
        /// if true, the distortion block is kept constant
        bool lockDistortion = true;
    };

    /**
     * @brief Replace the content with the used, non ignored, scale offset intrinsics of sfmData
     * and compute their locks from their state and the options.
     * Clears the container first, which invalidates all previously returned block pointers.
     * @param[in] sfmData the scene providing the intrinsics
     * @param[in] options what should be refined
     */
    void fill(const sfmData::SfMData& sfmData, const Options& options);

    /**
     * @brief Add all entries as parameter blocks of the problem, applying their locks and bounds.
     * Also adds the constant fake distortion block.
     * @param[in,out] problem the ceres problem
     * @param[in] useFocalPrior bound the focal around its initial value when it is known
     */
    void addToProblem(ceres::Problem& problem, bool useFocalPrior);

    /**
     * @brief Add all intrinsic and distortion blocks (including the fake distortion block) to a group of the linear solver ordering
     * @param[in,out] ordering the ceres parameter blocks ordering
     * @param[in] group the elimination group (lower groups are eliminated first)
     */
    void addToOrdering(ceres::ParameterBlockOrdering& ordering, int group);

    /**
     * @brief Copy the solved blocks back into the refined intrinsics of sfmData
     * @param[in,out] sfmData the scene to update
     */
    void updateSfmData(sfmData::SfMData& sfmData) const;

    /**
     * @brief Copy the current blocks into the intrinsic objects used by the cost functions
     */
    void updateObjects();

    /**
     * @brief Get the intrinsic blocks of all entries (not the distortions)
     * @return the block pointers, ordered by intrinsic id
     */
    std::vector<double*> getParameterBlocks();

    /**
     * @brief Check if an intrinsic has an entry
     * @param[in] intrinsicId the intrinsic id
     * @return true if the intrinsic is in the container
     */
    bool contains(IndexT intrinsicId) const;

    /**
     * @brief Get the intrinsic data block pointer, to give to ceres
     * @param[in] intrinsicId the intrinsic id, must be in the container
     * @return pointer to the intrinsic parameters block
     */
    double* getIntrinsicBlock(IndexT intrinsicId);

    /**
     * @brief Get the distortion data block pointer, to give to ceres
     * @param[in] intrinsicId the intrinsic id, must be in the container
     * @return pointer to the distortion block, or to the constant fake block if the intrinsic has no distortion
     */
    double* getDistortionBlock(IndexT intrinsicId);

    /**
     * @brief Get the intrinsic object copy used by the cost functions
     * @param[in] intrinsicId the intrinsic id, must be in the container
     * @return the intrinsic object
     */
    const std::shared_ptr<camera::IntrinsicBase>& getObject(IndexT intrinsicId) const;

    /**
     * @brief Remove all entries, invalidating all block pointers
     */
    void clear();

  private:
    /// entries indexed by intrinsic id
    std::map<IndexT, Entry> _entries;

    /// constant block used in place of a distortion, to trick ceres limitations
    double _fakeDistortionBlock = 0.0;
};

}  // namespace sfm
}  // namespace aliceVision
