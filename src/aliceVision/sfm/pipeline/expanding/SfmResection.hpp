// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/types.hpp>
#include <aliceVision/track/Track.hpp>
#include <aliceVision/sfmData/SfMData.hpp>

namespace aliceVision {
namespace sfm {

class SfmResection
{
public:
    SfmResection(size_t maxIterations, double precision) 
    : _maxIterations(maxIterations),
    _precision(precision)
    {

    }

    /**
     * Process resection
     * @param sfmData the actual state of the sfm
     * @param tracks the list of tracks for this scene
     * @param tracksPerView the list of tracks organized per views for the whole scene
     * @param randomNumberGenerator random number generator object
     * @param viewId the view id to process
     * @param updatedPose output estimated pose
     * @param updatedThreshold estimated threshold
     * @return false if a critical error occured
    */
    bool processView(
                const sfmData::SfMData & sfmData,
                const track::TracksMap & tracks,
                const track::TracksPerView & map_tracksPerView, 
                std::mt19937 &randomNumberGenerator,
                const IndexT viewId,
                Eigen::Matrix4d & updatedPose,
                double & updatedThreshold
            );

private:
    bool internalResection(
            std::shared_ptr<camera::IntrinsicBase> & intrinsic,
            std::mt19937 &randomNumberGenerator,
            const std::vector<Eigen::Vector3d> & structure,
            const std::vector<Eigen::Vector2d> & observations,
            const std::vector<feature::EImageDescriberType> & featureTypes,
            Eigen::Matrix4d & pose,
            std::vector<size_t> & inliers,
            double &errorMax
        );

    bool internalRefinement(
            const std::vector<Eigen::Vector3d> & structure,
            const std::vector<Eigen::Vector2d> & observations,
            const std::vector<size_t> & inliers,
            Eigen::Matrix4d & pose, 
            std::shared_ptr<camera::IntrinsicBase> & intrinsics
        );

private:
    double _precision;
    size_t _maxIterations;
};

} // namespace sfm
} // namespace aliceVision