// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/types.hpp>
#include <aliceVision/track/Track.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfm/pipeline/expanding/LocalizationValidationPolicy.hpp>
#include <memory>

namespace aliceVision {
namespace sfm {

class SfmResection
{
public:
    using uptr = std::unique_ptr<SfmResection>;

public:
    void setMaxIterations(size_t maxIterations)
    {
        _maxIterations = maxIterations;
    }

    /**
     * @brief set the maximal error allowed for ransac resection module
     * @param error the error value or <= 0 for automatic decision
    */
    void setResectionMaxError(double error)
    {
        _precision = error;
        if (_precision <= 0.0)
        {
            _precision = std::numeric_limits<double>::infinity();
        }
    }

    void setValidationPolicy(LocalizationValidationPolicy::uptr & policy)
    {
        _validationPolicy = std::move(policy);
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
     * @param inliersCount number of inliers for this resection
     * @return false if a critical error occurred
    */
    bool processView(
                const sfmData::SfMData & sfmData,
                const track::TracksMap & tracks,
                const track::TracksPerView & map_tracksPerView, 
                std::mt19937 &randomNumberGenerator,
                const IndexT viewId,
                Eigen::Matrix4d & updatedPose,
                double & updatedThreshold,
                size_t & inliersCount
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

    bool internalNodal(
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
            std::shared_ptr<camera::IntrinsicBase> & intrinsics,
            bool isNodal,
            double & errorMax
        );

private:
    std::unique_ptr<LocalizationValidationPolicy> _validationPolicy;

private:
    double _precision = std::numeric_limits<double>::infinity();
    size_t _maxIterations = 1024;
};

} // namespace sfm
} // namespace aliceVision