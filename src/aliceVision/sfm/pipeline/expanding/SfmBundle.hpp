// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/types.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfm/bundle/BundleAdjustment.hpp>
#include <aliceVision/track/TracksHandler.hpp>
#include <aliceVision/sfm/pipeline/expanding/LbaPolicy.hpp>
#include <aliceVision/sfm/pipeline/expanding/ExpansionHistory.hpp>

namespace aliceVision {
namespace sfm {

class SfmBundle
{
public:
    using uptr = std::unique_ptr<SfmBundle>;

public:
    /**
     * @brief Process bundle
     * @param sfmData the scene description to optimize
     * @param tracksHandler the tracks manager
     * @param viewIds the set of views to bundle
     * @return false if an error occurred
    */
    bool process(sfmData::SfMData & sfmData, const track::TracksHandler & tracksHandler, const std::set<IndexT> & viewIds);

    /**
     * @brief setup the expansion chunk handler
     * @param expansionChunk a unique ptr. Ownership will be taken
     */
    void setLbaPolicyHandler(LbaPolicy::uptr & lbaPolicy)
    {
        _lbaPolicy = std::move(lbaPolicy);
    }

    void setBundleAdjustmentMaxOutlier(size_t bundleAdjustmentMaxOutlier)
    {
        _bundleAdjustmentMaxOutlier = bundleAdjustmentMaxOutlier;
    }

    void setTemporalConstraintParams(aliceVision::sfm::TemporalConstraintParams tempConstrParams, bool bundleTemporalConstraint)
    {
        _tempConstrParams = tempConstrParams;
        _bundleTemporalConstraint = bundleTemporalConstraint;
    }

    /**
     * @brief set the minimal allowed parallax degree after bundle (Or the landmark will be removed)
     * @param angle the angle in DEGREES
    */
    void setMinAngleLandmark(double angle)
    {
        _minAngleForLandmark = angle;
    }

    /**
     * @brief set the maximal allowed error in pixels (Or the landmark will be removed)
     * @param error the error in pixels
    */
    void setMaxReprojectionError(double error)
    {
        _maxReprojectionError = error;
    }

    /**
     * @brief set the Minimal number of connected views to refine an intrinsic principal point
     * @param count the number of views
    */
    void setMinNbCamerasToRefinePrincipalPoint(size_t count)
    {
        _minNbCamerasToRefinePrincipalPoint = count;
    }

    /**
     * @brief set the maximum number of solver iterations
     * @param maxIterationCount the number of iterations
    */
    void setMaxIterationCount(unsigned int maxIterationCount)
    {
        _maxIterationCount = maxIterationCount;
    }

    /**
     * @brief Set whether to enable structure refinement in bundle adjustment
     * @param flag true to enable structure refinement, false to disable it
    */
    void setIsStructureRefinementEnabled(bool flag)
    {
        _isStructureRefinementEnabled = flag;
    }

    /**
     * @brief Set whether to enable density weighting of observations
     * @param flag true to enable observations density weighting, false to disable it
    */
    void setIsObservationsDensityWeightingEnabled(bool flag)
    {
        _enableObservationsDensityWeighting = flag;
    }

    /**
     * @brief Set whether to enable track weighting of observations
     * @param flag true to enable observations track weighting, false to disable it
    */
    void setIsObservationsTrackWeightingEnabled(bool flag)
    {
        _enableObservationsTrackWeighting = flag;
    }

    /**
     * @brief Set the fading size used to weight observations based on their temporal position within the track
     * @param fadingSize the number of frames used for the fading (0 means no fading)
    */
    void setObservationsTrackWeightingFadingSize(int fadingSize)
    {
        _obsWeightFadingSize = fadingSize;
    }

    /**
     * @brief Set the weight to give to long tracks vs. the reference weight of 1 given to shorter tracks
     * @param maxWeight the weight to give to long tracks
    */
    void setTrackLengthMaxWeight(double maxWeight)
    {
        _trackLengthMaxWeight = maxWeight;
    }

    /**
     * @brief Set the number of observations under which a landmark is given a weight of 1
     * @param lowerThreshold the lower threshold used to weight observations based on the number of observations of the landmark
    */
    void setTrackLengthLowerThreshold(int lowerThreshold)
    {
        _trackLengthLowerThreshold = lowerThreshold;
    }

    /**
     * @brief Set the number of observations above which a landmark is given the maximum weight
     * @param upperThreshold the upper threshold used to weight observations based on the number of observations of the landmark
    */
    void setTrackLengthUpperThreshold(int upperThreshold)
    {
        _trackLengthUpperThreshold = upperThreshold;
    }

    /**
     * @brief Get the number of valid points per pose required
     * @return the threshold used to discriminate a valid pose
    */
    size_t getMinPointsPerPose() const
    {
        return _minPointsPerPose;
    }

    /**
     * @brief Set the number of valid points per pose required
     * @param minPointsPerPose the threshold used to discriminate a valid pose
    */
    void setMinPointsPerPose(size_t minPointsPerPose)
    {
        _minPointsPerPose = minPointsPerPose;
    }

private:
    /**
     * Initialize bundle properties
     * @param sfmData the input sfmData
     * @param tracksHandler the tracks handler
     * @param viewIds the set of view ids which we are sure we want to be estimated
    */
    bool initializeIteration(sfmData::SfMData & sfmData, const track::TracksHandler & tracksHandler, const std::set<IndexT> & viewIds);

    /**
     * Cleanup sfmData
     * @param sfmData the scene to clean
     * @return true if enough change occurred during the cleaning
    */
    bool cleanup(sfmData::SfMData & sfmData);

private:
    LbaPolicy::uptr _lbaPolicy;

private:

    EFeatureConstraint _featureConstraint = EFeatureConstraint::SCALE;
    double _maxReprojectionError = 4.0;
    double _minAngleForLandmark = 2.0;
    double _maxConstraintDistance = 1.0;
    size_t _minTrackLength = 1;
    size_t _minPointsPerPose = 30;
    size_t _bundleAdjustmentMaxOutlier = 50;
    size_t _minNbCamerasToRefinePrincipalPoint = 3;
    unsigned int _maxIterationCount = 50;
    bool _useLBA = true;
    bool _bundleTemporalConstraint = false;
    aliceVision::sfm::TemporalConstraintParams _tempConstrParams;
    size_t _minNbCamerasLBA = 100;
    size_t _LBAGraphDistanceLimit = 1;
    size_t _LBAMinNbOfMatches = 50;
    bool _isStructureRefinementEnabled = true;
    bool _enableObservationsDensityWeighting = false;
    bool _enableObservationsTrackWeighting = false;
    int _obsWeightFadingSize = 5;
    double _trackLengthMaxWeight = 5.;
    int _trackLengthLowerThreshold = 5;
    int _trackLengthUpperThreshold = 20;
};

} // namespace sfm
} // namespace aliceVision
