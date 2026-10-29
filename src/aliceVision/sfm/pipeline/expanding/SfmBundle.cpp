// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "SfmBundle.hpp"

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/sfm/sfmFilters.hpp>
#include <aliceVision/sfm/utils/poseFilter.hpp>
#include <aliceVision/sfm/sfmStatistics.hpp>
#include <aliceVision/sfm/bundle/BundleAdjustmentCeres.hpp>
#include <aliceVision/sfm/pipeline/expanding/DistanceWeighting.hpp>

#include <sstream>

namespace aliceVision {
namespace sfm {

bool SfmBundle::process(sfmData::SfMData & sfmData, const track::TracksHandler & tracksHandler, const std::set<IndexT> & viewIds)
{
    ALICEVISION_LOG_INFO("SfmBundle::process start");

    BundleAdjustmentCeres::CeresOptions options(true, true, _maxIterationCount, _lossParameter);

    BundleAdjustment::ERefineOptions refineOptions = BundleAdjustment::REFINE_NONE;

    refineOptions |= BundleAdjustment::REFINE_ROTATION;
    refineOptions |= BundleAdjustment::REFINE_CENTER;

    if (_isStructureRefinementEnabled)
    {
        refineOptions |= BundleAdjustment::REFINE_STRUCTURE;
    }

    refineOptions |= BundleAdjustment::REFINE_INTRINSICS_ALL;

    if (_bundleTemporalConstraint)
    {
        refineOptions |= BundleAdjustment::REFINE_TEMPORAL_SMOOTHNESS_CONSTRAINT;
        options.temporalConstraintParams = _tempConstrParams;

        // Fill in the blanks in the views by interpolating new poses (position and orientation)

        sfm::poseFilter poseFilter;
        if (!poseFilter.interpolateMissingPoses(sfmData, false))
        {
            return false;
        }
    }

    options.setSparseBA();
    BundleAdjustmentCeres bundleObject(options, _minNbCamerasToRefinePrincipalPoint);

    const bool traceEnabled = system::isTraceEnabled();
    int pass = 0;

    //Repeat until nothing change
    do
    {
        ALICEVISION_LOG_TRACE("SfmBundle pass " << pass);
        pass++;

        // Compute statistics on residuals, normalized by the feature scale like the thresholds
        // of the Huber loss and of the residual outlier filter
        const std::vector<double> thresholds = {_lossParameter, _maxReprojectionError};
        NormalizedResidualStatistics residuals = computeNormalizedResidualStatistics(sfmData, traceEnabled, thresholds);
        ALICEVISION_LOG_INFO("SfmBundle normalized statistics before minimization - mean " << residuals.mean << ", median " << residuals.median);

        if (!initializeIteration(sfmData, tracksHandler, viewIds))
        {
            return false;
        }

        if (traceEnabled)
        {
            logTraceStatistics(sfmData, residuals, "before minimization");
        }

        const bool success = bundleObject.adjust(sfmData, refineOptions);
        if (!success)
        {
            return false;
        }

        residuals = computeNormalizedResidualStatistics(sfmData, traceEnabled, thresholds);
        ALICEVISION_LOG_INFO("SfmBundle normalized statistics after minimization - mean " << residuals.mean << ", median " << residuals.median);

        if (traceEnabled)
        {
            ALICEVISION_LOG_TRACE(bundleObject.getStatistics().toString());
            logTraceStatistics(sfmData, residuals, "after minimization");
        }
    }
    while (cleanup(sfmData));

    ALICEVISION_LOG_INFO("SfmBundle::process end");
    return true;
}

void SfmBundle::logTraceStatistics(const sfmData::SfMData & sfmData, const NormalizedResidualStatistics & residuals, const std::string & label) const
{
    ALICEVISION_LOG_TRACE("SfmBundle residuals " << label << " (normalized by feature scale, thresholds: loss parameter "
                          << _lossParameter << ", max reprojection error " << _maxReprojectionError << "):\n" << residuals);

    for (const IntrinsicUsage & usage : computeIntrinsicsUsage(sfmData))
    {
        ALICEVISION_LOG_TRACE("SfmBundle " << label << ", " << usage);
    }
}

bool SfmBundle::cleanup(sfmData::SfMData & sfmData)
{
    // Remove observations with too large residuals
    const std::size_t nbOutliersResidualErr = removeOutliersWithPixelResidualError(sfmData, _featureConstraint, _maxReprojectionError, _minTrackLength);

    // Remove landmarks without enough observed parallax
    const std::size_t nbOutliersAngleErr = removeOutliersWithAngleError(sfmData, _minAngleForLandmark);

    // Remove constraints which are too far away from landmark
    const std::size_t nbOutliersConstraints =  removeConstraints(sfmData, _maxConstraintDistance);

    // Remove poses without enough observations in an interactive fashion
    const std::size_t nbOutliers = nbOutliersResidualErr + nbOutliersAngleErr;
    std::set<IndexT> removedViewsIdIteration;
    bool somethingErased = false;
    if (!_bundleTemporalConstraint)
    {
        somethingErased = eraseUnstablePosesAndObservations(sfmData, _minPointsPerPose, _minTrackLength, &removedViewsIdIteration);
    }

    bool somethingChanged = /*somethingErased || */(nbOutliers > _bundleAdjustmentMaxOutlier) || (nbOutliersConstraints > 0);

    if (!removedViewsIdIteration.empty())
    {
        std::stringstream ss;
        for (const IndexT viewId : removedViewsIdIteration)
        {
            ss << " " << viewId;
        }
        ALICEVISION_LOG_TRACE("SfmBundle::cleanup removed " << removedViewsIdIteration.size() << " unstable views:" << ss.str());
    }

    ALICEVISION_LOG_TRACE("SfmBundle::cleanup another pass needed: " << somethingChanged
                          << " (outliers " << nbOutliers << " > max " << _bundleAdjustmentMaxOutlier << ": " << (nbOutliers > _bundleAdjustmentMaxOutlier)
                          << ", constraints removed: " << nbOutliersConstraints << ")");

    ALICEVISION_LOG_INFO("SfmBundle::cleanup : ");
    ALICEVISION_LOG_INFO(" - nbOutliersResidualErr : " << nbOutliersResidualErr);
    ALICEVISION_LOG_INFO(" - nbOutliersAngleErr : " << nbOutliersAngleErr);
    ALICEVISION_LOG_INFO(" - nbOutliersConstraints : " << nbOutliersConstraints);
    ALICEVISION_LOG_INFO(" - somethingErased : " << somethingErased);
    ALICEVISION_LOG_INFO(" - somethingChanged : " << somethingChanged);

    return somethingChanged;
}

bool SfmBundle::initializeIteration(sfmData::SfMData & sfmData, const track::TracksHandler & tracksHandler, const std::set<IndexT> & viewIds)
{
    const bool enableLocalStrategy = _lbaPolicy && (sfmData.getPoses().size() >= _minNbCamerasLBA);

    ALICEVISION_LOG_TRACE("SfmBundle local strategy: " << (enableLocalStrategy ? "enabled" : "disabled")
                          << " (" << sfmData.getPoses().size() << " poses, minimum " << _minNbCamerasLBA
                          << ", policy " << (_lbaPolicy ? "set" : "not set") << ")");

    // Fall back to a full bundle if the local strategy cannot be set up
    if (!enableLocalStrategy || !_lbaPolicy->build(sfmData, tracksHandler, viewIds))
    {
        if (enableLocalStrategy)
        {
            ALICEVISION_LOG_TRACE("SfmBundle local strategy could not be built, falling back to a full bundle");
        }
        sfmData.resetParameterStates();
    }

    if (_enableObservationsWeighting)
    {
        weightObservationsFromDistance(sfmData, 15, 5.0);
    }

    return true;
}

} // namespace sfm
} // namespace aliceVision