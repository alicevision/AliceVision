// This file is part of the AliceVision project.
// Copyright (c) 2020 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/track/Track.hpp>
#include <aliceVision/track/tracksUtils.hpp>
#include <aliceVision/stl/mapUtils.hpp>
#include <aliceVision/numeric/BoxStats.hpp>
#include <aliceVision/utils/Histogram.hpp>

namespace aliceVision {
namespace sfm {

/**
 * @brief Compute histogram of residual values between landmarks and features in all the views specified
 * @param[in] sfmData : scene containing the features and the landmarks
 * @param[out] out_stats : stats containing the residuals values
 * @param[out] out_histogram : histogram of the number of points for each residual value (0-4 px)
 * @param[in] specificViews: Limit stats to specific views. If empty, compute stats for all views.
 */
void computeResidualsHistogram(const sfmData::SfMData& sfmData,
                               BoxStats<double>& out_stats,
                               utils::Histogram<double>* out_histogram,
                               const std::set<IndexT>& specificViews = std::set<IndexT>());

/**
 * @brief Compute histogram of residual values between landmarks and features in all the views specified
 * @param[in] sfmData : scene containing the features and the landmarks
 * @param[in] specificViews: Limit stats to specific views. If empty, compute stats for all views.
 * @param[out] mean : mean of residuals
 * @param[out] median : median of residuals
 */
void computeResidualsMeanMedian(const sfmData::SfMData& sfmData,
                            double & mean,
                            double & median,
                            const std::set<IndexT>& specificViews = std::set<IndexT>());

/**
 * @brief Compute histogram of observations lengths
 * @param[in] sfmData: containing the observations
 * @param[out] out_stats: stats containing the observations lengths
 * @param[out] observationsLengthHistogram : histogram of the number of points for each observation length
 * @param[in] specificViews: Limit stats to specific views. If empty, compute stats for all views
 */
void computeObservationsLengthsHistogram(const sfmData::SfMData& sfmData,
                                         BoxStats<double>& out_stats,
                                         int& overallNbObservations,
                                         utils::Histogram<double>* observationsLengthHistogram,
                                         const std::set<IndexT>& specificViews = std::set<IndexT>());

/**
 * @brief Compute histogram of the number of landmarks per view
 * @param[in] sfmData: scene containing the views and the landmarks
 * @param[out] out_stats: stats containing the landmarks
 * @param[out] landmarksPerViewHistogram: histogram of the number of landmarks for each view
 */
void computeLandmarksPerViewHistogram(const sfmData::SfMData& sfmData,
                                      BoxStats<double>& out_stats,
                                      utils::Histogram<double>* landmarksPerViewHistogram);

/**
 * @brief Compute landmarks per view
 * @param[in] sfmData: scene containing the views and the landmarks
 * @param[out] outNbLandmarksPerView: vector containing the number of landmarks for each view
 */
void computeLandmarksPerView(const sfmData::SfMData& sfmData, std::vector<int>& outNbLandmarksPerView);

/**
 * @brief Compute features and matches per view
 * @param[in] sfmData: scene containing the views
 * @param[out] outFeatPerView: vector containing the number of features for each view
 * @param[out] outMatchPerView: vector containing the number of matches for each view
 */
void computeFeatMatchPerView(const sfmData::SfMData& sfmData, std::vector<size_t>& outFeatPerView, std::vector<size_t>& outMatchPerView);

/**
 * @brief Compute histogram of scale for observations
 * @param[in] sfmData: scene containing the observations
 * @param[out] outStats: stats containing the scales
 * @param[out] scaleHistogram: histogram of the number of points for each scale value
 * @param[in] specificViews: Limit stats to specific views. If empty, no stats computed
 */
void computeScaleHistogram(const sfmData::SfMData& sfmData,
                           BoxStats<double>& outStats,
                           utils::Histogram<double>* scaleHistogram,
                           const std::set<IndexT>& specificViews = std::set<IndexT>());

/**
 * @brief Compute different stats of residuals per view
 * @param[in] sfmData: scene containing the views and the landmarks
 * @param[in] nbViews: number of views to consider
 * @param[out] nbResidualsPerViewMin: vector containing the minimum residuals values
 * @param[out] nbResidualsPerViewMax: vector containing the max residuals values
 * @param[out] nbResidualsPerViewMean: vector containing the mean residuals values
 * @param[out] nbResidualsPerViewMedian: vector containing the median residuals values
 * @param[out] nbResidualsPerViewFirstQuartile: vector containing the first quartile residuals values
 * @param[out] nbResidualsPerViewThirdQuartile: vector containing the third quartile residuals values
 */
void computeResidualsPerView(const sfmData::SfMData& sfmData,
                             int& nbViews,
                             std::vector<double>& nbResidualsPerViewMin,
                             std::vector<double>& nbResidualsPerViewMax,
                             std::vector<double>& nbResidualsPerViewMean,
                             std::vector<double>& nbResidualsPerViewMedian,
                             std::vector<double>& nbResidualsPerViewFirstQuartile,
                             std::vector<double>& nbResidualsPerViewThirdQuartile);

/**
 * @brief Compute different stats of observations lengths per view
 * @param[in] sfmData: scene containing the views and the observations
 * @param[in] nbViews: number of views to consider
 * @param[out] nbObservationsLengthsPerViewMin : vector containing the minimum observations lengths
 * @param[out] nbObservationsLengthsPerViewMax: vector containing the maximum observations lengths
 * @param[out] nbObservationsLengthsPerViewMean: vector containing the mean observations lengths
 * @param[out] nbObservationsLengthsPerViewMedian: vector containing the median observations lengths
 * @param[out] nbResidualsPerViewFirstQuartile: vector containing the first quartile observations lengths
 * @param[out] nbResidualsPerViewThirdQuartile: vector containing the third quartile observations lengths
 */
void computeObservationsLengthsPerView(const sfmData::SfMData& sfmData,
                                       int& nbViews,
                                       std::vector<double>& nbObservationsLengthsPerViewMin,
                                       std::vector<double>& nbObservationsLengthsPerViewMax,
                                       std::vector<double>& nbObservationsLengthsPerViewMean,
                                       std::vector<double>& nbObservationsLengthsPerViewMedian,
                                       std::vector<double>& nbResidualsPerViewFirstQuartile,
                                       std::vector<double>& nbResidualsPerViewThirdQuartile);

/**
 * @brief Statistics on the residuals normalized by the feature scale
 * These are the units of the bundle adjustment cost and of the residual outlier filter
 */
struct NormalizedResidualStatistics
{
    /// minimum number of observations for a track to be considered long
    static constexpr std::size_t longTrackMinLength = 3;

    /// number of observations
    std::size_t nbObservations = 0;
    /// number of observations belonging to tracks of at least longTrackMinLength observations
    std::size_t nbObservationsLongTracks = 0;
    /// mean and quantiles of the residual norm
    double mean = 0.0;
    double median = 0.0;
    double p90 = 0.0;
    double p99 = 0.0;
    double max = 0.0;
    /// robust standard deviation (1.4826 * MAD) of the residual x and y components
    double sigma = 0.0;
    /// same as sigma, restricted to tracks of at least longTrackMinLength observations
    double sigmaLongTracks = 0.0;
    /// thresholds on the residual norm, and number of observations above each of them
    std::vector<double> thresholds;
    std::vector<std::size_t> nbAboveThresholds;
};

/**
 * @brief Compute statistics on the residuals normalized by the feature scale
 * Views without a defined pose or intrinsic are ignored
 * @param[in] sfmData : scene containing the features and the landmarks
 * @param[in] advancedStatistics : if false, only nbObservations, mean and median are computed
 * @param[in] thresholds : thresholds on the residual norm for which the number of observations above is counted (advanced only)
 * @param[in] specificViews: Limit stats to specific views. If empty, compute stats for all views.
 * @return the statistics
 */
NormalizedResidualStatistics computeNormalizedResidualStatistics(const sfmData::SfMData& sfmData,
                                                                 bool advancedStatistics,
                                                                 const std::vector<double>& thresholds = std::vector<double>(),
                                                                 const std::set<IndexT>& specificViews = std::set<IndexT>());

std::ostream& operator<<(std::ostream& os, const NormalizedResidualStatistics& statistics);

/**
 * @brief Parameters of an intrinsic and the number of reconstructed views using it
 */
struct IntrinsicUsage
{
    IndexT intrinsicId = UndefinedIndexT;
    /// number of views with a defined pose using this intrinsic
    std::size_t nbViews = 0;
    Vec2 focal = Vec2::Zero();
    Vec2 offset = Vec2::Zero();
    bool locked = false;
};

/**
 * @brief List the scale/offset intrinsics used by views with a defined pose
 * @param[in] sfmData : scene containing the views and the intrinsics
 * @return one entry per used intrinsic, ordered by intrinsic id
 */
std::vector<IntrinsicUsage> computeIntrinsicsUsage(const sfmData::SfMData& sfmData);

std::ostream& operator<<(std::ostream& os, const IntrinsicUsage& usage);

}  // namespace sfm
}  // namespace aliceVision
