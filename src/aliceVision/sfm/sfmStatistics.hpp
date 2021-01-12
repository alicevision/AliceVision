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
void computeResidualsHistogram(const sfmData::SfMData& sfmData, BoxStats<double>& out_stats, utils::Histogram<double>* out_histogram, const std::set<IndexT>& specificViews = std::set<IndexT>());

/**
 * @brief Compute histogram of observations lengths
 * @param[in] sfmData: containing the observations
 * @param[out] out_stats: stats containing the observations lengths
 * @param[out] observationsLengthHistogram : histogram of the number of points for each observation length
 * @param[in] specificViews: Limit stats to specific views. If empty, compute stats for all views
 */
void computeObservationsLengthsHistogram(const sfmData::SfMData& sfmData, BoxStats<double>& out_stats, int& overallNbObservations, utils::Histogram<double>* observationsLengthHistogram, const std::set<IndexT>& specificViews = std::set<IndexT>());

/**
 * @brief Compute histogram of the number of landmarks per view
 * @param[in] sfmData: scene containing the views and the landmarks
 * @param[out] out_stats: stats containing the landmarks
 * @param[out] landmarksPerViewHistogram: histogram of the number of landmarks for each view
 */
void computeLandmarksPerViewHistogram(const sfmData::SfMData& sfmData, BoxStats<double>& out_stats, utils::Histogram<double>* landmarksPerViewHistogram);

/**
 * @brief Compute landmarks per view
 * @param[in] sfmData: scene containing the views and the landmarks
 * @param[out] out_nbLandmarksPerView: vector containing the number of landmarks for each view
 */
void computeLandmarksPerView(const sfmData::SfMData& sfmData, std::vector<int>& out_nbLandmarksPerView);

/**
 * @brief Compute features and matches per view
 * @param[in] sfmData: scene containing the views
 * @param[out] out_featPerView: vector containing the number of features for each view
 * @param[out] out_matchPerView: vector containing the number of matches for each view
 */
void computeFeatMatchPerView(const sfmData::SfMData& sfmData, std::vector<size_t>& out_featPerView, std::vector<size_t>& out_matchPerView);

/**
 * @brief Compute histogram of scale for observations
 * @param[in] sfmData: scene containing the observations
 * @param[out] out_stats: stats containing the scales
 * @param[out] scaleHistogram: histogram of the number of points for each scale value
 * @param[in] specificViews: Limit stats to specific views. If empty, no stats computed
 */
void computeScaleHistogram(const sfmData::SfMData& sfmData, BoxStats<double>& out_stats, utils::Histogram<double>* scaleHistogram, const std::set<IndexT>& specificViews = std::set<IndexT>());

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
void computeResidualsPerView(const sfmData::SfMData& sfmData, int& nbViews, std::vector<double>& nbResidualsPerViewMin,
                                      std::vector<double>& nbResidualsPerViewMax, std::vector<double>& nbResidualsPerViewMean,
                                      std::vector<double>& nbResidualsPerViewMedian, std::vector<double>& nbResidualsPerViewFirstQuartile,
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
void computeObservationsLengthsPerView(const sfmData::SfMData& sfmData, int& nbViews, std::vector<double>& nbObservationsLengthsPerViewMin,
                                      std::vector<double>& nbObservationsLengthsPerViewMax, std::vector<double>& nbObservationsLengthsPerViewMean,
                                      std::vector<double>& nbObservationsLengthsPerViewMedian, std::vector<double>& nbResidualsPerViewFirstQuartile,
                                      std::vector<double>& nbResidualsPerViewThirdQuartile);

}
}

