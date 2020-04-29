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
 * @brief Compute histogram of residual values
 * @param[in] sfmData
 * @param[out] out_stats
 * @param[out] out_histogram
 * @param[in] specificViews: Limit stats to specific views. If empty, compute stats for all views.
 */
void computeResidualsHistogram(const sfmData::SfMData& sfmData, BoxStats<double>& out_stats, Histogram<double>* out_histogram, const std::set<IndexT>& specificViews = std::set<IndexT>());

/**
 * @brief Compute histogram of observations lengths
 * @param[in] sfmData
 * @param[out] out_stats
 * @param[out] observationsLengthHistogram
 * @param[in] specificViews: Limit stats to specific views. If empty, compute stats for all views
 */
void computeObservationsLengthsHistogram(const sfmData::SfMData& sfmData, BoxStats<double>& out_stats, int& overallNbObservations, Histogram<double>* observationsLengthHistogram, const std::set<IndexT>& specificViews = std::set<IndexT>());

/**
 * @brief Compute histogram of landmarks per view
 * @param[in] sfmData
 * @param[out] out_stats
 * @param[out] landmarksPerViewHistogram
 */
void computeLandmarksPerViewHistogram(const sfmData::SfMData& sfmData, BoxStats<double>& out_stats, Histogram<double>* landmarksPerViewHistogram);

/**
 * @brief computeLandmarksPerView
 * @param[in] sfmData
 * @param[out] out_nbLandmarksPerView
 */
void computeLandmarksPerView(const sfmData::SfMData& sfmData, std::vector<int>& out_nbLandmarksPerView);

/**
 * @brief computeFeatMatchPerView
 * @param[in] sfmData
 * @param[out] out_featPerView
 * @param[out] out_matchPerView
 */
void computeFeatMatchPerView(const sfmData::SfMData& sfmData, std::vector<size_t>& out_featPerView, std::vector<size_t>& out_matchPerView);

/**
 * @brief Compute histogram of scale for features and observations
 * @param[in] sfmData
 * @param[in] features
 * @param[out] out_stats
 * @param[out] scaleHistogram
 * @param[in] specificViews: Limit stats to specific views. If empty, no stats computed
 */
void computeScaleHistogram(const sfmData::SfMData& sfmData, BoxStats<double>& out_stats, Histogram<double>* scaleHistogram, const std::set<IndexT>& specificViews = std::set<IndexT>());

/**
 * @brief Compute histogram of residuals per view
 * @param[in] sfmData
 */
void computeResidualsPerView(const sfmData::SfMData& sfmData, int& nbViews, std::vector<double>& nbResidualsPerViewMin,
                                      std::vector<double>& nbResidualsPerViewMax, std::vector<double>& nbResidualsPerViewMean,
                                      std::vector<double>& nbResidualsPerViewMedian, std::vector<double>& nbResidualsPerViewFirstQuartile,
                                      std::vector<double>& nbResidualsPerViewThirdQuartile);

/**
 * @brief Compute histogram of observations lengths per view
 * @param[in] sfmData
 */
void computeObservationsLengthsPerView(const sfmData::SfMData& sfmData, int& nbViews, std::vector<double>& nbObservationsLengthsPerViewMin,
                                      std::vector<double>& nbObservationsLengthsPerViewMax, std::vector<double>& nbObservationsLengthsPerViewMean,
                                      std::vector<double>& nbObservationsLengthsPerViewMedian, std::vector<double>& nbResidualsPerViewFirstQuartile,
                                      std::vector<double>& nbResidualsPerViewThirdQuartile);

}
}

