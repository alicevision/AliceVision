// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/config.hpp>
#include <aliceVision/feature/PointFeature.hpp>
#include <aliceVision/feature/RegionsPerView.hpp>
#include <aliceVision/matching/MatchesCollections.hpp>
#include <aliceVision/matchingImageCollection/GeometricFilterMatrix.hpp>
#include <aliceVision/system/ProgressDisplay.hpp>
#include <aliceVision/matchingImageCollection/GeometricInfo.hpp>
#include <aliceVision/system/Logger.hpp>

#include <map>
#include <random>
#include <vector>

namespace aliceVision {
namespace matchingImageCollection {

using namespace aliceVision::matching;

/**
 * @brief Perform robust model estimation (with optional guided_matching)
 * or all the pairs and regions correspondences contained in the putativeMatches set.
 * Allow to keep only geometrically coherent matches.
 * It discards pairs that do not lead to a valid robust model estimation.
 * @param[out] out_geometricMatches
 * @param[in] sfmData
 * @param[in] regionsPerView
 * @param[in] functor
 * @param[in] putativeMatches
 * @param[in] guidedMatching
 * @param[in] distanceRatio
 * @param[in] randomNumberGenerator
 */
template<typename GeometryFunctor>
void robustModelEstimation(PairwiseMatches& out_geometricMatches,
                           const sfmData::SfMData& sfmData,
                           const feature::RegionsPerView& regionsPerView,
                           const GeometryFunctor& functor,
                           const PairwiseMatches& putativeMatches,
                           std::mt19937& randomNumberGenerator,
                           const bool guidedMatching = false,
                           const double distanceRatio = 0.6)
{
    out_geometricMatches.clear();

    auto progressDisplay = system::createConsoleProgressDisplay(putativeMatches.size(), "Robust Model Estimation\n");

#pragma omp parallel for schedule(dynamic)
    for (int i = 0; i < (int)putativeMatches.size(); ++i)
    {
        PairwiseMatches::const_iterator iter = putativeMatches.begin();
        std::advance(iter, i);

        const Pair currentPair = iter->first;
        const MatchesPerDescType& putativeMatchesPerType = iter->second;
        const Pair& imagePair = iter->first;

        // apply the geometric filter (robust model estimation)
        {
            MatchesPerDescType inliers;
            GeometryFunctor geometricFilter = functor;  // use a copy since we are in a multi-thread context
            const EstimationStatus state =
              geometricFilter.geometricEstimation(sfmData, regionsPerView, imagePair, putativeMatchesPerType, randomNumberGenerator, inliers);
            if (state.hasStrongSupport)
            {
                if (guidedMatching)
                {
                    MatchesPerDescType guidedGeometricInliers;
                    geometricFilter.Geometry_guided_matching(sfmData, regionsPerView, imagePair, distanceRatio, guidedGeometricInliers);
                    std::swap(inliers, guidedGeometricInliers);
                }

#pragma omp critical
                {
                    out_geometricMatches.emplace(currentPair, std::move(inliers));
                }
            }
        }
        ++progressDisplay;
    }
}

/**
 * @brief Perform robust model estimation (with optional guided_matching)
 * or all the pairs and regions correspondences contained in the putativeMatches set.
 * Allow to keep only geometrically coherent matches.
 * It discards pairs that do not lead to a valid robust model estimation.
 * @param[out] out_geometricInfos
 * @param[in] sfmData
 * @param[in] regionsPerView
 * @param[in] functor
 * @param[in] putativeMatches
 * @param[in] randomNumberGenerator
 * @param minMatches Minimum number of matches to accept a pair of images (or 0 to disable limit).
 */
template<typename GeometryFunctor>
void robustModelEstimation(PairwiseGeometricInfo& out_geometricInfos,
                           const sfmData::SfMData& sfmData,
                           const feature::RegionsPerView& regionsPerView,
                           const GeometryFunctor& functor,
                           const PairwiseMatches& putativeMatches,
                           std::mt19937& randomNumberGenerator,
                           size_t minMatches = 0)
{
    out_geometricInfos.clear();

    auto progressDisplay = system::createConsoleProgressDisplay(putativeMatches.size(), "Robust Model Estimation\n");

#pragma omp parallel for schedule(dynamic)
    for (int i = 0; i < (int)putativeMatches.size(); ++i)
    {
        PairwiseMatches::const_iterator iter = putativeMatches.begin();
        std::advance(iter, i);

        const Pair currentPair = iter->first;
        const MatchesPerDescType& putativeMatchesPerType = iter->second;
        const Pair& imagePair = iter->first;

        // apply the geometric filter (robust model estimation)
        {
            MatchesPerDescType inliers;
            GeometryFunctor geometricFilter = functor;  // use a copy since we are in a multi-thread context
            const EstimationStatus state = geometricFilter.geometricEstimation(sfmData, regionsPerView, imagePair, putativeMatchesPerType, randomNumberGenerator, inliers);
            
            ALICEVISION_LOG_DEBUG("Pair " << imagePair.first << ", " << imagePair.second << " : " << inliers.getNbAllMatches() << " inliers");

            //Filter out pairs that do not have enough inliers
            if (minMatches > 0 && inliers.getNbAllMatches() < minMatches)
            {
                ALICEVISION_LOG_DEBUG("Filtered by minMatches threshold");
                continue;
            }

            if (state.hasStrongSupport)
            {
#pragma omp critical
                {
                    PairGeometricInfo info;
                    info.model = geometricFilter.getMatrix();
                    info.inliers = inliers.getNbAllMatches();
                    info.threshold = geometricFilter.m_dPrecision_robust;
                    info.type = geometricFilter.getType();
                    out_geometricInfos.emplace(currentPair, info);
                }
            }
        }
        ++progressDisplay;
    }
}

/**
 * @brief removePoorlyOverlappingImagePairs Removes image pairs from the given list of geometric
 *  matches that have poor overlap according to the supplied criteria.
 * @param[in,out] geometricMatches List of geometric matches to clean up
 * @param putativeMatches List of putative matches
 * @param minimumRatio Minimum ratio of geometric to putative matches for image pair
 * @param minimumCount Minimum count of geometric matches for image pair
 */
void removePoorlyOverlappingImagePairs(PairwiseMatches& geometricMatches,
                                       const PairwiseMatches& putativeMatches,
                                       float minimumRatio,
                                       std::size_t minimumGeometricCount);

}  // namespace matchingImageCollection
}  // namespace aliceVision
