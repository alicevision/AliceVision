// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "matchesFiltering.hpp"

namespace aliceVision {
namespace matching {

void sortMatches_byFeaturesScale(const aliceVision::matching::IndMatches& inputMatches,
                                 const aliceVision::feature::Regions& regionsI,
                                 const aliceVision::feature::Regions& regionsJ,
                                 aliceVision::matching::IndMatches& outputMatches)
{
    const std::vector<aliceVision::feature::PointFeature>& vecFeatureI = regionsI.Features();
    const std::vector<aliceVision::feature::PointFeature>& vecFeatureJ = regionsJ.Features();

    // outputMatches will contain the sorted matches if inputMatches.
    outputMatches.reserve(inputMatches.size());

    // This vector is just a temporary container to link the index of the matches in the original vector inputMatches.
    // It will be used to retrieve the correct matches after the sort.
    std::vector<std::pair<float, size_t>> vecFeatureScale;

    for (size_t i = 0; i < inputMatches.size(); i++)
    {
        float scale1 = vecFeatureI[inputMatches[i]._i].scale();
        float scale2 = vecFeatureJ[inputMatches[i]._j].scale();
        vecFeatureScale.emplace_back((scale1 + scale2) / 2.0, i);
    }

    std::sort(vecFeatureScale.begin(), vecFeatureScale.end(), matchCompare);

    // The sorted match vector is filled according to the result of the sorting above.
    for (size_t i = 0; i < vecFeatureScale.size(); i++)
    {
        outputMatches.push_back(inputMatches[vecFeatureScale[i].second]);
    }
}

void sortMatches_byDistanceRatio(aliceVision::matching::IndMatches& matches)
{
    struct
    {
        bool operator()(const matching::IndMatch& m1, const matching::IndMatch& m2) const { return m1._distanceRatio < m2._distanceRatio; }
    } increasingLoweRatio;

    std::sort(matches.begin(), matches.end(), increasingLoweRatio);
}

bool matchCompare(const std::pair<float, size_t>& firstElem, const std::pair<float, size_t>& secondElem)
{
    return firstElem.first > secondElem.first;
}

void thresholdMatches(aliceVision::matching::IndMatches& outputMatches, const std::size_t uNumMatchesToKeep)
{
    if (outputMatches.size() > uNumMatchesToKeep)
    {
        outputMatches.resize(uNumMatchesToKeep);
    }
}

void matchesGridFiltering(const aliceVision::feature::Regions& lRegions,
                          const std::pair<std::size_t, std::size_t>& lImgSize,
                          const aliceVision::feature::Regions& rRegions,
                          const std::pair<std::size_t, std::size_t>& rImgSize,
                          const aliceVision::Pair& indexImagePair,
                          aliceVision::matching::IndMatches& outMatches,
                          size_t gridSize)
{
    const size_t leftCellWidth = divideRoundUp(lImgSize.first, gridSize);
    const size_t leftCellHeight = divideRoundUp(lImgSize.second, gridSize);
    const size_t rightCellWidth = divideRoundUp(rImgSize.first, gridSize);
    const size_t rightCellHeight = divideRoundUp(rImgSize.second, gridSize);

    std::vector<aliceVision::matching::IndMatches> completeGrid(gridSize * gridSize * 2);
    // Reserve all cells
    for (aliceVision::matching::IndMatches& cell : completeGrid)
    {
        cell.reserve(outMatches.size() / completeGrid.size());
    }
    // Split matches in grid cells
    for (const auto& match : outMatches)
    {
        const aliceVision::feature::PointFeature& leftPoint = lRegions.Features()[match._i];
        const aliceVision::feature::PointFeature& rightPoint = rRegions.Features()[match._j];

        const float leftGridIndex_f = std::floor(leftPoint.x() / (float)leftCellWidth) + std::floor(leftPoint.y() / (float)leftCellHeight) * gridSize;
        const float rightGridIndex_f =
          std::floor(rightPoint.x() / (float)rightCellWidth) + std::floor(rightPoint.y() / (float)rightCellHeight) * gridSize;
        // clamp the values if we have feature/marker centers outside the image size.
        const std::size_t leftGridIndex = clamp(leftGridIndex_f, 0.f, float(gridSize - 1));
        const std::size_t rightGridIndex = clamp(rightGridIndex_f, 0.f, float(gridSize - 1));

        aliceVision::matching::IndMatches& currentCaseL = completeGrid[leftGridIndex];
        aliceVision::matching::IndMatches& currentCaseR = completeGrid[rightGridIndex + gridSize * gridSize];

        if (currentCaseL.size() <= currentCaseR.size())
        {
            currentCaseL.push_back(match);
        }
        else
        {
            currentCaseR.push_back(match);
        }
    }

    // max Size of the cells:
    int maxSize = 0;
    for (const auto& cell : completeGrid)
    {
        if (cell.size() > maxSize)
        {
            maxSize = cell.size();
        }
    }

    aliceVision::matching::IndMatches finalMatches;
    finalMatches.reserve(outMatches.size());

    // Combine all cells into a global ordered vector
    for (int cmpt = 0; cmpt < maxSize; ++cmpt)
    {
        for (const auto& cell : completeGrid)
        {
            if (cmpt < cell.size())
            {
                finalMatches.push_back(cell[cmpt]);
            }
        }
    }
    outMatches.swap(finalMatches);
}

void matchesGridFilteringForAllPairs(const PairwiseMatches& geometricMatches,
                                     const sfmData::SfMData& sfmData,
                                     const feature::RegionsPerView& regionPerView,
                                     bool useGridSort,
                                     std::size_t numMatchesToKeep,
                                     PairwiseMatches& outPairwiseMatches)
{
    for (const auto& geometricMatch : geometricMatches)
    {
        // Get the image pair and their matches.
        const Pair& indexImagePair = geometricMatch.first;
        const MatchesPerDescType& matchesPerDesc = geometricMatch.second;

        for (const auto& match : matchesPerDesc)
        {
            const feature::EImageDescriberType descType = match.first;
            assert(descType != feature::EImageDescriberType::UNINITIALIZED);
            const IndMatches& inputMatches = match.second;

            const feature::Regions* rRegions = &regionPerView.getRegions(indexImagePair.second, descType);
            const feature::Regions* lRegions = &regionPerView.getRegions(indexImagePair.first, descType);

            // get the regions for the current view pair:
            if (rRegions && lRegions)
            {
                // sorting function:
                aliceVision::matching::IndMatches outMatches;
                sortMatches_byFeaturesScale(inputMatches, *lRegions, *rRegions, outMatches);

                if (useGridSort)
                {
                    // TODO: rename as matchesGridOrdering
                    matchesGridFiltering(*lRegions,
                                         sfmData.getView(indexImagePair.first).getImage().getImgSize(),
                                         *rRegions,
                                         sfmData.getView(indexImagePair.second).getImage().getImgSize(),
                                         indexImagePair,
                                         outMatches);
                }

                if (numMatchesToKeep > 0)
                {
                    size_t finalSize = std::min(numMatchesToKeep, outMatches.size());
                    outMatches.resize(finalSize);
                }

                // std::cout << "Left features: " << lRegions->Features().size() << ", right features: " << rRegions->Features().size() << ", num
                // matches: " << inputMatches.size() << ", num filtered matches: " << outMatches.size() << std::endl;
                outPairwiseMatches[indexImagePair].insert(std::make_pair(descType, outMatches));
            }
            else
            {
                ALICEVISION_LOG_INFO("You cannot perform the grid filtering with these regions");
            }
        }
    }
}

void filterMatchesByMin2DMotion(PairwiseMatches& mapPutativesMatches, const feature::RegionsPerView& regionPerView, double minRequired2DMotion)
{
    if (minRequired2DMotion < 0.0f)
        return;

    // For each image pair
    for (auto& imgPair : mapPutativesMatches)
    {
        const Pair viewPair = imgPair.first;
        IndexT viewI = viewPair.first;
        IndexT viewJ = viewPair.second;

        // For each descriptors in this image
        for (auto& descType : imgPair.second)
        {
            const feature::EImageDescriberType type = descType.first;

            const feature::Regions& regions_I = regionPerView.getRegions(viewI, type);
            const feature::Regions& regions_J = regionPerView.getRegions(viewJ, type);

            const auto& features_I = regions_I.Features();
            const auto& features_J = regions_J.Features();

            IndMatches& matches = descType.second;
            IndMatches updated_matches;

            for (auto& match : matches)
            {
                Vec2f pi = features_I[match._i].coords();
                Vec2f pj = features_J[match._j].coords();

                float scale = std::max(features_I[match._i].scale(), features_J[match._j].scale());
                float coeff = pow(2, scale);

                if ((pi - pj).norm() < (minRequired2DMotion * coeff))
                {
                    continue;
                }

                updated_matches.push_back(match);
            }

            matches = updated_matches;
        }
    }
}

}  // namespace matching
}  // namespace aliceVision
