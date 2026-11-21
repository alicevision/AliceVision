// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/matchingImageCollection/ImageCollectionMatcher_cascadeHashing.hpp>
#include <aliceVision/feature/regionsFactory.hpp>

#define BOOST_TEST_MODULE ImageCollectionMatcher_cascadeHashing

#include <boost/test/unit_test.hpp>

#include <algorithm>
#include <random>
#include <vector>

using namespace aliceVision;

// The matches of a pair must be computed between the regions of its two views
BOOST_AUTO_TEST_CASE(ImageCollectionMatcher_cascadeHashing_differentViews)
{
    const IndexT viewI = 3;
    const IndexT viewJ = 8;
    const std::size_t countI = 3000;
    const std::size_t countJ = 1000;
    const feature::EImageDescriberType descType = feature::EImageDescriberType::SIFT;

    std::mt19937 gen(0);
    std::uniform_real_distribution<float> distribution(0.f, 1.f);

    auto* regionsI = new feature::SIFT_Float_Regions();
    auto* regionsJ = new feature::SIFT_Float_Regions();

    for (std::size_t i = 0; i < countI; ++i)
    {
        feature::SIFT_Float_Regions::DescriptorT descriptor;
        for (std::size_t d = 0; d < descriptor.size(); ++d)
        {
            descriptor[d] = distribution(gen);
        }
        regionsI->Descriptors().push_back(descriptor);
        regionsI->Features().emplace_back(static_cast<float>(i), static_cast<float>(i), 1.f, 0.f);
    }

    // Each feature of the second view is a copy of a feature of the first view in a different order
    std::vector<std::size_t> sourceInI(countJ);
    for (std::size_t j = 0; j < countJ; ++j)
    {
        sourceInI[j] = countI - 1 - 3 * j;
        regionsJ->Descriptors().push_back(regionsI->Descriptors()[sourceInI[j]]);
        regionsJ->Features().emplace_back(static_cast<float>(j), static_cast<float>(j), 1.f, 0.f);
    }

    feature::RegionsPerView regionsPerView;
    regionsPerView.addRegions(viewI, descType, regionsI);
    regionsPerView.addRegions(viewJ, descType, regionsJ);

    matching::PairwiseMatches matches;
    std::mt19937 matcherGen(0);
    const matchingImageCollection::ImageCollectionMatcher_cascadeHashing matcher(0.8f);
    matcher.Match(matcherGen, regionsPerView, PairSet{{viewI, viewJ}}, descType, matches);

    BOOST_REQUIRE_EQUAL(matches.size(), 1u);
    const matching::IndMatches& pairMatches = matches.at({viewI, viewJ}).at(descType);
    BOOST_CHECK_EQUAL(pairMatches.size(), countJ);

    for (const matching::IndMatch& match : pairMatches)
    {
        BOOST_REQUIRE_LT(match._j, countJ);
        BOOST_CHECK_EQUAL(match._i, sourceInI[match._j]);
    }
}
