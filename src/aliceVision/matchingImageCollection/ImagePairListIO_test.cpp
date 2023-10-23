// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#define BOOST_TEST_MODULE imagePairListIO

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>

#include "ImagePairListIO.hpp"

using namespace aliceVision;
using namespace aliceVision::matchingImageCollection;

BOOST_AUTO_TEST_CASE(read_write_pairs_to_file)
{
    PairSet pairSetGT;
    pairSetGT.insert(std::make_pair(0, 1));
    pairSetGT.insert(std::make_pair(1, 2));
    pairSetGT.insert(std::make_pair(2, 0));

    PairSet pairSetGTsorted;
    pairSetGTsorted.insert(std::make_pair(0, 1));
    pairSetGTsorted.insert(std::make_pair(0, 2));
    pairSetGTsorted.insert(std::make_pair(1, 2));

    BOOST_CHECK(savePairsToFile("pairsT_IO.txt", pairSetGT));

    PairSet loaded_Pairs;
    BOOST_CHECK(loadPairsFromFile("pairsT_IO.txt", loaded_Pairs));
    BOOST_CHECK(std::equal(loaded_Pairs.begin(), loaded_Pairs.end(), pairSetGTsorted.begin()));
    std::remove("pairsT_IO.txt");
}

BOOST_AUTO_TEST_CASE(save_pairs)
{
    PairSet pairs = {{0, 2}, {0, 4}, {0, 5}, {8, 2}, {0, 1}, {5, 9}};

    std::stringstream output;
    savePairs(output, pairs);
    BOOST_CHECK_EQUAL(output.str(), std::string("0 1 2 4 5\n5 9\n8 2\n"));
}

BOOST_AUTO_TEST_CASE(load_multiple_pairs_per_line)
{
    std::stringstream input;
    input.str(R"( 0 2 4 5
        0 1
        5 9
)");

    PairSet loadedPairs;
    BOOST_CHECK(loadPairs(input, loadedPairs));

    PairSet expectedPairs = {{0, 2}, {0, 4}, {0, 5}, {0, 1}, {5, 9}};
    BOOST_CHECK(loadedPairs == expectedPairs);
}
