// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

//
// Created by Amir Yalamov (https://github.com/AmirYalamov)
// and Honson Tran (https://github.com/honsontran)
// on 2020-06-23.
//

#include "aliceVision/numeric/numeric.hpp"
#include "aliceVision/matching/ArrayMatcher_hnswlib.hpp"

#include <iostream>

#define BOOST_TEST_MODULE matching

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>

using namespace std;
using namespace aliceVision;
using namespace matching;

/*
 * This is a unit test for the hnswlib library (https://github.com/nmslib/hnswlib)
 *
 * It will test the different classes/functions that are needed for hnswlib
 */

// ----------------------- TESTING -----------------------

BOOST_AUTO_TEST_CASE(Matching_ArrayMatcher_Hnsw_NN)
{
    const float array[] = {0, 1, 2, 5, 6};
    // no 3, because it involve the same dist as 1,1

    ArrayMatcher_hnswlib<float> matcher;
    BOOST_CHECK( matcher.Build(array, 5, 1) );

    const float query[] {2};
    IndMatches vec_nIndice;
    vector<float> vec_fDistance;
    const int NN = 5;
    BOOST_CHECK( matcher.SearchNeighbours(query, 1, &vec_nIndice, &vec_fDistance, NN) );

    BOOST_CHECK_EQUAL( 5, vec_nIndice.size());
    BOOST_CHECK_EQUAL( 5, vec_fDistance.size() );

    // Check distances:
    BOOST_CHECK_SMALL(static_cast<double>(vec_fDistance[0]- Square(2.0f-2.0f)), 1e-6);
    BOOST_CHECK_SMALL(static_cast<double>(vec_fDistance[1]- Square(1.0f-2.0f)), 1e-6);
    BOOST_CHECK_SMALL(static_cast<double>(vec_fDistance[2]- Square(0.0f-2.0f)), 1e-6);
    BOOST_CHECK_SMALL(static_cast<double>(vec_fDistance[3]- Square(5.0f-2.0f)), 1e-6);
    BOOST_CHECK_SMALL(static_cast<double>(vec_fDistance[4]- Square(6.0f-2.0f)), 1e-6);

    // Check indexes:
    BOOST_CHECK_EQUAL(IndMatch(0,2), vec_nIndice[0]);
    BOOST_CHECK_EQUAL(IndMatch(0,1), vec_nIndice[1]);
    BOOST_CHECK_EQUAL(IndMatch(0,0), vec_nIndice[2]);
    BOOST_CHECK_EQUAL(IndMatch(0,3), vec_nIndice[3]);
    BOOST_CHECK_EQUAL(IndMatch(0,4), vec_nIndice[4]);
}
