// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "aliceVision/matching/filters.hpp"

#define BOOST_TEST_MODULE matchingFilters

#include <boost/test/unit_test.hpp>

using namespace aliceVision;
using namespace aliceVision::matching;
using namespace std;

/// Sorted vector intersection (increasing order)
BOOST_AUTO_TEST_CASE(matching_setIntersection)
{
  int tab0[] = {0, 1, 2, 3, 4, 5, 6, 7};
  int tab1[] = {0, 1, 8, 3, 4, 9, 6, 7};
  set<int> vec_0(tab0, tab0+8);
  set<int> vec_1(tab1, tab1+8);
  /// Array must be sorted

  vector<int> vec_intersect;
  IntersectMatches(vec_0.begin(), vec_0.end(),
    vec_1.begin(), vec_1.end(),
    vec_intersect);

  BOOST_CHECK_EQUAL(6, vec_intersect.size());
  BOOST_CHECK_EQUAL(0, vec_intersect[0]);
  BOOST_CHECK_EQUAL(1, vec_intersect[1]);
  BOOST_CHECK_EQUAL(3, vec_intersect[2]);
  BOOST_CHECK_EQUAL(4, vec_intersect[3]);
  BOOST_CHECK_EQUAL(6, vec_intersect[4]);
  BOOST_CHECK_EQUAL(7, vec_intersect[5]);
}
