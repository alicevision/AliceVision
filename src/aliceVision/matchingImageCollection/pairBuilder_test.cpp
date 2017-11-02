// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "aliceVision/matchingImageCollection/pairBuilder.hpp"
#include "aliceVision/sfm/SfMData.hpp"
#include "aliceVision/sfm/View.hpp"

#include <iostream>
#include <algorithm>
#include <memory>

#define BOOST_TEST_MODULE matchingImageCollectionPairBuilder
#include <boost/test/included/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

using namespace std;
using namespace aliceVision;

// Check pairs follow a weak ordering pair.first < pair.second
template<typename IterablePairs>
bool checkPairOrder(const IterablePairs & pairs)
{
  for (typename IterablePairs::const_iterator iterP = pairs.begin(); iterP != pairs.end();
    ++iterP)
  {
    if (iterP->first >= iterP->second)
      return false;
  }
  return true;
}

BOOST_AUTO_TEST_CASE(matchingImageCollection_exhaustivePairs)
{
  sfm::Views views;
  {
    // Empty
    PairSet pairSet = exhaustivePairs(views);
    BOOST_CHECK_EQUAL( 0, pairSet.size());
  }
  {
    std::vector<IndexT> indexes = {{ 12, 54, 89, 65 }};
    for( IndexT i: indexes )
    {
      views[i] = std::make_shared<sfm::View>("filepath", i);
    }


    PairSet pairSet = exhaustivePairs(views);
    BOOST_CHECK( checkPairOrder(pairSet) );
    BOOST_CHECK_EQUAL( 6, pairSet.size());
    BOOST_CHECK( pairSet.find(std::make_pair(12,54)) != pairSet.end() );
    BOOST_CHECK( pairSet.find(std::make_pair(12,89)) != pairSet.end() );
    BOOST_CHECK( pairSet.find(std::make_pair(12,65)) != pairSet.end() );
    BOOST_CHECK( pairSet.find(std::make_pair(54,89)) != pairSet.end() );
    BOOST_CHECK( pairSet.find(std::make_pair(54,65)) != pairSet.end() );
    BOOST_CHECK( pairSet.find(std::make_pair(65,89)) != pairSet.end() );
  }
}

BOOST_AUTO_TEST_CASE(matchingImageCollection_contiguousWithOverlap)
{
  sfm::Views views;
  {
    // Empty
    PairSet pairSet = contiguousWithOverlap(views, 1);
    BOOST_CHECK_EQUAL( 0, pairSet.size());
  }
  {
    std::vector<IndexT> indexes = {{ 12, 54, 65, 89 }};
    for( IndexT i: indexes )
    {
      views[i] = std::make_shared<sfm::View>("filepath", i);
    }

    PairSet pairSet = contiguousWithOverlap(views, 1);
    BOOST_CHECK( checkPairOrder(pairSet) );
    BOOST_CHECK_EQUAL( 3, pairSet.size());
    BOOST_CHECK( pairSet.find(std::make_pair(12, 54)) != pairSet.end() );
    BOOST_CHECK( pairSet.find(std::make_pair(54, 65)) != pairSet.end() );
    BOOST_CHECK( pairSet.find(std::make_pair(65, 89)) != pairSet.end() );
  }
  {
    std::vector<IndexT> indexes = {{ 11, 12, 54, 65, 89, 99 }};
    for( IndexT i: indexes )
    {
      views[i] = std::make_shared<sfm::View>("filepath", i);
    }

    PairSet pairSet = contiguousWithOverlap(views, 3);
    BOOST_CHECK( checkPairOrder(pairSet) );
    BOOST_CHECK_EQUAL( 12, pairSet.size());
    BOOST_CHECK( pairSet.find(std::make_pair(11, 12)) != pairSet.end() );
    BOOST_CHECK( pairSet.find(std::make_pair(11, 54)) != pairSet.end() );
    BOOST_CHECK( pairSet.find(std::make_pair(11, 65)) != pairSet.end() );
    BOOST_CHECK( pairSet.find(std::make_pair(12, 54)) != pairSet.end() );
    BOOST_CHECK( pairSet.find(std::make_pair(12,65)) != pairSet.end() );
    BOOST_CHECK( pairSet.find(std::make_pair(12,89)) != pairSet.end() );
    BOOST_CHECK( pairSet.find(std::make_pair(54,65)) != pairSet.end() );
    BOOST_CHECK( pairSet.find(std::make_pair(54,89)) != pairSet.end() );
    BOOST_CHECK( pairSet.find(std::make_pair(54,99)) != pairSet.end() );
    BOOST_CHECK( pairSet.find(std::make_pair(65,89)) != pairSet.end() );
    BOOST_CHECK( pairSet.find(std::make_pair(65,99)) != pairSet.end() );
    BOOST_CHECK( pairSet.find(std::make_pair(89,99)) != pairSet.end() );
  }
}

BOOST_AUTO_TEST_CASE(matchingImageCollection_IO)
{
  PairSet pairSetGT;
  pairSetGT.insert( std::make_pair(0,1) );
  pairSetGT.insert( std::make_pair(1,2) );
  pairSetGT.insert( std::make_pair(2,0) );

  PairSet pairSetGTsorted;
  pairSetGTsorted.insert( std::make_pair(0,1) );
  pairSetGTsorted.insert( std::make_pair(0,2) );
  pairSetGTsorted.insert( std::make_pair(1,2) );

  BOOST_CHECK( savePairs("pairsT_IO.txt", pairSetGT));

  PairSet loaded_Pairs;
  BOOST_CHECK( loadPairs("pairsT_IO.txt", loaded_Pairs));
  BOOST_CHECK( std::equal(loaded_Pairs.begin(), loaded_Pairs.end(), pairSetGTsorted.begin()) );
}
