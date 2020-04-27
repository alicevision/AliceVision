// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "DynamicBitset.hpp"

#define BOOST_TEST_MODULE stlDynamicBitset

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>

BOOST_AUTO_TEST_CASE(DYNAMIC_BITSET_InitAndReset_64)
{
  using namespace stl;

  const int nbBits = 64;
  dynamic_bitset mybitset(nbBits);

  // Check that there is nbBits bit stored
  BOOST_CHECK_EQUAL(64, mybitset.size());
  // Check that there is just the necessary count of BlockType allocated for storage
  BOOST_CHECK_EQUAL(64/dynamic_bitset::bits_per_block, mybitset.num_blocks());
  
  // Set some bits to 1
  for (int i = 0; i < mybitset.size(); i+=2)
    mybitset[i] = true;

  // Check that some bits have been correctly set to 1
  for (int i = 0; i < mybitset.size(); ++i)
  {
    BOOST_CHECK_EQUAL(!(i%2), mybitset[i]);
  }
  
  // Reset the value to 0
  mybitset.reset();
  for (int i = 0; i < mybitset.size(); ++i)
  {
    BOOST_CHECK_EQUAL(false, mybitset[i]);
  }
}

// Create a dynamic_bitset that is shorter than the internal used bit container
BOOST_AUTO_TEST_CASE(DYNAMIC_BITSET_InitAndReset_4)
{
  using namespace stl;

  const int nbBits = 4;
  dynamic_bitset mybitset(nbBits);
  
  BOOST_CHECK_EQUAL(4, mybitset.size());
  BOOST_CHECK_EQUAL(1, mybitset.num_blocks());

  // Set some bits to 1
  for (int i = 0; i < mybitset.size(); i+=2)
    mybitset[i] = true;

  // Check that some bits have been correctly set to 1
  for (int i = 0; i < mybitset.size(); ++i)
  {
    BOOST_CHECK_EQUAL(!(i%2), mybitset[i]);
  }
  
  // Reset the value to 0
  mybitset.reset();
  for (int i = 0; i < mybitset.size(); ++i)
  {
    BOOST_CHECK_EQUAL(false, mybitset[i]);
  }
}
