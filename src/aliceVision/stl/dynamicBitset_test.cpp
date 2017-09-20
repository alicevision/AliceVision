// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include "testing/testing.h"

#include "DynamicBitset.hpp"

TEST(DYNAMIC_BITSET, InitAndReset_64)
{
  using namespace stl;

  const int nbBits = 64;
  dynamic_bitset mybitset(nbBits);

  // Check that there is nbBits bit stored
  EXPECT_EQ(64, mybitset.size());
  // Check that there is just the necessary count of BlockType allocated for storage
  EXPECT_EQ(64/dynamic_bitset::bits_per_block, mybitset.num_blocks());
  
  // Set some bits to 1
  for (int i = 0; i < mybitset.size(); i+=2)
    mybitset[i] = true;

  // Check that some bits have been correctly set to 1
  for (int i = 0; i < mybitset.size(); ++i)
  {
    EXPECT_EQ(!(i%2), mybitset[i]);
  }
  
  // Reset the value to 0
  mybitset.reset();
  for (int i = 0; i < mybitset.size(); ++i)
  {
    EXPECT_EQ(false, mybitset[i]);
  }
}

// Create a dynamic_bitset that is shorter than the internal used bit container
TEST(DYNAMIC_BITSET, InitAndReset_4)
{
  using namespace stl;

  const int nbBits = 4;
  dynamic_bitset mybitset(nbBits);
  
  EXPECT_EQ(4, mybitset.size());
  EXPECT_EQ(1, mybitset.num_blocks());

  // Set some bits to 1
  for (int i = 0; i < mybitset.size(); i+=2)
    mybitset[i] = true;

  // Check that some bits have been correctly set to 1
  for (int i = 0; i < mybitset.size(); ++i)
  {
    EXPECT_EQ(!(i%2), mybitset[i]);
  }
  
  // Reset the value to 0
  mybitset.reset();
  for (int i = 0; i < mybitset.size(); ++i)
  {
    EXPECT_EQ(false, mybitset[i]);
  }
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */

