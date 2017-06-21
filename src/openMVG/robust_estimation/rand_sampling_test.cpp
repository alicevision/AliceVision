
// Copyright (c) 2012, 2013 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "openMVG/robust_estimation/rand_sampling.hpp"
#include "testing/testing.h"
#include <set>
#include <vector>

using namespace openMVG;
using namespace openMVG::robust;

// Assert that each time exactly N random number are picked (no repetition)
TEST(UniformSampleTest, NoRepetions) {

  for(std::size_t upperBound = 1; upperBound < 513; upperBound *= 2)
  { 
    //Size of the data set
    for(std::size_t numSamples = 1; numSamples <= upperBound; numSamples *= 2)
    { 
      //Size of the consensus set
      std::vector<std::size_t> samples;
      std::cout << "Upper " << upperBound << " Lower " << 0 << " numSamples " << numSamples << "\n";
      UniformSample(numSamples, upperBound, &samples);
      std::set<std::size_t> myset;
      for(const auto& s : samples) 
      {
        myset.insert(s);
        CHECK(s >= 0);
        CHECK(s < upperBound);
      }
      CHECK_EQUAL(numSamples, myset.size());
    }
  }
}

TEST(UniformSampleTest, UniformSampleSet) {

  for(std::size_t upperBound = 1; upperBound < 513; upperBound *= 2)
  { 
    //Size of the data set
    for(std::size_t numSample = 1; numSample <= upperBound; numSample *= 2)
    { 
      //Size of the consensus set
      std::cout << "Upper " << upperBound << " Lower " << 0 << " numSamples " << numSample << "\n";
      std::set<std::size_t> samples;
      UniformSample(numSample, upperBound, samples);
      CHECK_EQUAL(numSample, samples.size());
      for(const auto& s : samples) 
      {
        CHECK(s >= 0);
        CHECK(s < upperBound);
      }
    }
  }
}

TEST(UniformSampleTest, NoRepetionsBeginEnd) {

  for(std::size_t upperBound = 1; upperBound < 513; upperBound *= 2)
  { 
    //Size of the data set
    for(std::size_t numSamples = 1; numSamples <= upperBound; numSamples *= 2)
    { 
      //Size of the consensus set
      assert((upperBound-numSamples) >= 0);
      const std::size_t begin = upperBound-numSamples;
      std::cout << "Upper " << upperBound << " Lower " << begin << " numSamples " << numSamples << "\n";
      std::vector<std::size_t> samples;
      UniformSample(begin, upperBound, numSamples, samples);
      std::set<std::size_t> myset;
      for(const auto& s : samples) 
      {
        myset.insert(s);
        CHECK(s >= begin);
        CHECK(s < upperBound);
      }
      CHECK_EQUAL(numSamples, myset.size());
    }
  }
}

TEST(UniformSampleTest, randSample) {
  
  for(std::size_t upperBound = 1; upperBound < 513; upperBound *= 2)
  { 
    for(std::size_t numSamples = 1; numSamples <= upperBound; numSamples *= 2)
    { 
      assert((upperBound-numSamples) >= 0);
      const std::size_t lowerBound = upperBound-numSamples;
      const auto samples = randSample<std::size_t>(lowerBound, upperBound, numSamples);
      
      std::set<std::size_t> myset;
      std::cout << "Upper " << upperBound << " Lower " << lowerBound << " numSamples " << numSamples << "\n";
      for(const auto& s : samples) 
      {
//        std::cout << samples[i] << " ";
        myset.insert(s);
        CHECK(s >= lowerBound);
        CHECK(s < upperBound);
      }
//      std::cout  << "\n";
      // this verifies no repetitions
      CHECK_EQUAL(numSamples, myset.size());
    }
  }
}


/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
