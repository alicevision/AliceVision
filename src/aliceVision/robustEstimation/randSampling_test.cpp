// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "aliceVision/robustEstimation/randSampling.hpp"
#include <set>
#include <vector>
#include <iostream>

#define BOOST_TEST_MODULE randSampling

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>

using namespace aliceVision;
using namespace aliceVision::robustEstimation;

// Assert that each time exactly N random number are picked (no repetition)
BOOST_AUTO_TEST_CASE(UniformSampleTest_NoRepetions) {

  std::mt19937 randomNumberGenerator; 

  for(std::size_t upperBound = 1; upperBound < 513; upperBound *= 2)
  { 
    //Size of the data set
    for(std::size_t numSamples = 1; numSamples <= upperBound; numSamples *= 2)
    { 
      //Size of the consensus set
      std::vector<std::size_t> samples;
      std::cout << "Upper " << upperBound << " Lower " << 0 << " numSamples " << numSamples << "\n";
      uniformSample(randomNumberGenerator, numSamples, upperBound, samples);
      std::set<std::size_t> myset;
      for(const auto& s : samples) 
      {
        myset.insert(s);
        BOOST_CHECK(s >= 0);
        BOOST_CHECK(s < upperBound);
      }
      BOOST_CHECK_EQUAL(numSamples, myset.size());
    }
  }
}

BOOST_AUTO_TEST_CASE(UniformSampleTest_UniformSampleSet) {

  std::mt19937 randomNumberGenerator; 

  for(std::size_t upperBound = 1; upperBound < 513; upperBound *= 2)
  { 
    //Size of the data set
    for(std::size_t numSample = 1; numSample <= upperBound; numSample *= 2)
    { 
      //Size of the consensus set
      std::cout << "Upper " << upperBound << " Lower " << 0 << " numSamples " << numSample << "\n";
      std::set<std::size_t> samples;
      uniformSample(randomNumberGenerator, numSample, upperBound, samples);
      BOOST_CHECK_EQUAL(numSample, samples.size());
      for(const auto& s : samples) 
      {
        BOOST_CHECK(s >= 0);
        BOOST_CHECK(s < upperBound);
      }
    }
  }
}

BOOST_AUTO_TEST_CASE(UniformSampleTest_NoRepetionsBeginEnd) {

  std::mt19937 randomNumberGenerator; 

  for(std::size_t upperBound = 1; upperBound < 513; upperBound *= 2)
  { 
    //Size of the data set
    for(std::size_t numSamples = 1; numSamples <= upperBound; numSamples *= 2)
    { 
      //Size of the consensus set
      assert(upperBound >= numSamples);
      const std::size_t begin = upperBound-numSamples;
      std::cout << "Upper " << upperBound << " Lower " << begin << " numSamples " << numSamples << "\n";
      std::vector<std::size_t> samples;
      uniformSample(randomNumberGenerator, begin, upperBound, numSamples, samples);
      std::set<std::size_t> myset;
      for(const auto& s : samples) 
      {
        myset.insert(s);
        BOOST_CHECK(s >= begin);
        BOOST_CHECK(s < upperBound);
      }
      BOOST_CHECK_EQUAL(numSamples, myset.size());
    }
  }
}

BOOST_AUTO_TEST_CASE(UniformSampleTest_randSample) {
  std::mt19937 randomNumberGenerator;

  for(std::size_t upperBound = 1; upperBound < 513; upperBound *= 2)
  { 
    for(std::size_t numSamples = 1; numSamples <= upperBound; numSamples *= 2)
    { 
      assert(upperBound >= numSamples);
      const std::size_t lowerBound = upperBound-numSamples;
      const auto samples = randSample<std::size_t>(randomNumberGenerator, lowerBound, upperBound, numSamples);
      
      std::set<std::size_t> myset;
      std::cout << "Upper " << upperBound << " Lower " << lowerBound << " numSamples " << numSamples << "\n";
      for(const auto& s : samples) 
      {
//        std::cout << samples[i] << " ";
        myset.insert(s);
        BOOST_CHECK(s >= lowerBound);
        BOOST_CHECK(s < upperBound);
      }
//      std::cout  << "\n";
      // this verifies no repetitions
      BOOST_CHECK_EQUAL(numSamples, myset.size());
    }
  }
}
