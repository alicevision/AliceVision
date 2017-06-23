
// Copyright (c) 2007, 2008 libmv authors.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.

// Copyright (c) 2012, 2013 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#pragma once

#include <set>
#include <unordered_set>
#include <algorithm>
#include <cstdlib>
#include <random>
#include <cassert>

namespace openMVG {
namespace robust{


/**
 * @brief Generate a unique random samples without replacement in the 
 * range [lowerBound upperBound).
 * It is modeled after Matlab function with the same name, and it tries to optimize
 * the generation of the random samples: if the number of required samples is a
 * large ratio of the range, then it shuffles a vector containing all the numbers 
 * in the range and it takes the first numSamples elements. Otherwise it proceeds
 * by drawing random numbers until the numSamples elements are generated, using  
 * Robert Floyd's algorithm.
 * 
 * @param[in] lowerBound The lower bound of the range.
 * @param[in] upperBound The upper bound of the range (not included).
 * @param[in] numSamples Number of unique samples to draw.
 * @return samples The vector containing the samples.
 */
template<typename IntT>
inline std::vector<IntT> randSample(IntT lowerBound,
                                    IntT upperBound,
                                    IntT numSamples)
{
  const auto rangeSize = upperBound - lowerBound;
  
  assert(lowerBound < upperBound);
  assert(numSamples <= rangeSize);
  static_assert(std::is_integral<IntT>::value, "Only integer types are supported");

  
  std::random_device rd;
  std::mt19937 generator(rd());

  if(numSamples * 1.5 > rangeSize)
  {
    // if the number of required samples is a large fraction of the range size
    // generate a vector with all the elements in the range, shuffle it and 
    // return the first numSample elements.
    // this should be more time efficient than drawing at each time.
    std::vector<IntT> result(rangeSize);
    std::iota(result.begin(), result.end(), lowerBound);
    std::shuffle(result.begin(), result.end(), generator);
    result.resize(numSamples);
    return result;
  }
  else
  {
    // otherwise if the number of required samples is small wrt the range
    // use the optimized Robert Floyd algorithm.
    // this has linear complexity and minimize the memory usage.
    std::unordered_set<IntT> samples;
    for(IntT d = upperBound - numSamples; d < upperBound; ++d)
    {
      IntT t = std::uniform_int_distribution<>(0, d)(generator) + lowerBound;
      if(samples.find(t) == samples.end())
        samples.insert(t);
      else
        samples.insert(d);
    }
    assert(samples.size() == numSamples);
    std::vector<IntT> result(std::make_move_iterator(samples.begin()),
                             std::make_move_iterator(samples.end()));
    return result;
  }
}

/**
* @brief Pick a random subset of the integers in the range [0, upperBound).
*
* @param[in] numSamples The number of samples to produce.
* @param[in] upperBound The upper bound of the range.
* @param[out] samples The set containing the random numbers in the range [0, upperBound)
*/
template<typename IntT>
inline void UniformSample(std::size_t numSamples,
                          std::size_t upperBound,
                          std::set<IntT> &samples)
{
  assert(numSamples <= upperBound);
  static_assert(std::is_integral<IntT>::value, "Only integer types are supported");
  
  const auto vecSamples = randSample<IntT>(0, upperBound, numSamples);
  for(const auto& s : vecSamples)
  {
    samples.insert(s);
  }
  assert(samples.size() == numSamples);
}

/**
 * @brief Generate a unique random samples in the range [lowerBound upperBound).
 * 
 * @param[in] lowerBound The lower bound of the range.
 * @param[in] upperBound The upper bound of the range (not included).
 * @param[in] numSamples Number of unique samples to draw.
 * @param[out] samples The vector containing the samples.
 */
template<typename IntT>
inline void UniformSample(std::size_t lowerBound,
                          std::size_t upperBound,
                          std::size_t numSamples,
                          std::vector<IntT> &samples)
{
  samples = randSample<IntT>(lowerBound, upperBound, numSamples);
}

/**
 * @brief Generate a unique random samples in the range [0 upperBound).
 * 
 * @param[in] numSamples Number of unique samples to draw.
 * @param[in] upperBound The value at the end of the range (not included).
 * @param[out] samples The vector containing the samples.
 */
template<typename IntT>
inline void UniformSample(std::size_t numSamples,
                          std::size_t upperBound,
                          std::vector<IntT> &samples)
{
  UniformSample(0, upperBound, numSamples, samples);
}

/**
 * @brief Generate a random sequence containing a sampling without replacement of
 * of the elements of the input vector.
 * 
 * @param[in] sampleSize The size of the sample to generate.
 * @param[in] elements The possible data indices.
 * @param[out] sample The random sample of sizeSample indices.
 */
inline void UniformSample(std::size_t sampleSize,
                          const std::vector<std::size_t>& elements,
                          std::vector<std::size_t>& sample)
{
  sample = randSample<std::size_t>(0, elements.size(), sampleSize);
  assert(sample.size() == sampleSize);
  for(auto& s : sample)
  {
    s = elements[ s ];
  }
}

} // namespace robust
} // namespace openMVG
