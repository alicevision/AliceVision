
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
* Pick a random subset of the integers in the range [0, total), in random order.
*
* This uses a quadratic rejection strategy and should only be used for small
* num_samples.
*
* @param num_samples   The number of samples to produce.
* @param upperBound    The upper bound of the range.
* @param samples       num_samples of numbers in [0, total_samples) is placed
*                      here on return.
* @warning Argument values should respect: num_samples <= total_samples
*/
template<typename IntT>
inline void UniformSample(
  std::size_t num_samples,
  std::size_t upperBound,
  std::set<IntT> *samples)
{
  assert(num_samples <= upperBound);
  static_assert(std::is_integral<IntT>::value, "Only integer types are supported");
  std::random_device rd;
  std::default_random_engine e1(rd());
  std::uniform_int_distribution<IntT> uniform_dist(0, upperBound-1);
  while (samples->size() < num_samples)
  {
    IntT sample = uniform_dist(e1);
    samples->insert(sample);
  }
}

/**
 * @brief Generate a unique random samples in the range [lowerBound upperBound).
 * @param[in] lowerBound The lower bound of the range.
 * @param[in] upperBound The upper bound of the range (not included).
 * @param[in] num_samples Number of unique samples to draw.
 * @param[out] samples The vector containing the samples.
 */
template<typename IntT>
inline void UniformSample(
  std::size_t lowerBound,
  std::size_t upperBound,
  std::size_t num_samples,
  std::vector<IntT> *samples)
{
  assert(lowerBound < upperBound);
  assert(num_samples <= (lowerBound-upperBound));
  static_assert(std::is_integral<IntT>::value, "Only integer types are supported");
  samples->resize(0);
  samples->reserve(num_samples);
  std::random_device rd;
  std::default_random_engine e1(rd());
  std::uniform_int_distribution<IntT> uniform_dist(lowerBound, upperBound-1);
  std::set<IntT> set_samples;
  while (set_samples.size() < num_samples)
  {
    IntT sample = uniform_dist(e1);
    if(set_samples.count(sample) == 0)
    {
      set_samples.insert(sample);
      samples->push_back(sample);
    }
  }
  assert(samples->size() == num_samples);
}

/**
 * @brief Generate a unique random samples without replacement in the range [lowerBound upperBound).
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
  assert(numSamples <= (rangeSize));
  static_assert(std::is_integral<IntT>::value, "Only integer types are supported");

  
  std::random_device rd;
  std::mt19937 generator(rd());

  if(numSamples * 1.5 > (rangeSize))
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
 * @brief Generate a unique random samples in the range [0 upperBound).
 * @param[in] num_samples Number of unique samples to draw.
 * @param[in] upperBound The value at the end of the range (not included).
 * @param[out] samples The vector containing the samples.
 */
template<typename IntT>
inline void UniformSample(
  std::size_t num_samples,
  std::size_t upperBound,
  std::vector<IntT> *samples)
{
  UniformSample(0, upperBound, num_samples, samples);
}

/// Get a (sorted) random sample of size X in [0:n-1]
inline void random_sample(std::size_t X, std::size_t upperBound, std::vector<std::size_t> *samples)
{
  samples->resize(X);
  for(std::size_t i=0; i < X; ++i)
  {
    std::size_t r = (std::rand()>>3)%(upperBound-i), j;
    for(j=0; j<i && r>=(*samples)[j]; ++j)
    {
      ++r;
    }
    std::size_t j0 = j;
    for(j=i; j > j0; --j)
    {
      (*samples)[j] = (*samples)[j-1];
    }
    (*samples)[j0] = r;
  }
}


/**
 * @brief 
 * @param sizeSample The size of the sample.
 * @param vec_index The possible data indices.
 * @param sample The random sample of sizeSample indices (output).
 */
inline void UniformSample(std::size_t sampleSize,
                          const std::vector<std::size_t>& vec_index,
                          std::vector<std::size_t>& sample)
{
  sample.resize(sampleSize);
  sample = randSample<std::size_t>(0, vec_index.size(), sampleSize);
  assert(sample.size() == sampleSize);
  for(auto &element : sample)
  {
    element = vec_index[ element ];
  }
}

} // namespace robust
} // namespace openMVG
