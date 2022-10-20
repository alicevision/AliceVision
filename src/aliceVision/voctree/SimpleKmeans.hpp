// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "distance.hpp"
#include "DefaultAllocator.hpp"

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/ParallelFor.hpp>

#include <boost/function.hpp>
#include <boost/foreach.hpp>

#include <algorithm>
#include <mutex>
#include <numeric>
#include <vector>
#include <limits>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

namespace aliceVision{
namespace voctree{

/**
 * @brief Initializer for K-means that randomly selects k features as the cluster centers.
 */
struct InitRandom
{

  template<class Feature, class Distance, class FeatureAllocator>
  void operator()(const std::vector<Feature*>& features, size_t k, std::vector<Feature, FeatureAllocator>& centers, Distance distance, const int verbose = 0)
  {
    ALICEVISION_LOG_DEBUG("#\t\tRandom initialization");
    // Construct a random permutation of the features using a Fisher-Yates shuffle
    std::vector<Feature*> features_perm = features;
    for(size_t i = features.size(); i > 1; --i)
    {
      size_t k = rand() % i;
      std::swap(features_perm[i - 1], features_perm[k]);
    }
    // Take the first k permuted features as the initial centers
    for(size_t i = 0; i < centers.size(); ++i)
      centers[i] = *features_perm[i];
  }
};

/**
 * @brief Initializer for K-means using the K-means++ algorithm.
 * 
 *  Arthur, D. and Vassilvitskii, S. (2007). "k-means++: the advantages of 
 *  careful seeding" Proceedings of the 18th annual ACM-SIAM symposium on 
 *  Discrete algorithms. Society for Industrial and Applied Mathematics 
 * Philadelphia, PA, USA. pp. 1027–1035.
 */
struct InitKmeanspp
{

  template<class Feature, class Distance, class FeatureAllocator>
  void operator()(const std::vector<Feature*>& features, size_t k, std::vector<Feature, FeatureAllocator>& centers, Distance distance, const int verbose = 0)
  {
    typedef typename Distance::result_type squared_distance_type;

    int numTrials = 5;
    squared_distance_type currSum = 0;

    // Algorithm:
    // 1. Choose one center uniformly at random from among the data points.
    // 2. For each data point x, compute D(x), the distance between x and the nearest 
    //    center that has already been chosen.
    // 3. Add one new data point as a center. Each point x is chosen with probability 
    //    proportional to D(x)^2.
    // 4. Repeat Steps 2 and 3 until k centers have been chosen.
    if(verbose > 0) ALICEVISION_LOG_DEBUG("Kmeanspp initialization");

    centers.clear();
    centers.resize(k);

    auto threadCount = std::min(numTrials, system::getMaxParallelThreadCount());
    auto subTrialThreadCount = system::getMaxParallelThreadCount() / threadCount;

    std::vector<squared_distance_type> dists(features.size(), std::numeric_limits<squared_distance_type>::max());
    std::vector<squared_distance_type> distsTempBest(features.size(), std::numeric_limits<squared_distance_type>::max());
    std::vector<std::vector<squared_distance_type>> trialDistsTemp(numTrials);
    std::vector<std::vector<squared_distance_type>> trialThreadDistSum(numTrials);
    for (int i = 0; i < numTrials; ++i)
    {
        // Data will be overwritten, can be initialized to anything.
        trialDistsTemp[i].resize(features.size());
        trialThreadDistSum[i].resize(system::getMaxParallelThreadCount());
    }

    typename std::vector<squared_distance_type>::iterator dstiter;
    typename std::vector<Feature*>::const_iterator featiter;

    // 1. Choose a random center
    size_t randCenter = rand() % features.size();

    // add it to the centers
    centers[0] = *features[ randCenter ];

    if(verbose > 2) ALICEVISION_LOG_DEBUG("First center picked randomly " << randCenter << ": " << centers[0]);

    // compute the distances
    for(dstiter = dists.begin(), featiter = features.begin(); dstiter != dists.end(); ++dstiter, ++featiter)
    {
      *dstiter = distance(*(*featiter), centers[0]);
      currSum += *dstiter;
    }

    std::mutex bestSumMutex;

    std::vector<float> trialPercs(numTrials);

    // iterate k-1 times
    for(int i = 1; i < k; ++i)
    {
      if(verbose > 1) ALICEVISION_LOG_DEBUG("Finding initial center " << i + 1);

      squared_distance_type bestSum = std::numeric_limits<squared_distance_type>::max();
      std::size_t bestCenter = -1;

      for (auto& perc : trialPercs)
      {
          perc = (float)std::rand() / RAND_MAX;
      }

      //make it a little bit more robust and try several guesses
      // choose the one with the global minimal distance
      system::parallelFor(0, numTrials, system::ParallelSettings().setThreadCount(threadCount),
                          [&](int j)
      {
        // draw an element from 0 to currSum
        // in order to choose a point with a probability proportional to D(x)^2
        // let's compute the overall sum of D(x)^2 and draw a number between
        // 0 and this sum, then start compute the sum from the first element again
        // until the partial sum is greater than the number drawn: the
        // the previous element is what we are looking for
        const float perc = trialPercs[j];
        squared_distance_type partial = (squared_distance_type)(currSum * perc);
        // look for the element that cap the partial sum that has been
        // drawn
        auto dstiter = dists.begin();
        while((partial > 0) && (dstiter != dists.end()))
        {
          assert(dstiter != dists.end());
          // safeguard against unsigned types that do not allow negative numbers
          if(partial > *dstiter)
          {
            partial -= *dstiter;
          }
          else
          {
            partial = 0;
          }
          ++dstiter;
        }

        // get the index
        std::size_t featidx;
        if(dstiter == dists.end())
          featidx = features.size() - 1;
        else
          featidx = dstiter - dists.begin();

        // 2. compute the distance of each feature from the current center
        auto& distsTemp = trialDistsTemp[j];
        auto& threadDistSum = trialThreadDistSum[j];
        std::fill(threadDistSum.begin(), threadDistSum.end(), 0);

        Feature newCenter = *features[ featidx ];
        system::parallelForWithPerThreadData<std::ptrdiff_t>(
                    0, features.size(), threadDistSum,
                    system::ParallelSettings().setThreadCount(subTrialThreadCount),
                    [&](std::ptrdiff_t it, squared_distance_type& threadSum)
        {
          distsTemp[it] = std::min(distance(*(features[it]), newCenter), dists[it]);
          threadSum += distsTemp[it];
        });

        squared_distance_type distSum = std::accumulate(threadDistSum.begin(), threadDistSum.end(),
                                                        squared_distance_type{0});

        if(verbose > 2) ALICEVISION_LOG_DEBUG("trial " << j << " found feat " << featidx << ": " << *features[ featidx ] << " with sum: " << distSum);

        {
            std::lock_guard<std::mutex> lock(bestSumMutex);
            if (distSum < bestSum)
            {
                // save the best so far
                bestSum = distSum;
                bestCenter = featidx;
                std::swap(distsTemp, distsTempBest);
            }
        }

      });
      if(verbose > 2) ALICEVISION_LOG_DEBUG("feature found feat " << bestCenter << ": " << *features[ bestCenter ]);

      // 3. add new data
      centers[i] = *features[ bestCenter ];
      currSum = bestSum;
      std::swap(dists, distsTempBest);

    }
    if(verbose > 1) ALICEVISION_LOG_DEBUG("Done!");

  }
};

/**
 * @brief Dummy initializer for K-means that leaves the centers as-is.
 */
struct InitGiven
{

  template<class Feature, class Distance, class FeatureAllocator>
  void operator()(const std::vector<Feature*>& features, std::size_t k, std::vector<Feature, FeatureAllocator>& centers, Distance distance, const int verbose = 0)
  {
    // Do nothing!
  }
};

template<class Feature>
inline void printFeat(const Feature &f)
{
  ALICEVISION_LOG_DEBUG(f);
}

template<class Feature, class FeatureAllocator = typename DefaultAllocator<Feature>::type>
void printFeatVector(const std::vector<Feature, FeatureAllocator> &f)
{
  for(std::size_t j = 0; j < f.size(); ++j)
  {
    printFeat(f[j]);
  }
}

/**
 * Check for "strange values" (nan or large values) in a feature, if some
 * of these values are found it prints them out and in the end it returns false
 * 
 * @param f the feature to check (supposed to have a size() method)
 * @param str somethig to write as debug
 * @return true if everything is ok
 */
template<class Feature>
bool checkElements(const Feature &f, const char* str)
{
  bool correct = true;
  // here we are supposing that Feature has a size method... (bleah!)
  for(std::size_t i = 0; i < f.size(); ++i)
  {
    if(f[i] > 10e6 || std::isnan(f[i]))
    {
      correct = false;
      printf("%s\t%.3f %ld", str, (float) f[i], i);
    }
  }
  if(!correct) printf("\n");
  return correct;
}

/**
 * Check for "strange values" (nan or large values) in a set of feature, if some
 * of these values are found it prints them out and in the end it returns false
 * 
 * @param f the vector of feature to check
 * @param str omethig to write as debug
 * @return true if everything is ok
 * @see checkElements( const Feature &f, const char* str )
 */
template<class Feature, class FeatureAllocator = typename DefaultAllocator<Feature>::type>
bool checkVectorElements(const std::vector<Feature, FeatureAllocator> &f, const char* str)
{
  bool correct = true;
  for(std::size_t i = 0; i < f.size(); ++i)
  {
    if(!checkElements(f[i], str))
      correct = false;
  }
  return correct;
}

/**
 * @brief Class for performing K-means clustering, optimized for a particular feature type and metric.
 *
 * The standard Lloyd's algorithm is used. By default, cluster centers are initialized randomly.
 */
template<class Feature,
         class Distance = L2<Feature, Feature>,
         class FeatureAllocator = typename DefaultAllocator<Feature>::type>
class SimpleKmeans
{
public:
  typedef typename Distance::result_type squared_distance_type;
  typedef boost::function<void(const std::vector<Feature*>&, std::size_t, std::vector<Feature, FeatureAllocator>&, Distance, const int verbose) > Initializer;

  /**
   * @brief Constructor
   *
   * @param zero Object representing zero in the feature space
   * @param d    Functor for calculating squared distance
   * 
   * @todo FeatureAllocator parameter
   */
  SimpleKmeans(const Feature& zero = Feature(), Distance d = Distance(), const int verbose = 0);

  /// Set function object used to choose initial cluster centers.

  void setInitMethod(const Initializer& init)
  {
    choose_centers_ = init;
  }

  std::size_t getMaxIterations() const
  {
    return max_iterations_;
  }

  void setMaxIterations(std::size_t iters)
  {
    max_iterations_ = iters;
  }

  std::size_t getRestarts() const
  {
    return restarts_;
  }

  void setRestarts(std::size_t restarts)
  {
    restarts_ = restarts;
  }

  int getVerbose() const
  {
    return verbose_;
  }

  void setVerbose(const int verboseLevel)
  {
    verbose_ = verboseLevel;
  }

  /**
   * @brief Partition a set of features into k clusters.
   *
   * @param      features   The features to be clustered.
   * @param      k          The number of clusters.
   * @param[out] centers    A set of k cluster centers.
   * @param[out] membership Cluster assignment for each feature
   */
  squared_distance_type cluster(const std::vector<Feature, FeatureAllocator>& features, std::size_t k,
                                std::vector<Feature, FeatureAllocator>& centers,
                                std::vector<unsigned int>& membership) const;

  /**
   * @brief Partition a set of features into k clusters.
   *
   * This version is more convenient for hierarchical clustering, as you do not have to copy
   * feature objects.
   *
   * @param      features   The features to be clustered.
   * @param      k          The number of clusters.
   * @param[out] centers    A set of k cluster centers.
   * @param[out] membership Cluster assignment for each feature
   */
  squared_distance_type clusterPointers(const std::vector<Feature*>& features, std::size_t k,
                                        std::vector<Feature, FeatureAllocator>& centers,
                                        std::vector<unsigned int>& membership) const;

private:

  squared_distance_type clusterOnce(const std::vector<Feature*>& features, std::size_t k,
                                    std::vector<Feature, FeatureAllocator>& centers,
                                    std::vector<unsigned int>& membership) const;

  Feature zero_;
  Distance distance_;
  Initializer choose_centers_;
  std::size_t max_iterations_;
  std::size_t restarts_;
  int verbose_;
};

template < class Feature, class Distance, class FeatureAllocator >
SimpleKmeans<Feature, Distance, FeatureAllocator>::SimpleKmeans(const Feature& zero, Distance d, const int verbose)
: zero_(zero),
distance_(d),
//    choose_centers_( InitRandom( ) ),
choose_centers_(InitKmeanspp()),
max_iterations_(100),
verbose_(verbose),
restarts_(1)
{
}

template < class Feature, class Distance, class FeatureAllocator >
typename SimpleKmeans<Feature, Distance, FeatureAllocator>::squared_distance_type
SimpleKmeans<Feature, Distance, FeatureAllocator>::cluster(const std::vector<Feature, FeatureAllocator>& features, size_t k,
                                                           std::vector<Feature, FeatureAllocator>& centers,
                                                           std::vector<unsigned int>& membership) const
{
  std::vector<Feature*> feature_ptrs;
  feature_ptrs.reserve(features.size());
  BOOST_FOREACH(const Feature& f, features)
  feature_ptrs.push_back(const_cast<Feature*> (&f));
  return clusterPointers(feature_ptrs, k, centers, membership);
}

template < class Feature, class Distance, class FeatureAllocator >
typename SimpleKmeans<Feature, Distance, FeatureAllocator>::squared_distance_type
SimpleKmeans<Feature, Distance, FeatureAllocator>::clusterPointers(const std::vector<Feature*>& features, size_t k,
                                                                   std::vector<Feature, FeatureAllocator>& centers,
                                                                   std::vector<unsigned int>& membership) const
{
  std::vector<Feature, FeatureAllocator> new_centers(centers);
  new_centers.resize(k);
  std::vector<unsigned int> new_membership(features.size());

  squared_distance_type least_sse = std::numeric_limits<squared_distance_type>::max();
  assert(restarts_ > 0);
  for(std::size_t starts = 0; starts < restarts_; ++starts)
  {
    if(verbose_ > 0) ALICEVISION_LOG_DEBUG("Trial " << starts + 1 << "/" << restarts_);
    choose_centers_(features, k, new_centers, distance_, verbose_);
    squared_distance_type sse = clusterOnce(features, k, new_centers, new_membership);
    if(verbose_ > 0) ALICEVISION_LOG_DEBUG("End of Trial " << starts + 1 << "/" << restarts_);
    if(sse < least_sse)
    {
      least_sse = sse;
      centers = new_centers;
      membership = new_membership;
    }
  }
 
  assert(least_sse != std::numeric_limits<squared_distance_type>::max());
  assert(membership.size() >= features.size());
  return least_sse;
}

template < class Feature, class Distance, class FeatureAllocator >
typename SimpleKmeans<Feature, Distance, FeatureAllocator>::squared_distance_type
SimpleKmeans<Feature, Distance, FeatureAllocator>::clusterOnce(const std::vector<Feature*>& features, std::size_t k,
                                                               std::vector<Feature, FeatureAllocator>& centers,
                                                               std::vector<unsigned int>& membership) const
{
  typedef typename std::vector<Feature, FeatureAllocator>::value_type centerType;
  typedef typename Distance::value_type feature_value_type;

  std::vector<std::size_t> new_center_counts(k);
  std::vector<Feature, FeatureAllocator> new_centers(k);
  std::vector<std::mutex> centersLocks(k);
  squared_distance_type max_center_shift = std::numeric_limits<squared_distance_type>::max();

  if(verbose_ > 0) ALICEVISION_LOG_DEBUG("Iterations");
  for(std::size_t iter = 0; iter < max_iterations_; ++iter)
  {
    if(verbose_ > 0) ALICEVISION_LOG_DEBUG("*");
    // Zero out new centers and counts
    std::fill(new_center_counts.begin(), new_center_counts.end(), 0);
    //		for(std::size_t i = 0; i < k; checkElements(new_centers[i++], "bef"));
    std::fill(new_centers.begin(), new_centers.end(), zero_);
    //		for(std::size_t i = 0; i < k; checkElements(new_centers[i++], "aft"));
    assert(checkVectorElements(new_centers, "newcenters init"));
    bool is_stable = true;

    // On small problems enabling multithreading does much more harm than good because thread
    // creation is relatively expensive.
    // TODO: Ideally a thread pool would be created before the first iteration and the iterations
    // would reuse the existing threads.
    bool enableMultithreading = features.size() * k > 1000000;

    // Assign data objects to current centers
    std::mutex centersMutex;
    system::parallelFor<std::ptrdiff_t>(0, features.size(),
                                        system::ParallelSettings().setEnableMultithreading(enableMultithreading),
                                        [&](std::ptrdiff_t i)
    {
      squared_distance_type d_min = std::numeric_limits<squared_distance_type>::max();
      unsigned int nearest = 0;
      bool found = false;

      // @todo if k is large, let's say k>100 use FLAAN to retrieve the 
      // cluster center

      // Find the nearest cluster center to feature i
      for(unsigned int j = 0; j < k; ++j)
      {
        squared_distance_type distance = distance_(*features[i], centers[j]);
        //		printf("\t\tdistance %f\n", (float)distance);
        //		PrintFeat(*features[i] );
        if(distance < d_min)
        {
          d_min = distance;
          nearest = j;
          found = true;
        }
        //		 printf("\t distance %f %f\n", (double)distance, (double)d_min);
      }
      assert(found);
      // Assign feature i to the cluster it is nearest to
      if(membership[i] != nearest)
      {
        is_stable = false;
        membership[i] = nearest;
      }
      // Accumulate the cluster center and its membership count
      //	  printf("\t nearest %d\n", nearest);
      //			checkElements(*features[i], "feat");
      {
        std::lock_guard<std::mutex> lock{centersLocks[nearest]};
        new_centers[nearest] += *features[i];
        //			checkElements(new_centers[nearest], "sum");
        //	  printf("\t new_centers[nearest] += *features[i];\n");
        ++new_center_counts[nearest];
        //	  printf("\taccumulate\n");
      }
    });

    if(is_stable) break;

    if(iter > 0)
      max_center_shift = 0;
    // Assign new centers
    for(std::size_t i = 0; i < k; ++i)
    {
      if(new_center_counts[i] > 0)
      {
        //		  printf("%d - %d\n", i, new_center_counts[i] );
        //		  PrintFeat(new_centers[i] );
        //		checkElements(new_centers[i], "bef");
        new_centers[i] = new_centers[i] / new_center_counts[i];

        squared_distance_type shift = distance_(new_centers[i], centers[i]);

        max_center_shift = std::max(max_center_shift, shift);

        centers[i] = new_centers[i];
        //					centers[i] = new_centers[i] / new_center_counts[i];
        //		checkElements(new_centers[i], "aft");
        //		PrintFeat(centers[i] );
      }
      else
      {
        // Choose a new center randomly from the input features
        // @todo use a better strategy like taking splitting the largest cluster
        unsigned int index = rand() % features.size();
        centers[i] = *features[index];
        ALICEVISION_LOG_DEBUG("Choosing a new center: " << index);
      }
    }
    //			ALICEVISION_LOG_DEBUG("max_center_shift: " << max_center_shift);  
    if(max_center_shift <= 10e-10) break;
  }
  if(verbose_ > 0) ALICEVISION_LOG_DEBUG("");

  // Return the sum squared error
  /// @todo Kahan summation?
  squared_distance_type sse = squared_distance_type(0);
  assert(features.size() > 0);
  for(std::size_t i = 0; i < features.size(); ++i)
  {
    sse += distance_(*features[i], centers[membership[i]]);
  }
  return sse;
}

}
}
