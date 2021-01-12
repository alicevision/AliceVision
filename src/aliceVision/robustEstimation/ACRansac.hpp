// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// Copyright (c) 2012, 2013 Lionel MOISAN.
// Copyright (c) 2012, 2013 Pascal MONASSE.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/robustEstimation/randSampling.hpp>
#include <aliceVision/system/Logger.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <iterator>
#include <limits>
#include <numeric>
#include <vector>

/**
 * @brief Generic implementation of ACRANSAC
 *
 * The A contrario parametrization have been first explained in [1] and
 * later extended to generic model estimation in [2] (with a demonstration for
 * the homography) and extended and use at large scale for Structure from Motion in [3].
 *
 * @ref [1] Lionel Moisan, Berenger Stival.
 *          A probalistic criterion to detect rigid point matches between
 *          two images and estimate the fundamental matrix.
 *          IJCV 04.
 *
 * @ref [2] Lionel Moisan, Pierre Moulon, Pascal Monasse.
 *          Automatic Homographic Registration of a Pair of Images,
 *          with A Contrario Elimination of Outliers
 *          Image Processing On Line (IPOL), 2012.
 *          http://dx.doi.org/10.5201/ipol.2012.mmm-oh
 *
 * @ref [3] Pierre Moulon, Pascal Monasse and Renaud Marlet.
 *          Adaptive Structure from Motion with a contrario mode estimation.
 *          In 11th Asian Conference on Computer Vision (ACCV 2012)
 */

namespace aliceVision {
namespace robustEstimation{

/**
 * @brief Logarithm (base 10) of binomial coefficient
 */
template <typename T>
T logcombi(std::size_t k, std::size_t n, const std::vector<T>& vec_log10) // vec_log10: lookuptable in [0,n+1]

{
  if (k>=n || k<=0) return(0.0f);
  if (n-k<k) k=n-k;
  T r = 0.0f;
  for (std::size_t i = 1; i <= k; ++i)
    r += vec_log10[n-i+1] - vec_log10[i];
  return r;
}

/**
 * @brief Tabulate logcombi(.,n)
 */
template<typename Type>
void makelogcombi_n(std::size_t n, std::vector<Type>& l, std::vector<Type>& vec_log10) // vec_log10: lookuptable [0,n+1]
{
  l.resize(n+1);
  for (std::size_t k = 0; k <= n; ++k)
    l[k] = logcombi<Type>(k, n, vec_log10);
}

/**
 * @brief Tabulate logcombi(k,.)
 */
template<typename Type>
void makelogcombi_k(std::size_t k, std::size_t nmax, std::vector<Type>& l, std::vector<Type>& vec_log10) // vec_log10: lookuptable [0,n+1]
{
  l.resize(nmax+1);
  for (std::size_t n = 0; n <= nmax; ++n)
    l[n] = logcombi<Type>(k, n, vec_log10);
}

template <typename Type>
void makelogcombi(std::size_t k, std::size_t n, std::vector<Type>& vec_logc_k, std::vector<Type>& vec_logc_n)
{
  // compute a lookuptable of log10 value for the range [0,n+1]
  std::vector<Type> vec_log10(n + 1);
  for (std::size_t k = 0; k <= n; ++k)
    vec_log10[k] = log10((Type)k);

  makelogcombi_n(n, vec_logc_n, vec_log10);
  makelogcombi_k(k, n, vec_logc_k, vec_log10);
}

/**
 * @brief NFA and associated index
 */
using ErrorIndex = std::pair<double,size_t>;

/**
 * @brief Find best NFA and its index wrt square error threshold in e.
 */
inline ErrorIndex bestNFA(int startIndex, //number of point required for estimation
                          double logalpha0,
                          const std::vector<ErrorIndex>& e,
                          double loge0,
                          double maxThreshold,
                          const std::vector<float> &logc_n,
                          const std::vector<float> &logc_k,
                          double multError = 1.0)
{
  ErrorIndex bestIndex(std::numeric_limits<double>::infinity(), startIndex);
  const size_t n = e.size();
  for(size_t k = startIndex + 1; k <= n && e[k - 1].first <= maxThreshold; ++k)
  {
    const double logalpha = logalpha0 +
      multError * log10(e[k - 1].first + std::numeric_limits<float>::epsilon());
    ErrorIndex index(loge0 +
                     logalpha * (double) (k - startIndex) +
                     logc_n[k] +
                     logc_k[k], k);

    if(index.first < bestIndex.first)
      bestIndex = index;
  }
  return bestIndex;
}


/**
 * @brief ACRANSAC routine (ErrorThreshold, NFA)
 *
 * @param[in] kernel model and metric object
 * @param[out] vec_inliers points that fit the estimated model
 * @param[in] nIter maximum number of consecutive iterations
 * @param[out] model returned model if found
 * @param[in] precision upper bound of the precision (squared error)
 *
 * @return (errorMax, minNFA)
 */
template<typename Kernel>
std::pair<double, double> ACRANSAC(const Kernel& kernel,
                                   std::mt19937 &randomNumberGenerator,
                                   std::vector<size_t>& vec_inliers,
                                   std::size_t nIter = 1024,
                                   typename Kernel::ModelT* model = nullptr,
                                   double precision = std::numeric_limits<double>::infinity())
{
  vec_inliers.clear();

  const std::size_t sizeSample = kernel.getMinimumNbRequiredSamples();
  const std::size_t nData = kernel.nbSamples();
  if (nData <= (std::size_t)sizeSample)
    return std::make_pair(0.0,0.0);

  const double maxThreshold = (precision==std::numeric_limits<double>::infinity()) ?
    std::numeric_limits<double>::infinity() :
    precision * kernel.normalizer2()(0,0) * kernel.normalizer2()(0,0);

  std::vector<ErrorIndex> vec_residuals(nData); // [residual,index]
  std::vector<double> vec_residuals_(nData);

  // Possible sampling indices [0,..,nData] (will change in the optimization phase)
  std::vector<size_t> vec_index(nData);
  std::iota(vec_index.begin(), vec_index.end(), 0);

  // Precompute log combi
  const double loge0 = log10((double)kernel.getMaximumNbModels() * (nData-sizeSample));
  std::vector<float> vec_logc_n, vec_logc_k;
  makelogcombi(sizeSample, nData, vec_logc_k, vec_logc_n);

  // Output parameters
  double minNFA = std::numeric_limits<double>::infinity();
  double errorMax = std::numeric_limits<double>::infinity();

  // Reserve 10% of iterations for focused sampling
  size_t nIterReserve = nIter/10;
  nIter -= nIterReserve;

  bool bACRansacMode = (precision == std::numeric_limits<double>::infinity());

  // Main estimation loop.
  for(std::size_t iter = 0; iter < nIter; ++iter)
  {
    std::vector<std::size_t> vec_sample(sizeSample); // Sample indices
    if (bACRansacMode)
      uniformSample(randomNumberGenerator, sizeSample, vec_index, vec_sample); // Get random sample
    else
      uniformSample(randomNumberGenerator, sizeSample, nData, vec_sample); // Get random sample

    std::vector<typename Kernel::ModelT> vec_models; // Up to max_models solutions
    kernel.fit(vec_sample, vec_models);

    // Evaluate models
    bool better = false;
    for (std::size_t k = 0; k < vec_models.size(); ++k)
    {
      // Residuals computation and ordering
      kernel.errors(vec_models[k], vec_residuals_);

      if (!bACRansacMode)
      {
        unsigned int nInlier = 0;
        for (std::size_t i = 0; i < nData; ++i)
        {
          if (vec_residuals_[i] <= maxThreshold)
            ++nInlier;
        }
        if (nInlier > 2.5 * sizeSample) // does the model is meaningful
          bACRansacMode = true;
      }
      if (bACRansacMode)
      {
        for (size_t i = 0; i < nData; ++i)
        {
          const double error = vec_residuals_[i];
          vec_residuals[i] = ErrorIndex(error, i);
        }
        std::sort(vec_residuals.begin(), vec_residuals.end());

        // Most meaningful discrimination inliers/outliers
        const ErrorIndex best = bestNFA(
          sizeSample,
          kernel.logalpha0(),
          vec_residuals,
          loge0,
          maxThreshold,
          vec_logc_n,
          vec_logc_k,
          kernel.multError());

        if (best.first < minNFA /*&& vec_residuals[best.second-1].first < errorMax*/)
        {
          // A better model was found
          better = true;
          minNFA = best.first;
          vec_inliers.resize(best.second);
          for (size_t i=0; i<best.second; ++i)
            vec_inliers[i] = vec_residuals[i].second;
          errorMax = vec_residuals[best.second-1].first; // Error threshold
          if(model) *model = vec_models[k];

          ALICEVISION_LOG_TRACE("  nfa=" << minNFA
            << " inliers=" << best.second << "/" << nData
            << " precisionNormalized=" << errorMax
            << " precision=" << kernel.unormalizeError(errorMax)
            << " (iter=" << iter
            << ",sample=" << vec_sample
            << ")");
        }
      } //if(bACRansacMode)
    } //for(size_t k...
    
    // Early exit test -> no meaningful model found after nIterReserve*2 iterations
    if (!bACRansacMode && iter > nIterReserve*2)
      break;

    // ACRANSAC optimization: draw samples among best set of inliers so far
    if (bACRansacMode && ((better && minNFA<0) || (iter+1==nIter && nIterReserve)))
    {
      if (vec_inliers.empty())
      {
        // No model found at all so far
        ++nIter; // Continue to look for any model, even not meaningful
        --nIterReserve;
      }
      else
      {
        // ACRANSAC optimization: draw samples among best set of inliers so far
        vec_index = vec_inliers;
        if(nIterReserve)
        {
          nIter = iter + 1 + nIterReserve;
          nIterReserve = 0;
        }
      }
    }
  }

  if(minNFA >= 0)
    vec_inliers.clear();

  if (!vec_inliers.empty())
  {
    if (model)
      kernel.unnormalize(*model);
    errorMax = kernel.unormalizeError(errorMax);
  }

  return std::make_pair(errorMax, minNFA);
}

} // namespace robustEstimation
} // namespace aliceVision
