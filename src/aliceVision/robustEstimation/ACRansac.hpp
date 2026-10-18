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
#include <limits>
#include <numeric>
#include <vector>

namespace aliceVision {
namespace robustEstimation {

/**
 * @brief Logarithm (base 10) of binomial coefficient
 */
template<typename T>
T logcombi(std::size_t k, std::size_t n, const std::vector<T>& vec_log10)  // vec_log10: lookuptable in [0,n+1]

{
    if (k >= n || k <= 0)
        return (0.0f);
    if (n - k < k)
        k = n - k;
    T r = 0.0f;
    for (std::size_t i = 1; i <= k; ++i)
        r += vec_log10[n - i + 1] - vec_log10[i];
    return r;
}

/**
 * @brief Tabulate logcombi(.,n)
 */
template<typename Type>
void makelogcombi_n(std::size_t n, std::vector<Type>& l, std::vector<Type>& vec_log10)  // vec_log10: lookuptable [0,n+1]
{
    l.resize(n + 1);
    for (std::size_t k = 0; k <= n; ++k)
        l[k] = logcombi<Type>(k, n, vec_log10);
}

/**
 * @brief Tabulate logcombi(k,.)
 */
template<typename Type>
void makelogcombi_k(std::size_t k, std::size_t nmax, std::vector<Type>& l, std::vector<Type>& vec_log10)  // vec_log10: lookuptable [0,n+1]
{
    l.resize(nmax + 1);
    for (std::size_t n = 0; n <= nmax; ++n)
        l[n] = logcombi<Type>(k, n, vec_log10);
}

template<typename Type>
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
using ErrorIndex = std::pair<double, size_t>;

/**
 * @brief Find best NFA and its index wrt square error threshold in e.
 */
inline ErrorIndex bestNFA(int startIndex,  // number of point required for estimation
                          double logalpha0,
                          const std::vector<ErrorIndex>& e,
                          double loge0,
                          double maxThreshold,
                          const std::vector<float>& logc_n,
                          const std::vector<float>& logc_k,
                          double errorVectorDimension = 1.0)
{
    ErrorIndex bestIndex(std::numeric_limits<double>::infinity(), startIndex);
    const size_t n = e.size();

    for (size_t k = startIndex + 1; k <= n && e[k - 1].first <= maxThreshold; ++k)
    {
        double squaredResidual = e[k - 1].first;
        double residual = sqrt(squaredResidual) + std::numeric_limits<float>::epsilon();

        const double logalpha = logalpha0 + errorVectorDimension * log10(residual);
        const double nfa = loge0 + logalpha * (double)(k - startIndex) + logc_n[k] + logc_k[k];

        if (nfa < bestIndex.first)
        {
            bestIndex = ErrorIndex(nfa, k);
        }
    }

    return bestIndex;
}

/**
 * @brief Compute a lower bound of bestNFA(...).first in linear time, without sorting the errors.
 *
 * The errors are grouped into ordered bins, 16 bins per power of two (from std::frexp).
 * Neighbouring bins are merged when the errors span a wide range, so that there are at most max(n, 64) bins.
 * Each rank k that falls into a bin is evaluated with the smallest error of that bin.
 * The NFA at a given rank increases with the error, so the minimum over all ranks cannot exceed
 * the best NFA found on the sorted errors.
 *
 * @param[in] startIndex number of samples required to estimate a model
 * @param[in] logalpha0 log10 of the probability of a random point to fall within unit error, from the kernel
 * @param[in] errors unsorted square errors of every data point, as computed by the kernel
 * @param[in] loge0 log10 of the number of tests (models per sample times the number of candidate ranks)
 * @param[in] maxThreshold maximum error; ranks with a larger error are not evaluated, as in bestNFA()
 * @param[in] logc_n tabulated log10 of the binomial coefficients C(n, k), from makelogcombi()
 * @param[in] logc_k tabulated log10 of the binomial coefficients C(k, startIndex), from makelogcombi()
 * @param[in] errorVectorDimension dimension of the error vector, from the kernel
 * @param[in,out] binCount buffer for the number of errors per bin, reused between calls to avoid allocations
 * @param[in,out] binMin buffer for the smallest error per bin, reused between calls to avoid allocations
 * @return a value that never exceeds the best NFA,
 *         +infinity if there are no errors,
 *         -infinity if an error is negative or NaN (no bound can be given)
 */
inline double bestNFALowerBound(int startIndex,
                                double logalpha0,
                                const std::vector<double>& errors,
                                double loge0,
                                double maxThreshold,
                                const std::vector<float>& logc_n,
                                const std::vector<float>& logc_k,
                                double errorVectorDimension,
                                std::vector<std::size_t>& binCount,
                                std::vector<double>& binMin)
{
    const std::size_t n = errors.size();
    const double infinity = std::numeric_limits<double>::infinity();

    // bin of a positive finite error: its power of two and the first 4 bits of its mantissa, in increasing order
    const auto keyOf = [](double value) {
        int exponent;
        const double mantissa = std::frexp(value, &exponent);  // value = mantissa * 2^exponent, mantissa in [0.5, 1)
        return exponent * 16 + static_cast<int>((mantissa - 0.5) * 32.0);
    };

    int minKey = std::numeric_limits<int>::max();
    int maxKey = std::numeric_limits<int>::min();
    for (const double e : errors)
    {
        if (!(e >= 0.0))
            return -infinity;
        if (e > 0.0 && e < infinity)
        {
            const int key = keyOf(e);
            minKey = std::min(minKey, key);
            maxKey = std::max(maxKey, key);
        }
    }
    if (n == 0)
        return infinity;
    if (minKey > maxKey)  // only zeros and infinities
        minKey = maxKey = 0;

    // zeros go in a bin below every positive error, infinities in a bin above
    const int firstKey = minKey - 1;
    const auto binKeyOf = [&](double e) { return e == 0.0 ? firstKey : (e == infinity ? maxKey + 1 : keyOf(e)); };

    // merge neighbouring bins when the errors span a wide range, so the histogram stays smaller than the data
    const std::size_t keyRange = static_cast<std::size_t>(maxKey + 1 - firstKey);
    int mergeShift = 0;
    while ((keyRange >> mergeShift) + 1 > std::max<std::size_t>(n, 64))
        ++mergeShift;
    const std::size_t nbBins = (keyRange >> mergeShift) + 1;

    binCount.assign(nbBins, 0);
    binMin.assign(nbBins, infinity);
    for (const double e : errors)
    {
        const std::size_t bin = static_cast<std::size_t>(binKeyOf(e) - firstKey) >> mergeShift;
        ++binCount[bin];
        binMin[bin] = std::min(binMin[bin], e);
    }

    double lowerBound = std::numeric_limits<double>::infinity();
    std::size_t rankEnd = 0;  // 1-based rank of the last error in the bins seen so far
    for (std::size_t bin = 0; bin < nbBins; ++bin)
    {
        if (binCount[bin] == 0)
            continue;
        const std::size_t rankBegin = rankEnd + 1;
        rankEnd += binCount[bin];

        // bestNFA() stops at the first error above maxThreshold
        if (binMin[bin] > maxThreshold)
            break;

        // same expression as bestNFA(), with the smallest error of the bin
        const double residual = sqrt(binMin[bin]) + std::numeric_limits<float>::epsilon();
        const double logalpha = logalpha0 + errorVectorDimension * log10(residual);
        for (std::size_t k = std::max<std::size_t>(rankBegin, startIndex + 1); k <= rankEnd; ++k)
        {
            const double nfa = loge0 + logalpha * (double)(k - startIndex) + logc_n[k] + logc_k[k];
            lowerBound = std::min(lowerBound, nfa);
        }
    }
    return lowerBound;
}

/**
 * @brief An implementation of the "Random Sample Consensus" algorithm based on a-contrario estimator
 * to automatically estimate the error threshold.
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
 *
 * @param[in] kernel model and metric object
 * @param[out] vec_inliers points that fit the estimated model
 * @param[in] nIter maximum number of consecutive iterations
 * @param[out] model returned model if found
 * @param[in] precision upper bound of the precision
 *
 * @return (errorMax, minNFA)
 */
template<typename Kernel>
std::pair<double, double> ACRANSAC(const Kernel& kernel,
                                   std::mt19937& randomNumberGenerator,
                                   std::vector<size_t>& vec_inliers,
                                   std::size_t nIter = 1024,
                                   typename Kernel::ModelT* model = nullptr,
                                   double precision = std::numeric_limits<double>::infinity())
{
    vec_inliers.clear();

    const std::size_t sizeSample = kernel.getMinimumNbRequiredSamples();
    const std::size_t nData = kernel.nbSamples();
    if (nData <= (std::size_t)sizeSample)
        return std::make_pair(0.0, 0.0);

    const double maxThreshold = (precision == std::numeric_limits<double>::infinity())
                                  ? std::numeric_limits<double>::infinity()
                                  : precision * precision * kernel.thresholdNormalizer() * kernel.thresholdNormalizer();

    std::vector<ErrorIndex> vec_residuals(nData);  // [residual,index]
    std::vector<double> vec_residuals_(nData);

    // Possible sampling indices [0,..,nData] (will change in the optimization phase)
    std::vector<size_t> vec_index(nData);
    std::iota(vec_index.begin(), vec_index.end(), 0);

    // Precompute log combi
    const double loge0 = log10((double)kernel.getMaximumNbModels() * (nData - sizeSample));
    std::vector<float> vec_logc_n, vec_logc_k;
    makelogcombi(sizeSample, nData, vec_logc_k, vec_logc_n);

    // Output parameters
    double minNFA = std::numeric_limits<double>::infinity();
    double errorMax = std::numeric_limits<double>::infinity();

    // Reserve 10% of iterations for focused sampling
    size_t nIterReserve = nIter / 10;
    nIter -= nIterReserve;

    bool bACRansacMode = (precision == std::numeric_limits<double>::infinity());

    // Buffers for bestNFALowerBound()
    std::vector<std::size_t> binCount;
    std::vector<double> binMin;
    // Margin on the lower bound, in log10 units, so that rounding in log10() can never reject a better model
    const double lowerBoundMargin = 1e-6;

    // Main estimation loop.
    for (std::size_t iter = 0; iter < nIter; ++iter)
    {
        std::vector<std::size_t> vec_sample(sizeSample);  // Sample indices
        if (bACRansacMode)
            uniformSample(randomNumberGenerator, sizeSample, vec_index, vec_sample);  // Get random sample
        else
            uniformSample(randomNumberGenerator, sizeSample, nData, vec_sample);  // Get random sample

        std::vector<typename Kernel::ModelT> vec_models;  // Up to max_models solutions
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
                if (nInlier > 2.5 * sizeSample)  // does the model is meaningful
                    bACRansacMode = true;
            }
            if (bACRansacMode)
            {
                // Skip the sort when this model cannot beat the best NFA found so far
                if (minNFA != std::numeric_limits<double>::infinity())
                {
                    const double lowerBound = bestNFALowerBound(sizeSample, kernel.logalpha0(), vec_residuals_, loge0, maxThreshold, vec_logc_n,
                                                                vec_logc_k, kernel.errorVectorDimension(), binCount, binMin);
                    if (lowerBound - lowerBoundMargin >= minNFA)
                        continue;
                }

                for (size_t i = 0; i < nData; ++i)
                {
                    const double error = vec_residuals_[i];
                    vec_residuals[i] = ErrorIndex(error, i);
                }
                std::sort(vec_residuals.begin(), vec_residuals.end());

                // Most meaningful discrimination inliers/outliers
                const ErrorIndex best =
                  bestNFA(sizeSample, kernel.logalpha0(), vec_residuals, loge0, maxThreshold, vec_logc_n, vec_logc_k, kernel.errorVectorDimension());

                if (best.first < minNFA /*&& vec_residuals[best.second-1].first < errorMax*/)
                {
                    // A better model was found
                    better = true;
                    minNFA = best.first;
                    vec_inliers.resize(best.second);
                    for (size_t i = 0; i < best.second; ++i)
                        vec_inliers[i] = vec_residuals[i].second;
                    errorMax = vec_residuals[best.second - 1].first;  // Error threshold
                    if (model)
                        *model = vec_models[k];

                    ALICEVISION_LOG_TRACE("  nfa=" << minNFA << " inliers=" << best.second << "/" << nData << " precisionNormalized=" << errorMax
                                                   << " precision=" << kernel.unormalizeError(errorMax) << " (iter=" << iter
                                                   << ",sample=" << vec_sample << ")");
                }
            }  // if(bACRansacMode)
        }      // for(size_t k...

        // Early exit test -> no meaningful model found after nIterReserve*2 iterations
        if (!bACRansacMode && iter > nIterReserve * 2)
            break;

        // ACRANSAC optimization: draw samples among best set of inliers so far
        if (bACRansacMode && ((better && minNFA < 0) || (iter + 1 == nIter && nIterReserve)))
        {
            if (vec_inliers.empty())
            {
                // No model found at all so far
                ++nIter;  // Continue to look for any model, even not meaningful
                --nIterReserve;
            }
            else
            {
                // ACRANSAC optimization: draw samples among best set of inliers so far
                vec_index = vec_inliers;
                if (nIterReserve)
                {
                    nIter = iter + 1 + nIterReserve;
                    nIterReserve = 0;
                }
            }
        }
    }

    if (minNFA >= 0)
        vec_inliers.clear();

    if (!vec_inliers.empty())
    {
        if (model)
            kernel.unnormalize(*model);
        errorMax = kernel.unormalizeError(errorMax);
    }

    return std::make_pair(errorMax, minNFA);
}

}  // namespace robustEstimation
}  // namespace aliceVision
