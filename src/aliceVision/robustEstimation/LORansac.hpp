// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/robustEstimation/randSampling.hpp>
#include <aliceVision/robustEstimation/ACRansac.hpp>
#include <aliceVision/robustEstimation/ransacTools.hpp>
#include <aliceVision/robustEstimation/IRansacKernel.hpp>
#include <limits>
#include <numeric>
#include <iostream>
#include <vector>
#include <iterator>

namespace aliceVision {
namespace robustEstimation{

/**
 * @brief It performs an iterative reweighted least square (IRLS) estimation of the problem
 * defined by \p Kernel. At each step it perform a LS estimation using weights
 * for each data element computed iteratively on some residual error.
 * This implementation follow the Algorithm 3 described in  
 * 
 * Karel Lebeda, Jiri Matas, Ondrej Chum:
 * Fixing the Locally Optimized RANSAC. BMVC 2012: 1-11
 * 
 * @tparam Kernel The kernel used in the LORansac estimator which must provide a
 * minimum solver and a LS solver, the latter used here for the IRLS 
 * @see aliceVision/robustEstimation/LORansacKernel.hpp
 * @tparam Scorer The scorer used in the LORansac estimator @see ScoreEvaluator
 * 
 * @param[in] kernel The kernel used in the LORansac estimator.
 * @param[in] scorer The scorer used in the LORansac estimator.
 * @param[out] best_model The best model found at the end of the iterations.
 * @param[out] inliers The inliers supporting the best model.
 * @param[in] mtheta A threshold multiplier used to vary the threshold of the scorer. 
 * @param[in] numIter The max number of iterations to run.
 * @return the score of the best model as computed by the Scorer, infinity if the
 * process does not converge.
 */
template<typename Kernel, typename Scorer>
double iterativeReweightedLeastSquares(const Kernel &kernel,
                                       const Scorer &scorer,
                                       typename Kernel::ModelT &best_model,
                                       std::vector<std::size_t> &inliers,
                                       double mtheta = std::sqrt(2),
                                       std::size_t numIter = 4,
                                       bool verbose = false)
{
  const std::size_t total_samples = kernel.nbSamples();
  const std::size_t min_samples = kernel.getMinimumNbRequiredSamplesLS();
  double theta = scorer.getThreshold();  
  // used in the iterations to update (reduce) the threshold value
  const double deltaTetha = (mtheta*theta - theta) / (numIter-1);
  
  std::vector<std::size_t> all_samples(total_samples);
  std::iota(all_samples.begin(), all_samples.end(), 0);
  
  // find inliers from best model with threshold theta
  inliers.clear();
  scorer.score(kernel, best_model, all_samples, inliers, theta);
  
  if(inliers.size() < min_samples)
  {
    inliers.clear();
    if(verbose)
    {
      ALICEVISION_LOG_WARNING("[IRLS] returning cause inliers.size() < min_samples");
    }
    return std::numeric_limits<double>::infinity();
  }
  
  // LS model from the above inliers
  std::vector<typename Kernel::ModelT> models;
  kernel.fitLS(inliers, models);
  if (models.size() == 0) 
  {
    inliers.clear();
    if(verbose)
    {
      ALICEVISION_LOG_WARNING("[IRLS] returning cause models.size() == 0");
    }
    return std::numeric_limits<double>::infinity();
  }

  // change threshold for refinement
  theta *= mtheta;
  
  // iterative refinement
  for(std::size_t i = 0; i < numIter; ++i)
  {
    // find inliers on the best-so-far model
    // @todo maybe inliers instead of all samples to save some computation
    inliers.clear();
    scorer.score(kernel, models[0], all_samples, inliers, theta);

    if(inliers.size() < min_samples)
    {
      inliers.clear();
      if(verbose)
      {
        ALICEVISION_LOG_WARNING("[IRLS] returning cause inliers.size() < min_samples");
      }
      return std::numeric_limits<double>::infinity();
    }
//    ALICEVISION_LOG_DEBUG("[IRLS] #" << i 
//            << " theta: " << theta
//            << " num inliers: " << inliers.size());
    
    // compute the weights for the inliers
    std::vector<double> weights;
    kernel.computeWeights(models[0], inliers, weights);
    
    // LS with weights on inliers
    models.clear();
    kernel.fitLS(inliers, models, &weights);
    if(models.size() != 1)   // LS fitting must always return 1 model
    {
      if(verbose)
      {
        ALICEVISION_LOG_WARNING("[IRLS] found "<< models.size() << " models, aborting...");
      }
      return std::numeric_limits<double>::infinity();
    }
    
    // update the threshold
    theta -= deltaTetha;
  }
  
  assert(models.size()==1);
  best_model = models[0];
  inliers.clear();
  const double score = scorer.score(kernel, best_model, all_samples, inliers, theta);
  if(verbose)
  {
    ALICEVISION_LOG_DEBUG("[IRLS] returning with num inliers: " << inliers.size() 
            << " and score " << score);
  }
  return score;
}


/**
 * @brief The local optimization step used by LORansac. It takes as input a model
 * and its inliers computed by a minimal solver and refine the solution by using 
 * IRLS (@see iterativeReweightedLeastSquares()). It first estimates a new model 
 * using LS and its associated inliers. Then it repeatedly (\p numRep) re-estimate
 * a new model by LS on a randomly drawn sample of those inliers and then refine 
 * the model by IRLS. The best found model (in terms of number of supporting inliers
 * is then returned.
 * 
 * This implementation follow the Algorithm 2 described in  
 * 
 * Karel Lebeda, Jiri Matas, Ondrej Chum:
 * Fixing the Locally Optimized RANSAC. BMVC 2012: 1-11 
 * 
 * @tparam Kernel The kernel used in the LORansac estimator which must provide a
 * minimum solver and a LS solver, the latter used here for the IRLS 
 * @see aliceVision/robustEstimation/LORansacKernel.hpp
 * @tparam Scorer The scorer used in the LORansac estimator @see ScoreEvaluator
 * 
 * @param[in] kernel The kernel used in the LORansac estimator.
 * @param[in] scorer The scorer used in the LORansac estimator.
 * @param[in,out] best_model In input the model estimated by a minimum solver, as
 * output the best model found.
 * @param[out] bestInliers The inliers supporting the best model.
 * @param[in] mtheta A threshold multiplier used for IRLS.
 * @param[in] numRep The number of re-sampling/re-estimation of the model.
 * @param[in] minSampleSize Size of the inner sample used for re-estimation.
 * @return the best score of the best model as computed by Scorer.
 */
template<typename Kernel, typename Scorer>
double localOptimization(const Kernel& kernel,
                         const Scorer& scorer,
                         std::mt19937 &randomNumberGenerator,
                         typename Kernel::ModelT& bestModel,
                         std::vector<std::size_t>& bestInliers,
                         double mtheta = std::sqrt(2),
                         std::size_t numRep = 10,
                         std::size_t minSampleSize = 10,
                         bool verbose = false)
{
  const std::size_t total_samples = kernel.nbSamples();
  const std::size_t min_samples = kernel.getMinimumNbRequiredSamplesLS();
  assert((total_samples > min_samples) && 
          "[localOptimization] not enough data to estimate the model!");
  
  const double theta = scorer.getThreshold();
  
  std::vector<std::size_t> all_samples(total_samples);
  std::iota(all_samples.begin(), all_samples.end(), 0);
  
  std::size_t debugInit = 0;
  if(!bestInliers.empty())
  {
    debugInit = bestInliers.size(); 
    bestInliers.clear();
  }
  double bestScore = scorer.score(kernel, bestModel, all_samples, bestInliers, theta);
  if(debugInit != 0) assert(debugInit == bestInliers.size());
  
  // so far this is the best model
  std::size_t bestNumInliers = bestInliers.size();
  if(verbose)
  {
    ALICEVISION_LOG_DEBUG("[localOptim] so far best num inliers: " << bestNumInliers);
    ALICEVISION_LOG_DEBUG("[localOptim] so far best model:\n" << bestModel.getMatrix());
    ALICEVISION_LOG_DEBUG("[localOptim] so far best score: " << bestScore);
  }
     
  // find inliers from best model with larger threshold t*m over all the samples
  std::vector<std::size_t> inliersBase;
  scorer.score(kernel, bestModel, all_samples, inliersBase, theta*mtheta);
  assert((inliersBase.size() > min_samples) && 
          "[localOptimization] not enough data in inliersBase to estimate the model!");
  
  // LS model from the above inliers
  std::vector<typename Kernel::ModelT> models;
//  ALICEVISION_LOG_DEBUG("[localOptim] before: ");
  kernel.fitLS(inliersBase, models);
  if (models.size()== 0) 
  {
    return bestScore;
  }

  // find inliers with t again over all the samples
  inliersBase.clear();
  scorer.score(kernel, models[0], all_samples, inliersBase, theta);
    
  // sample of size sampleSize from the last best inliers
  const std::size_t sampleSize = std::min(minSampleSize, inliersBase.size()/2);
  if(sampleSize <= kernel.getMinimumNbRequiredSamplesLS())
  {
    if(verbose)
    {
      ALICEVISION_LOG_DEBUG("breaking cause sampleSize is " << sampleSize);
    }
    return bestScore;
  }
  
  // do numRep resampling + iterative LS
  for(std::size_t i = 0; i < numRep; ++i)
  {
    std::vector<std::size_t> sample;
    uniformSample(randomNumberGenerator, sampleSize, inliersBase, sample);
    assert(sampleSize > kernel.getMinimumNbRequiredSamplesLS());
    assert(sample.size() > kernel.getMinimumNbRequiredSamplesLS());
  
    // LS estimation from the sample
    models.clear();
    kernel.fitLS(sample, models);
    if (models.size() == 0)
    {
      continue;
    }

    // IRLS 
    std::vector<std::size_t> inliers;
    const double score = iterativeReweightedLeastSquares(kernel, scorer, models[0], inliers);
    
    // store new best model if it is the case
    if((inliers.size() > bestNumInliers) || 
       ((inliers.size() == bestNumInliers) && (score < bestScore)))
    {
      bestNumInliers = inliers.size();
      bestScore = score;
      bestModel = models[0];
      bestInliers.swap(inliers);
      if(verbose)
      {
        ALICEVISION_LOG_DEBUG("[localOptim] new best num inliers: " << bestNumInliers);
      }
    }
  }
  return bestScore;
}

//@todo make visible parameters for the optimization step
/**
 * @brief Implementation of the LORansac framework.
 * 
 * This implementation follow the Algorithm 1 described in  
 * 
 * Karel Lebeda, Jiri Matas, Ondrej Chum:
 * Fixing the Locally Optimized RANSAC. BMVC 2012: 1-11 
 * 
 * @tparam Kernel The kernel used in the LORansac estimator which must provide a
 * minimum solver and a LS solver, the latter used here for the IRLS 
 * @see aliceVision/robustEstimation/LORansacKernel.hpp
 * @tparam Scorer The scorer used in the LORansac estimator @see ScoreEvaluator
 * 
 * @param[in] kernel The kernel containing the problem to solve.
 * @param[in] scorer The scorer used to asses the model quality.
 * @param[out] best_inliers The indices of the samples supporting the best model.
 * @param[out] best_score The score of the best model, ie the number of inliers
 * supporting the best model.
 * @param[in] bVerbose Enable/Disable log messages
 * @param[in] max_iterations Maximum number of iterations for the ransac part.
 * @param[in] outliers_probability The wanted probability of picking outliers.
 * @return The best model found.
 */
template<typename Kernel, typename Scorer>
typename Kernel::ModelT LO_RANSAC(const Kernel& kernel,
                                  const Scorer& scorer,
                                  std::mt19937 & randomNumberGenerator,
                                  std::vector<std::size_t>* best_inliers = NULL,
                                  double* best_score = NULL,
                                  bool bVerbose = false,
                                  std::size_t max_iterations = 100,
                                  double outliers_probability = 1e-2)
{
  assert(outliers_probability < 1.0);
  assert(outliers_probability > 0.0);
  std::size_t iteration = 0;
  const std::size_t min_samples = kernel.getMinimumNbRequiredSamples();
  const std::size_t total_samples = kernel.nbSamples();

  const std::size_t really_max_iterations = 4096;

  std::size_t bestNumInliers = 0;
  double bestInlierRatio = 0.0;
  typename Kernel::ModelT bestModel;

  // Test if we have sufficient points for the kernel.
  if (total_samples < min_samples) 
  {
    if (best_inliers) {
      best_inliers->clear();
    }
    return bestModel;
  }

  // In this robust estimator, the scorer always works on all the data points
  // at once. So precompute the list ahead of time [0,..,total_samples].
  std::vector<std::size_t> all_samples(total_samples);
  std::iota(all_samples.begin(), all_samples.end(), 0);

  for(iteration = 0; iteration < max_iterations; ++iteration) 
  {
    std::vector<std::size_t> sample;
    uniformSample(randomNumberGenerator, min_samples, total_samples, sample);

    std::vector<typename Kernel::ModelT> models;
    kernel.fit(sample, models);

    // Compute the inlier list for each fit.
    for(std::size_t i = 0; i < models.size(); ++i) 
    {
      std::vector<std::size_t> inliers;
      double score = scorer.score(kernel, models.at(i), all_samples, inliers);
      if(bVerbose)
      {
        ALICEVISION_LOG_DEBUG("sample=" << sample);
        ALICEVISION_LOG_DEBUG("model " << i << " e: " << score);
      }

      if (bestNumInliers <= inliers.size()) 
      {
        bestModel = models[i];
        //** LOCAL OPTIMIZATION
        if(bVerbose)
        {
          ALICEVISION_LOG_DEBUG("Before Optim: num inliers: " << inliers.size() 
                  << " score: " << score
                  << " kernel minimum nb required samples LS: " << kernel.getMinimumNbRequiredSamplesLS()
                 );

          ALICEVISION_LOG_DEBUG("Model:\n" << bestModel.getMatrix());
        }
        
        if(inliers.size() > kernel.getMinimumNbRequiredSamplesLS())
        {
          score = localOptimization(kernel, scorer, randomNumberGenerator, bestModel, inliers);
        }
        
        if(bVerbose)
        {
          ALICEVISION_LOG_DEBUG("After Optim: num inliers: " << inliers.size()
                  << " score: " << score);
          ALICEVISION_LOG_DEBUG("Model:\n" << bestModel.getMatrix());
        }
        
        bestNumInliers = inliers.size();
        bestInlierRatio = inliers.size() / double(total_samples);

        if (best_inliers) 
        {
          best_inliers->swap(inliers);
        }

        if(bVerbose)
        {
          ALICEVISION_LOG_DEBUG(" inliers=" << bestNumInliers << "/" << total_samples
                    << " (iter=" << iteration
                    << " ,i=" << i
                    << " ,sample=" << sample
                    << ")");
        }
        if (bestInlierRatio) 
        {
          max_iterations = iterationsRequired(min_samples,
                                              outliers_probability,
                                              bestInlierRatio);
          // safeguard to not get stuck in a big number of iterations
          max_iterations = std::min(max_iterations, really_max_iterations);
          if(bVerbose)
            ALICEVISION_LOG_DEBUG("New max_iteration: " << max_iterations);
        }
      }
    }
  }
  if (best_score)
    *best_score = bestNumInliers;
  
  if(bestNumInliers)
    kernel.unnormalize(bestModel);
  
  return bestModel;  
}

} // namespace robustEstimation
} // namespace aliceVision
