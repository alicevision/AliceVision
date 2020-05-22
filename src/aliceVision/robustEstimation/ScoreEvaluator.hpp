// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

namespace aliceVision {
namespace robustEstimation{

using namespace std;

/**
 * @brief Templated Functor class to evaluate a given model over a set of samples.
 */
template<typename Kernel>
class ScoreEvaluator
{
public:
  explicit ScoreEvaluator(double threshold)
    : _threshold(threshold)
  {}

  template <typename T>
  double score(const Kernel& kernel,
               const typename Kernel::ModelT& model,
               const std::vector<T>& samples,
               std::vector<T>& inliers,
               double threshold) const
  {
    double cost = 0.0;
    for(std::size_t j = 0; j < samples.size(); ++j)
    {
      double error = kernel.error(samples.at(j), model);
      if (error < threshold) 
      {
        cost += error;
        inliers.push_back(samples[j]);
      } 
      else 
      {
//        cost += threshold;
        cost += error;
      }
    }
    return cost;
  }

  template <typename T>
  double score(const Kernel &kernel,
               const typename Kernel::ModelT& model,
               const std::vector<T>& samples,
               std::vector<T>& inliers) const
  {
    return score(kernel, model, samples, inliers, _threshold);
  }
  
  double getThreshold() const {return _threshold;}
  
private:
  double _threshold;
};

} // namespace robustEstimation
} // namespace aliceVision
