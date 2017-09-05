// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#ifndef OPENMVG_ROBUST_ESTIMATION_SCORE_EVALUATOR_H_
#define OPENMVG_ROBUST_ESTIMATION_SCORE_EVALUATOR_H_

namespace openMVG {
namespace robust{

using namespace std;

/// Templated Functor class to evaluate a given model over a set of samples.
template<typename Kernel>
class ScorerEvaluator {
public:
  ScorerEvaluator(double threshold) : threshold_(threshold) {}

  template <typename T>
  double Score(const Kernel &kernel,
               const typename Kernel::Model &model,
               const std::vector<T> &samples,
               std::vector<T> *inliers,
               double threshold) const
  {
    double cost = 0.0;
    for (size_t j = 0; j < samples.size(); ++j) 
    {
      double error = kernel.Error(samples[j], model);
      if (error < threshold) 
      {
        cost += error;
        inliers->push_back(samples[j]);
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
  double Score(const Kernel &kernel,
               const typename Kernel::Model &model,
               const std::vector<T> &samples,
               std::vector<T> *inliers) const
  {
    return Score(kernel, model, samples, inliers, threshold_);
  }
  
  double getThreshold() const {return threshold_;} 
  
private:
  double threshold_;
};

} // namespace robust
} // namespace openMVG

#endif // OPENMVG_ROBUST_ESTIMATION_SCORE_EVALUATOR_H_
