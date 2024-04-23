// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.
#include <aliceVision/robustEstimation/ISolver.hpp>
#include <aliceVision/robustEstimation/IRansacKernel.hpp>

namespace aliceVision {
namespace multiview {

/**
 * Triangulation kernel which works for any camera model.
 * The estimation is done on lifted points but error is computed in pixels
*/
class TriangulationSphericalKernel : public robustEstimation::IRansacKernel<robustEstimation::MatrixModel<Vec4>>
{
public:
    using ModelT = robustEstimation::MatrixModel<Vec4>;
public:  
  /**
   * @brief Constructor.
   * @param[in] observations The observations 2d points in pixels.
   * @param[in] poses the transformation for each observation.
   * @param[in] intrinsics the camera intrinsic for each observation.
   */
  TriangulationSphericalKernel(
        const std::vector<Vec2> & observations, 
        const std::vector<Eigen::Matrix4d>& poses, 
        std::vector<std::shared_ptr<camera::IntrinsicBase>> & intrinsics
    )
  : _observations(observations)
  , _poses(poses)
  , _intrinsics(intrinsics)
  {
    for (int id = 0; id < observations.size(); id++)
    {
        //Lift all points onto the metric unit sphere
        const Vec2 & obs = _observations[id];
        std::shared_ptr<camera::IntrinsicBase> camera = _intrinsics[id];
        _lifted.push_back(camera->toUnitSphere(camera->removeDistortion(camera->ima2cam(obs))));
    }
  }

  /**
   * @brief Return the minimum number of required samples
   * @return minimum number of required samples
   */
  inline std::size_t getMinimumNbRequiredSamples() const override
  {
    return 2;
  }

  /**
   * @brief Return the maximum number of models
   * @return maximum number of models
   */
  inline std::size_t getMaximumNbModels() const override
  {
    return 1;
  }

  inline std::size_t getMinimumNbRequiredSamplesLS() const override
  {
    return 2;
  }

  /**
   * @brief Triangulate 2 points.
   * @param[in] samples The index of two points to triangulate.
   * @param[out] models The estimated 3D points.
   */
  void fit(const std::vector<std::size_t>& samples, std::vector<ModelT>& models) const override
  {
    std::vector<Vec3> sampledPts;
    std::vector<Eigen::Matrix4d> sampledMats;
    
    for (int i = 0; i < samples.size(); i++)
    {
        std::size_t idx = samples[i];
        sampledMats.push_back(_poses[idx]);
        sampledPts.push_back(_lifted[idx]);
    }

    _solver.solve(sampledPts, sampledMats, models);
  }

  /**
   * @brief Triangulate N points using the least squared solver.
   * @param[in] inliers The index of the N points to triangulate.
   * @param[out] models The estimated 3D point.
   * @param[in] weights The optional array of weight for each of the N points.
   */
  void fitLS(const std::vector<std::size_t>& inliers,
             std::vector<ModelT>& models,
             const std::vector<double> *weights = nullptr) const override
  {
    std::vector<Vec3> sampledPts;
    std::vector<Eigen::Matrix4d> sampledMats;
    
    for (int i = 0; i < inliers.size(); i++)
    {
        std::size_t idx = inliers[i];
        sampledMats.push_back(_poses[idx]);
        sampledPts.push_back(_lifted[idx]);
    }

    _solver.solve(sampledPts, sampledMats, models, *weights);
  }


  /**
   * @brief Compute the weights..
   * @param[in] model The 3D point for which the weights are computed.
   * @param[in] inliers The array of the indices of the data to be used.
   * @param[out] vec_weights The array of weight of the same size as \p inliers.
   * @param[in] eps An optional threshold to max out the value of the threshold (typically
   * to avoid division by zero or too small numbers).
   */
  void computeWeights(const ModelT & model,
                      const std::vector<std::size_t> &inliers, 
                      std::vector<double>& weights,
                      const double eps = 0.001) const override
  {
    const auto numInliers = inliers.size();
    weights.resize(numInliers);

    for(std::size_t sample = 0; sample < numInliers; ++sample)
    {
      const auto idx = inliers[sample];
      weights[sample] = 1.0 / std::pow(std::max(eps,  error(idx, model)), 2);
    }
  }
  
  /**
   * @brief Error for the i-th view
   * @param[in] sample The index of the view for which the error is computed.
   * @param[in] model The 3D point.
   * @return The estimation error for the given view and 3D point.
   */
  double error(std::size_t sample, const ModelT & model) const override
  {
    Vec4 X = model.getMatrix();
    if (std::abs(X(3)) > 1e-16)
    {
        X = X / X(3);
    }

    Vec2 residual = _intrinsics[sample]->residual(_poses[sample], X, _observations[sample]);

    return residual.norm();
  }

  /**
   * @brief Error for each view.
   * @param[in] model The 3D point.
   * @param[out] vec_errors The vector containing all the estimation errors for every view.
   */
  void errors(const ModelT & model, std::vector<double>& errors) const override
  {
    errors.resize(nbSamples());
    for(Mat::Index i = 0; i < _lifted.size(); ++i)
    {
      errors[i] = error(i, model);
    }
  }

  /**
   * @brief Unnormalize the model. (not used)
   * @param[in,out] model the 3D point.
   */
  void unnormalize(robustEstimation::MatrixModel<Vec4> & model) const override
  {
    
  }

  /**
   * @brief Return the number of view.
   * @return the number of view.
   */
  std::size_t nbSamples() const override
  {
    return _lifted.size();
  }
  
  double logalpha0() const override
  {
    std::runtime_error("Method 'logalpha0()' is not defined for 'NViewsTriangulationLORansac'.");
    return 0.0;
  }

  double errorVectorDimension() const override
  {
    std::runtime_error("Method 'errorVectorDimension()' is not defined for 'NViewsTriangulationLORansac'.");
    return 0.0;
  }

  double unormalizeError(double val) const override
  {
    std::runtime_error("Method 'unormalizeError()' is not defined for 'NViewsTriangulationLORansac'.");
    return 0.0;
  }

  Mat3 normalizer1() const override
  {
    std::runtime_error("Method 'normalizer1()' is not defined for 'NViewsTriangulationLORansac'.");
    return Mat3();
  }

  double thresholdNormalizer() const override
  {
    std::runtime_error("Method 'thresholdNormalizer()' is not defined for 'NViewsTriangulationLORansac'.");
    return 0.0;
  }

private:
  std::vector<Vec3> _lifted;
  const std::vector<Vec2> _observations;
  const std::vector<Eigen::Matrix4d> _poses;
  const std::vector<std::shared_ptr<camera::IntrinsicBase>> _intrinsics;
  multiview::TriangulateNViewsSphericalSolver _solver;
};

}
}