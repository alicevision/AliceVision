// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/robustEstimation/LORansacKernelAdaptor.hpp>
#include <aliceVision/multiview/conditioning.hpp>
#include <aliceVision/multiview/triangulation/Triangulation.hpp>
#include <aliceVision/robustEstimation/LORansac.hpp> 
#include <aliceVision/robustEstimation/LORansacKernelAdaptor.hpp> 

#include <vector>
#include <cstddef>
#include <algorithm>

namespace aliceVision {

/**
 * @brief The kernel for triangulating a point from N views to be used with
 * the LORansac framework.
 * 
 * @tparam SolverArg The minimal solver able to find a solution from a
 * minimum set of points.
 * @tparam ErrorArg The functor computing the error for each data sample with
 * respect to the estimated model, usually a reprojection error functor.
 * @tparam UnnormalizerArg The functor used to normalize the data before the 
 * estimation of the model.
 * @tparam ModelArg = Vec4 The type of the model to estimate, the 3D point.
 * @tparam SolverLSArg = SolverArg The least square solver that is used to find
 * a solution from any set of data larger than the minimum required, usually a 
 * DLT algorithm.
 */
template <typename SolverArg,
          typename ErrorArg,
          typename UnnormalizerArg,
          typename SolverLSArg,
          typename ModelArg = Vec4>
class NViewsTriangulationLORansac : 
    public robustEstimation::LORansacGenericKernel<2, 3, ModelArg>
{
   
  private:
    const Mat2X& _pt2d;
    const std::vector< Mat34 >& _projMatrices;
    
  public:
    typedef SolverArg Solver;
    typedef ModelArg Model;
    typedef ErrorArg ErrorT;
    typedef SolverLSArg SolverLS;
    
  enum
  {
    MINIMUM_SAMPLES = Solver::MINIMUM_SAMPLES,
    MINIMUM_LSSAMPLES = SolverLS::MINIMUM_SAMPLES
  };
  
  /**
   * @brief Constructor.
   * @param[in] _pt2d The feature points, a 2xN matrix.
   * @param[in] projMatrices The N projection matrix for each view.
   */
  NViewsTriangulationLORansac(const Mat2X &_pt2d, const std::vector< Mat34 > &projMatrices)
  : _pt2d(_pt2d),
    _projMatrices(projMatrices)
  {
    assert(_projMatrices.size() == _pt2d.cols());
  }

  /**
   * @brief Triangulate 2 points.
   * @param[in] samples The index of two points to triangulate.
   * @param[out] models The estimated 3D points.
   */
  void Fit(const std::vector<std::size_t> &samples, std::vector<Model> *models) const override
  {
    const Mat p2d = ExtractColumns(_pt2d, samples);
    std::vector< Mat34 > sampledMats;
    pick(sampledMats, _projMatrices, samples);
    Solver::Solve(p2d, sampledMats, *models);
  }

  /**
   * @brief Triangulate N points using the least squared solver.
   * @param[in] inliers The index of the N points to triangulate.
   * @param[out] models The estimated 3D point.
   * @param[in] weights The optional array of weight for each of the N points.
   */
  void FitLS(const std::vector<std::size_t> &inliers, 
              std::vector<Model> *models, 
              const std::vector<double> *weights = nullptr) const override
  {
    const Mat p2d = ExtractColumns(_pt2d, inliers);
    std::vector< Mat34 > sampledMats;
    pick(sampledMats, _projMatrices, inliers);
    SolverLS::Solve(p2d, sampledMats, *models, *weights);
  }


  /**
   * @brief Compute the weights..
   * @param[in] model The 3D point for which the weights are computed.
   * @param[in] inliers The array of the indices of the data to be used.
   * @param[out] vec_weights The array of weight of the same size as \p inliers.
   * @param[in] eps An optional threshold to max out the value of the threshold (typically
   * to avoid division by zero or too small numbers).
   */
  void computeWeights(const Model & model, 
                      const std::vector<std::size_t> &inliers, 
                      std::vector<double> & vec_weights, 
                      const double eps = 0.001) const override
  {
    const auto numInliers = inliers.size();
    vec_weights.resize(numInliers);
    for(std::size_t sample = 0; sample < numInliers; ++sample)
    {
      const auto idx = inliers[sample];
      vec_weights[sample] = ErrorT::Error(_pt2d.col(idx), _projMatrices[idx], model);
      // avoid division by zero
      vec_weights[sample] = 1/std::pow(std::max(eps, vec_weights[sample]), 2);
    }
  }
  
  /**
   * @brief Error for the i-th view
   * @param[in] sample The index of the view for which the error is computed.
   * @param[in] model The 3D point.
   * @return The estimation error for the given view and 3D point.
   */
  double Error(std::size_t sample, const Model &model) const override
  {
    return ErrorT::Error(_pt2d.col(sample), _projMatrices[sample], model);
  }

  /**
   * @brief Error for each view.
   * @param[in] model The 3D point.
   * @param[out] vec_errors The vector containing all the estimation errors for every view.
   */
  void Errors(const Model & model, std::vector<double> & vec_errors) const override
  {
    vec_errors.resize(NumSamples());
    for(Mat::Index i = 0; i < _pt2d.cols(); ++i)
    {
      vec_errors[i] = Error(i, model);
    }
  }

  /**
   * @brief Unnormalize the model. (not used)
   * @param[in,out] model the 3D point.
   */
  void Unnormalize(Model * model) const override
  {
    
  }

  /**
   * @brief Return the number of view.
   * @return the number of view.
   */
  std::size_t NumSamples() const override
  {
    return _pt2d.cols();
  }
  
};

/**
 * @brief Functor used to compute the reprojection error as the pixel error.
 */
struct ReprojectionError
{
  /**
   * @brief Compute the reprojection error.
   * @param[in] pt2d The 2d image point.
   * @param[in] projMatrix The 3x4 projection matrix.
   * @param[in] pt3d The 3d point.
   * @return the reprojection error.
   */
  static double Error(const Vec2 &pt2d, const Mat34 &projMatrix, const Vec4 &pt3d)
  {
    const Vec3 proj = projMatrix * pt3d;
    return (pt2d - proj.hnormalized()).norm();
  }
};

/**
 * @brief Functor used to compute the error as the angular error.
 */
struct AngularError
{
  /**
   * @brief Compute the error as the angular error.
   * @param[in] pt2d The 2d image point.
   * @param[in] projMatrix The 3x4 projection matrix.
   * @param[in] pt3d The 3d point.
   * @return the error as the angular error between the direction of the
   * 3D point and the projection ray associated with the image point.
   */
  static double Error(const Vec2 &pt2d, const Mat34 &projMatrix, const Vec4 &pt3d)
  {
    const Vec3 ray1 = pt2d.homogeneous();
    const Vec3 ray2 = projMatrix * pt3d;
    return std::acos(ray1.normalized().transpose() * ray2.normalized());
  }
};

/// A kernel for robust triangulation with reprojection error
//typedef NViewsTriangulationLORansac<TriangulateNViewsSolver, 
//                                    ReprojectionError, 
//                                    UnnormalizerT,
//                                    TriangulateNViewsSolver> LORansacTriangulationKernel;
template<typename ErrorCost = ReprojectionError>
using LORansacTriangulationKernel =  NViewsTriangulationLORansac<TriangulateNViewsSolver, 
                                    ErrorCost, 
                                    UnnormalizerT,
                                    TriangulateNViewsSolver>;
                                    
/**************************/

//template <typename SolverArg,
//          typename ErrorArg,
//          typename UnnormalizerArg,
//          typename SolverLSArg,
//          typename ModelArg = Vec4>
//class AdvancedTriangulationLORansac : 
//    public robust::LORansacGenericKernel<SolverArg, ErrorArg, UnnormalizerArg, SolverLSArg, ModelArg>
//{
//   
//  private:
//    const Mat2X& _pt2d;
//    const std::vector< Mat34 >& _projMatrices;
//    
//  public:
//    typedef SolverArg Solver;
//    typedef ModelArg Model;
//    typedef ErrorArg ErrorT;
//    typedef SolverLSArg SolverLS;
//    
//  enum
//  {
//    MINIMUM_SAMPLES = Solver::MINIMUM_SAMPLES,
//    MINIMUM_LSSAMPLES = SolverLS::MINIMUM_SAMPLES
//  };
//  
//  /**
//   * @brief 
//   * @param x
//   * @param Ps
//   */
//  AdvancedTriangulationLORansac(const Mat2X &x, const std::vector< Mat34 > &Ps) 
//  : _pt2d(x),
//    _projMatrices(Ps)
//  {
//    assert(_projMatrices.size() == _pt2d.cols());
//  }
//  
//  void Fit(const std::vector<std::size_t> &samples, std::vector<Model> *models) const override
//  {
//    const Mat p2d = ExtractColumns(_pt2d, samples);
//    std::vector< Mat34 > sampledMats;
//    pick(sampledMats, _projMatrices, samples);
//    Solver::Solve(p2d, sampledMats, *models);
//  }
//  
//  void FitLS(const std::vector<std::size_t> &inliers, 
//              std::vector<Model> *models, 
//              const std::vector<double> *weights = nullptr) const override
//  {
//    const Mat p2d = ExtractColumns(_pt2d, inliers);
//    std::vector< Mat34 > sampledMats;
//    pick(sampledMats, _projMatrices, inliers);
//    SolverLS::Solve(p2d, sampledMats, *models, *weights);
//  }
//  
//  void computeWeights(const Model & model, 
//                      const std::vector<std::size_t> &inliers, 
//                      std::vector<double> & vec_weights, 
//                      const double eps = 0.001) const override
//  {
//    const auto numInliers = inliers.size();
//    vec_weights.resize(numInliers);
//    for(std::size_t sample = 0; sample < numInliers; ++sample)
//    {
//      const auto idx = inliers[sample];
//      vec_weights[sample] = ErrorT::Error(_pt2d.col(idx), _projMatrices[idx], model);
//      // avoid division by zero
//      vec_weights[sample] = 1/std::pow(std::max(eps, vec_weights[sample]), 2);
//    }
//  }
//  
//  /**
//   * @brief Error for the i-th view
//   * @param sample
//   * @param model
//   * @return 
//   */
//  double Error(std::size_t sample, const Model &model) const override
//  {
//    return ErrorT::Error(_pt2d.col(sample), _projMatrices[sample], model);
//  }
//
//  /**
//   * @brief Error for each view
//   * @param model
//   * @param vec_errors
//   */
//  void Errors(const Model & model, std::vector<double> & vec_errors) const override
//  {
//    vec_errors.resize(NumSamples());
//    for(Mat::Index i = 0; i < _pt2d.cols(); ++i)
//    {
//      vec_errors[i] = Error(i, model);
//    }
//  }
//  
//  void Unnormalize(Model * model) const override
//  {
//    
//  }
//  
//  std::size_t NumSamples() const override
//  {
//    return _pt2d.cols();
//  }
//  
//};                                    
}
