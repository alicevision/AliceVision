// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// Copyright (c) 2010 libmv contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/robustEstimation/ISolver.hpp>

#include <vector>
#include <random>

namespace aliceVision {
namespace multiview {

/**
 * @brief Compute a 3D position of a point from several images of it. In particular,
 * compute the projective point X in R^4 such that x = PX.
 * Algorithm is the standard DLT; for derivation see appendix of Keir's thesis.
 * It also allows to specify some (optional) weight for each point (solving the 
 * weighted least squared problem)
 * 
 * @param[in] x are 2D coordinates (x,y,1) in each image
 * @param[in] Ps is the list of projective matrices for each camera
 * @param[out] X is the estimated 3D point
 * @param[in] weights a (optional) list of weights for each point
 */
void TriangulateNView(const Mat2X &x, 
                      const std::vector< Mat34 > &Ps, 
                      Vec4 *X, 
                      const std::vector<double> *weights = nullptr);

/**
 * @brief Compute a 3D position of a point from several images of it. In particular,
 * compute the projective point X in R^4 such that x ~ PX.
 * Algorithm is the standard DLT
 * It also allows to specify some (optional) weight for each point (solving the 
 * weighted least squared problem)
 * 
 * @param[in] x are 2D coordinates (x,y,1) in each image
 * @param[in] Ps is the list of projective matrices for each camera
 * @param[out] X is the estimated 3D point
 * @param[in] weights a (optional) list of weights for each point
 */
void TriangulateNViewAlgebraic(const Mat2X &x, 
                               const std::vector< Mat34 > &Ps,
                               Vec4 *X, 
                               const std::vector<double> *weights = nullptr);

/**
 * @brief Compute a 3D position of a point from several images of it. In particular,
 * compute the projective point X in R^4 such that x ~ PX.
 * Algorithm is Lo-RANSAC
 * It can return the the list of the cameras set as intlier by the Lo-RANSAC algorithm.
 * 
 * @param[in] x are 2D coordinates (x,y,1) in each image
 * @param[in] Ps is the list of projective matrices for each camera
 * @param[out] X is the estimated 3D point
 * @param[out] inliersIndex (optional) store the index of the cameras (following Ps ordering, not the view_id) set as Inliers by Lo-RANSAC
 * @param[in] thresholdError (optional) set a threashold value to the Lo-RANSAC scorer
 */                               
void TriangulateNViewLORANSAC(const Mat2X &x, 
                              const std::vector< Mat34 > &Ps,
                              std::mt19937 & generator,
                              Vec4 *X,
                              std::vector<std::size_t> *inliersIndex = NULL,
                              const double & thresholdError = 4.0);                               

//Iterated linear method

class Triangulation
{
public:

  std::size_t size() const { return views.size();}

  void clear()  { views.clear();}

  void add(const Mat34& projMatrix, const Vec2 & p)
  {
    views.emplace_back(projMatrix, p);
  }

  // Return squared L2 sum of error
  double error(const Vec3 &X) const;

  Vec3 compute(int iter = 3) const;

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Accessors

  // These values are defined after a successful call to compute
  double minDepth() const { return zmin; }
  double maxDepth() const { return zmax; }
  double error()    const { return err; }

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Data members

protected:
  mutable double zmin; // min depth, mutable since modified in compute(...) const;
  mutable double zmax; // max depth, mutable since modified in compute(...) const;
  mutable double err; // re-projection error, mutable since modified in compute(...) const;
  std::vector< std::pair<Mat34, Vec2> > views; // Proj matrix and associated image point
};

struct TriangulateNViewsSolver 
{

  /**
   * @brief Return the minimum number of required samples
   * @return minimum number of required samples
   */
  inline std::size_t getMinimumNbRequiredSamples() const
  {
    return 2;
  }

  /**
   * @brief Return the maximum number of models
   * @return maximum number of models
   */
  inline std::size_t getMaximumNbModels() const
  {
    return 1;
  }

  void solve(const Mat2X& x, const std::vector<Mat34>& Ps, std::vector<robustEstimation::MatrixModel<Vec4>>& X) const;
  
  void solve(const Mat2X& x, const std::vector<Mat34>& Ps, std::vector<robustEstimation::MatrixModel<Vec4>>& X, const std::vector<double>& weights) const;

};

} // namespace multiview
} // namespace aliceVision
