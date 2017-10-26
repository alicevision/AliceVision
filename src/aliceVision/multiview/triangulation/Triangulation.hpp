// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>

#include <vector>

namespace aliceVision {

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
  enum { MINIMUM_SAMPLES = 2 };
  enum { MAX_MODELS = 1 };

  static void Solve(const Mat2X &x, const std::vector< Mat34 > &Ps, std::vector<Vec4> &X);
  
  static void Solve(const Mat2X &x, const std::vector< Mat34 > &Ps, std::vector<Vec4> &X, const std::vector<double> &weights);

};

} // namespace aliceVision
