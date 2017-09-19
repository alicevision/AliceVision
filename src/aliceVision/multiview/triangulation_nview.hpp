// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#ifndef ALICEVISION_MULTIVIEW_TRIANGULATION_NVIEW_H
#define ALICEVISION_MULTIVIEW_TRIANGULATION_NVIEW_H

#include "aliceVision/numeric/numeric.hpp"

#include <vector>

namespace aliceVision {

/// Compute a 3D position of a point from several images of it. In particular,
///  compute the projective point X in R^4 such that x = PX.
/// Algorithm is the standard DLT; for derivation see appendix of Keir's thesis.
void TriangulateNView(const Mat2X &x, // x's are 2D coordinates (x,y,1) in each image
                      const std::vector< Mat34 > &Ps, // Ps are projective cameras
                      Vec4 *X);

// This method uses the algebraic distance approximation.
// Note that this method works better when the 2D points are normalized
// with an isotopic normalization.
void TriangulateNViewAlgebraic(const Mat2X &x, // x's are 2D coordinates (x,y,1) in each image
                               const std::vector< Mat34 > &Ps, // Ps are projective cameras.
                               Vec4 *X);

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

} // namespace aliceVision

#endif  // ALICEVISION_MULTIVIEW_TRIANGULATION_NVIEW_H
