// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// Copyright (c) 2010 libmv contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>

#include <vector>

namespace aliceVision {
namespace multiview {
	
/**
 * @brief Build a 9 x n matrix from point matches, where each row is equivalent to the
 * equation x'T*F*x = 0 for a single correspondence pair (x', x). The domain of
 * the matrix is a 9 element vector corresponding to F. In other words, set up
 * the linear system
 *
 *   Af = 0,
 *
 * where f is the F matrix as a 9-vector rather than a 3x3 matrix (row
 * major). If the points are well conditioned and there are 8 or more, then
 * the nullspace should be rank one. If the nullspace is two dimensional,
 * then the rank 2 constraint must be enforced to identify the appropriate F
 * matrix.
 *
 * @note that this does not resize the matrix A; it is expected to have the
 * appropriate size already.
 */
template<typename TMatX, typename TMatA>
inline void encodeEpipolarEquation(const TMatX& x1, const TMatX& x2, TMatA* A, const std::vector<double> *weights = nullptr)
{
  assert(x1.cols()==x2.cols());

  if(weights != nullptr)
  {
    assert(x1.cols()==weights->size());
  }

  for(typename TMatX::Index i = 0; i < x1.cols(); ++i)
  {
    const Vec2 xx1 = x1.col(i);
    const Vec2 xx2 = x2.col(i);

    A->row(i) <<
      xx2(0) * xx1(0),  // 0 represents x coords,
      xx2(0) * xx1(1),  // 1 represents y coords.
      xx2(0),
      xx2(1) * xx1(0),
      xx2(1) * xx1(1),
      xx2(1),
      xx1(0),
      xx1(1),
      1.0;

    if(weights != nullptr)
      A->row(i) *= (*weights)[i];
  }
}


template <typename TMatX, typename TMatA>
inline void encodeEpipolarSphericalEquation(const TMatX& x1, const TMatX& x2, TMatA* A,
                                            const std::vector<double>* weights = nullptr)
{
    assert(x1.cols() == x2.cols());
    if(weights)
    {
        assert(x1.cols() == weights->size());
    }
    for(typename TMatX::Index i = 0; i < x1.cols(); ++i)
    {
        const Vec3 xx1 = x1.col(i);
        const Vec3 xx2 = x2.col(i);
        A->row(i) <<
            xx2(0) * xx1(0),  // 0 represents x coords,
            xx2(0) * xx1(1),  // 1 represents y coords.
            xx2(0) * xx1(2),
            xx2(1) * xx1(0),
            xx2(1) * xx1(1),
            xx2(1) * xx1(2),
            xx2(2) * xx1(0),
            xx2(2) * xx1(1),
            xx2(2) * xx1(2);

        if(weights)
        {
            A->row(i) *= (*weights)[i];
        }
    }
}


} // namespace multiview
} // namespace aliceVision
