// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <Eigen/Core>
#include <Eigen/StdVector>

namespace aliceVision {
namespace voctree {

/**
 * @brief Meta-function to get the default allocator for a particular feature type.
 *
 * Defaults to \c std::allocator<Feature>.
 */
template<class Feature>
struct DefaultAllocator
{
  typedef std::allocator<Feature> type;
};

// Specialization to use aligned allocator for Eigen::Matrix types.

template<typename Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
struct DefaultAllocator< Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols> >
{
  typedef Eigen::aligned_allocator<Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols> > type;
};

}
}
