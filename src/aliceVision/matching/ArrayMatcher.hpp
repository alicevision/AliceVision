// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "aliceVision/numeric/numeric.hpp"
#include "aliceVision/matching/IndMatch.hpp"

#include <vector>
#include <random>

namespace aliceVision {
namespace matching {


template < typename Scalar, typename Metric >
class ArrayMatcher
{
  public:
  typedef Scalar ScalarT;
  typedef typename Metric::ResultType DistanceType;
  typedef Metric MetricT;

  ArrayMatcher() {}
  virtual ~ArrayMatcher() {};

  /**
   * Build the matching structure
   *
   * \param[in] dataset   Input data.
   * \param[in] nbRows    The number of component.
   * \param[in] dimension Length of the data contained in the dataset.
   *
   * \return True if success.
   */
  virtual bool Build(std::mt19937 & randomNumberGenerator,const Scalar * dataset, int nbRows, int dimension)=0;

  /**
   * Search the nearest Neighbor of the scalar array query.
   *
   * \param[in]   query     The query array
   * \param[out]  indice    The indice of array in the dataset that
   *  have been computed as the nearest array.
   * \param[out]  distance  The distance between the two arrays.
   *
   * \return True if success.
   */
  virtual bool SearchNeighbour( const Scalar * query,
                                int * indice, DistanceType * distance)=0;


/**
   * Search the N nearest Neighbor of the scalar array query.
   *
   * \param[in]   query     The query array
   * \param[in]   nbQuery   The number of query rows
   * \param[out]  indices   The corresponding (query, neighbor) indices
   * \param[out]  distances The distances between the matched arrays.
   * \param[out]  NN        The number of maximal neighbor that could
   *  will be searched.
   *
   * \return True if success.
   */
  virtual bool SearchNeighbours( const Scalar * query, int nbQuery,
                                  IndMatches * indices,
                                  std::vector<DistanceType> * distances,
                                  size_t NN)=0;
};

}  // namespace matching
}  // namespace aliceVision
