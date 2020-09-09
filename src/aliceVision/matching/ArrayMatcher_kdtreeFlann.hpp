// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/matching/ArrayMatcher.hpp>
#include <aliceVision/config.hpp>
#include <aliceVision/alicevision_omp.hpp>

#include "flann/flann.hpp"

#include <memory>

namespace aliceVision {
namespace matching  {

/// Implement ArrayMatcher as a FLANN KDtree matcher.
// http://www.cs.ubc.ca/~mariusm/index.php/FLANN/FLANN
// David G. Lowe and Marius Muja
//
// By default use squared L2 metric (flann::L2<Scalar>)
// sqrt is monotonic so for performance reason we do not compute it.

template < typename Scalar = float, typename  Metric = flann::L2<Scalar> >
class ArrayMatcher_kdtreeFlann : public ArrayMatcher<Scalar, Metric>
{
  public:
  typedef typename Metric::ResultType DistanceType;

  ArrayMatcher_kdtreeFlann() = default;

  virtual ~ArrayMatcher_kdtreeFlann() = default;

  /**
   * Build the matching structure
   *
   * \param[in] dataset   Input data.
   * \param[in] nbRows    The number of component.
   * \param[in] dimension Length of the data contained in the each
   *  row of the dataset.
   *
   * \return True if success.
   */
  bool Build(std::mt19937 & randomNumberGenerator,const Scalar * dataset, int nbRows, int dimension)
  {
    if (nbRows <= 0)
      return false;

    _dimension = dimension;
    //-- Build Flann Matrix container (map to already allocated memory)
    _datasetM.reset(
        new flann::Matrix<Scalar>((Scalar*)dataset, nbRows, dimension));

    //-- Build FLANN index
    _index.reset(
        new flann::Index<Metric> (*_datasetM, flann::KDTreeIndexParams(4)));
    _index->buildIndex();

    return true;
  }

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
  bool SearchNeighbour( const Scalar * query, int * indice, DistanceType * distance)
  {
    if (_index.get() == nullptr)
      return false;
    
    int * indicePTR = indice;
    DistanceType * distancePTR = distance;
    flann::Matrix<Scalar> queries((Scalar*)query, 1, _dimension);

    flann::Matrix<int> indices(indicePTR, 1, 1);
    flann::Matrix<DistanceType> dists(distancePTR, 1, 1);
    // do a knn search, using 128 checks
    return (_index->knnSearch(queries, indices, dists, 1, flann::SearchParams(128)) > 0);
  }

  /**
   * Search the N nearest Neighbor of the scalar array query.
   *
   * \param[in]   query           The query array
   * \param[in]   nbQuery         The number of query rows
   * \param[out]  indices   The corresponding (query, neighbor) indices
   * \param[out]  pvec_distances  The distances between the matched arrays.
   * \param[out]  NN              The number of maximal neighbor that will be searched.
   *
   * \return True if success.
   */
  bool SearchNeighbours
  (
    const Scalar * query, int nbQuery,
    IndMatches * pvec_indices,
    std::vector<DistanceType> * pvec_distances,
    size_t NN
  )
  {
    if (_index.get() == nullptr || NN > _datasetM->rows)
      return false;

    std::vector<DistanceType> vec_distances(nbQuery * NN);
    DistanceType * distancePTR = &(vec_distances[0]);
    flann::Matrix<DistanceType> dists(distancePTR, nbQuery, NN);

    std::vector<int> vec_indices(nbQuery * NN);
    int * indicePTR = &(vec_indices[0]);
    flann::Matrix<int> indices(indicePTR, nbQuery, NN);

    flann::Matrix<Scalar> queries((Scalar*)query, nbQuery, _dimension);
    // do a knn search, using 128 checks
    flann::SearchParams params(128);
    params.cores = omp_get_max_threads();

    if (_index->knnSearch(queries, indices, dists, NN, params) <= 0)
      return false;

    // Save the resulting found indices
    pvec_indices->reserve(nbQuery * NN);
    pvec_distances->reserve(nbQuery * NN);
    for (size_t i = 0; i < nbQuery; ++i)
    {
      for (size_t j = 0; j < NN; ++j)
      {
          pvec_indices->emplace_back(i, vec_indices[i*NN+j]);
          pvec_distances->emplace_back(vec_distances[i*NN+j]);
      }
    }
    return true;
  }

  private:

  std::unique_ptr< flann::Matrix<Scalar> > _datasetM;
  std::unique_ptr< flann::Index<Metric> > _index;
  std::size_t _dimension;
};

} // namespace matching
} // namespace aliceVision
