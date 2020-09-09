// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/matching/ArrayMatcher.hpp>
#include <aliceVision/feature/metric.hpp>
#include <aliceVision/stl/indexedSort.hpp>

#include <aliceVision/config.hpp>

#include <memory>
#include <iostream>


namespace aliceVision {
namespace matching {

// By default compute square(L2 distance).
template < typename Scalar = float, typename Metric = feature::L2_Simple<Scalar> >
class ArrayMatcher_bruteForce  : public ArrayMatcher<Scalar, Metric>
{
  public:
  typedef typename Metric::ResultType DistanceType;

  ArrayMatcher_bruteForce()   {}
  virtual ~ArrayMatcher_bruteForce() {
    memMapping.reset();
  }

  /**
   * Build the matching structure
   *
   * \param[in] dataset   Input data.
   * \param[in] nbRows    The number of component.
   * \param[in] dimension Length of the data contained in the dataset.
   *
   * \return True if success.
   */
  bool Build(std::mt19937 & randomNumberGenerator,const Scalar * dataset, int nbRows, int dimension) {
    if (nbRows < 1) {
      memMapping.reset(nullptr);
      return false;
    }
    memMapping.reset(new Eigen::Map<BaseMat>( (Scalar*)dataset, nbRows, dimension) );
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
  bool SearchNeighbour( const Scalar * query,
                        int * indice, DistanceType * distance)
  {
    if (memMapping.get() == nullptr)
      return false;

      //matrix representation of the input data;
      Eigen::Map<BaseMat> mat_query((Scalar*)query, 1, (*memMapping).cols() );
      Metric metric;
      std::vector<DistanceType> vec_dist((*memMapping).rows(), 0.0);
    for (int i = 0; i < (*memMapping).rows(); ++i)
    {
        // Compute Distance Metric
        vec_dist[i] = metric( (Scalar*)query, (*memMapping).row(i).data(), (*memMapping).cols() );
      }
      if (!vec_dist.empty())
      {
        // Find the minimum distance :
        typename std::vector<DistanceType>::const_iterator min_iter =
          min_element( vec_dist.begin(), vec_dist.end());
        *indice =std::distance(
          typename std::vector<DistanceType>::const_iterator(vec_dist.begin()),
          min_iter);
        *distance = static_cast<DistanceType>(*min_iter);
      }
      return true;
  }


/**
   * Search the N nearest Neighbor of the scalar array query.
   *
   * \param[in]   query     The query array
   * \param[in]   nbQuery   The number of query rows
   * \param[out]  indices   The corresponding (query, neighbor) indices
   * \param[out]  distances The distances between the matched arrays.
   * \param[out]  NN        The number of maximal neighbor that will be searched.
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
    if (memMapping.get() == nullptr)  {
      return false;
    }

    if (NN > (*memMapping).rows() || nbQuery < 1) {
      return false;
    }

    //matrix representation of the input data;
    Eigen::Map<BaseMat> mat_query((Scalar*)query, nbQuery, (*memMapping).cols());
    Metric metric;

    pvec_distances->resize(nbQuery * NN);
    pvec_indices->resize(nbQuery * NN);

    #pragma omp parallel for schedule(dynamic)
    for (int queryIndex=0; queryIndex < nbQuery; ++queryIndex) 
    {
      std::vector<DistanceType> vec_distance((*memMapping).rows(), 0.0);
      const Scalar * queryPtr = mat_query.row(queryIndex).data();
      const Scalar * rowPtr = (*memMapping).data();
      for (int i = 0; i < (*memMapping).rows(); ++i)
      {
        vec_distance[i] = metric( queryPtr,
          rowPtr, (*memMapping).cols() );
        rowPtr += (*memMapping).cols();
      }

      // Find the N minimum distances:
      const int maxMinFound = (int) std::min( size_t(NN), vec_distance.size());
      using namespace stl::indexed_sort;
      std::vector< sort_index_packet_ascend< DistanceType, int> > packet_vec(vec_distance.size());
      sort_index_helper(packet_vec, &vec_distance[0], maxMinFound);

      for (int i = 0; i < maxMinFound; ++i)
      {
        (*pvec_distances)[queryIndex*NN+i] = packet_vec[i].val;
        (*pvec_indices)[queryIndex*NN+i] = IndMatch(queryIndex, packet_vec[i].index);
      }
    }
    return true;
  };

private:
  typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> BaseMat;
  /// Use a memory mapping in order to avoid memory re-allocation
  std::unique_ptr< Eigen::Map<BaseMat> > memMapping;
};

}  // namespace matching
}  // namespace aliceVision
