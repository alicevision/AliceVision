// Copyright (c) 2012, 2013 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef OPENMVG_MATCHING_ARRAYMATCHER_BRUTE_FORCE_H
#define OPENMVG_MATCHING_ARRAYMATCHER_BRUTE_FORCE_H

#include "openMVG/numeric/numeric.h"
#include "openMVG/matching/matching_interface.hpp"
#include "openMVG/matching/metric.hpp"
#include "openMVG/stl/indexed_sort.hpp"
#include <openMVG/config.hpp>
#include <memory>

#include <iostream>
#include <stdio.h>
#include <fstream>

#include <typeinfo>
#include <inttypes.h>
#include <math.h>
#include <cmath>


namespace openMVG {
namespace matching {

// By default compute square(L2 distance).
template < typename Scalar = float, typename Metric = L2_Simple<Scalar> >
class ArrayMatcherBruteForce  : public ArrayMatcher<Scalar, Metric>
{
  public:
  typedef typename Metric::ResultType DistanceType;

  ArrayMatcherBruteForce()   {}
  virtual ~ArrayMatcherBruteForce() {
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
  bool Build(const Scalar * dataset, int nbRows, int dimension) {
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
   
    //#pragma omp parallel for schedule(dynamic)
    for (int queryIndex=0; queryIndex < nbQuery; ++queryIndex) //641 times
      {
	std::vector<DistanceType> vec_distance((*memMapping).rows(), 0.0);
	const Scalar * queryPtr = mat_query.row(queryIndex).data();
	const Scalar * rowPtr = (*memMapping).data();


    //std::cout << "a = " << std::bitset<128 * 8>(queryPtr)  << std::endl;
    
	int diff0, diff1, diff2, diff3, dota;
	
	int best = 0; //size
	int nextBest = 0; 
	
	int tmp;
	int bestIndex = -1;
	int nextBestIndex = -1;
	int res;
	
	for (int i = 0; i < (*memMapping).rows(); ++i) {//477 times
	  

	  res = 0;
	  for(int iter = 0; iter < (*memMapping).cols(); iter+=4) { //calculating dot product
          
          
	    diff0 = rowPtr[iter] * queryPtr[iter];
	    diff1 = rowPtr[iter+1] * queryPtr[iter+1];
	    diff2 = rowPtr[iter+2] * queryPtr[iter+2];
	    diff3 = rowPtr[iter+3] * queryPtr[iter+3];
  
	    res += (diff0 + diff1 + diff2 + diff3);	   
	  }           

	  vec_distance[i] = res;

	  //replace best result
	  if(res > best) {	      	    	      
	    nextBest = best;
	    best = res;
	    nextBestIndex = bestIndex;
	    bestIndex = i;
	  } else if(res > nextBest) {
	    nextBest = res;
	    nextBestIndex = i;
	  }
	    	    
	  rowPtr += (*memMapping).cols();
	}

	//calculate the two bet vectors using L2_Vectorized
	const Scalar * tmp1 = (*memMapping).data();
	tmp1 += bestIndex*(*memMapping).cols();
	vec_distance[bestIndex] = metric(queryPtr, tmp1, (*memMapping).cols()); //
	const Scalar * tmp2 = (*memMapping).data();
	tmp2 += nextBestIndex*(*memMapping).cols();
	vec_distance[nextBestIndex] = metric(queryPtr, tmp2, (*memMapping).cols());

	(*pvec_distances)[queryIndex*NN+0] = vec_distance[bestIndex];
	(*pvec_indices)[queryIndex*NN+0] = IndMatch(queryIndex, bestIndex);
	
	(*pvec_distances)[queryIndex*NN+1] = vec_distance[nextBestIndex];
	(*pvec_indices)[queryIndex*NN+1] = IndMatch(queryIndex, nextBestIndex);
	

      }
    return true;
  };
    
  /*
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
    for (int queryIndex=0; queryIndex < nbQuery; ++queryIndex) //641 times
      {
	std::vector<DistanceType> vec_distance((*memMapping).rows(), 0.0);
	const Scalar * queryPtr = mat_query.row(queryIndex).data();
	const Scalar * rowPtr = (*memMapping).data();
		
	for (int i = 0; i < (*memMapping).rows(); ++i) //477 times
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
	
	if(queryIndex == 0) {
	  std::cout << packet_vec[0].index << std::endl;
	  std::cout << packet_vec[1].index << std::endl;
	}
	
	for (int i = 0; i < maxMinFound; ++i)
	  {
	    (*pvec_distances)[queryIndex*NN+i] = packet_vec[i].val;
	    (*pvec_indices)[queryIndex*NN+i] = IndMatch(queryIndex, packet_vec[i].index);
	  }
	
      }
    return true;
  };
  */  
private:
  typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> BaseMat;
  /// Use a memory mapping in order to avoid memory re-allocation
  std::unique_ptr< Eigen::Map<BaseMat> > memMapping;
};

}  // namespace matching
}  // namespace openMVG

#endif  // OPENMVG_MATCHING_ARRAYMATCHER_BRUTE_FORCE_H
