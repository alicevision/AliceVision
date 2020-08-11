// This file is part of the AliceVision project.
// Copyright (c) 2020 AliceVision contributors.
// Copyright (c) 2019 openMVG contributors.
// Copyright (c) 2019 Romain Janvier and Pierre Moulon
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/matching/ArrayMatcher.hpp>
#include <aliceVision/matching/CascadeHasher.hpp>
#include <aliceVision/matching/IndMatch.hpp>

#include <aliceVision/system/Logger.hpp>

#include <dependencies/hnswlib/hnswlib/hnswlib.h>

#include <memory>

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OPENMP)
#include <omp.h>
#endif

#include <vector>

#include <random>
#include <cmath>

using hnswlib::HierarchicalNSW;
using hnswlib::L2Space;
using hnswlib::L2SpaceI;
using hnswlib::SpaceInterface;

namespace aliceVision
{
namespace matching
{


// Implementation of descriptor matching using the cascade hashing method of [1].
// If you use this matcher, please cite the paper.
// template Metric parameter is ignored (by default compute square(L2 distance)).
/**
 * @brief Implementation of descriptor matching using the cascade hashing method of @cite Cheng2014.
 * @tparam Scalar the type of data
 * @tparam Metric is ignored (by default compute square(L2 distance)).
 * @detail Jian Cheng, Cong Leng, Jiaxiang Wu, Hainan Cui, and Hanqing Lu. 2014. Fast and Accurate Image Matching with Cascade Hashing for 3D Reconstruction. In Proceedings of the 2014 IEEE Conference on Computer Vision and Pattern Recognition (CVPR ’14). IEEE Computer Society, USA, 1–8. DOI:https://doi.org/10.1109/CVPR.2014.8
 */
template <typename Scalar = float, typename Metric = L2_Simple<Scalar>>
class ArrayMatcher_hnswlib : public ArrayMatcher<Scalar, Metric>
{
public:
    using DistanceType = typename Metric::ResultType;

    // Some initialization
    ArrayMatcher_hnswlib() = default;
    ~ArrayMatcher_hnswlib() override = default;

    /**
     * Build the matching structure
     *
     * @param[in] dataset   Input data.
     * @param[in] nbRows    The number of component.
     * @param[in] dimension Length of the data contained in the dataset.
     *
     * \return True if success.
     */
    bool Build(const Scalar* dataset, int nbRows, int dimension)
    {
        reset();

        if(nbRows < 1)
            return false;

        dimension_ = dimension;

        // Here this is tricky since there is no specialization
        if(typeid(DistanceType) == typeid(int))
        {
            HNSWmetric.reset(dynamic_cast<SpaceInterface<DistanceType>*>(new L2SpaceI(dimension)));
        }
        else if(typeid(DistanceType) == typeid(float))
        {
            HNSWmetric.reset(dynamic_cast<SpaceInterface<DistanceType>*>(new L2Space(dimension)));
        }
        else
        {
            ALICEVISION_LOG_WARNING("HNSW matcher: this type of distance is not handled Yet");
            return false;
        }

        HNSWmatcher.reset(new HierarchicalNSW<DistanceType>(HNSWmetric.get(), nbRows, 16, 100));
        HNSWmatcher->setEf(16);

        // add first point..
        HNSWmatcher->addPoint((void*)(dataset), (size_t)0);
        //...and the other in //
        #ifdef ALICEVISION_IS_DEFINED
        #pragma omp parallel for
        #endif
        for(int i = 1; i < nbRows; i++)
        {
            HNSWmatcher->addPoint((void*)(dataset + dimension * i), (size_t)i);
        }

        return true;
    };

    /**
     * Search the nearest Neighbor of the scalar array query.
     *
     * @param[in]   query     The query array
     * @param[out]  indice    The indice of array in the dataset that
     *  have been computed as the nearest array.
     * @param[out]  distance  The distance between the two arrays.
     *
     * @return True if success.
     */
    bool SearchNeighbour(const Scalar* query, int* indice, DistanceType* distance)
    {
        ALICEVISION_LOG_WARNING("This matcher is not made to match a single query");
        return false;
    }

    /**
     * Search the N nearest Neighbor of the scalar array query.
     *
     * @param[in]   query     The query array
     * @param[in]   nbQuery   The number of query rows
     * @param[out]  indices   The corresponding (query, neighbor) indices
     * @param[out]  distances  The distances between the matched arrays.
     * @param[out]  NN        The number of maximal neighbor that will be searched.
     *
     * @return True if success.
     */
    bool SearchNeighbours(const Scalar* query, int nbQuery, IndMatches* pvec_indices,
                          std::vector<DistanceType>* pvec_distances, size_t NN)
    {
      if(HNSWmatcher.get() == nullptr)
      {
        return false;
      }

      pvec_distances->resize(nbQuery * NN);
      pvec_indices->resize(nbQuery * NN);

      #ifdef ALICEVISION_IS_DEFINED
      #pragma omp parallel for
      #endif
      for(int i = 0; i < nbQuery; ++i)
      {
        auto result = HNSWmatcher->searchKnn(
          (const void*)(query + dimension_ * i), NN,
          [](const std::pair<DistanceType, size_t>& var1, const std::pair<DistanceType, size_t>& var2) -> bool {
              return var1.first < var2.first;
          });

        /* direct indexing approach chosen over omp critical (OpenMP) approach due
         * to possible performance loss from waiting threads in OpenMP approach
        */
        {
          for (int j = 0; j < NN; ++j) {
            const auto &res = result[j];
            (*pvec_indices)[i * NN + j] = IndMatch(i, res.second);
            (*pvec_distances)[i * NN + j] = res.first;
          }
        }

      }

      return true;
    };

private:
    void reset() {
      HNSWmetric.reset(nullptr);
      HNSWmatcher.reset(nullptr);
      dimension_ = 0;
    }

private:
  int dimension_{0};
  std::unique_ptr<SpaceInterface<DistanceType>> HNSWmetric;
  std::unique_ptr<HierarchicalNSW<DistanceType>> HNSWmatcher;
};

} // namespace matching
} // namespace aliceVision
