// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/matchingImageCollection/ImageCollectionMatcher_cascadeHashing.hpp>
#include <aliceVision/matching/ArrayMatcher_cascadeHashing.hpp>
#include <aliceVision/matching/IndMatchDecorator.hpp>
#include <aliceVision/matching/filters.hpp>
#include <aliceVision/config.hpp>

#include <boost/progress.hpp>

namespace aliceVision {
namespace matchingImageCollection {

using namespace aliceVision::matching;
using namespace aliceVision::feature;

ImageCollectionMatcher_cascadeHashing
::ImageCollectionMatcher_cascadeHashing
(
  float distRatio
):IImageCollectionMatcher(), f_dist_ratio_(distRatio)
{
}

namespace impl
{
template <typename ScalarT>
void Match
(
  std::mt19937 & gen,
  const feature::RegionsPerView& regionsPerView,
  const PairSet & pairs,
  EImageDescriberType descType,
  float fDistRatio,
  PairwiseMatches & map_PutativesMatches // the pairwise photometric corresponding points
)
{
  boost::progress_display my_progress_bar( pairs.size() );

  // Collect used view indexes
  std::set<IndexT> used_index;
  // Sort pairs according the first index to minimize later memory swapping
  typedef std::map<IndexT, std::vector<IndexT> > Map_vectorT;
  Map_vectorT map_Pairs;
  for (PairSet::const_iterator iter = pairs.begin(); iter != pairs.end(); ++iter)
  {
    map_Pairs[iter->first].push_back(iter->second);
    used_index.insert(iter->first);
    used_index.insert(iter->second);
  }

  typedef Eigen::Matrix<ScalarT, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> BaseMat;

  // Init the cascade hasher
  CascadeHasher cascade_hasher;
  if (!used_index.empty())
  {
    const IndexT I = *used_index.begin();
    const feature::Regions &regionsI = regionsPerView.getRegions(I, descType);
    const size_t dimension = regionsI.DescriptorLength();
    cascade_hasher.Init(gen, dimension);
  }

  std::map<IndexT, HashedDescriptions> hashed_base_;

  // Compute the zero mean descriptor that will be used for hashing (one for all the image regions)
  Eigen::VectorXf zero_mean_descriptor;
  {
    Eigen::MatrixXf matForZeroMean;
    for (int i =0; i < used_index.size(); ++i)
    {
      std::set<IndexT>::const_iterator iter = used_index.begin();
      std::advance(iter, i);
      const IndexT I = *iter;
      const feature::Regions &regionsI = regionsPerView.getRegions(I, descType);
      const ScalarT * tabI =
        reinterpret_cast<const ScalarT*>(regionsI.DescriptorRawData());
      const size_t dimension = regionsI.DescriptorLength();
      if (i==0)
      {
        matForZeroMean.resize(used_index.size(), dimension);
        matForZeroMean.fill(0.0f);
      }
      if (regionsI.RegionCount() > 0)
      {
        Eigen::Map<BaseMat> mat_I( (ScalarT*)tabI, regionsI.RegionCount(), dimension);
        matForZeroMean.row(i) = CascadeHasher::GetZeroMeanDescriptor(mat_I);
      }
    }
    zero_mean_descriptor = CascadeHasher::GetZeroMeanDescriptor(matForZeroMean);
  }

  // Index the input regions
  #pragma omp parallel for schedule(dynamic)
  for (int i =0; i < used_index.size(); ++i)
  {
    std::set<IndexT>::const_iterator iter = used_index.begin();
    std::advance(iter, i);
    const IndexT I = *iter;
    const feature::Regions &regionsI = regionsPerView.getRegions(I, descType);
    const ScalarT * tabI =
      reinterpret_cast<const ScalarT*>(regionsI.DescriptorRawData());
    const size_t dimension = regionsI.DescriptorLength();

    Eigen::Map<BaseMat> mat_I( (ScalarT*)tabI, regionsI.RegionCount(), dimension);
    HashedDescriptions hashed_description = cascade_hasher.CreateHashedDescriptions(mat_I,
      zero_mean_descriptor);
    #pragma omp critical
    {
      hashed_base_[I] = std::move(hashed_description);
    }
  }

  // Perform matching between all the pairs
  for (Map_vectorT::const_iterator iter = map_Pairs.begin();
    iter != map_Pairs.end(); ++iter)
  {
    const IndexT I = iter->first;
    const std::vector<IndexT> & indexToCompare = iter->second;

    const feature::Regions &regionsI = regionsPerView.getRegions(I, descType);
    if (regionsI.RegionCount() == 0)
    {
      my_progress_bar += indexToCompare.size();
      continue;
    }

    const std::vector<feature::PointFeature> pointFeaturesI = regionsI.GetRegionsPositions();
    const ScalarT * tabI =
      reinterpret_cast<const ScalarT*>(regionsI.DescriptorRawData());
    const size_t dimension = regionsI.DescriptorLength();
    Eigen::Map<BaseMat> mat_I( (ScalarT*)tabI, regionsI.RegionCount(), dimension);
    #pragma omp parallel for schedule(dynamic)
    for (int j = 0; j < (int)indexToCompare.size(); ++j)
    {
      size_t J = indexToCompare[j];
      const feature::Regions &regionsJ = regionsPerView.getRegions(I, descType);

      if (!regionsPerView.viewExist(J)
          || regionsI.Type_id() != regionsJ.Type_id())
      {
        #pragma omp critical
        ++my_progress_bar;
        continue;
      }

      // Matrix representation of the query input data;
      const ScalarT * tabJ = reinterpret_cast<const ScalarT*>(regionsJ.DescriptorRawData());
      Eigen::Map<BaseMat> mat_J( (ScalarT*)tabJ, regionsJ.RegionCount(), dimension);

      IndMatches pvec_indices;
      typedef typename Accumulator<ScalarT>::Type ResultType;
      std::vector<ResultType> pvec_distances;
      pvec_distances.reserve(regionsJ.RegionCount() * 2);
      pvec_indices.reserve(regionsJ.RegionCount() * 2);

      // Match the query descriptors to the database
      cascade_hasher.Match_HashedDescriptions<BaseMat, ResultType>(
        hashed_base_[J], mat_J,
        hashed_base_[I], mat_I,
        &pvec_indices, &pvec_distances);

      std::vector<int> vec_nn_ratio_idx;
      // Filter the matches using a distance ratio test:
      //   The probability that a match is correct is determined by taking
      //   the ratio of distance from the closest neighbor to the distance
      //   of the second closest.
      matching::NNdistanceRatio(
        pvec_distances.begin(), // distance start
        pvec_distances.end(),   // distance end
        2, // Number of neighbor in iterator sequence (minimum required 2)
        vec_nn_ratio_idx, // output (indices that respect the distance Ratio)
        Square(fDistRatio));

      matching::IndMatches vec_putative_matches;
      vec_putative_matches.reserve(vec_nn_ratio_idx.size());
      for (size_t k=0; k < vec_nn_ratio_idx.size(); ++k)
      {
        const size_t index = vec_nn_ratio_idx[k];
        vec_putative_matches.emplace_back(pvec_indices[index*2]._j, pvec_indices[index*2]._i);
      }

      // Remove duplicates
      matching::IndMatch::getDeduplicated(vec_putative_matches);

      // Remove matches that have the same (X,Y) coordinates
      const std::vector<feature::PointFeature> pointFeaturesJ = regionsJ.GetRegionsPositions();
      matching::IndMatchDecorator<float> matchDeduplicator(vec_putative_matches,
        pointFeaturesI, pointFeaturesJ);
      matchDeduplicator.getDeduplicated(vec_putative_matches);

      #pragma omp critical
      {
        ++my_progress_bar;
        if (!vec_putative_matches.empty())
        {
          assert(map_PutativesMatches.count(std::make_pair(I,J)) == 0);
          map_PutativesMatches[std::make_pair(I,J)].emplace(descType, std::move(vec_putative_matches));
        }
      }
    }
  }
}
} // namespace impl

void ImageCollectionMatcher_cascadeHashing::Match
(
  std::mt19937 & gen,
  const feature::RegionsPerView& regionsPerView,
  const PairSet & pairs,
  feature::EImageDescriberType descType,
  PairwiseMatches & map_PutativesMatches // the pairwise photometric corresponding points
) const
{

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OPENMP)
  ALICEVISION_LOG_DEBUG("Using the OPENMP thread interface");
#endif

  if (regionsPerView.isEmpty())
    return;

  const feature::Regions& regions = regionsPerView.getFirstViewRegions(descType);

  if (regions.IsBinary())
    return;

  if(regions.Type_id() == typeid(unsigned char).name())
  {
    impl::Match<unsigned char>(
      gen,
      regionsPerView,
      pairs,
      descType,
      f_dist_ratio_,
      map_PutativesMatches);
  }
  else
  if(regions.Type_id() == typeid(float).name())
  {
    impl::Match<float>(
      gen,
      regionsPerView,
      pairs,
      descType,
      f_dist_ratio_,
      map_PutativesMatches);
  }
  else
  {
    ALICEVISION_LOG_WARNING("Matcher not implemented for this region type");
  }
}

} // namespace aliceVision
} // namespace matchingImageCollection
