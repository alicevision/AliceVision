// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/matching/IndMatch.hpp>
#include <aliceVision/feature/RegionsPerView.hpp>
#include <aliceVision/feature/Regions.hpp>
#include <aliceVision/camera/IntrinsicBase.hpp>

#include <vector>

namespace aliceVision {
namespace matching {

/**
 * @brief Guided Matching (features only):
 *        Use a model to find valid correspondences:
 *        Keep the best corresponding points for the given model under the
 *        user specified distance.
 *
 * @tparam ModelT The used model type
 * @tparam ErrorT The metric to compute distance to the model
 *
 * @param[in] mod The model
 * @param[in] xLeft The left data points
 * @param[in] xRight The right data points
 * @param[in] errorTh Maximal authorized error threshold
 * @param[out] out_validMatches Ouput corresponding index
 */
template<typename ModelT, typename ErrorT> 
void guidedMatching(const ModelT& mod, const Mat& xLeft, const Mat& xRight, double errorTh, matching::IndMatches& out_validMatches)
{
  assert(xLeft.rows() == xRight.rows());

  const ErrorT errorEstimator = ErrorT();

  // looking for the corresponding points that have
  // the smallest distance (smaller than the provided Threshold)
  for(Mat::Index i = 0; i < xLeft.cols(); ++i)
  {
    double min = std::numeric_limits<double>::max();
    matching::IndMatch match;
    for(Mat::Index j = 0; j < xRight.cols(); ++j)
    {
      // compute the geometric error: error to the model
      const double err = errorEstimator.error(mod, xLeft.col(i), xRight.col(j));

      // if smaller error update corresponding index
      if(err < errorTh && err < min)
      {
        min = err;
        match = matching::IndMatch(i, j);
      }
    }
    if(min < errorTh)
    {
      // save the best corresponding index
      out_validMatches.push_back(match);
    }
  }

  // remove duplicates (when multiple points at same position exist)
  matching::IndMatch::getDeduplicated(out_validMatches);
}

/**
 * @brief Struct to help filtering of correspondence according update of two smallest distance.
 * @note useful for descriptor distance ratio filtering
 */
template <typename DistT>
struct distanceRatio
{
  /// best and second best distance
  DistT bd, sbd;
  /// best corresponding index
  std::size_t idx;

  distanceRatio()
    : bd(std::numeric_limits<DistT>::max())
    , sbd(std::numeric_limits<DistT>::max())
    , idx(0)
  {}

  /**
   * @brief Update match according the provided distance
   */
  inline bool update(std::size_t index, DistT dist)
  {
    if(dist < bd) // best than any previous
    {
      idx = index;
      // update and swap
      sbd = dist;
      std::swap(bd, sbd);
      return true;
    }
    else if(dist < sbd)
    {
      sbd = dist;
      return true;
    }
    return false;
  }

  /**
   * @brief Return if the ratio of distance is ok or not
   */
  inline bool isValid(const double distRatio) const
  {
    // check:
    // - that two best distance have been found
    // - the distance ratio
    return (sbd != std::numeric_limits<DistT>::max()
            && bd < distRatio * sbd);
  }
};


/**
 * @brief Guided Matching (features + descriptors with distance ratio):
 *        Use a model to find valid correspondences:
 *        Keep the best corresponding points for the given model under the
 *        user specified distance.
 *
 * @tparam ModelT The used model type
 * @tparam ErrorT The metric to compute distance to the model
 * @tparam DescriptorT The descriptor type
 * @tparam MetricT The metric to compare two descriptors
 *
 * @param[in] mod The model
 * @param[in] xLeft The left data points
 * @param[in] lDescriptors The left descriptors
 * @param[in] xRight The right data points
 * @param[in] rDescriptors The right descriptors
 * @param[in] errorTh Maximal authorized error threshold
 * @param[in] distRatio Maximal authorized distance ratio
 * @param[out] vec_corresponding_index Ouput corresponding index
 */
template<typename ModelT, typename ErrorT, typename DescriptorT, typename MetricT>
void guidedMatching(const ModelT& mod,
                    const Mat& xLeft,
                    const std::vector<DescriptorT>& lDescriptors,
                    const Mat& xRight,
                    const std::vector<DescriptorT>& rDescriptors,
                    double errorTh,
                    double distRatio,
                    matching::IndMatches& vec_corresponding_index)
{
  assert(xLeft.rows() == xRight.rows());
  assert(xLeft.cols() == lDescriptors.size());
  assert(xRight.cols() == rDescriptors.size());

  MetricT metric;

  // Looking for the corresponding points that have to satisfy:
  //   1. a geometric distance below the provided Threshold
  //   2. a distance ratio between descriptors of valid geometric correspondencess

  for(Mat::Index i = 0; i < xLeft.cols(); ++i)
  {

    distanceRatio<typename MetricT::ResultType > dR;
    for(Mat::Index j = 0; j < xRight.cols(); ++j)
    {
      // Compute the geometric error: error to the model
      const double geomErr = ErrorT::Error(mod, // The model
                                             xLeft.col(i), 
                                             xRight.col(j)); // The corresponding points
      if(geomErr < errorTh)
      {
        const typename MetricT::ResultType descDist =
                metric(lDescriptors[i].getData(), rDescriptors[j].getData(), DescriptorT::static_size);
        // Update the corresponding points & distance (if required)
        dR.update(j, descDist);
      }
    }
    // Add correspondence only iff the distance ratio is valid
    if(dR.isValid(distRatio))
    {
      // save the best corresponding index
      vec_corresponding_index.emplace_back(i, dR.idx);
    }
  }

  // Remove duplicates (when multiple points at same position exist)
  matching::IndMatch::getDeduplicated(vec_corresponding_index);
}

/**
 * @brief Guided Matching (features + descriptors with distance ratio):
 *        Use a model to find valid correspondences:
 *        Keep the best corresponding points for the given model under the
 *        user specified distance ratio.
 *
 * @tparam ModelT The used model type
 * @tparam ErrorT The metric to compute distance to the model
 *
 * @param[in] mod The model
 * @param[in] camL Optional camera (in order to undistord on the fly feature positions, can be NULL)
 * @param[in] lRegions regions (point features & corresponding descriptors)
 * @param[in] camR Optional camera (in order to undistord on the fly feature positions, can be NULL)
 * @param[in] rRegions regions (point features & corresponding descriptors)
 * @param[in] errorTh Maximal authorized error threshold
 * @param[in] distRatio Maximal authorized distance ratio
 * @param[out] out_matches Ouput corresponding index
 */
template<typename ModelT, typename ErrorT>
void guidedMatching(const ModelT& mod,
                    const camera::IntrinsicBase* camL,
                    const feature::Regions& lRegions,
                    const camera::IntrinsicBase* camR,
                    const feature::Regions& rRegions,
                    double errorTh,
                    double distRatio,
                    matching::IndMatches& out_matches)
{
  // looking for the corresponding points that have to satisfy:
  //   1. a geometric distance below the provided Threshold
  //   2. a distance ratio between descriptors of valid geometric correspondencess

  const ErrorT errorEstimator = ErrorT();

  // build region positions arrays (in order to un-distord on-demand point position once)
  std::vector<Vec2> lRegionsPos(lRegions.RegionCount());
  std::vector<Vec2> rRegionsPos(rRegions.RegionCount());

  if(camL && camL->isValid())
  {
    for(std::size_t i = 0; i < lRegions.RegionCount(); ++i)
      lRegionsPos[i] = camL->get_ud_pixel(lRegions.GetRegionPosition(i));
  }
  else
  {
    for(std::size_t i = 0; i < lRegions.RegionCount(); ++i)
      lRegionsPos[i] = lRegions.GetRegionPosition(i);
  }
  if(camR && camR->isValid())
  {
    for(std::size_t i = 0; i < rRegions.RegionCount(); ++i)
      rRegionsPos[i] = camR->get_ud_pixel(rRegions.GetRegionPosition(i));
  }
  else
  {
    for(std::size_t i = 0; i < rRegions.RegionCount(); ++i)
      rRegionsPos[i] = rRegions.GetRegionPosition(i);
  }

  for(std::size_t i = 0; i < lRegions.RegionCount(); ++i)
  {
    distanceRatio<double> dR;
    for(std::size_t j = 0; j < rRegions.RegionCount(); ++j)
    {
      // compute the geometric error: error to the model
      const double geomErr = errorEstimator.error(mod, lRegionsPos[i], rRegionsPos[j]);

      if(geomErr < errorTh)
      {
        // update the corresponding points & distance (if required)
        dR.update(j, lRegions.SquaredDescriptorDistance(i, &rRegions, j));
      }
    }
    // add correspondence only iff the distance ratio is valid
    if(dR.isValid(distRatio))
    {
      // save the best corresponding index
      out_matches.emplace_back(i, dR.idx);
    }
  }

  // remove duplicates (when multiple points at same position exist)
  matching::IndMatch::getDeduplicated(out_matches);
}

/**
 * @brief Guided Matching (features + descriptors with distance ratio):
 *        Use a model to find valid correspondences:
 *        Keep the best corresponding points for the given model under the
 *        user specified distance ratio.
 *
 * @tparam ModelT The used model type
 * @tparam ErrorT The metric to compute distance to the model
 *
 * @param[in] mod The model
 * @param[in] camL Optional camera (in order to undistord on the fly feature positions, can be NULL)
 * @param[in] lRegions regions (point features & corresponding descriptors)
 * @param[in] camR Optional camera (in order to undistord on the fly feature positions, can be NULL)
 * @param[in] rRegions regions (point features & corresponding descriptors)
 * @param[in] errorTh Maximal authorized error threshold
 * @param[in] distRatio Maximal authorized distance ratio
 * @param[out] out_matchesPerDesc Ouput corresponding index
 */
template<typename ModelT, typename ErrorT>
void guidedMatching(const ModelT& mod,
                    const camera::IntrinsicBase* camL,
                    const feature::MapRegionsPerDesc& lRegions,
                    const camera::IntrinsicBase* camR,
                    const feature::MapRegionsPerDesc& rRegions,
                    double errorTh,
                    double distRatio,
                    matching::MatchesPerDescType& out_matchesPerDesc)
{
  const std::vector<feature::EImageDescriberType> descTypes = getCommonDescTypes(lRegions, rRegions);
  if(descTypes.empty())
    return;

  for(const feature::EImageDescriberType descType: descTypes)
  {
    guidedMatching<ModelT, ErrorT>(mod, camL, *lRegions.at(descType), camR, *rRegions.at(descType), errorTh, distRatio, out_matchesPerDesc[descType]);
  }
}

/**
 * @brief Compute a bucket index from an epipolar point
 *        (the one that is closer to image border intersection)
 */
unsigned int pix_to_bucket(const Vec2i &x, int W, int H);

/**
 * @brief Compute intersection of the epipolar line with the image border
 */
bool line_to_endPoints(const Vec3& line, int W, int H, Vec2& x0, Vec2& x1);

/**
 * @brief Guided Matching (features + descriptors with distance ratio):
 *        Cluster correspondences per epipolar line (faster than exhaustive search).
 *        Keep the best corresponding points for the given model under the
 *        user specified distance ratio.
 *        Can be seen as a variant of robustEstimation method [1].
 *
 * @note implementation done here use a pixel grid limited to image border.
 *
 * @ref [1] Rajvi Shah, Vanshika Shrivastava, and P J Narayanan
 *          Geometry-aware Feature Matching for Structure from Motion Applications.
 *          WACV 2015.
 *
 * @tparam ErrorT The metric to compute distance to the model
 *
 * @param[in] FMat The fundamental matrix
 * @param[in] epipole2 Epipole2 (camera center1 in image plane2; must not be normalized)
 * @param[in] camL Optional camera (in order to undistord on the fly feature positions, can be NULL)
 * @param[in] lRegions regions (point features & corresponding descriptors)
 * @param[in] camR Optional camera (in order to undistord on the fly feature positions, can be NULL)
 * @param[in] rRegions regions (point features & corresponding descriptors)
 * @param[in] widthR
 * @param[in] heightR
 * @param[in] errorTh Maximal authorized error threshold (consider it's a square threshold)
 * @param[in] distRatio Maximal authorized distance ratio
 * @param[out] vec_corresponding_index Ouput corresponding index
 */
template<typename ErrorT>
void guidedMatchingFundamentalFast(const Mat3& FMat,
                                   const Vec3& epipole2,
                                   const camera::IntrinsicBase* camL,
                                   const feature::Regions& lRegions,
                                   const camera::IntrinsicBase* camR,
                                   const feature::Regions& rRegions,
                                   const int widthR,
                                   const int heightR,
                                   double errorTh,
                                   double distRatio,
                                   matching::IndMatches& vec_corresponding_index)
{
  // Looking for the corresponding points that have to satisfy:
  //   1. a geometric distance below the provided Threshold
  //   2. a distance ratio between descriptors of valid geometric correspondencess
  //
  // - Cluster left point according their epipolar line border intersection.
  // - For each right point, compute threshold limited bandwidth and compare only
  //   points that belong to this range (limited buckets).

  // Normalize F and epipole for (ep2->p2) line adequation
  Mat3 F = FMat;
  Vec3 ep2 = epipole2;
  if(ep2(2) > 0.0)
  {
    F = -F;
    ep2 = -ep2;
  }
  ep2 = ep2 / ep2(2);

  //--
  //-- Store point in the corresponding epipolar line bucket
  //--
  using Bucket_vec = std::vector<IndexT>;
  using Buckets_vec = std::vector<Bucket_vec>;
  const int nb_buckets = 2 * (widthR + heightR - 2);

  Buckets_vec buckets(nb_buckets);
  for(std::size_t i = 0; i < lRegions.RegionCount(); ++i)
  {
    // Compute epipolar line
    const Vec2 l_pt = (camL && camL->isValid()) ? camL->get_ud_pixel(lRegions.GetRegionPosition(i)) : lRegions.GetRegionPosition(i);
    const Vec3 line = F * Vec3(l_pt(0), l_pt(1), 1.);
    // If the epipolar line exists in Right image
    Vec2 x0, x1;
    if(line_to_endPoints(line, widthR, heightR, x0, x1))
    {
      // Find in which cluster the point belongs
      const auto bucket = pix_to_bucket(x0.cast<int>(), widthR, heightR);
      buckets[bucket].push_back(i);
    }
  }

  // For each point in right image, find if there is good candidates.
  std::vector<distanceRatio<double > > dR(lRegions.RegionCount());
  for(std::size_t j = 0; j < rRegions.RegionCount(); ++j)
  {
    // According the point:
    // - Compute it's epipolar line from the epipole
    // - compute the range of possible bucket by computing
    //    the epipolar line gauge limitation introduced by the tolerated pixel error

    const Vec2 xR = (camR && camR->isValid()) ? camR->get_ud_pixel(rRegions.GetRegionPosition(j)) : rRegions.GetRegionPosition(j);
    const Vec3 l2 = ep2.cross(Vec3(xR(0), xR(1), 1.));
    const Vec2 n = l2.head<2>() * (sqrt(errorTh) / l2.head<2>().norm());

    const Vec3 l2min = ep2.cross(Vec3(xR(0) - n(0), xR(1) - n(1), 1.));
    const Vec3 l2max = ep2.cross(Vec3(xR(0) + n(0), xR(1) + n(1), 1.));

    // Compute corresponding buckets
    Vec2 x0, x1;
    if(!line_to_endPoints(l2min, widthR, heightR, x0, x1))
      continue;
    
    const auto bucket_start = pix_to_bucket(x0.cast<int>(), widthR, heightR);

    if(!line_to_endPoints(l2max, widthR, heightR, x0, x1))
      continue;
    
    const auto bucket_stop = pix_to_bucket(x0.cast<int>(), widthR, heightR);

    if(bucket_stop > bucket_start)
    {
      // test candidate buckets
      for(Buckets_vec::const_iterator itBs = buckets.begin() + bucket_start;
              itBs != buckets.begin() + bucket_stop; ++itBs)
      {
        const Bucket_vec & bucket = *itBs;
        for(unsigned int i : bucket)
        {
          // Compute descriptor distance
          const double descDist = lRegions.SquaredDescriptorDistance(i, &rRegions, j);
          // Update the corresponding points & distance (if required)
          dR[i].update(j, descDist);
        }
      }
    }
  }
  // Check distance ratio validity
  for(std::size_t i = 0; i < dR.size(); ++i)
  {
    if(dR[i].isValid(distRatio))
    {
      // save the best corresponding index
      vec_corresponding_index.emplace_back(i, dR[i].idx);
    }
  }
}

}
}
