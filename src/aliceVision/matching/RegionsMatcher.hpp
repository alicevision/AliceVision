// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "aliceVision/matching/matcherType.hpp"
#include "aliceVision/matching/IndMatch.hpp"
#include "aliceVision/matching/IndMatchDecorator.hpp"
#include "aliceVision/matching/filters.hpp"

#include "aliceVision/numeric/numeric.hpp"
#include "aliceVision/feature/Regions.hpp"
#include "aliceVision/feature/RegionsPerView.hpp"

#include <vector>
#include <random>

namespace aliceVision {
namespace matching {

/**
 * @brief Match two Regions according to a chosen MatcherType using the ratio test
 * to assure the robustness of the matches.
 * It returns the matching features.
 * @param[in] randomNumberGenerator random Number generator
 * @param[in] dist_ratio The threshold for the ratio test.
 * @param[in] eMatcherType The type of matcher to use.
 * @param[in] regions_I The first Region to match
 * @param[in] regions_J The second Region to match
 * @param[out] matches It contains the indices of the matching features
 */
void DistanceRatioMatch
(
  std::mt19937 & randomNumberGenerator, 
  float dist_ratio,   // Distance ratio
  matching::EMatcherType eMatcherType, // Matcher
  const feature::Regions & regions_I, // database
  const feature::Regions & regions_J, // query
  matching::IndMatches & matches // corresponding points
);


/**
 * @brief Interface for a matcher of Regions that keep on of the Regions stored
 * as a "database". The other region is then a query that is matched with respect
 * to the database.
 */
class IRegionsMatcher
{
public:
  const feature::Regions& regions_;

  /**
   * @brief The destructor.
   */
  virtual ~IRegionsMatcher() = 0;

  IRegionsMatcher(const feature::Regions& regions)
    : regions_(regions)
  {

  }

  /**
   * @brief Match a Regions to the internal database using the test ratio to improve
   * the robustness of the match.
   *
   * @param[in] f_dist_ratio The threshold for the ratio test.
   * @param[in] query_regions The Regions to match.
   * @param[out] vec_putative_matches It contains the indices of the matching features
   * of the database and the query Regions.
   * @return True if everything went well.
   */
  virtual bool Match(
    const float f_dist_ratio,
    const feature::Regions& query_regions,
    matching::IndMatches & vec_putative_matches
  ) = 0;

  const feature::Regions& getDatabaseRegions() const { return regions_; }
};

inline IRegionsMatcher::~IRegionsMatcher()
{}

/**
 * Match two Regions with one stored as a "database" according a Template ArrayMatcher.
 */
template < class ArrayMatcherT >
class RegionsMatcher : public IRegionsMatcher
{
private:
  ArrayMatcherT matcher_;
  bool b_squared_metric_; // Store if the metric is squared or not
public:
  typedef typename ArrayMatcherT::ScalarT Scalar;
  typedef typename ArrayMatcherT::DistanceType DistanceType;

  /**
   * @brief Empty constructor, by default it initializes the database to an empty database.
   */
  RegionsMatcher() {}

  /**
   * @brief Initialize the matcher with a Regions that will be used as database
   * 
   * @param regions The Regions to be used as database.
   * @param b_squared_metric Whether to use a squared metric for the ratio test 
   * when matching two Regions.
   */
  RegionsMatcher(std::mt19937 & randomNumberGenerator,const feature::Regions& regions, bool b_squared_metric = false)
    : IRegionsMatcher(regions), b_squared_metric_(b_squared_metric)
  {
    if (regions_.RegionCount() == 0)
      return;

    const Scalar * tab = reinterpret_cast<const Scalar *>(regions_.DescriptorRawData());
    matcher_.Build(randomNumberGenerator, tab, regions_.RegionCount(), regions_.DescriptorLength());
  }

  /**
   * @brief Match a Regions to the internal database using the test ratio to improve
   * the robustness of the match.
   * 
   * @param[in] f_dist_ratio The threshold for the ratio test.
   * @param[in] query_regions The Regions to match.
   * @param[out] vec_putative_matches It contains the indices of the matching features
     * of the database and the query Regions.
   * @return True if everything went well.
   */
  bool Match(const float f_dist_ratio,
             const feature::Regions& queryregions_,
             matching::IndMatches & vec_putative_matches)
  {

    const Scalar * queries = reinterpret_cast<const Scalar *>(queryregions_.DescriptorRawData());

    const size_t NNN__ = 2;
    matching::IndMatches vec_nIndice;
    std::vector<DistanceType> vec_fDistance;

    // Search the 2 closest features neighbours for each query descriptor
    if (!matcher_.SearchNeighbours(queries, queryregions_.RegionCount(), &vec_nIndice, &vec_fDistance, NNN__))
      return false;

    assert(vec_nIndice.size() == vec_fDistance.size());

    std::vector<int> vec_nn_ratio_idx;
    std::vector<float> vec_distanceRatio;
    // Filter the matches using a distance ratio test:
    //   The probability that a match is correct is determined by taking
    //   the ratio of distance from the closest neighbor to the distance
    //   of the second closest.
    matching::NNdistanceRatio(
      vec_fDistance.begin(), // distance start
      vec_fDistance.end(),   // distance end
      NNN__, // Number of neighbor in iterator sequence (minimum required 2)
      vec_nn_ratio_idx, // output (indices that respect the distance Ratio)
      b_squared_metric_ ? Square(f_dist_ratio) : f_dist_ratio,
      &vec_distanceRatio);

    vec_putative_matches.reserve(vec_nn_ratio_idx.size());
    for (size_t k=0; k < vec_nn_ratio_idx.size(); ++k)
    {
      const size_t index = vec_nn_ratio_idx[k] * NNN__;
      vec_putative_matches.emplace_back(vec_nIndice[index]._j, vec_nIndice[index]._i
          , vec_distanceRatio[k]
  #ifdef ALICEVISION_DEBUG_MATCHING
          , (float) vec_fDistance[index]
  #endif
      );
    }

    // Remove duplicates
    matching::IndMatch::getDeduplicated(vec_putative_matches);

    // Remove matches that have the same (X,Y) coordinates
    matching::IndMatchDecorator<float> matchDeduplicator(vec_putative_matches,
      regions_.GetRegionsPositions(), queryregions_.GetRegionsPositions());
    matchDeduplicator.getDeduplicated(vec_putative_matches);

    return (!vec_putative_matches.empty());
  }

};


/**
 * @brief A simple class that allows to match Regions with respect a given Regions
 * used as "database".
 */
class RegionsDatabaseMatcher
{
  public:
    /**
     * @brief Empty constructor, by default it initializes the matcher to BRUTE_FORCE_L2
     * and the database to an empty database.
     */
    RegionsDatabaseMatcher();

    /**
     * @brief Initialize the internal database
     *
     * @param[in] matcherType The type of matcher to use to match the Regions.
     * @param[in] database_regions The Regions that will be used as database to
     * match other Regions (query).
     */
    RegionsDatabaseMatcher(
      std::mt19937 & randomNumberGenerator,
      matching::EMatcherType matcherType,
      const feature::Regions & database_regions);

    /**
     * @brief Find corresponding points between the query Regions and the database one
     *
     * @param[in] distRatio The threshold for the ratio test used to discard spurious correspondence.
     * @param[in] queryRegions The Regions to match.
     * @param[out] matches It contains the indices of the matching features (photometric corresponding points)
     *                     of the database and the query Regions.
     * @return True if everything went well.
     */
    bool Match(
      float distRatio,
      const feature::Regions & queryRegions,
      matching::IndMatches & matches) const;

    const feature::Regions& getDatabaseRegions() const { return _regionsMatcher->getDatabaseRegions(); }

  private:
  // Matcher Type
  matching::EMatcherType _matcherType;
  // The matching interface
  std::unique_ptr<IRegionsMatcher> _regionsMatcher;
};

class RegionsDatabaseMatcherPerDesc
{
public:
  RegionsDatabaseMatcherPerDesc(
      std::mt19937 & randomNumberGenerator,
      matching::EMatcherType matcherType,
      const feature::MapRegionsPerDesc & queryRegions)
    : _databaseRegions(queryRegions)
  {
    for(const auto& queryRegionsIt: queryRegions)
    {
      _mapMatchers[queryRegionsIt.first] = RegionsDatabaseMatcher(randomNumberGenerator, matcherType, *queryRegionsIt.second);
    }
  }

  bool Match(
    float distRatio,
    const feature::MapRegionsPerDesc & matchedRegions,
    matching::MatchesPerDescType & out_putativeFeatureMatches)
  {
    bool res = false;
    for(auto& matcherIt: _mapMatchers)
    {
      const feature::EImageDescriberType descType = matcherIt.first;
      res |= matcherIt.second.Match(
            distRatio,
            *matchedRegions.at(descType),
            out_putativeFeatureMatches[descType]);
    }
    return res;
  }

  const feature::MapRegionsPerDesc & getDatabaseRegionsPerDesc() const { return _databaseRegions; }

  const feature::Regions & getDatabaseRegions(feature::EImageDescriberType descType) const { return _mapMatchers.at(descType).getDatabaseRegions(); }

  std::map<feature::EImageDescriberType, RegionsDatabaseMatcher> & getData() { return _mapMatchers; }

private:
  const feature::MapRegionsPerDesc & _databaseRegions;
  std::map<feature::EImageDescriberType, RegionsDatabaseMatcher> _mapMatchers;
};

std::unique_ptr<IRegionsMatcher> createRegionsMatcher(std::mt19937 & randomNumberGenerator,const feature::Regions & regions, matching::EMatcherType matcherType);

}  // namespace matching
}  // namespace aliceVision
