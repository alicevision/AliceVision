// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include "aliceVision/matching/matcherType.hpp"
#include "aliceVision/matching/RegionsMatcher.hpp"
#include "aliceVision/matching/ArrayMatcher_bruteForce.hpp"
#include "aliceVision/matching/ArrayMatcher_kdtreeFlann.hpp"
#include "aliceVision/matching/ArrayMatcher_cascadeHashing.hpp"

namespace aliceVision {
namespace matching {

void DistanceRatioMatch(
  float f_dist_ratio,
  matching::EMatcherType eMatcherType,
  const feature::Regions & regions_I, // database
  const feature::Regions & regions_J, // query
  matching::IndMatches & matches)
{
  RegionsDatabaseMatcher matcher(eMatcherType, regions_I);
  matcher.Match(f_dist_ratio, regions_J, matches);
}

bool RegionsDatabaseMatcher::Match(
  float distRatio,
  const feature::Regions & queryRegions,
  matching::IndMatches & matches) const
{
  if (queryRegions.RegionCount() == 0)
    return false;

  if (!_regionsMatcher)
    return false;

  return _regionsMatcher->Match(distRatio, queryRegions, matches);
}

RegionsDatabaseMatcher::RegionsDatabaseMatcher():
  _matcherType(BRUTE_FORCE_L2),
  _regionsMatcher(nullptr)
{}


RegionsDatabaseMatcher::RegionsDatabaseMatcher(
  matching::EMatcherType matcherType,
  const feature::Regions & databaseRegions)
  : _matcherType(matcherType)
{
  _regionsMatcher = createRegionsMatcher(databaseRegions, matcherType);
}


std::unique_ptr<IRegionsMatcher> createRegionsMatcher(const feature::Regions & regions, matching::EMatcherType matcherType)
{
  std::unique_ptr<IRegionsMatcher> out;

  // Handle invalid request
  if (regions.IsScalar() && matcherType == BRUTE_FORCE_HAMMING)
    return out;
  if (regions.IsBinary() && matcherType != BRUTE_FORCE_HAMMING)
    return out;

  // Switch regions type ID, matcher & Metric: initialize the Matcher interface
  if (regions.IsScalar())
  {
    if (regions.Type_id() == typeid(unsigned char).name())
    {
      // Build on the fly unsigned char based Matcher
      switch (matcherType)
      {
        case BRUTE_FORCE_L2:
        {
          typedef L2_Vectorized<unsigned char> MetricT;
          typedef ArrayMatcher_bruteForce<unsigned char, MetricT> MatcherT;
          out.reset(new matching::RegionsMatcher<MatcherT>(regions, true));
        }
        break;
        case ANN_L2:
        {
          typedef ArrayMatcher_kdtreeFlann<unsigned char> MatcherT;
          out.reset(new matching::RegionsMatcher<MatcherT>(regions, true));
        }
        break;
        case CASCADE_HASHING_L2:
        {
          typedef L2_Vectorized<unsigned char> MetricT;
          typedef ArrayMatcher_cascadeHashing<unsigned char, MetricT> MatcherT;
          out.reset(new matching::RegionsMatcher<MatcherT>(regions, true));
        }
        break;
        default:
          ALICEVISION_LOG_WARNING("Using unknown matcher type");
      }
    }
    else if (regions.Type_id() == typeid(float).name())
    {
      // Build on the fly float based Matcher
      switch (matcherType)
      {
        case BRUTE_FORCE_L2:
        {
          typedef L2_Vectorized<float> MetricT;
          typedef ArrayMatcher_bruteForce<float, MetricT> MatcherT;
          out.reset(new matching::RegionsMatcher<MatcherT>(regions, true));
        }
        break;
        case ANN_L2:
        {
          typedef ArrayMatcher_kdtreeFlann<float> MatcherT;
          out.reset(new matching::RegionsMatcher<MatcherT>(regions, true));
        }
        break;
        case CASCADE_HASHING_L2:
        {
          typedef L2_Vectorized<float> MetricT;
          typedef ArrayMatcher_cascadeHashing<float, MetricT> MatcherT;
          out.reset(new matching::RegionsMatcher<MatcherT>(regions, true));
        }
        break;
        default:
          ALICEVISION_LOG_WARNING("Using unknown matcher type");
      }
    }
    else if (regions.Type_id() == typeid(double).name())
    {
      // Build on the fly double based Matcher
      switch (matcherType)
      {
        case BRUTE_FORCE_L2:
        {
          typedef L2_Vectorized<double> MetricT;
          typedef ArrayMatcher_bruteForce<double, MetricT> MatcherT;
          out.reset(new matching::RegionsMatcher<MatcherT>(regions, true));
        }
        break;
        case ANN_L2:
        {
          typedef ArrayMatcher_kdtreeFlann<double> MatcherT;
          out.reset(new matching::RegionsMatcher<MatcherT>(regions, true));
        }
        break;
        case CASCADE_HASHING_L2:
        {
          ALICEVISION_LOG_WARNING("Not yet implemented");
        }
        break;
        default:
          ALICEVISION_LOG_WARNING("Using unknown matcher type");
      }
    }
  }
  else if (regions.IsBinary() && regions.Type_id() == typeid(unsigned char).name())
  {
    switch (matcherType)
    {
      case BRUTE_FORCE_HAMMING:
      {
        typedef Hamming<unsigned char> Metric;
        typedef ArrayMatcher_bruteForce<unsigned char, Metric> MatcherT;
        out.reset(new matching::RegionsMatcher<MatcherT>(regions, false));
      }
      break;
      default:
          ALICEVISION_LOG_WARNING("Using unknown matcher type");
    }
  }
  else
  {
    ALICEVISION_LOG_WARNING("Please consider add this region type_id to Matcher_Regions_Database::Match(...)\n"
      << "typeid: " << regions.Type_id());
  }
  return out;
}


}  // namespace matching
}  // namespace aliceVision
