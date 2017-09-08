// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include "aliceVision/matching/matcher_type.hpp"
#include "aliceVision/matching/regions_matcher.hpp"
#include "aliceVision/matching/matcher_brute_force.hpp"
#include "aliceVision/matching/matcher_kdtree_flann.hpp"
#include "aliceVision/matching/matcher_cascade_hashing.hpp"

namespace aliceVision {
namespace matching {

void DistanceRatioMatch(
  float f_dist_ratio,
  matching::EMatcherType eMatcherType,
  const features::Regions & regions_I, // database
  const features::Regions & regions_J, // query
  matching::IndMatches & matches)
{
  RegionsDatabaseMatcher matcher(eMatcherType, regions_I);
  matcher.Match(f_dist_ratio, regions_J, matches);
}

bool RegionsDatabaseMatcher::Match(
  float distRatio,
  const features::Regions & queryRegions,
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
  const features::Regions & databaseRegions)
  : _matcherType(matcherType)
{
  _regionsMatcher = createRegionsMatcher(databaseRegions, matcherType);
}


std::unique_ptr<IRegionsMatcher> createRegionsMatcher(const features::Regions & regions, matching::EMatcherType matcherType)
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
          typedef ArrayMatcherBruteForce<unsigned char, MetricT> MatcherT;
          out.reset(new matching::RegionsMatcher<MatcherT>(regions, true));
        }
        break;
        case ANN_L2:
        {
          typedef ArrayMatcher_Kdtree_Flann<unsigned char> MatcherT;
          out.reset(new matching::RegionsMatcher<MatcherT>(regions, true));
        }
        break;
        case CASCADE_HASHING_L2:
        {
          typedef L2_Vectorized<unsigned char> MetricT;
          typedef ArrayMatcherCascadeHashing<unsigned char, MetricT> MatcherT;
          out.reset(new matching::RegionsMatcher<MatcherT>(regions, true));
        }
        break;
        default:
          OPENMVG_LOG_WARNING("Using unknown matcher type");
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
          typedef ArrayMatcherBruteForce<float, MetricT> MatcherT;
          out.reset(new matching::RegionsMatcher<MatcherT>(regions, true));
        }
        break;
        case ANN_L2:
        {
          typedef ArrayMatcher_Kdtree_Flann<float> MatcherT;
          out.reset(new matching::RegionsMatcher<MatcherT>(regions, true));
        }
        break;
        case CASCADE_HASHING_L2:
        {
          typedef L2_Vectorized<float> MetricT;
          typedef ArrayMatcherCascadeHashing<float, MetricT> MatcherT;
          out.reset(new matching::RegionsMatcher<MatcherT>(regions, true));
        }
        break;
        default:
          OPENMVG_LOG_WARNING("Using unknown matcher type");
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
          typedef ArrayMatcherBruteForce<double, MetricT> MatcherT;
          out.reset(new matching::RegionsMatcher<MatcherT>(regions, true));
        }
        break;
        case ANN_L2:
        {
          typedef ArrayMatcher_Kdtree_Flann<double> MatcherT;
          out.reset(new matching::RegionsMatcher<MatcherT>(regions, true));
        }
        break;
        case CASCADE_HASHING_L2:
        {
          OPENMVG_LOG_WARNING("Not yet implemented");
        }
        break;
        default:
          OPENMVG_LOG_WARNING("Using unknown matcher type");
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
        typedef ArrayMatcherBruteForce<unsigned char, Metric> MatcherT;
        out.reset(new matching::RegionsMatcher<MatcherT>(regions, false));
      }
      break;
      default:
          OPENMVG_LOG_WARNING("Using unknown matcher type");
    }
  }
  else
  {
    OPENMVG_LOG_WARNING("Please consider add this region type_id to Matcher_Regions_Database::Match(...)\n"
      << "typeid: " << regions.Type_id());
  }
  return out;
}


}  // namespace matching
}  // namespace aliceVision
