
// Copyright (c) 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "openMVG/matching/regions_matcher.hpp"
#include "openMVG/matching/matcher_brute_force.hpp"
#include "openMVG/matching/matcher_kdtree_flann.hpp"
#include "openMVG/matching/matcher_cascade_hashing.hpp"
#include "openMVG/matching/matcher_voctree.hpp"

namespace openMVG {
namespace matching {

void DistanceRatioMatch
(
  float f_dist_ratio,
  matching::EMatcherType eMatcherType,
  const features::Regions & regions_I, // database
  const features::Regions & regions_J, // query
  matching::IndMatches & matches // photometric corresponding points
)
{
  Matcher_Regions_Database matcher(eMatcherType, regions_I);
  matcher.Match(f_dist_ratio, regions_J, matches);
}

bool Matcher_Regions_Database::Match
(
  float dist_ratio, // Distance ratio used to discard spurious correspondence
  const features::Regions & query_regions,
  matching::IndMatches & matches // photometric corresponding points
)const
{
  if (query_regions.RegionCount() == 0)
    return false;

  assert(_matching_interface);

  return _matching_interface->Match(dist_ratio, query_regions, matches);
}

Matcher_Regions_Database::Matcher_Regions_Database():
  _eMatcherType(BRUTE_FORCE_L2),
  _matching_interface(nullptr)
{}

Matcher_Regions_Database::Matcher_Regions_Database
(
  matching::EMatcherType eMatcherType,
  const features::Regions & database_regions // database
):
  _eMatcherType(eMatcherType)
{
  // Handle invalid request
  if (database_regions.IsScalar() && eMatcherType == BRUTE_FORCE_HAMMING)
    throw std::logic_error("Error: Incompatible regions type (scalar) and matcher type.");
  if (database_regions.IsBinary() && eMatcherType != BRUTE_FORCE_HAMMING)
    throw std::logic_error("Error: Incompatible regions type (binary) and matcher type.");

  // Switch regions type ID, matcher & Metric: initialize the Matcher interface
  if (database_regions.IsScalar())
  {
    if (database_regions.Type_id() == typeid(unsigned char).name())
    {
      // Build on the fly unsigned char based Matcher
      switch (_eMatcherType)
      {
        case BRUTE_FORCE_L2:
        {
          typedef L2_Vectorized<unsigned char> MetricT;
          typedef ArrayMatcherBruteForce<unsigned char, MetricT> MatcherT;
          _matching_interface.reset(new matching::RegionsMatcherT<MatcherT>(database_regions, true));
        }
        break;
        case ANN_L2:
        {
          typedef flann::L2<unsigned char> MetricT;
          typedef ArrayMatcher_Kdtree_Flann<unsigned char, MetricT> MatcherT;
          _matching_interface.reset(new matching::RegionsMatcherT<MatcherT>(database_regions, true));
        }
        break;
        case CASCADE_HASHING_L2:
        {
          typedef L2_Vectorized<unsigned char> MetricT;
          typedef ArrayMatcherCascadeHashing<unsigned char, MetricT> MatcherT;
          _matching_interface.reset(new matching::RegionsMatcherT<MatcherT>(database_regions, true));
        }
        case VOCTREE_MATCHER:
        {
          const char* voctreeFile = std::getenv("OPENMVG_VOCTREE");
          if(voctreeFile == NULL)
            throw std::logic_error("Error: Voctree matcher needs an environment variable OPENMVG_VOCTREE with the path to the voctree file.");
          _matching_interface.reset(new matching::MatcherVoctree(database_regions, voctreeFile));
        }
        break;
        default:
          throw std::logic_error("Error: Unknown matcher type.");
      }
    }
    else if (database_regions.Type_id() == typeid(float).name())
    {
      // Build on the fly float based Matcher
      switch (eMatcherType)
      {
        case BRUTE_FORCE_L2:
        {
          typedef L2_Vectorized<float> MetricT;
          typedef ArrayMatcherBruteForce<float, MetricT> MatcherT;
          _matching_interface.reset(new matching::RegionsMatcherT<MatcherT>(database_regions, true));
        }
        break;
        case ANN_L2:
        {
          typedef flann::L2<float> MetricT;
          typedef ArrayMatcher_Kdtree_Flann<float, MetricT> MatcherT;
          _matching_interface.reset(new matching::RegionsMatcherT<MatcherT>(database_regions, true));
        }
        break;
        case CASCADE_HASHING_L2:
        {
          typedef L2_Vectorized<float> MetricT;
          typedef ArrayMatcherCascadeHashing<float, MetricT> MatcherT;
          _matching_interface.reset(new matching::RegionsMatcherT<MatcherT>(database_regions, true));
        }
        break;
        default:
          throw std::logic_error("Error: Unknown matcher type.");
      }
    }
    else if (database_regions.Type_id() == typeid(double).name())
    {
      // Build on the fly double based Matcher
      switch (eMatcherType)
      {
        case BRUTE_FORCE_L2:
        {
          typedef L2_Vectorized<double> MetricT;
          typedef ArrayMatcherBruteForce<double, MetricT> MatcherT;
          _matching_interface.reset(new matching::RegionsMatcherT<MatcherT>(database_regions, true));
        }
        break;
        case ANN_L2:
        {
          typedef flann::L2<double> MetricT;
          typedef ArrayMatcher_Kdtree_Flann<double, MetricT> MatcherT;
          _matching_interface.reset(new matching::RegionsMatcherT<MatcherT>(database_regions, true));
        }
        break;
        case CASCADE_HASHING_L2:
        {
          throw std::logic_error("Error: CASCADE_HASHING_L2 not yet implemented.");
        }
        break;
        default:
          throw std::logic_error("Error: Unknown matcher type.");
      }
    }
  }
  else if (database_regions.IsBinary() && database_regions.Type_id() == typeid(unsigned char).name())
  {
    switch (eMatcherType)
    {
      case BRUTE_FORCE_HAMMING:
      {
        typedef Hamming<unsigned char> Metric;
        typedef ArrayMatcherBruteForce<unsigned char, Metric> MatcherT;
        _matching_interface.reset(new matching::RegionsMatcherT<MatcherT>(database_regions, false));
      }
      break;
      default:
        throw std::logic_error("Error: Unknown matcher type.");
    }
  }
  else
  {
    throw std::logic_error("Error: Unknown region type in matching.");
  }
}

}  // namespace matching
}  // namespace openMVG
