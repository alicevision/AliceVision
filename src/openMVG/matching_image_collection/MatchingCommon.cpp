#include "MatchingCommon.hpp"

#include "openMVG/matching_image_collection/Matcher_Regions_AllInMemory.hpp"
#include "openMVG/matching_image_collection/Cascade_Hashing_Matcher_Regions_AllInMemory.hpp"

#include <exception>
#include <cassert>

namespace openMVG {
namespace matching_image_collection {
  

std::unique_ptr<Matcher> createMatcher(matching::EMatcherType matcherType, float distRatio)
{
  std::unique_ptr<Matcher> matcherPtr;
  
  switch(matcherType)
  {
    case matching::BRUTE_FORCE_L2:          matcherPtr.reset(new Matcher_Regions_AllInMemory(distRatio, matching::BRUTE_FORCE_L2)); break;
    case matching::ANN_L2:                  matcherPtr.reset(new Matcher_Regions_AllInMemory(distRatio, matching::ANN_L2)); break;
    case matching::CASCADE_HASHING_L2:      matcherPtr.reset(new Matcher_Regions_AllInMemory(distRatio, matching::CASCADE_HASHING_L2)); break;
    case matching::FAST_CASCADE_HASHING_L2: matcherPtr.reset(new Cascade_Hashing_Matcher_Regions_AllInMemory(distRatio)); break;
    case matching::BRUTE_FORCE_HAMMING:     matcherPtr.reset(new Matcher_Regions_AllInMemory(distRatio, matching::BRUTE_FORCE_HAMMING)); break;
    
    default: throw std::out_of_range("Invalid imageDescriber enum");
  }
  assert(matcherPtr != nullptr);
  
  return matcherPtr;
}

} // namespace matching_image_collection
} // namespace openMVG
