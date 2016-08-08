#pragma once

#include "regions_matcher.hpp"
#include "openMVG/voctree/database.hpp"
#include "openMVG/features/regions.hpp"
#include "openMVG/matching/indMatch_utils.hpp"

namespace openMVG {
namespace matching {

static const int DIMENSION = 128;
typedef openMVG::features::Descriptor<float, DIMENSION> DescriptorFloat;
typedef openMVG::features::Descriptor<unsigned char, DIMENSION> DescriptorUChar;
typedef features::FeatDesc_Regions<features::SIOPointFeature, unsigned char, 128> SiftRegion;

class MatcherVoctree : public RegionsMatcher
{
  
private:
  const features::Regions* regions_;
  voctree::SparseHistogram histogram_;

public:
  openMVG::voctree::VocabularyTree<DescriptorFloat> voctree_;
  
  MatcherVoctree(const features::Regions& regions, const std::string& voctreeFile);
  
  /**
   * @brief Initialize the matcher by setting one region as a "database".
   * 
   * @param[in] regions The Regions to be used as reference when matching another Region.
   */
  void Init_database(const features::Regions& regions);

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
  bool Match(
    const float f_dist_ratio,
    const features::Regions& query_regions,
    matching::IndMatches & vec_putative_matches
  );

};

} // namespace matching
} // namespace openMVG
