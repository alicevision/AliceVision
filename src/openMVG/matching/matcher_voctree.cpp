#include "matcher_voctree.hpp"

namespace openMVG {
namespace matching {
 
MatcherVoctree::MatcherVoctree(const features::Regions& regions, const std::string& voctreeFile)
{
  voctree_ = voctree::VocabularyTree<DescriptorFloat>(voctreeFile);
  Init_database(regions);
}

void MatcherVoctree::Init_database(const features::Regions& regions)
{
  regions_ = &regions;
  if(regions_->RegionCount() == 0)
    return;
  
  typedef features::FeatDesc_Regions<features::SIOPointFeature, unsigned char, 128> SiftRegion;
  const SiftRegion* castedRegion = dynamic_cast<const SiftRegion*>(&regions);
  if(castedRegion == NULL)
  {
    std::cerr << "MatcherVoctree error: only support SIFT regions." << std::endl;
    return;
  }
  
  const std::vector<DescriptorUChar>& descriptors = castedRegion->Descriptors();
  if(descriptors.size() > 20000)
  {
    std::cout << "ERROR: "<<std::endl;
    std::cout << "Regions count: " << regions.RegionCount() << std::endl;
    std::cout << "Descriptors size: " << descriptors.size() << std::endl;
  }
  //Compute histogram
  histogram_ = voctree_.softQuantizeToSparse(descriptors);
  //histogram_ = voctree_.quantizeToSparse(descriptors);
}
      
bool MatcherVoctree::Match(const float f_dist_ratio, const features::Regions& query_regions, matching::IndMatches& vec_putative_matches)
{
  typedef features::FeatDesc_Regions<features::SIOPointFeature, unsigned char, 128> SiftRegion;
  const SiftRegion* castedRegion = dynamic_cast<const SiftRegion*>(&query_regions);
  if(castedRegion == NULL)
  {
    std::cerr << "MatcherVoctree error: only support SIFT regions." << std::endl;
    return false;
  }
  const std::vector<DescriptorUChar>& queryDescriptors = castedRegion->Descriptors();
  if(queryDescriptors.size() > 20000)
  {
    std::cout <<"ERROR: "<<std::endl;
    std::cout << queryDescriptors.size() << std::endl;
  }

  //Compute histogram for the second region
  //voctree::SparseHistogram queryHistogram = voctree_.softQuantizeToSparse(queryDescriptors);
  voctree::SparseHistogram queryHistogram = voctree_.quantizeToSparse(queryDescriptors);
        
  // Matching points
  // Version 1: matches only features that are alone in their leaves
  // Version 2: bruteForce in each leave
  for(const auto& currentLeaf: histogram_)
  {
    auto queryLeafIt = queryHistogram.find(currentLeaf.first);
    if (queryLeafIt == queryHistogram.end())
      continue;
    //Version 1
    if(currentLeaf.second.size() != 1 || queryLeafIt->second.size() != 1)
    {
      continue;
    }
    double dist = regions_->SquaredDescriptorDistance(currentLeaf.second[0], castedRegion, queryLeafIt->second[0]);
    openMVG::matching::IndMatch currentMatch = openMVG::matching::IndMatch(currentLeaf.second[0], queryLeafIt->second[0], dist);
    vec_putative_matches.push_back(currentMatch);
    //Version 2
    /*for(const auto& leftPoint: currentLeaf.second)
    {
      for(const auto& rightPoint: queryLeafIt->second)
      {
        double dist = regions_->SquaredDescriptorDistance(leftPoint, castedRegion, rightPoint);
        openMVG::matching::IndMatch currentMatch = openMVG::matching::IndMatch(leftPoint, rightPoint, dist);
        vec_putative_matches.push_back(currentMatch);
      }
    }*/
  }
  
  // Remove duplicates
  matching::IndMatch::getDeduplicated(vec_putative_matches);

  // Remove matches that have the same (X,Y) coordinates
  matching::IndMatchDecorator<float> matchDeduplicator(vec_putative_matches,
    regions_->GetRegionsPositions(), query_regions.GetRegionsPositions());
  matchDeduplicator.getDeduplicated(vec_putative_matches);

  return true;
}


}
}
