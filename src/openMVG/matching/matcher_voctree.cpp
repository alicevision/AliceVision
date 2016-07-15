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
  //voctree::SparseHistogram queryHistogram = voctree_.quantizeToSparse(queryDescriptors);

  const std::vector<SiftRegion::FeatureT> queryFeatures = castedRegion->Features();
  std::vector<IndexT> queryDescUID;
  createVectFeatures(queryFeatures, queryDescUID);
  voctree::SparseHistogram queryHistogram = voctree_.quantizeMultiToSparse(queryDescriptors, queryDescUID);
        
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
  }
  
  // Remove duplicates
  matching::IndMatch::getDeduplicated(vec_putative_matches);

  // Remove matches that have the same (X,Y) coordinates
  matching::IndMatchDecorator<float> matchDeduplicator(vec_putative_matches,
    regions_->GetRegionsPositions(), query_regions.GetRegionsPositions());
  matchDeduplicator.getDeduplicated(vec_putative_matches);

  return true;
}


void MatcherVoctree::createVectFeatures(const std::vector<SiftRegion::FeatureT> &features, 
        std::vector<IndexT>& indexVect)
{

  std::map<std::pair<float,float>, IndexT> mapId2Index;

  IndexT cpt = 0;
  for(auto& feat : features)
  {
    std::pair<float,float> currentPair = make_pair(feat.x(), feat.y());
    std::map<std::pair<float,float>, IndexT>::iterator it = mapId2Index.find(currentPair);
    if(it != mapId2Index.end())
    {
      indexVect.push_back(it->second);
    }
    else
    {
      mapId2Index.insert(std::pair<std::pair<float,float>, IndexT>(currentPair,cpt));
      indexVect.push_back(cpt);
    }
    cpt++;
  }
}


}
}
