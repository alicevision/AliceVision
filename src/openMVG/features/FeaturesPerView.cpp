// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include "FeaturesPerView.hpp"

namespace aliceVision {
namespace features {

const features::PointFeatures& FeaturesPerView::getFeatures(IndexT viewId, features::EImageDescriberType descType) const
{
  assert(descType != features::EImageDescriberType::UNINITIALIZED);
  // Have an empty feature set in order to deal with non existing view_id
  static const features::PointFeatures emptyFeats = features::PointFeatures();

  MapFeaturesPerView::const_iterator itView = _data.find(viewId);

  if (itView != _data.end())
  {
    MapFeaturesPerDesc::const_iterator itDesc = itView->second.find(descType);

    if(itDesc != itView->second.end())
    {
      return itDesc->second;
    }
  }
  return emptyFeats;
}

std::vector<features::EImageDescriberType> FeaturesPerView::getCommonDescTypes(const Pair& pair) const
{
  std::vector<features::EImageDescriberType> descTypes;

  const auto& featuresA = _data.at(pair.first);
  const auto& featuresB = _data.at(pair.second);

  for(const auto& featuresPerDesc : featuresA)
  {
    const auto desc = featuresPerDesc.first;
    if ( featuresB.count(desc) > 0 )
    {
      descTypes.push_back(desc);
    }
  }
  return descTypes;
}

} // namespace features
} // namespace aliceVision
