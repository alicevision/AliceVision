// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "FeaturesPerView.hpp"

namespace aliceVision {
namespace feature {

const feature::PointFeatures& FeaturesPerView::getFeatures(IndexT viewId, feature::EImageDescriberType descType) const
{
  assert(descType != feature::EImageDescriberType::UNINITIALIZED);
  // Have an empty feature set in order to deal with non existing view_id
  static const feature::PointFeatures emptyFeats = feature::PointFeatures();

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

std::vector<feature::EImageDescriberType> FeaturesPerView::getCommonDescTypes(const Pair& pair) const
{
  std::vector<feature::EImageDescriberType> descTypes;

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

} // namespace feature
} // namespace aliceVision
