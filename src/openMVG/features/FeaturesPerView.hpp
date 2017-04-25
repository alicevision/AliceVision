
// Copyright (c) 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.
#pragma once

#include "openMVG/types.hpp"
#include "openMVG/features/features.hpp"
#include "openMVG/features/ImageDescriberCommon.hpp"
#include "openMVG/multiview/test_data_sets.hpp" // synthetic data

#include <memory>
#include <random>

namespace openMVG {
namespace features {


using MapFeaturesPerDesc = Hash_Map<features::EImageDescriberType, features::PointFeatures>;
using MapFeaturesPerView = Hash_Map<IndexT, MapFeaturesPerDesc>;

/**
 * @brief Container for all Features for each View.
 */
class FeaturesPerView
{
public:
  /**
   * @brief Get the PointFeatures belonging to the View, 
   * if the view does not exist it returns an empty PointFeatures.
   * 
   * @param viewId
   * @return PointFeatures or an empty PointFeatures.
   */
  const features::PointFeatures& getFeatures(IndexT viewId, features::EImageDescriberType descType) const
  {
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

  std::vector<features::EImageDescriberType> getCommonDescTypes(const Pair& pair) const
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
  
  /**
   * 
   * @param viewId
   * @return 
   */
  bool viewExist(IndexT viewId) const
  {
    return _data.count(viewId) > 0;
  }
  
  /**
   * 
   * @return 
   */
  bool isEmpty() const
  {
    return _data.empty();
  }
  
  /**
   * 
   * @param viewId
   * @param regionsPtr
   */
  void addFeatures(IndexT viewId, features::EImageDescriberType descType, features::PointFeatures pointFeatures)
  {
    _data[viewId][descType] = pointFeatures;
  }
  
  /**
   * 
   * @param FeaturesPerView
   * @param synthetic_data
   * @param noise
   * @return 
   */
  // Create from a synthetic scene (NViewDataSet) some SfM pipelines data provider:
//  - for each view store the observations point as PointFeatures
  template <typename NoiseGenerator>
  bool createSyntheticData(features::EImageDescriberType descType, const NViewDataSet & synthetic_data, NoiseGenerator & noise)
  {
    std::default_random_engine generator;

    for (int j = 0; j < synthetic_data._n; ++j) // For each view
    {
      for (int i = 0; i < synthetic_data._x[j].cols(); ++i) // For each new point visibility
      {
        const Vec2 pt = synthetic_data._x[j].col(i);
        _data[j][descType].push_back(features::PointFeature(pt(0) + noise(generator), pt(1) + noise(generator)));
      }
    }
    return true;
  }
  
  /**
   * 
   * @return 
   */
  MapFeaturesPerView& getData()
  {
    return _data;
  }
  
private:
  
  /// PointFeature array per ViewId of the considered SfM_Data container
  MapFeaturesPerView _data;
};

} // namespace features
} // namespace openMVG
