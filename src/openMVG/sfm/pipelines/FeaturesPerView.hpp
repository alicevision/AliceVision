
// Copyright (c) 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.
#pragma once

#include "openMVG/types.hpp"
#include "openMVG/features/features.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/features/ImageDescriberCommon.hpp"
#include "openMVG/multiview/test_data_sets.hpp" // synthetic data

#include "third_party/progress/progress.hpp"

#include <memory>
#include <random>

namespace openMVG {
namespace sfm {
  
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
  const features::PointFeatures& getFeatures(IndexT viewId) const
  {
    // Have an empty feature set in order to deal with non existing view_id
    static const features::PointFeatures emptyFeats = features::PointFeatures();

    Hash_Map<IndexT, features::PointFeatures>::const_iterator it = _data.find(viewId);
    
    if (it == _data.end())
    {
      return emptyFeats;
    }
    return it->second;
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
  void addFeatures(IndexT viewId, features::PointFeatures pointFeatures)
  {
    _data[viewId] = pointFeatures;
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
  bool createSyntheticData(const NViewDataSet & synthetic_data, NoiseGenerator & noise)
  {
    std::default_random_engine generator;

    for (int j = 0; j < synthetic_data._n; ++j) // For each view
    {
      for (int i = 0; i < synthetic_data._x[j].cols(); ++i) // For each new point visibility
      {
        const Vec2 pt = synthetic_data._x[j].col(i);
        _data[j].push_back(features::PointFeature(pt(0) + noise(generator), pt(1) + noise(generator)));
      }
    }
    return true;
  }
  
  /**
   * 
   * @return 
   */
  Hash_Map<IndexT, features::PointFeatures>& getData()
  {
    return _data;
  }
  
private:
  
  /// PointFeature array per ViewId of the considered SfM_Data container
  Hash_Map<IndexT, features::PointFeatures> _data;
};


bool loadFeaturesPerView(FeaturesPerView& featuresPerView,
                    const SfM_Data& sfmData,
                    const std::string& storageDirectory,
                    features::EImageDescriberType imageDescriberType);


} // namespace sfm
} // namespace openMVG