
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
  const features::PointFeatures& getFeatures(IndexT viewId, features::EImageDescriberType descType) const;

  MapFeaturesPerDesc& getFeaturesPerDesc(IndexT viewId)
  {
    return _data.at(viewId);
  }

  const MapFeaturesPerDesc& getFeaturesPerDesc(IndexT viewId) const
  {
    return _data.at(viewId);
  }

  const MapFeaturesPerDesc& getDataPerDesc(IndexT viewId) const
  {
    return _data.at(viewId);
  }

  /**
   * @brief Get the list of common describerTypes between two views
   * @param pair
   * @return vector of describerType
   */
  std::vector<features::EImageDescriberType> getCommonDescTypes(const Pair& pair) const;
  
  /**
   * @brief Return true if the given viewId is present in the container
   * @param viewId
   * @return boolean
   */
  bool viewExist(IndexT viewId) const
  {
    return _data.count(viewId) > 0;
  }
  
  /**
   * @brief Return true if the container is empty
   * @return boolean
   */
  bool isEmpty() const
  {
    return _data.empty();
  }
  
  /**
   * @brief Add a list of features for a given view and describerType
   * @param viewId
   * @param regionsPtr
   */
  void addFeatures(IndexT viewId, features::EImageDescriberType descType, const features::PointFeatures& pointFeatures)
  {
    assert(descType != features::EImageDescriberType::UNINITIALIZED);
    _data[viewId][descType] = pointFeatures;
  }
  
  /**
   * @brief Add synthtic data into the container for the given describerType
   * @param synthetic_data
   * @param noise
   * @return true if success
   */
  // Create from a synthetic scene (NViewDataSet) some SfM pipelines data provider:
//  - for each view store the observations point as PointFeatures
  template <typename NoiseGenerator>
  bool createSyntheticData(features::EImageDescriberType descType, const NViewDataSet & synthetic_data, NoiseGenerator & noise)
  {
    assert(descType != features::EImageDescriberType::UNINITIALIZED);
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
   * @brief Get a reference of private container data
   * @return MapFeaturesPerView reference
   */
  features::MapFeaturesPerView& getData()
  {
    return _data;
  }
  
private:
  
  /// PointFeature array per ViewId of the considered SfM_Data container
  MapFeaturesPerView _data;
};

} // namespace features
} // namespace openMVG
