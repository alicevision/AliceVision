// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#pragma once

#include "aliceVision/types.hpp"
#include "aliceVision/feature/feature.hpp"
#include "aliceVision/feature/imageDescriberCommon.hpp"
#include "aliceVision/multiview/NViewDataSet.hpp" // synthetic data

#include <memory>
#include <random>

namespace aliceVision {
namespace feature {


using MapFeaturesPerDesc = HashMap<feature::EImageDescriberType, feature::PointFeatures>;
using MapFeaturesPerView = HashMap<IndexT, MapFeaturesPerDesc>;

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
  const feature::PointFeatures& getFeatures(IndexT viewId, feature::EImageDescriberType descType) const;

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
  std::vector<feature::EImageDescriberType> getCommonDescTypes(const Pair& pair) const;
  
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
  void addFeatures(IndexT viewId, feature::EImageDescriberType descType, const feature::PointFeatures& pointFeatures)
  {
    assert(descType != feature::EImageDescriberType::UNINITIALIZED);
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
  bool createSyntheticData(feature::EImageDescriberType descType, const NViewDataSet & synthetic_data, NoiseGenerator & noise)
  {
    assert(descType != feature::EImageDescriberType::UNINITIALIZED);
    std::default_random_engine generator;

    for (std::size_t j = 0; j < synthetic_data._n; ++j) // For each view
    {
      for (Mat::Index i = 0; i < synthetic_data._x[j].cols(); ++i) // For each new point visibility
      {
        const Vec2 pt = synthetic_data._x[j].col(i);
        _data[j][descType].push_back(feature::PointFeature(pt(0) + noise(generator), pt(1) + noise(generator)));
      }
    }
    return true;
  }
  
  /**
   * @brief Get a reference of private container data
   * @return MapFeaturesPerView reference
   */
  feature::MapFeaturesPerView& getData()
  {
    return _data;
  }
  
private:
  
  /// PointFeature array per ViewId of the considered SfMData container
  MapFeaturesPerView _data;
};

} // namespace feature
} // namespace aliceVision
