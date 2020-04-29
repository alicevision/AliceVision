// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/types.hpp>
#include <aliceVision/feature/feature.hpp>
#include <aliceVision/feature/imageDescriberCommon.hpp>

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

  std::size_t getNbFeatures(IndexT viewId) const
  {
      MapFeaturesPerDesc f = getFeaturesPerDesc(viewId);
      std::size_t count = 0;
      for(const auto& it: f)
      {
          count += it.second.size();
      }
      return count;
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
