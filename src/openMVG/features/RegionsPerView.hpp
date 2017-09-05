// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#pragma once

#include <openMVG/types.hpp>
#include <openMVG/features/regions.hpp>
#include <openMVG/features/image_describer.hpp>
#include <openMVG/features/ImageDescriberCommon.hpp>

#include <memory>

namespace openMVG {
namespace features {

/// Regions per ViewId of the considered SfM_Data container
class MapRegionsPerDesc : public std::map<features::EImageDescriberType, std::unique_ptr<features::Regions>>
{
public:
  std::size_t getNbAllRegions() const
  {
    std::size_t nb = 0;
    for(const auto& it: *this)
      nb += it.second->RegionCount();
    return nb;
  }

  template<class T>
  T getRegions(features::EImageDescriberType descType) { return dynamic_cast<T&>(*this->at(descType)); }

  template<class T>
  const T getRegions(features::EImageDescriberType descType) const { return dynamic_cast<const T&>(*this->at(descType)); }
};

using MapRegionsPerView = std::map<IndexT, MapRegionsPerDesc>;

template<class MapFeatOrRegionsPerDesc>
inline std::vector<features::EImageDescriberType> getCommonDescTypes(const MapFeatOrRegionsPerDesc& regionsA, const MapFeatOrRegionsPerDesc& regionsB)
{
  std::vector<features::EImageDescriberType> descTypes;
  for(const auto& regionsPerDesc : regionsA)
  {
    const auto desc = regionsPerDesc.first;
    if(regionsB.count(desc) > 0)
    {
      descTypes.push_back(desc);
    }
  }
  return descTypes;
}

/**
 * @brief Container for all Regions (Features and Descriptors) for each View.
 */
class RegionsPerView
{
public:
  MapRegionsPerView& getData()
  {
    return _data;
  }

  const MapRegionsPerView& getData() const
  {
    return _data;
  }

  // TODO: to remove
  const features::Regions& getFirstViewRegions(features::EImageDescriberType descType) const
  {
    assert(descType != features::EImageDescriberType::UNINITIALIZED);
    return *(_data.begin()->second.at(descType).get());
  }

  const features::MapRegionsPerDesc& getRegionsPerDesc(IndexT viewId) const
  {
    return _data.at(viewId);
  }
  const features::MapRegionsPerDesc& getDataPerDesc(IndexT viewId) const
  {
    return _data.at(viewId);
  }

  const features::Regions& getRegions(IndexT viewId, features::EImageDescriberType descType) const
  {
    assert(descType != features::EImageDescriberType::UNINITIALIZED);
    return *(_data.at(viewId).at(descType).get());
  }
  
  const MapRegionsPerDesc& getAllRegions(IndexT viewId) const
  {
    return _data.at(viewId);
  }
  
  bool viewExist(IndexT viewId) const
  {
    return _data.count(viewId) > 0;
  }
  
  bool isEmpty() const
  {
    return _data.empty();
  }
  
  void addRegions(IndexT viewId, features::EImageDescriberType descType, features::Regions* regionsPtr)
  {
    assert(descType != features::EImageDescriberType::UNINITIALIZED);
    _data[viewId][descType].reset(regionsPtr);
  }

  std::vector<features::EImageDescriberType> getCommonDescTypes(const Pair& pair) const
  {
    const auto& regionsA = getAllRegions(pair.first);
    const auto& regionsB = getAllRegions(pair.second);
    return openMVG::features::getCommonDescTypes(regionsA, regionsB);
  }
  
  void clearDescriptors()
  {
    for(auto& itA: _data)
    {
      for(auto& itB: itA.second)
      {
        itB.second->clearDescriptors();
      }
    }
  }
  
private:
  MapRegionsPerView _data;
};

} // namespace features
} // namespace openMVG


