// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#pragma once

#include <aliceVision/types.hpp>
#include <aliceVision/feature/Regions.hpp>
#include <aliceVision/feature/ImageDescriber.hpp>
#include <aliceVision/feature/imageDescriberCommon.hpp>

#include <memory>

namespace aliceVision {
namespace feature {

/// Regions per ViewId of the considered SfM_Data container
class MapRegionsPerDesc : public std::map<feature::EImageDescriberType, std::unique_ptr<feature::Regions>>
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
  T getRegions(feature::EImageDescriberType descType) { return dynamic_cast<T&>(*this->at(descType)); }

  template<class T>
  const T getRegions(feature::EImageDescriberType descType) const { return dynamic_cast<const T&>(*this->at(descType)); }
};

using MapRegionsPerView = std::map<IndexT, MapRegionsPerDesc>;

template<class MapFeatOrRegionsPerDesc>
inline std::vector<feature::EImageDescriberType> getCommonDescTypes(const MapFeatOrRegionsPerDesc& regionsA, const MapFeatOrRegionsPerDesc& regionsB)
{
  std::vector<feature::EImageDescriberType> descTypes;
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
  const feature::Regions& getFirstViewRegions(feature::EImageDescriberType descType) const
  {
    assert(descType != feature::EImageDescriberType::UNINITIALIZED);
    return *(_data.begin()->second.at(descType).get());
  }

  const feature::MapRegionsPerDesc& getRegionsPerDesc(IndexT viewId) const
  {
    return _data.at(viewId);
  }
  const feature::MapRegionsPerDesc& getDataPerDesc(IndexT viewId) const
  {
    return _data.at(viewId);
  }

  const feature::Regions& getRegions(IndexT viewId, feature::EImageDescriberType descType) const
  {
    assert(descType != feature::EImageDescriberType::UNINITIALIZED);
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
  
  void addRegions(IndexT viewId, feature::EImageDescriberType descType, feature::Regions* regionsPtr)
  {
    assert(descType != feature::EImageDescriberType::UNINITIALIZED);
    _data[viewId][descType].reset(regionsPtr);
  }

  std::vector<feature::EImageDescriberType> getCommonDescTypes(const Pair& pair) const
  {
    const auto& regionsA = getAllRegions(pair.first);
    const auto& regionsB = getAllRegions(pair.second);
    return aliceVision::feature::getCommonDescTypes(regionsA, regionsB);
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

} // namespace feature
} // namespace aliceVision


