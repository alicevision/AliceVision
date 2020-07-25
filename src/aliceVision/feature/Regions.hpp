// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/types.hpp>
#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/feature/PointFeature.hpp>
#include <aliceVision/feature/Descriptor.hpp>
#include <aliceVision/feature/metric.hpp>

#include <string>
#include <cstddef>
#include <typeinfo>
#include <memory>


namespace aliceVision {
namespace feature {

/**
 * @brief Store a featureIndex and the associated point3dId
 */
struct FeatureInImage
{
  FeatureInImage(IndexT featureIndex, IndexT point3dId)
    : _featureIndex(featureIndex)
    , _point3dId(point3dId)
  {}

  IndexT _featureIndex;
  IndexT _point3dId;

  bool operator<(const FeatureInImage& other) const
  {
    return _featureIndex < other._featureIndex;
  }
};

/// Describe an image a set of regions (position, ...) + attributes
/// Each region is described by a set of attributes (descriptor)
/**
 * @brief The Regions class describe a set of regions extracted from an image.
 *        It contains both a feature (position, scale, orientation) and a descriptor (photometric neighborhood description).
 */
class Regions
{
public:

  virtual ~Regions() = 0;

protected:
  std::vector<PointFeature> _vec_feats;    // region features

public:
  void LoadFeatures(const std::string& sfileNameFeats)
  {
    loadFeatsFromFile(sfileNameFeats, _vec_feats);
  }

  PointFeatures GetRegionsPositions() const
  {
    return PointFeatures(_vec_feats.begin(), _vec_feats.end());
  }

  Vec2 GetRegionPosition(std::size_t i) const
  {
    return Vec2f(_vec_feats[i].coords()).cast<double>();
  }

  /// Return the number of defined regions
  std::size_t RegionCount() const {return _vec_feats.size();}

  /// Mutable and non-mutable PointFeature getters.
  inline std::vector<PointFeature> & Features() { return _vec_feats; }
  inline const std::vector<PointFeature> & Features() const { return _vec_feats; }

  //--
  // IO - one file for region features, one file for region descriptors
  //--

  virtual void Load(
    const std::string& sfileNameFeats,
    const std::string& sfileNameDescs) = 0;

  virtual void Save(
    const std::string& sfileNameFeats,
    const std::string& sfileNameDescs) const = 0;

  virtual void SaveDesc(const std::string& sfileNameDescs) const = 0;

  //--
  //- Basic description of a descriptor [Type, Length]
  //--
  virtual bool IsScalar() const = 0;
  virtual bool IsBinary() const = 0;

  /// basis element used for description
  virtual std::string Type_id() const = 0;
  virtual std::size_t DescriptorLength() const = 0;

  /**
   * @brief Return a blind pointer to the container of the descriptors array.
   *
   * @note: Descriptors are always stored as an std::vector<DescType>.
   */
  virtual const void* blindDescriptors() const = 0;

  /**
   * @brief Return a pointer to the first value of the descriptor array.
   *
   * @note: Descriptors are always stored as a flat array of descriptors.
   */
  virtual const void * DescriptorRawData() const = 0;

  virtual void clearDescriptors() = 0;

  /// Return the squared distance between two descriptors
  // A default metric is used according the descriptor type:
  // - Scalar: L2,
  // - Binary: Hamming
  virtual double SquaredDescriptorDistance(std::size_t i, const Regions *, std::size_t j) const = 0;

  /// Add the Inth region to another Region container
  virtual void CopyRegion(std::size_t i, Regions *) const = 0;

  virtual Regions * EmptyClone() const = 0;

  virtual std::unique_ptr<Regions> createFilteredRegions(
                     const std::vector<FeatureInImage>& featuresInImage,
                     std::vector<IndexT>& out_associated3dPoint,
                     std::map<IndexT, IndexT>& out_mapFullToLocal) const = 0;

};

inline Regions::~Regions() {}

enum class ERegionType: bool
{
  Binary = 0,
  Scalar = 1
};


template<typename T, ERegionType regionType>
struct SquaredMetric;

template<typename T>
struct SquaredMetric<T, ERegionType::Scalar>
{
  using Metric = L2_Vectorized<T>;
};

template<typename T>
struct SquaredMetric<T, ERegionType::Binary>
{
  using Metric = SquaredHamming<T>;
};


template<typename T, std::size_t L, ERegionType regionType>
class FeatDescRegions : public Regions
{
public:
  typedef FeatDescRegions<T, L, regionType> This;
  /// Region descriptor
  typedef Descriptor<T, L> DescriptorT;
  /// Container for multiple regions description
  typedef std::vector<DescriptorT> DescsT;

protected:
  std::vector<DescriptorT> _vec_descs; // region descriptions

public:
  std::string Type_id() const override {return typeid(T).name();}
  std::size_t DescriptorLength() const override {return static_cast<std::size_t>(L);}

  bool IsScalar() const override { return regionType == ERegionType::Scalar; }
  bool IsBinary() const override { return regionType == ERegionType::Binary; }

  Regions * EmptyClone() const override
  {
    return new This();
  }

  /// Read from files the regions and their corresponding descriptors.
  void Load(
    const std::string& sfileNameFeats,
    const std::string& sfileNameDescs) override
  {
    loadFeatsFromFile(sfileNameFeats, this->_vec_feats);
    loadDescsFromBinFile(sfileNameDescs, _vec_descs);
  }

  /// Export in two separate files the regions and their corresponding descriptors.
  void Save(
    const std::string& sfileNameFeats,
    const std::string& sfileNameDescs) const override
  {
    saveFeatsToFile(sfileNameFeats, this->_vec_feats);
    saveDescsToBinFile(sfileNameDescs, _vec_descs);
  }

  void SaveDesc(const std::string& sfileNameDescs) const override
  {
    saveDescsToBinFile(sfileNameDescs, _vec_descs);
  }

  /// Mutable and non-mutable DescriptorT getters.
  inline std::vector<DescriptorT> & Descriptors() { return _vec_descs; }
  inline const std::vector<DescriptorT> & Descriptors() const { return _vec_descs; }

  inline const void* blindDescriptors() const override { return &_vec_descs; }

  inline const void* DescriptorRawData() const override { return &_vec_descs[0];}

  inline void clearDescriptors() override { _vec_descs.clear(); }

  inline void swap(This& other)
  {
    this->_vec_feats.swap(other._vec_feats);
    _vec_descs.swap(other._vec_descs);
  }

  // Return the distance between two descriptors
  double SquaredDescriptorDistance(std::size_t i, const Regions * genericRegions, std::size_t j) const override
  {
    assert(i < this->_vec_descs.size());
    assert(genericRegions);
    assert(j < genericRegions->RegionCount());

    const This * regionsT = dynamic_cast<const This*>(genericRegions);
    static typename SquaredMetric<T, regionType>::Metric metric;
    return metric(this->_vec_descs[i].getData(), regionsT->_vec_descs[j].getData(), DescriptorT::static_size);
  }

  /**
   * @brief Add the Inth region to another Region container
   * @param[in] i: index of the region to copy
   * @param[out] outRegionContainer: the output region group to add the region
   */
  void CopyRegion(std::size_t i, Regions * outRegionContainer) const override
  {
    assert(i < this->_vec_feats.size() && i < this->_vec_descs.size());
    static_cast<This*>(outRegionContainer)->_vec_feats.push_back(this->_vec_feats[i]);
    static_cast<This*>(outRegionContainer)->_vec_descs.push_back(this->_vec_descs[i]);
  }

  /**
   * @brief Duplicate only reconstructed regions.
   * @param[in] featuresInImage list of features with an associated 3D point Id
   * @param[out] out_associated3dPoint
   * @param[out] out_mapFullToLocal
   */
  std::unique_ptr<Regions> createFilteredRegions(
                     const std::vector<FeatureInImage>& featuresInImage,
                     std::vector<IndexT>& out_associated3dPoint,
                     std::map<IndexT, IndexT>& out_mapFullToLocal) const override
  {
    out_associated3dPoint.clear();
    out_mapFullToLocal.clear();

    This* regionsPtr = new This;
    std::unique_ptr<Regions> regions(regionsPtr);
    regionsPtr->Features().reserve(featuresInImage.size());
    regionsPtr->Descriptors().reserve(featuresInImage.size());
    out_associated3dPoint.reserve(featuresInImage.size());
    for(std::size_t i = 0; i < featuresInImage.size(); ++i)
    {
      const FeatureInImage & feat = featuresInImage[i];
      regionsPtr->Features().push_back(this->_vec_feats[feat._featureIndex]);
      regionsPtr->Descriptors().push_back(this->_vec_descs[feat._featureIndex]);

      // This assert should be valid in theory, but in the context of CameraLocalization
      // we can have the same 2D feature associated to different 3D points (2 in practice).
      // In this particular case, currently it returns the last one...
      //
      // assert(out_mapFullToLocal.count(feat._featureIndex) == 0);

      out_mapFullToLocal[feat._featureIndex] = i;
      out_associated3dPoint.push_back(feat._point3dId);
    }
    return regions;
  }
};


template<typename T, std::size_t L>
using ScalarRegions = FeatDescRegions<T, L, ERegionType::Scalar>;

template<std::size_t L>
using BinaryRegions = FeatDescRegions<unsigned char, L, ERegionType::Binary>;



} // namespace feature
} // namespace aliceVision
