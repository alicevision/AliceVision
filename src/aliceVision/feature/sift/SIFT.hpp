// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/feature/Descriptor.hpp>
#include <aliceVision/feature/ImageDescriber.hpp>
#include <aliceVision/feature/regionsFactory.hpp>
#include <aliceVision/config.hpp>
#include <aliceVision/system/Logger.hpp>

extern "C" {
#include "nonFree/sift/vl/sift.h"
}

#include <iostream>
#include <numeric>
#include <stdexcept>

namespace aliceVision {
namespace feature {

/**
 * Bibliography:
 * [1] R. Arandjelović, A. Zisserman.
 * Three things everyone should know to improve object retrieval. CVPR2012.
 */

struct SiftParams
{
  SiftParams(int firstOctave = 0,
             int numOctaves = 6,
             int numScales = 3,
             float edgeThreshold = 10.0f,
             float peakThreshold = 0.04f,
             std::size_t gridSize = 4,
             std::size_t maxTotalKeypoints = 1000,
             bool rootSift = true)
    : _firstOctave(firstOctave)
    , _numOctaves(numOctaves)
    , _numScales(numScales)
    , _edgeThreshold(edgeThreshold)
    , _peakThreshold(peakThreshold)
    , _gridSize(gridSize)
    , _maxTotalKeypoints(maxTotalKeypoints)
    , _rootSift(rootSift)
  {}

  // Parameters

  /// Use original image, or perform an upscale if == -1
  int _firstOctave;
  /// Max octaves count
  int _numOctaves;
  /// Scales per octave
  int _numScales;
  /// Max ratio of Hessian eigenvalues
  float _edgeThreshold;
  /// Min contrast
  float _peakThreshold;
  std::size_t _gridSize;
  std::size_t _maxTotalKeypoints;
  /// see [1]
  bool _rootSift;
  
  void setPreset(EImageDescriberPreset preset)
  {
    switch(preset)
    {
      case EImageDescriberPreset::LOW:
      {
        _maxTotalKeypoints = 1000;
        _peakThreshold = 0.04f;
        _firstOctave = 2;
        break;
      }
      case EImageDescriberPreset::MEDIUM:
      {
        _maxTotalKeypoints = 5000;
        _peakThreshold = 0.04f;
        _firstOctave = 1;
        break;
      }
      case EImageDescriberPreset::NORMAL:
      {
        _maxTotalKeypoints = 10000;
        _peakThreshold = 0.02f;
        _firstOctave = 0;
        break;
      }
      case EImageDescriberPreset::HIGH:
      {
        _maxTotalKeypoints = 50000;
        _peakThreshold = 0.01f;
        _firstOctave = 0;
        break;
      }
      case EImageDescriberPreset::ULTRA:
      {
        _maxTotalKeypoints = 100000;
        _peakThreshold = 0.01f;
        _firstOctave = -1;
        break;
      }
      default:
        throw std::out_of_range("Invalid image describer preset enum");
    }
  }
};

// VLFeat Instance management
class VLFeatInstance
{
public:

  static void initialize();

  static void destroy();

private:
  static int nbInstances;
};

//convertSIFT
//////////////////////////////
template < typename TOut > 
inline void convertSIFT(
  const vl_sift_pix* descr,
  Descriptor<TOut,128> & descriptor,
  bool rootSift
  );

template <> 
inline void convertSIFT<float>(
  const vl_sift_pix* descr,
  Descriptor<float,128> &descriptor,
  bool rootSift)
{
  if(rootSift)
  {
    const float sum = std::accumulate(descr, descr + 128, 0.0f);
    for(int k = 0; k < 128; ++k)
      descriptor[k] = std::floor(512.f*sqrt(descr[k] / sum));
  }
  else
  {
    for(int k = 0; k < 128; ++k)
      descriptor[k] = std::floor(512.f*descr[k]);
  }
}

template <> 
inline void convertSIFT<unsigned char>(
  const vl_sift_pix* descr,
  Descriptor<unsigned char,128> & descriptor,
  bool rootSift)
{
  if (rootSift)
  {
    // rootsift = sqrt( sift / sum(sift) );
    const float sum = std::accumulate(descr, descr+128, 0.0f);
    for (int k=0;k<128;++k)
      descriptor[k] = static_cast<unsigned char>(512.f*sqrt(descr[k]/sum));
  }
  else
  {
    for (int k=0;k<128;++k)
      descriptor[k] = static_cast<unsigned char>(512.f*descr[k]);
  }
}

/**
 * @brief Get the total amount of RAM needed for a
 * feature extraction of an image of the given dimension.
 * @param[in] width The image width
 * @param[in] height The image height
 * @return total amount of memory needed
 */
std::size_t getMemoryConsumptionVLFeat(std::size_t width, std::size_t height, const SiftParams& params);

/**
 * @brief Extract SIFT regions (in float or unsigned char).
 *
 * @param image
 * @param regions
 * @param params
 * @param orientation
 * @param mask
 * @return
 */
template <typename T>
bool extractSIFT(const image::Image<float>& image,
    std::unique_ptr<Regions>& regions,
    const SiftParams& params,
    bool orientation,
    const image::Image<unsigned char>* mask)
{
  const int w = image.Width(), h = image.Height();
  VlSiftFilt *filt = vl_sift_new(w, h, params._numOctaves, params._numScales, params._firstOctave);
  if (params._edgeThreshold >= 0)
    vl_sift_set_edge_thresh(filt, params._edgeThreshold);
  if (params._peakThreshold >= 0)
    vl_sift_set_peak_thresh(filt, params._peakThreshold/params._numScales);

  Descriptor<vl_sift_pix, 128> vlFeatDescriptor;
  Descriptor<T, 128> descriptor;

  // Process SIFT computation
  vl_sift_process_first_octave(filt, image.data());

  typedef ScalarRegions<SIOPointFeature,T,128> SIFT_Region_T;
  regions.reset( new SIFT_Region_T );
  
  // Build alias to cached data
  SIFT_Region_T * regionsCasted = dynamic_cast<SIFT_Region_T*>(regions.get());
  // reserve some memory for faster keypoint saving
  const std::size_t reserveSize = (params._gridSize && params._maxTotalKeypoints) ? params._maxTotalKeypoints : 2000;
  regionsCasted->Features().reserve(reserveSize);
  regionsCasted->Descriptors().reserve(reserveSize);

  while (true)
  {
    vl_sift_detect(filt);

    VlSiftKeypoint const *keys  = vl_sift_get_keypoints(filt);
    const int nkeys = vl_sift_get_nkeypoints(filt);

    // Update gradient before launching parallel extraction
    vl_sift_update_gradient(filt);

    #pragma omp parallel for private(vlFeatDescriptor, descriptor)
    for (int i = 0; i < nkeys; ++i)
    {

      // Feature masking
      if (mask)
      {
        const image::Image<unsigned char> & maskIma = *mask;
        if (maskIma(keys[i].y, keys[i].x) > 0)
          continue;
      }

      double angles [4] = {0.0, 0.0, 0.0, 0.0};
      int nangles = 1; // by default (1 upright feature)
      if (orientation)
      { // compute from 1 to 4 orientations
        nangles = vl_sift_calc_keypoint_orientations(filt, angles, keys+i);
      }

      for (int q=0 ; q < nangles ; ++q)
      {
        vl_sift_calc_keypoint_descriptor(filt, &vlFeatDescriptor[0], keys+i, angles[q]);
        const SIOPointFeature fp(keys[i].x, keys[i].y,
          keys[i].sigma, static_cast<float>(angles[q]));

        convertSIFT<T>(&vlFeatDescriptor[0], descriptor, params._rootSift);
        
        #pragma omp critical
        {
          regionsCasted->Descriptors().push_back(descriptor);
          regionsCasted->Features().push_back(fp);
        }
        
      }
    }
    
    if (vl_sift_process_next_octave(filt))
      break; // Last octave
  }
  vl_sift_delete(filt);

  const auto& features = regionsCasted->Features();
  const auto& descriptors = regionsCasted->Descriptors();
  assert(features.size() == descriptors.size());
  
  //Sorting the extracted features according to their scale
  {
    std::vector<std::size_t> indexSort(features.size());
    std::iota(indexSort.begin(), indexSort.end(), 0);
    std::sort(indexSort.begin(), indexSort.end(), [&](std::size_t a, std::size_t b){ return features[a].scale() > features[b].scale(); });
    
    std::vector<typename SIFT_Region_T::FeatureT> sortedFeatures(features.size());
    std::vector<typename SIFT_Region_T::DescriptorT> sortedDescriptors(features.size());
    for(std::size_t i: indexSort)
    {
      sortedFeatures[i] = features[indexSort[i]];
      sortedDescriptors[i] = descriptors[indexSort[i]];
    }
    regionsCasted->Features().swap(sortedFeatures);
    regionsCasted->Descriptors().swap(sortedDescriptors);
  }

  // Grid filtering of the keypoints to ensure a global repartition
  if(params._gridSize && params._maxTotalKeypoints)
  {
    // Only filter features if we have more features than the maxTotalKeypoints
    if(features.size() > params._maxTotalKeypoints)
    {
      std::vector<IndexT> filtered_indexes;
      std::vector<IndexT> rejected_indexes;
      filtered_indexes.reserve(std::min(features.size(), params._maxTotalKeypoints));
      rejected_indexes.reserve(features.size());

      const std::size_t sizeMat = params._gridSize * params._gridSize;
      std::vector<std::size_t> countFeatPerCell(sizeMat, 0);
      for (int Indice = 0; Indice < sizeMat; Indice++)
      {
    	  countFeatPerCell[Indice] = 0;
      }
      const std::size_t keypointsPerCell = params._maxTotalKeypoints / sizeMat;
      const double regionWidth = w / double(params._gridSize);
      const double regionHeight = h / double(params._gridSize);

      for(IndexT i = 0; i < features.size(); ++i)
      {
        const auto& keypoint = features.at(i);
        
        const std::size_t cellX = std::min(std::size_t(keypoint.x() / regionWidth), params._gridSize);
        const std::size_t cellY = std::min(std::size_t(keypoint.y() / regionHeight), params._gridSize);

        std::size_t &count = countFeatPerCell[cellX*params._gridSize + cellY];
        ++count;

        if(count < keypointsPerCell)
          filtered_indexes.push_back(i);
        else
          rejected_indexes.push_back(i);
      }
      // If we don't have enough features (less than maxTotalKeypoints) after the grid filtering (empty regions in the grid for example).
      // We add the best other ones, without repartition constraint.
      if( filtered_indexes.size() < params._maxTotalKeypoints )
      {
        const std::size_t remainingElements = std::min(rejected_indexes.size(), params._maxTotalKeypoints - filtered_indexes.size());
        ALICEVISION_LOG_TRACE("Grid filtering -- Copy remaining points: " << remainingElements);
        filtered_indexes.insert(filtered_indexes.end(), rejected_indexes.begin(), rejected_indexes.begin() + remainingElements);
      }

      std::vector<typename SIFT_Region_T::FeatureT> filtered_features(filtered_indexes.size());
      std::vector<typename SIFT_Region_T::DescriptorT> filtered_descriptors(filtered_indexes.size());
      for(IndexT i = 0; i < filtered_indexes.size(); ++i)
      {
        filtered_features[i] = features[filtered_indexes[i]];
        filtered_descriptors[i] = descriptors[filtered_indexes[i]];
      }
      regionsCasted->Features().swap(filtered_features);
      regionsCasted->Descriptors().swap(filtered_descriptors);
    }
  }
  assert(features.size() == descriptors.size());
  
  return true;
}

} //namespace feature
} //namespace aliceVision
