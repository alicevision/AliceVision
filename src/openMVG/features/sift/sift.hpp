#pragma once

#include <openMVG/features/descriptor.hpp>
#include <openMVG/features/image_describer.hpp>
#include <openMVG/features/regions_factory.hpp>
#include <openMVG/voctree/distance.hpp>

#include <opencv2/opencv.hpp>
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/eigen.hpp"

#include "third_party/vectorGraphics/svgDrawer.hpp"

extern "C" {
#include "nonFree/sift/vl/sift.h"
}

#include <cereal/cereal.hpp>

#include <map>
#include <iostream>
#include <iomanip>
#include <numeric>
#include <stdlib.h>

namespace openMVG {
namespace features {

struct SiftParams
{
  SiftParams(
    int first_octave = 0,
    int num_octaves = 6,
    int num_scales = 3,
    float edge_threshold = 10.0f,
    float peak_threshold = 0.04f,
    std::size_t gridSize = 4,
    std::size_t maxTotalKeypoints = 1000,
    bool root_sift = true
  )
    : _first_octave(first_octave)
    , _num_octaves(num_octaves)
    , _num_scales(num_scales)
    , _edge_threshold(edge_threshold)
    , _peak_threshold(peak_threshold)
    , _gridSize(gridSize)
    , _maxTotalKeypoints(maxTotalKeypoints)
    , _root_sift(root_sift)
  {}

  template<class Archive>
  void serialize( Archive & ar )
  {
    ar(
      cereal::make_nvp("first_octave", _first_octave),      
      cereal::make_nvp("num_octaves",_num_octaves),
      cereal::make_nvp("num_scales",_num_scales),
      cereal::make_nvp("edge_threshold",_edge_threshold),
      cereal::make_nvp("peak_threshold",_peak_threshold),
      //
      cereal::make_nvp("grid_size", _gridSize),
      cereal::make_nvp("max_total_keypoints", _maxTotalKeypoints),
      //
      cereal::make_nvp("root_sift",_root_sift));
  }

  // Parameters
  int _first_octave;      // Use original image, or perform an upscale if == -1
  int _num_octaves;       // Max octaves count
  int _num_scales;        // Scales per octave
  float _edge_threshold;  // Max ratio of Hessian eigenvalues
  float _peak_threshold;  // Min contrast
  std::size_t _gridSize;  // Square Grid width
  std::size_t _maxTotalKeypoints; // Hard threshold to limit the number of keypoints
  bool _root_sift;        // see [1]

  bool setPreset(EDESCRIBER_PRESET preset)
  {
    switch(preset)
    {
    case LOW_PRESET:
    {
      _maxTotalKeypoints = 1000;
      _peak_threshold = 0.04f;
      _first_octave = 2;
      break;
    }
    case MEDIUM_PRESET:
    {
      _maxTotalKeypoints = 5000;
      _peak_threshold = 0.04f;
      _first_octave = 1;
      break;
    }
    case NORMAL_PRESET:
    {
      _maxTotalKeypoints = 10000;
      _peak_threshold = 0.04f;
      break;
    }
    case HIGH_PRESET:
    {
      _maxTotalKeypoints = 20000;
      _peak_threshold = 0.01f;
      break;
    }
    case ULTRA_PRESET:
    {
      _maxTotalKeypoints = 40000;
      _peak_threshold = 0.01f;
      _first_octave = -1;
      break;
    }
    default:
      return false;
    }
    return true;
  }
};


/**
 * @brief Extract SIFT regions (in float or unsigned char).
 * Classic SIFT extraction
 * @param image - The input image
 * @param regions - The detected regions and attributes (the caller must delete the allocated data)
 * @param params - The parameters of the SIFT extractor
 * @param bOrientation - Compute orientation of SIFT descriptor (for the first extraction only)
 * @param mask - 8-bit gray image for keypoint filtering (optional).
 */
template < typename T >
bool extractSIFT(const image::Image<unsigned char>& image,
    std::unique_ptr<Regions>& regions,
    const SiftParams& params,
    const bool bOrientation,
    const image::Image<unsigned char> * mask);


} //namespace features
} //namespace openMVG

