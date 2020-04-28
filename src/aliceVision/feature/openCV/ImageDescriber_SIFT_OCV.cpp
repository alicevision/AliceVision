// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ImageDescriber_SIFT_OCV.hpp"

#include <aliceVision/image/all.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/system/Logger.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/xfeatures2d.hpp>

namespace aliceVision {
namespace feature {

void SIFT_openCV_Params::setConfigurationPreset(EImageDescriberPreset preset)
{
    switch(preset)
    {
      case EImageDescriberPreset::LOW:
        contrastThreshold = 0.01;
        maxTotalKeypoints = 1000;
      break;
      case EImageDescriberPreset::MEDIUM:
        contrastThreshold = 0.005;
        maxTotalKeypoints = 5000;
      break;
      case EImageDescriberPreset::NORMAL:
        contrastThreshold = 0.005;
        edgeThreshold = 15;
        maxTotalKeypoints = 10000;
      break;
      case EImageDescriberPreset::HIGH:
        contrastThreshold = 0.005;
        edgeThreshold = 20;
        maxTotalKeypoints = 20000;
      break;
      case EImageDescriberPreset::ULTRA:
        contrastThreshold = 0.005;
        edgeThreshold = 20;
        maxTotalKeypoints = 40000;
      break;
      default:
        throw std::out_of_range("Invalid image describer preset enum");
    }
}

bool ImageDescriber_SIFT_openCV::describe(const image::Image<unsigned char>& image,
                                          std::unique_ptr<Regions> &regions,
                                          const image::Image<unsigned char> * mask)
{
  // Convert for opencv
  cv::Mat img;
  cv::eigen2cv(image.GetMat(), img);

  // Create a SIFT detector
  std::vector< cv::KeyPoint > v_keypoints;
  cv::Mat m_desc;
  std::size_t maxDetect = 0; //< No max value by default
  if(_params.maxTotalKeypoints)
    if(!_params.gridSize) //< If no grid filtering, use opencv to limit the number of features
      maxDetect = _params.maxTotalKeypoints;

  cv::Ptr<cv::Feature2D> siftdetector = cv::xfeatures2d::SIFT::create(maxDetect,
                                                                      _params.nOctaveLayers,
                                                                      _params.contrastThreshold,
                                                                      _params.edgeThreshold,
                                                                      _params.sigma);

  // Detect SIFT keypoints
  auto detect_start = std::chrono::steady_clock::now();
  siftdetector->detect(img, v_keypoints);
  auto detect_end = std::chrono::steady_clock::now();
  auto detect_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(detect_end - detect_start);

  ALICEVISION_LOG_TRACE("SIFT: contrastThreshold: " << _params.contrastThreshold << ", edgeThreshold: " << _params.edgeThreshold << std::endl);
  ALICEVISION_LOG_TRACE("Detect SIFT: " << detect_elapsed.count() << " milliseconds." << std::endl);
  ALICEVISION_LOG_TRACE("Image size: " << img.cols << " x " << img.rows << std::endl);
  ALICEVISION_LOG_TRACE("Grid size: " << _params.gridSize << ", maxTotalKeypoints: " << _params.maxTotalKeypoints << std::endl);
  ALICEVISION_LOG_TRACE("Number of detected features: " << v_keypoints.size() << std::endl);

  // cv::KeyPoint::response: the response by which the most strong keypoints have been selected.
  // Can be used for the further sorting or subsampling.
  std::sort(v_keypoints.begin(), v_keypoints.end(), [](const cv::KeyPoint& a, const cv::KeyPoint& b) { return a.size > b.size; });

  // Grid filtering of the keypoints to ensure a global repartition
  if(_params.gridSize && _params.maxTotalKeypoints)
  {
    // Only filter features if we have more features than the maxTotalKeypoints
    if(v_keypoints.size() > _params.maxTotalKeypoints)
    {
      std::vector< cv::KeyPoint > filtered_keypoints;
      std::vector< cv::KeyPoint > rejected_keypoints;
      filtered_keypoints.reserve(std::min(v_keypoints.size(), _params.maxTotalKeypoints));
      rejected_keypoints.reserve(v_keypoints.size());

      cv::Mat countFeatPerCell(_params.gridSize, _params.gridSize, cv::DataType<int>::type, cv::Scalar(0));
      const std::size_t keypointsPerCell = _params.maxTotalKeypoints / countFeatPerCell.total();
      const double regionWidth = image.Width() / double(countFeatPerCell.cols);
      const double regionHeight = image.Height() / double(countFeatPerCell.rows);

      ALICEVISION_LOG_TRACE ("Grid filtering -- keypointsPerCell: " << keypointsPerCell
                        << ", regionWidth: " << regionWidth
                        << ", regionHeight: " << regionHeight << std::endl);

      for(const cv::KeyPoint& keypoint: v_keypoints)
      {
        const std::size_t cellX = std::min(std::size_t(keypoint.pt.x / regionWidth), _params.gridSize);
        const std::size_t cellY = std::min(std::size_t(keypoint.pt.y / regionHeight), _params.gridSize);
        // std::cout << "- keypoint.pt.x: " << keypoint.pt.x << ", keypoint.pt.y: " << keypoint.pt.y << std::endl;
        // std::cout << "- cellX: " << cellX << ", cellY: " << cellY << std::endl;
        // std::cout << "- countFeatPerCell: " << countFeatPerCell << std::endl;
        // std::cout << "- gridSize: " << _params.gridSize << std::endl;

        const int count = countFeatPerCell.at<int>(cellX, cellY);
        countFeatPerCell.at<int>(cellX, cellY) = count + 1;
        if(count < keypointsPerCell)
          filtered_keypoints.push_back(keypoint);
        else
          rejected_keypoints.push_back(keypoint);
      }
      // If we don't have enough features (less than maxTotalKeypoints) after the grid filtering (empty regions in the grid for example).
      // We add the best other ones, without repartition constraint.
      if( filtered_keypoints.size() < _params.maxTotalKeypoints )
      {
        const std::size_t remainingElements = std::min(rejected_keypoints.size(), _params.maxTotalKeypoints - filtered_keypoints.size());
        ALICEVISION_LOG_TRACE("Grid filtering -- Copy remaining points: " << remainingElements << std::endl);
        filtered_keypoints.insert(filtered_keypoints.end(), rejected_keypoints.begin(), rejected_keypoints.begin() + remainingElements);
      }

      v_keypoints.swap(filtered_keypoints);
    }
  }
  ALICEVISION_LOG_TRACE("Number of features: " << v_keypoints.size() << std::endl);

  // Compute SIFT descriptors
  auto desc_start = std::chrono::steady_clock::now();
  siftdetector->compute(img, v_keypoints, m_desc);
  auto desc_end = std::chrono::steady_clock::now();
  auto desc_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(desc_end - desc_start);
  ALICEVISION_LOG_TRACE("Compute descriptors: " << desc_elapsed.count() << " milliseconds." << std::endl);

  allocate(regions);

  // Build alias to cached data
  SIFT_Regions * regionsCasted = dynamic_cast<SIFT_Regions*>(regions.get());
  // Reserve some memory for faster keypoint saving
  regionsCasted->Features().reserve(v_keypoints.size());
  regionsCasted->Descriptors().reserve(v_keypoints.size());

  // Prepare a column vector with the sum of each descriptor
  cv::Mat m_siftsum;
  cv::reduce(m_desc, m_siftsum, 1, cv::REDUCE_SUM);

  // Copy keypoints and descriptors in the regions
  int cpt = 0;
  for(const auto& i_kp : v_keypoints)
  {
    PointFeature feat(i_kp.pt.x, i_kp.pt.y, i_kp.size, i_kp.angle);
    regionsCasted->Features().push_back(feat);

    Descriptor<unsigned char, 128> desc;
    for(std::size_t j = 0; j < 128; ++j)
    {
      desc[j] = static_cast<unsigned char>(512.0*sqrt(m_desc.at<float>(cpt, j)/m_siftsum.at<float>(cpt, 0)));
    }
    regionsCasted->Descriptors().push_back(desc);
    ++cpt;
  }

  return true;
}

} //namespace feature
} //namespace aliceVision
