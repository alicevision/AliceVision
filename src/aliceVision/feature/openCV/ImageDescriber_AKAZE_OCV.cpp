// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ImageDescriber_AKAZE_OCV.hpp"

#include "aliceVision/image/all.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

namespace aliceVision {
namespace feature {

bool ImageDescriber_AKAZE_OCV::describe(const image::Image<unsigned char>& image,
                                           std::unique_ptr<Regions>& regions,
                                           const image::Image<unsigned char>* mask)
{
  cv::Mat img;
  cv::eigen2cv(image.GetMat(), img);

  std::vector< cv::KeyPoint > vec_keypoints;
  cv::Mat m_desc;

  cv::Ptr<cv::Feature2D> extractor = cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_KAZE);
  extractor->detectAndCompute(img, cv::Mat(), vec_keypoints, m_desc);

  if (vec_keypoints.empty())
  {
    return false;
  }

  allocate(regions);

  // Build alias to cached data
  AKAZE_Float_Regions* regionsCasted = dynamic_cast<AKAZE_Float_Regions*>(regions.get());

  // Reserve some memory for faster keypoint saving
  regionsCasted->Features().reserve(vec_keypoints.size());
  regionsCasted->Descriptors().reserve(vec_keypoints.size());

  typedef Descriptor<float, 64> DescriptorT;
  DescriptorT descriptor;
  int cpt = 0;

  for(const auto& i_keypoint : vec_keypoints)
  {
    PointFeature feat(i_keypoint.pt.x, i_keypoint.pt.y, i_keypoint.size, i_keypoint.angle);
    regionsCasted->Features().push_back(feat);

    memcpy(descriptor.getData(),
           m_desc.ptr<typename DescriptorT::bin_type>(cpt),
           DescriptorT::static_size*sizeof(DescriptorT::bin_type));

    regionsCasted->Descriptors().push_back(descriptor);
    ++cpt;
  }

  return true;
}

} //namespace feature
} //namespace aliceVision
