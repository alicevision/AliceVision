// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ImageDescriber_BRISK_OCV.hpp"

#include "aliceVision/image/all.hpp"

#include <opencv2/opencv.hpp>   
#include <opencv2/core/eigen.hpp>

namespace aliceVision {
    namespace feature {

        bool ImageDescriber_BRISK_OCV::describe(const image::Image<unsigned char>& image,
                                                std::unique_ptr<Regions>& regions,
                                                const image::Image<unsigned char>* mask)
        {
        cv::Mat img;
        cv::eigen2cv(image.GetMat(), img);

        std::vector< cv::KeyPoint > vec_keypoints;
        cv::Mat m_desc;

        cv::Ptr<cv::Feature2D> extractor = cv::BRISK::create(int thresh = 30,
                                                             int octaves = 3,
                                                             float patternScale = 1.0f);
        
        extractor->detectAndCompute(img, cv::Mat(), vec_keypoints, m_desc);

        if (vec_keypoints.empty())
        {
            return false;
        }

        allocate(regions);

        // Build alias to cached data
        BRISK_Float_Regions* regionsCasted = dynamic_cast<BRISK_Float_Regions*>(regions.get());

        // Reserve some memory for faster keypoint saving
        regionsCasted->Features().reserve(vec_keypoints.size());
        regionsCasted->Descriptors().reserve(vec_keypoints.size());

        typedef Descriptor<float, 64> DescriptorT;
        DescriptorT descriptor;
        int cpt = 0;

        for(const auto& i_keypoint : vec_keypoints)
        {
            SIOPointFeature feat(i_keypoint.pt.x, i_keypoint.pt.y, i_keypoint.size, i_keypoint.angle);
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
