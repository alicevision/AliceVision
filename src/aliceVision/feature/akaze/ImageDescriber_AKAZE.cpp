// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ImageDescriber_AKAZE.hpp"

namespace aliceVision {
namespace feature {

using namespace std;

bool ImageDescriber_AKAZE::describe(const image::Image<float>& image,
                                    std::unique_ptr<Regions>& regions,
                                    const image::Image<unsigned char>* mask)
{
  _params.options.descFactor =
    (_params.akazeDescriptorType == AKAZE_MSURF ||
     _params.akazeDescriptorType == AKAZE_LIOP) ? 10.f * sqrtf(2.f)
    : 11.f * sqrtf(2.f); // MLDB


  std::vector<AKAZEKeypoint> keypoints;
  keypoints.reserve(_params.options.maxTotalKeypoints * 2);

  AKAZE akaze(image, _params.options);
  akaze.computeScaleSpace();
  akaze.featureDetection(keypoints);
  akaze.subpixelRefinement(keypoints);
  akaze.gridFiltering(keypoints);

  allocate(regions);

  switch(_params.akazeDescriptorType)
  {
    case AKAZE_MSURF:
    {
      // build alias to cached data
      AKAZE_Float_Regions* regionsCasted = dynamic_cast<AKAZE_Float_Regions*>(regions.get());
      regionsCasted->Features().resize(keypoints.size());
      regionsCasted->Descriptors().resize(keypoints.size());

#pragma omp parallel for
      for(int i = 0; i < static_cast<int>(keypoints.size()); ++i)
      {
        AKAZEKeypoint point = keypoints.at(i);

        // feature masking
        if(mask)
        {
          const image::Image<unsigned char>& maskIma = *mask;
          if(maskIma(point.y, point.x) > 0)
            continue;
        }

        const AKAZE::TEvolution& cur_slice = akaze.getSlices()[point.class_id];

        if(_isOriented)
          akaze.computeMainOrientation(point, cur_slice.Lx, cur_slice.Ly);
        else
          point.angle = 0.0f;

        regionsCasted->Features()[i] =
          PointFeature(point.x, point.y, point.size, point.angle);

        ComputeMSURFDescriptor(cur_slice.Lx, cur_slice.Ly, point.octave,
          regionsCasted->Features()[i],
          regionsCasted->Descriptors()[i]);
      }
    }
    break;
    case AKAZE_LIOP:
    {
      // build alias to cached data
      AKAZE_Liop_Regions* regionsCasted = dynamic_cast<AKAZE_Liop_Regions*>(regions.get());
      regionsCasted->Features().resize(keypoints.size());
      regionsCasted->Descriptors().resize(keypoints.size());

      // init LIOP extractor
      DescriptorExtractor_LIOP liop_extractor;

#pragma omp parallel for
      for(int i = 0; i < static_cast<int>(keypoints.size()); ++i)
      {
        AKAZEKeypoint point = keypoints[i];

        // feature masking
        if(mask)
        {
          const image::Image<unsigned char>& maskIma = *mask;
          if(maskIma(point.y, point.x) > 0)
            continue;
        }

        const AKAZE::TEvolution& cur_slice = akaze.getSlices()[point.class_id];

        if(_isOriented)
          akaze.computeMainOrientation(point, cur_slice.Lx, cur_slice.Ly);
        else
          point.angle = 0.0f;

        regionsCasted->Features()[i] =
          PointFeature(point.x, point.y, point.size, point.angle);

        // compute LIOP descriptor (do not need rotation computation, since
        // LIOP descriptor is rotation invariant).
        // rescale for LIOP patch extraction
        const PointFeature fp = PointFeature(point.x, point.y, point.size/2.0, point.angle);

        float desc[144];
        liop_extractor.extract(image, fp, desc);
        for(int j=0; j < 144; ++j)
          regionsCasted->Descriptors()[i][j] = static_cast<unsigned char>(desc[j] * 255.f + .5f);
      }
    }
    break;
    case AKAZE_MLDB:
    {
      // Build alias to cached data
      AKAZE_BinaryRegions* regionsCasted = dynamic_cast<AKAZE_BinaryRegions*>(regions.get());
      regionsCasted->Features().resize(keypoints.size());
      regionsCasted->Descriptors().resize(keypoints.size());

#pragma omp parallel for
      for (int i = 0; i < static_cast<int>(keypoints.size()); ++i)
      {
        AKAZEKeypoint point = keypoints[i];

        // Feature masking
        if(mask)
        {
          const image::Image<unsigned char> & maskIma = *mask;
          if (maskIma(point.y, point.x) > 0)
            continue;
        }

        const AKAZE::TEvolution& cur_slice = akaze.getSlices()[point.class_id];

        if(_isOriented)
          akaze.computeMainOrientation(point, cur_slice.Lx, cur_slice.Ly);
        else
          point.angle = 0.0f;

        regionsCasted->Features()[i] = PointFeature(point.x, point.y, point.size, point.angle);

        // compute MLDB descriptor
        Descriptor<bool,486> desc;
        ComputeMLDBDescriptor(cur_slice.cur, cur_slice.Lx, cur_slice.Ly, point.octave, regionsCasted->Features()[i], desc);
        // convert the bool vector to the binary unsigned char array
        unsigned char* ptr = reinterpret_cast<unsigned char*>(&regionsCasted->Descriptors()[i]);
        memset(ptr, 0, regionsCasted->DescriptorLength()*sizeof(unsigned char));

        for(int j = 0; j < std::ceil(486./8.); ++j, ++ptr) // for each byte
        {
          // set the corresponding 8bits to the good values
          for(int iBit = 0; iBit < 8 && j*8+iBit < 486; ++iBit)
          {
            *ptr |= desc[j*8+iBit] << iBit;
          }
        }
      }
    }
    break;
  }
  return true;
}

} // namespace feature
} // namespace aliceVision
