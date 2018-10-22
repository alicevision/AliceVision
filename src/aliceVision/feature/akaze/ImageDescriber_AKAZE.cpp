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
  _params._options.fDesc_factor =
    (_params._eAkazeDescriptor == AKAZE_MSURF ||
    _params._eAkazeDescriptor == AKAZE_LIOP) ? 10.f*sqrtf(2.f)
    : 11.f*sqrtf(2.f); // MLDB

  AKAZE akaze(image, _params._options);
  akaze.Compute_AKAZEScaleSpace();
  std::vector<AKAZEKeypoint> kpts;
  kpts.reserve(5000);
  akaze.Feature_Detection(kpts);
  akaze.Do_Subpixel_Refinement(kpts);
  akaze.gridFiltering(kpts);

  allocate(regions);

  switch(_params._eAkazeDescriptor)
  {
    case AKAZE_MSURF:
    {
      // Build alias to cached data
      AKAZE_Float_Regions * regionsCasted = dynamic_cast<AKAZE_Float_Regions*>(regions.get());
      regionsCasted->Features().resize(kpts.size());
      regionsCasted->Descriptors().resize(kpts.size());

#pragma omp parallel for
      for (int i = 0; i < static_cast<int>(kpts.size()); ++i)
      {
        AKAZEKeypoint ptAkaze = kpts[i];

        // Feature masking
        if (mask)
        {
          const image::Image<unsigned char> & maskIma = *mask;
          if (maskIma(ptAkaze.y, ptAkaze.x) > 0)
            continue;
        }

        const TEvolution & cur_slice = akaze.getSlices()[ptAkaze.class_id];

        if (_bOrientation)
          akaze.Compute_Main_Orientation(ptAkaze, cur_slice.Lx, cur_slice.Ly);
        else
          ptAkaze.angle = 0.0f;

        regionsCasted->Features()[i] =
          SIOPointFeature(ptAkaze.x, ptAkaze.y, ptAkaze.size, ptAkaze.angle);

        ComputeMSURFDescriptor(cur_slice.Lx, cur_slice.Ly, ptAkaze.octave,
          regionsCasted->Features()[i],
          regionsCasted->Descriptors()[i]);
      }
    }
    break;
    case AKAZE_LIOP:
    {
      // Build alias to cached data
      AKAZE_Liop_Regions * regionsCasted = dynamic_cast<AKAZE_Liop_Regions*>(regions.get());
      regionsCasted->Features().resize(kpts.size());
      regionsCasted->Descriptors().resize(kpts.size());

      // Init LIOP extractor
      DescriptorExtractor_LIOP liop_extractor;

#pragma omp parallel for
      for (int i = 0; i < static_cast<int>(kpts.size()); ++i)
      {
        AKAZEKeypoint ptAkaze = kpts[i];

        // Feature masking
        if (mask)
        {
          const image::Image<unsigned char> & maskIma = *mask;
          if (maskIma(ptAkaze.y, ptAkaze.x) > 0)
            continue;
        }

        const TEvolution & cur_slice = akaze.getSlices()[ptAkaze.class_id];

        if (_bOrientation)
          akaze.Compute_Main_Orientation(ptAkaze, cur_slice.Lx, cur_slice.Ly);
        else
          ptAkaze.angle = 0.0f;

        regionsCasted->Features()[i] =
          SIOPointFeature(ptAkaze.x, ptAkaze.y, ptAkaze.size, ptAkaze.angle);

        // Compute LIOP descriptor (do not need rotation computation, since
        //  LIOP descriptor is rotation invariant).
        // Rescale for LIOP patch extraction
        const SIOPointFeature fp =
            SIOPointFeature(ptAkaze.x, ptAkaze.y,
            ptAkaze.size/2.0, ptAkaze.angle);

        float desc[144];
        liop_extractor.extract(image, fp, desc);
        for(int j=0; j < 144; ++j)
          regionsCasted->Descriptors()[i][j] =
            static_cast<unsigned char>(desc[j] * 255.f +.5f);
      }
    }
    break;
    case AKAZE_MLDB:
    {
      // Build alias to cached data
      AKAZE_BinaryRegions * regionsCasted = dynamic_cast<AKAZE_BinaryRegions*>(regions.get());
      regionsCasted->Features().resize(kpts.size());
      regionsCasted->Descriptors().resize(kpts.size());

#pragma omp parallel for
      for (int i = 0; i < static_cast<int>(kpts.size()); ++i)
      {
        AKAZEKeypoint ptAkaze = kpts[i];

        // Feature masking
        if (mask)
        {
          const image::Image<unsigned char> & maskIma = *mask;
          if (maskIma(ptAkaze.y, ptAkaze.x) > 0)
            continue;
        }

        const TEvolution & cur_slice = akaze.getSlices()[ptAkaze.class_id];

        if (_bOrientation)
          akaze.Compute_Main_Orientation(ptAkaze, cur_slice.Lx, cur_slice.Ly);
        else
          ptAkaze.angle = 0.0f;

        regionsCasted->Features()[i] =
          SIOPointFeature(ptAkaze.x, ptAkaze.y, ptAkaze.size, ptAkaze.angle);

        // Compute MLDB descriptor
        Descriptor<bool,486> desc;
        ComputeMLDBDescriptor(cur_slice.cur, cur_slice.Lx, cur_slice.Ly,
          ptAkaze.octave, regionsCasted->Features()[i], desc);
        // convert the bool vector to the binary unsigned char array
        unsigned char * ptr = reinterpret_cast<unsigned char*>(&regionsCasted->Descriptors()[i]);
        memset(ptr, 0, regionsCasted->DescriptorLength()*sizeof(unsigned char));
        // For each byte
        for (int j = 0; j < std::ceil(486./8.); ++j, ++ptr) {
          // set the corresponding 8bits to the good values
          for (int iBit = 0; iBit < 8 && j*8+iBit < 486; ++iBit)  {
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
