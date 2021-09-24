// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ImageDescriber_APRILTAG.hpp"
#include <aliceVision/gpu/gpu.hpp>
#include <aliceVision/system/Logger.hpp>

#include <apriltag/apriltag.h>
#include <apriltag/tag16h5.h>

namespace aliceVision {
namespace feature {

void ImageDescriber_APRILTAG::AprilTagParameters::setPreset(EImageDescriberPreset preset)
{
}

ImageDescriber_APRILTAG::ImageDescriber_APRILTAG()
  : ImageDescriber()
  {}

void ImageDescriber_APRILTAG::allocate(std::unique_ptr<Regions> &regions) const
{
  regions.reset( new APRILTAG_Regions );
}

void ImageDescriber_APRILTAG::setConfigurationPreset(ConfigurationPreset preset)
{
  _params.setPreset(preset.descPreset);
}

EImageDescriberType ImageDescriber_APRILTAG::getDescriberType() const
{
  return EImageDescriberType::APRILTAG16H5;
}

bool ImageDescriber_APRILTAG::describe(const image::Image<unsigned char>& image,
    std::unique_ptr<Regions> &regions,
    const image::Image<unsigned char> * mask)
{
  allocate(regions);
  ALICEVISION_LOG_DEBUG(" regions.get() = " << regions.get());
  APRILTAG_Regions * regionsCasted = dynamic_cast<APRILTAG_Regions*>(regions.get());
  ALICEVISION_LOG_DEBUG(" regionsCasted = " << regionsCasted);

  apriltag_family_t *tf = tag16h5_create();
  apriltag_detector_t *td = apriltag_detector_create();
  apriltag_detector_add_family(td, tf);
  td->quad_decimate = 2.0;
  td->quad_sigma = 0.0; // blur
  td->nthreads = 1;
  td->debug = 0;
  td->refine_edges = 1;

  // Make an image_u8_t header for the image
  image_u8_t im = { .width = image.Width(),
      .height = image.Height(),
      .stride = image.Width(),
      .buf = (uint8_t*) image.data()
  };
  zarray_t *detections = apriltag_detector_detect(td, &im);
  // Draw detection outlines
  for (int i = 0; i < zarray_size(detections); i++) {
    apriltag_detection_t *det;
    zarray_get(detections, i, &det);
    if (det->hamming == 0) {
      // compute center point as the intersection of the two diagonals
      // note: AprilTag corner points are listed counter-clockwise:
      Vec2 tl(det->p[0][0], det->p[0][1]);
      Vec2 bl(det->p[1][0], det->p[1][1]);
      Vec2 br(det->p[2][0], det->p[2][1]);
      Vec2 tr(det->p[3][0], det->p[3][1]);
      double denominator = ((tl[0]-br[0]) * (bl[1]-tr[1]) - (tl[1]-br[1]) * (bl[0]-tr[0]));
      Vec2 center(
        ((tl[0]*br[1] - tl[1]*br[0]) * (bl[0]-tr[0]) - (tl[0]-br[0]) * (bl[0]*tr[1] - bl[1]*tr[0])) / denominator,
        ((tl[0]*br[1] - tl[1]*br[0]) * (bl[1]-tr[1]) - (tl[1]-br[1]) * (bl[0]*tr[1] - bl[1]*tr[0])) / denominator
      );
      Vec2 points[5] = { center, tl, bl, br, tr };
      std::size_t indices[5] = { det->id, 30 + det->id, 60 + det->id, 90 + det->id, 120 + det->id};
      // compute scale from max side length and diagonals (divided by sqare root of 2):
      const double scale = 0.5 * std::max({(tl-bl).norm(), (bl-br).norm(), (br-tr).norm(), (tr-tl).norm(), 0.707*(tl-br).norm(), 0.707*(tr-bl).norm()});
      ALICEVISION_LOG_DEBUG(" New AprilTag: Id " << det->id << " ; Center location ( " << center[0] << " , " << center[1] << " ) ; Scale " << scale);
      // ignore orientation for now:
      const float orientation = 0.0f;
      for (size_t j = 0; j < 5; ++j) {
        // Add its associated descriptor
        APRILTAG_Regions::DescriptorT desc;
        for(size_t k = 0; k < desc.size(); ++k)
        {
          desc[k] = (unsigned char) 0;
        }
        desc[indices[j]] = (unsigned char) 255;
        regionsCasted->Descriptors().push_back(desc);
        regionsCasted->Features().push_back(PointFeature(points[j][0], points[j][1], scale, orientation));
      }
    }
  }
  apriltag_detections_destroy(detections);
  apriltag_detector_destroy(td);
  tag16h5_destroy(tf);

  return true;
}

} // namespace feature
} // namespace aliceVision

