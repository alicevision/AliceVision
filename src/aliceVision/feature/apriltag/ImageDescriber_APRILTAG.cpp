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
#include <opencv2/core/core.hpp>

namespace aliceVision {
namespace feature {

ImageDescriber_APRILTAG::AprilTagParameters::AprilTagParameters()
{
}

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

  const cv::Mat graySrc(cv::Size(image.Width(), image.Height()), CV_8UC1, (unsigned char *) image.data(), cv::Mat::AUTO_STEP);
  // Make an image_u8_t header for the Mat data
  image_u8_t im = { .width = graySrc.cols,
      .height = graySrc.rows,
      .stride = graySrc.cols,
      .buf = graySrc.data
  };
  zarray_t *detections = apriltag_detector_detect(td, &im);
  // Draw detection outlines
  for (int i = 0; i < zarray_size(detections); i++) {
    apriltag_detection_t *det;
    zarray_get(detections, i, &det);
    if (det->hamming == 0) {
      // compute center point as the intersection of the two diagonals
      // note: AprilTag corner points are listed counter-clockwise:
      cv::Point2d tl(det->p[0][0], det->p[0][1]);
      cv::Point2d bl(det->p[1][0], det->p[1][1]);
      cv::Point2d br(det->p[2][0], det->p[2][1]);
      cv::Point2d tr(det->p[3][0], det->p[3][1]);
      double denominator = ((tl.x-br.x) * (bl.y-tr.y) - (tl.y-br.y) * (bl.x-tr.x));
      cv::Point2d center(
        ((tl.x*br.y - tl.y*br.x) * (bl.x-tr.x) - (tl.x-br.x) * (bl.x*tr.y - bl.y*tr.x)) / denominator,
        ((tl.x*br.y - tl.y*br.x) * (bl.y-tr.y) - (tl.y-br.y) * (bl.x*tr.y - bl.y*tr.x)) / denominator
      );
      cv::Point2d points[5] = { center, tl, bl, br, tr };
      std::size_t indices[5] = { det->id, 30 + det->id, 60 + det->id, 90 + det->id, 120 + det->id};
      // compute scale from max side length and diagonals (divided by sqare root of 2):
      double scale = std::max({cv::norm(tl-bl), cv::norm(bl-br), cv::norm(br-tr), cv::norm(tr-tl), 0.707*cv::norm(tl-br), 0.707*cv::norm(tr-bl)});
      ALICEVISION_LOG_DEBUG(" New AprilTag: Id " << det->id << " ; Center location ( " << center.x << " , " << center.y << " ) ; Scale " << scale);
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
        regionsCasted->Features().push_back(PointFeature(points[j].x, points[j].y, scale, orientation));
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

