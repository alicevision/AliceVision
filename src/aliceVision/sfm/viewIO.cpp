// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "viewIO.hpp"

#include <aliceVision/camera/camera.hpp>
#include <aliceVision/image/image.hpp>
#include <aliceVision/exif/EasyExifIO.hpp>

#include <stdexcept>

namespace aliceVision {
namespace sfm {

void updateIncompleteView(View& view)
{
  // check if the image format is supported
  if(aliceVision::image::GetFormat(view.getImagePath().c_str()) == aliceVision::image::Unknown)
  {
    ALICEVISION_LOG_ERROR("Error: Unknown image file format '" << stlplus::filename_part(view.getImagePath()) << "'." << std::endl);
    throw std::invalid_argument("Error: Unknown image file format '" + stlplus::filename_part(view.getImagePath()) + "'.");
  }

  // read image header
  image::ImageHeader imgHeader;
  if(!aliceVision::image::ReadImageHeader(view.getImagePath().c_str(), &imgHeader))
  {
    ALICEVISION_LOG_ERROR("Error: Can't read image header '" << stlplus::filename_part(view.getImagePath()) << "'." << std::endl);
    throw std::invalid_argument("Error: Can't read image header '" + stlplus::filename_part(view.getImagePath()) + "'.");
  }

  // reset width and height
  view.setWidth(imgHeader.width);
  view.setHeight(imgHeader.height);

  //check dimensions
  if(view.getWidth() <= 0 || view.getHeight() <= 0)
  {
    ALICEVISION_LOG_ERROR("Error: Image size is invalid '" << stlplus::filename_part(view.getImagePath())  << "'." << std::endl
                        << "\t- width: " << view.getWidth() << std::endl
                        << "\t- height: " << view.getHeight() << std::endl);
    throw std::invalid_argument("Error: Image size is invalid '" + stlplus::filename_part(view.getImagePath()) + "'.");
   }

  // read exif metadata
  exif::EasyExifIO exifReader;
  exifReader.open(view.getImagePath());

  // reset viewId
  view.setViewId(computeUID(exifReader, stlplus::filename_part(view.getImagePath())));

  // reset metadata
  if(view.getMetadata().empty())
    view.setMetadata(exifReader.getExifData());

  if(view.getPoseId() == UndefinedIndexT)
  {
    // check if the rig poseId id is defined
    if(view.isPartOfRig())
    {
      ALICEVISION_LOG_ERROR("Error: Can't find poseId for'" << stlplus::filename_part(view.getImagePath()) << "' marked as part of a rig." << std::endl);
      throw std::invalid_argument("Error: Can't find poseId for'" + stlplus::filename_part(view.getImagePath()) + "' marked as part of a rig.");
    }
    else
      view.setPoseId(view.getViewId());
  }
  else if((!view.isPartOfRig()) && (view.getPoseId() != view.getViewId()))
  {
    ALICEVISION_LOG_ERROR("Error: Bad poseId for image '" << stlplus::filename_part(view.getImagePath()) << "' (viewId should be equal to poseId)." << std::endl);
    throw std::invalid_argument("Error: Bad poseId for image '" + stlplus::filename_part(view.getImagePath()) + "'.");
  }
}

std::shared_ptr<camera::IntrinsicBase> getViewIntrinsic(const View& view,
                                                        double sensorWidth,
                                                        double defaultFocalLengthPx,
                                                        camera::EINTRINSIC defaultIntrinsicType,
                                                        double defaultPPx,
                                                        double defaultPPy)
{
  // get view informations
  const std::string cameraBrand = view.hasMetadata("camera_make") ? view.getMetadata("camera_make") : "";
  const std::string cameraModel = view.hasMetadata("camera_model") ? view.getMetadata("camera_model") : "";
  const std::string bodySerialNumber = view.hasMetadata("serial_number") ? view.getMetadata("serial_number") : "";
  const std::string lensSerialNumber = view.hasMetadata("lens_serial_number") ? view.getMetadata("lens_serial_number") : "";

  float mmFocalLength = view.hasMetadata("lens_focal_length") ? std::stof(view.getMetadata("lens_focal_length")) : -1;
  double pxFocalLength = defaultFocalLengthPx;
  camera::EINTRINSIC intrinsicType = defaultIntrinsicType;

  double ppx = view.getWidth() / 2.0;
  double ppy = view.getHeight() / 2.0;

  bool isResized = false;

  if(view.hasMetadata("image_width") && view.hasMetadata("image_height")) // has metadata
  {
    // check if the image is resized
    int exifWidth = std::stoi(view.getMetadata("image_width"));
    int exifHeight = std::stoi(view.getMetadata("image_height"));

    // if metadata is rotated
    if(exifWidth == view.getHeight() && exifHeight == view.getWidth())
      std::swap(exifWidth, exifHeight);


    if(exifWidth > 0 && exifHeight > 0 &&
       (exifWidth != view.getWidth() || exifHeight != view.getHeight()))
    {
      ALICEVISION_LOG_WARNING("Warning: Resized image detected: " << stlplus::filename_part(view.getImagePath()) << std::endl
                          << "\t- real image size: " <<  view.getWidth() << "x" <<  view.getHeight() << std::endl
                          << "\t- image size from exif metadata is: " << exifWidth << "x" << exifHeight << std::endl);
      isResized = true;
    }
  }
  else if(defaultPPx > 0 && defaultPPy > 0) // use default principal point
  {
    ppx = defaultPPx;
    ppy = defaultPPy;
  }

  // handle case where focal length (mm) is unset or false
  if(mmFocalLength <= .0f)
  {
    ALICEVISION_LOG_WARNING("Warning: image '" << stlplus::filename_part(view.getImagePath()) << "' focal length (in mm) metadata is missing." << std::endl
                                               << "Can't compute focal length (px), use default." << std::endl);
  }
  else if(sensorWidth > 0)
  {
    // Retrieve the focal from the metadata in mm and convert to pixel.
    pxFocalLength = std::max(view.getWidth(), view.getHeight()) * mmFocalLength / sensorWidth;
  }

  // choose intrinsic type
  if(cameraBrand == "Custom")
  {
    intrinsicType = camera::EINTRINSIC_stringToEnum(cameraModel);
  }
  else if(isResized)
  {
    // if the image has been resized, we assume that it has been undistorted
    // and we use a camera without lens distortion.
    intrinsicType = camera::PINHOLE_CAMERA;
  }
  else if(mmFocalLength > 0.0 && mmFocalLength < 15)
  {
    // if the focal lens is short, the fisheye model should fit better.
    intrinsicType = camera::PINHOLE_CAMERA_FISHEYE;
  }
  else if(intrinsicType == camera::PINHOLE_CAMERA_START)
  {
    // choose a default camera model if no default type
    // use standard lens with radial distortion by default
    intrinsicType = camera::PINHOLE_CAMERA_RADIAL3;
  }

  // create the desired intrinsic
  std::shared_ptr<camera::IntrinsicBase> intrinsic = camera::createPinholeIntrinsic(intrinsicType, view.getWidth(), view.getHeight(), pxFocalLength, ppx, ppy);
  intrinsic->setInitialFocalLengthPix(pxFocalLength);

  // initialize distortion parameters
  switch(intrinsicType)
  {
    case camera::PINHOLE_CAMERA_FISHEYE:
    {
      if(cameraBrand == "GoPro")
        intrinsic->updateFromParams({pxFocalLength, ppx, ppy, 0.0524, 0.0094, -0.0037, -0.0004});
      break;
    }
    case camera::PINHOLE_CAMERA_FISHEYE1:
    {
      if(cameraBrand == "GoPro")
        intrinsic->updateFromParams({pxFocalLength, ppx, ppy, 1.04});
      break;
    }
    default: break;
  }

  // create serial number
  intrinsic->setSerialNumber(bodySerialNumber + lensSerialNumber);

  return intrinsic;
}

} // namespace sfm
} // namespace aliceVision
