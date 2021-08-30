// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/config.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/image/Image.hpp>
#include <aliceVision/image/Sampler.hpp>
#include <aliceVision/camera/cameraCommon.hpp>
#include <aliceVision/camera/IntrinsicBase.hpp>
#include <aliceVision/camera/Pinhole.hpp>
#include <aliceVision/image/io.hpp>

#include <memory>

namespace aliceVision {
namespace camera {

/// Undistort an image according a given camera and its distortion model
template <typename T>
void UndistortImage(
  const image::Image<T>& imageIn,
  const camera::IntrinsicBase* intrinsicPtr,
  image::Image<T>& image_ud,
  T fillcolor,
  bool correctPrincipalPoint = false, 
  const oiio::ROI & roi = oiio::ROI())
{
  if (!intrinsicPtr->hasDistortion()) // no distortion, perform a direct copy
  {
    image_ud = imageIn;
  }
  else // There is distortion
  {
    const Vec2 center(imageIn.Width() * 0.5, imageIn.Height() * 0.5);
    /*Vec2 ppCorrection(0.0, 0.0);

    if(correctPrincipalPoint)
    {
      if(camera::isPinhole(intrinsicPtr->getType()))
      {
        const camera::Pinhole* pinholePtr = dynamic_cast<const camera::Pinhole*>(intrinsicPtr);
        ppCorrection = pinholePtr->getPrincipalPoint() - center;
      }
    }*/
    
    int widthRoi = imageIn.Width();
    int heightRoi = imageIn.Height();
    int xOffset = 0;
    int yOffset = 0;
    if(roi.defined())
    {
        widthRoi = roi.width();
        heightRoi = roi.height();
        xOffset = roi.xbegin;
        yOffset = roi.ybegin;
    }

    image_ud.resize(widthRoi, heightRoi, true, fillcolor);
    const image::Sampler2d<image::SamplerLinear> sampler;
    
    std::shared_ptr<camera::IntrinsicBase> corrected(intrinsicPtr->clone());

    std::shared_ptr<camera::IntrinsicsScaleOffsetDisto> camdist = std::dynamic_pointer_cast<camera::IntrinsicsScaleOffsetDisto>(corrected);
    if (camdist)
    {
      std::vector<double> tmp = camdist->getDistortionParams();
      for (double & v : tmp) v = 0;
      camdist->setDistortionParams(tmp);
    }


    if (correctPrincipalPoint)
    {
      std::shared_ptr<camera::IntrinsicsScaleOffset> camso = std::dynamic_pointer_cast<camera::IntrinsicsScaleOffset>(corrected);
      if (camso)
      {
        camso->setOffset(0.0, 0.0);
      }
    }
    

    #pragma omp parallel for
    for(int j = 0; j < heightRoi; ++j)
        for(int i = 0; i < widthRoi; ++i)
        {       
            const Vec2 refPix(i + xOffset, j + yOffset); 

            // compute coordinates with distortion
            const Vec2 disto_pix = intrinsicPtr->cam2ima(intrinsicPtr->addDistortion(corrected->ima2cam(refPix)));
           
            // pick pixel if it is in the image domain
            if(imageIn.Contains(disto_pix(1), disto_pix(0)))
                image_ud(j, i) = sampler(imageIn, disto_pix(1), disto_pix(0));
        }
  }
}

} // namespace camera
} // namespace aliceVision


