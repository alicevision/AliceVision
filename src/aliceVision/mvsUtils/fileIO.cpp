// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "fileIO.hpp"
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/image/LegacyImage.hpp>
#include <aliceVision/image/imageAlgo.hpp>
#include <aliceVision/image/io.hpp>
#include <aliceVision/mvsUtils/common.hpp>
#include <aliceVision/mvsUtils/MultiViewParams.hpp>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

namespace aliceVision {
namespace mvsUtils {

std::string getFileNameFromViewId(const MultiViewParams& mp, int viewId, EFileType fileType, int scale, const std::string& customSuffix)
{
  std::string folder = mp._imagesFolder;
  std::string suffix;
  std::string ext;

  switch(fileType)
  {
      case EFileType::P:
      {
          suffix = "_P";
          ext = "txt";
          break;
      }
      case EFileType::K:
      {
          suffix = "_K";
          ext = "txt";
          break;
      }
      case EFileType::iK:
      {
          suffix = "_iK";
          ext = "txt";
          break;
      }
      case EFileType::R:
      {
          suffix = "_R";
          ext = "txt";
          break;
      }
      case EFileType::iR:
      {
          suffix = "_iR";
          ext = "txt";
          break;
      }
      case EFileType::C:
      {
          suffix = "_C";
          ext = "txt";
          break;
      }
      case EFileType::iP:
      {
          suffix = "_iP";
          ext = "txt";
          break;
      }
      case EFileType::har:
      {
          suffix = "_har";
          ext = "bin";
          break;
      }
      case EFileType::prematched:
      {
          suffix = "_prematched";
          ext = "bin";
          break;
      }
      case EFileType::growed:
      {
          suffix = "_growed";
          ext = "bin";
          break;
      }
      case EFileType::occMap:
      {
          suffix = "_occMap";
          ext = "bin";
          break;
      }
      case EFileType::nearMap:
      {
          suffix = "_nearMap";
          ext = "bin";
          break;
      }
      case EFileType::op:
      {
          suffix = "_op";
          ext = "bin";
          break;
      }
      case EFileType::wshed:
      {
          suffix = "_wshed";
          ext = "bin";
          break;
      }
      case EFileType::img:
      {
          suffix = "_img";
          ext = "bin";
          break;
      }
      case EFileType::imgT:
      {
          suffix = "_imgT";
          ext = "bin";
          break;
      }
      case EFileType::graphCutMap:
      {
          suffix = "_graphCutMap";
          ext = "bin";
          break;
      }
      case EFileType::graphCutPts:
      {
          suffix = "_graphCutPts";
          ext = "bin";
          break;
      }
      case EFileType::growedMap:
      {
          suffix = "_growedMap";
          ext = "bin";
          break;
      }
      case EFileType::agreedMap:
      {
          suffix = "_agreedMap";
          ext = "bin";
          break;
      }
      case EFileType::agreedPts:
      {
          suffix = "_agreedPts";
          ext = "bin";
          break;
      }
      case EFileType::refinedMap:
      {
          suffix = "_refinedMap";
          ext = "bin";
          break;
      }
      case EFileType::seeds_sfm:
      {
          suffix = "_seeds_sfm";
          ext = "bin";
          break;
      }
      case EFileType::radial_disortion:
      {
          suffix = "_rd";
          ext = "bin";
          break;
      }
      case EFileType::graphCutMesh:
      {
          suffix = "_graphCutMesh";
          ext = "bin";
          break;
      }
      case EFileType::agreedMesh:
      {
          suffix = "_agreedMesh";
          ext = "bin";
          break;
      }
      case EFileType::nearestAgreedMap:
      {
          suffix = "_nearestAgreedMap";
          ext = "bin";
          break;
      }
      case EFileType::segPlanes:
      {
          suffix = "_segPlanes";
          ext = "bin";
          break;
      }
      case EFileType::agreedVisMap:
      {
          suffix = "_agreedVisMap";
          ext = "bin";
          break;
      }
      case EFileType::diskSizeMap:
      {
          suffix = "_diskSizeMap";
          ext = "bin";
          break;
      }
      case EFileType::depthMap:
      {
          if(scale == 0)
              folder = mp.getDepthMapsFilterFolder();
          else
              folder = mp.getDepthMapsFolder();
          suffix = "_depthMap";
          ext = "exr";
          break;
      }
      case EFileType::normalMap:
      {
          folder = mp.getDepthMapsFilterFolder();
          suffix = "_normalMap";
          ext = "exr";
          break;
      }
      case EFileType::simMap:
      {
          if(scale == 0)
              folder = mp.getDepthMapsFilterFolder();
          else
              folder = mp.getDepthMapsFolder();
          suffix = "_simMap";
          ext = "exr";
          break;
      }
      case EFileType::mapPtsTmp:
      {
          suffix = "_mapPts";
          ext = "tmp";
          break;
      }
      case EFileType::mapPtsSimsTmp:
      {
          suffix = "_mapPtsSims";
          ext = "tmp";
          break;
      }
      case EFileType::camMap:
      {
          suffix = "_camMap";
          ext = "bin";
          break;
      }
      case EFileType::nmodMap:
      {
          folder = mp.getDepthMapsFilterFolder();
          suffix = "_nmodMap";
          ext = "png";
          break;
      }
      case EFileType::D:
      {
          suffix = "_D";
          ext = "txt";
          break;
      }
  }
  if(scale > 1)
  {
      suffix += "_scale" + num2str(scale);
  }

  std::string fileName = folder + std::to_string(viewId) + suffix + customSuffix + "." + ext;
  return fileName;
}

std::string getFileNameFromIndex(const MultiViewParams& mp, int index, EFileType mv_file_type, int scale, const std::string& customSuffix)
{
    return getFileNameFromViewId(mp, mp.getViewId(index), mv_file_type, scale, customSuffix);
}

FILE* mv_openFile(const MultiViewParams& mp, int index, EFileType mv_file_type, const char* readWrite)
{
    const std::string fileName = getFileNameFromIndex(mp, index, mv_file_type, 0, "");
    FILE* out = fopen(fileName.c_str(), readWrite);
    if (out==NULL)
        throw std::runtime_error(std::string("Cannot create file: ") + fileName);
    return out;
}


Matrix3x4 load3x4MatrixFromFile(std::istream& in)
{
    Matrix3x4 m;

    // M[col*3 + row]
    float a, b, c, d;
    in >> a >> b >> c >> d;
    m.m11 = a;
    m.m12 = b;
    m.m13 = c;
    m.m14 = d;
    in >> a >> b >> c >> d;
    m.m21 = a;
    m.m22 = b;
    m.m23 = c;
    m.m24 = d;
    in >> a >> b >> c >> d;
    m.m31 = a;
    m.m32 = b;
    m.m33 = c;
    m.m34 = d;

    return m;
}

template<class T>
void compensateExposure(LegacyImage<T>& image, float exposureCompensation)
{
    for(int pix = 0; pix < image.size(); ++pix)
        image[pix] = image[pix] * exposureCompensation;
}

template<class T>
void compensateExposure(image::Image<T>& image, float exposureCompensation)
{
    auto* data = image.data();
    for (std::size_t i = 0; i < image.Width() * image.Height(); ++i, ++data)
        *data = *data * exposureCompensation;
}

template<class Image>
void loadImage(const std::string& path, const MultiViewParams& mp, int camId, Image& img,
               image::EImageColorSpace colorspace, ECorrectEV correctEV)
{
    // check image size
    auto checkImageSize = [&path, &mp, camId, &img](){
        if((mp.getOriginalWidth(camId) != img.Width()) || (mp.getOriginalHeight(camId) != img.Height()))
        {
            std::stringstream s;
            s << "Bad image dimension for camera : " << camId << "\n";
            s << "\t- image path : " << path << "\n";
            s << "\t- expected dimension : " << mp.getOriginalWidth(camId) << "x" << mp.getOriginalHeight(camId) << "\n";
            s << "\t- real dimension : " << img.Width() << "x" << img.Height() << "\n";
            throw std::runtime_error(s.str());
        }
    };

    if(correctEV == ECorrectEV::NO_CORRECTION)
    {
        image::readImage(path, img, colorspace);
        checkImageSize();
    }
    // if exposure correction, apply it in linear colorspace and then convert colorspace
    else
    {
        image::readImage(path, img, image::EImageColorSpace::LINEAR);
        checkImageSize();

        const auto metadata = image::readImageMetadata(path);

        float exposureCompensation = metadata.get_float("AliceVision:EVComp", -1);

        if(exposureCompensation == -1)
        {
            exposureCompensation = 1.0f;
            ALICEVISION_LOG_INFO("Cannot compensate exposure. PrepareDenseScene needs to be update");
        }
        else
        {
            ALICEVISION_LOG_INFO("  exposure compensation for image " << camId + 1 << ": " << exposureCompensation);

            compensateExposure(img, exposureCompensation);

            imageAlgo::colorconvert(img, image::EImageColorSpace::LINEAR, colorspace);
        }
    }

    // scale choosed by the user and apply during the process
    const int processScale = mp.getProcessDownscale();

    if(processScale > 1)
    {
        ALICEVISION_LOG_DEBUG("Downscale (x" << processScale << ") image: " << mp.getViewId(camId) << ".");
        Image bmpr;
        imageAlgo::resizeImage(processScale, img, bmpr);
        img.swap(bmpr);
    }
}

template void loadImage<ImageRGBAf>(const std::string& path, const MultiViewParams& mp, int camId,
                                    ImageRGBAf& img, image::EImageColorSpace colorspace, ECorrectEV correctEV);
template void loadImage<image::Image<image::RGBfColor>>(const std::string& path, const MultiViewParams& mp, int camId,
                                                        image::Image<image::RGBfColor>& img,
                                                        image::EImageColorSpace colorspace, ECorrectEV correctEV);
template void loadImage<image::Image<image::RGBAfColor>>(const std::string& path, const MultiViewParams& mp, int camId,
                                                         image::Image<image::RGBAfColor>& img,
                                                         image::EImageColorSpace colorspace, ECorrectEV correctEV);

} // namespace mvsUtils
} // namespace aliceVision
