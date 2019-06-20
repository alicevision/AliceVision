// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "fileIO.hpp"
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/mvsUtils/common.hpp>
#include <aliceVision/mvsUtils/MultiViewParams.hpp>
#include <aliceVision/mvsData/imageIO.hpp>
#include <aliceVision/mvsData/Image.hpp>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

namespace aliceVision {
namespace mvsUtils {

bool FileExists(const std::string& filePath)
{
    return boost::filesystem::exists(filePath);
}

bool FolderExists(const std::string& folderPath)
{
    return boost::filesystem::is_directory(folderPath);
}

std::string getFileNameFromViewId(const MultiViewParams* mp, int viewId, EFileType fileType, int scale)
{
  std::string folder = mp->_imagesFolder;
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
              folder = mp->getDepthMapsFilterFolder();
          else
              folder = mp->getDepthMapsFolder();
          suffix = "_depthMap";
          ext = "exr";
          break;
      }
      case EFileType::normalMap:
      {
          folder = mp->getDepthMapsFilterFolder();
          suffix = "_normalMap";
          ext = "exr";
          break;
      }
      case EFileType::simMap:
      {
          if(scale == 0)
              folder = mp->getDepthMapsFilterFolder();
          else
              folder = mp->getDepthMapsFolder();
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
          folder = mp->getDepthMapsFilterFolder();
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

  std::string fileName = folder + std::to_string(viewId) + suffix + "." + ext;
  return fileName;
}

std::string getFileNameFromIndex(const MultiViewParams* mp, int index, EFileType mv_file_type, int scale)
{
    return getFileNameFromViewId(mp, mp->getViewId(index), mv_file_type, scale);
}

FILE* mv_openFile(const MultiViewParams* mp, int index, EFileType mv_file_type, const char* readWrite)
{
    const std::string fileName = getFileNameFromIndex(mp, index, mv_file_type);
    FILE* out = fopen(fileName.c_str(), readWrite);
    if (out==NULL)
        throw std::runtime_error(std::string("Cannot create file: ") + fileName);
    return out;
}


Matrix3x4 load3x4MatrixFromFile(FILE* fi)
{
    Matrix3x4 m;

    // M[col*3 + row]
    float a, b, c, d;
    fscanf(fi, "%f %f %f %f \n", &a, &b, &c, &d);
    m.m11 = a;
    m.m12 = b;
    m.m13 = c;
    m.m14 = d;
    fscanf(fi, "%f %f %f %f \n", &a, &b, &c, &d);
    m.m21 = a;
    m.m22 = b;
    m.m23 = c;
    m.m24 = d;
    fscanf(fi, "%f %f %f %f \n", &a, &b, &c, &d);
    m.m31 = a;
    m.m32 = b;
    m.m33 = c;
    m.m34 = d;

    return m;
}

void loadImage(const std::string& path, const MultiViewParams* mp, int camId, Image& img, int bandType, imageIO::EImageColorSpace colorspace)
{
    imageIO::readImage(path, img, colorspace);

    // check image size
    if((mp->getOriginalWidth(camId) != img.width()) || (mp->getOriginalHeight(camId) != img.height()))
    {
        std::stringstream s;
        s << "Bad image dimension for camera : " << camId << "\n";
        s << "\t- image path : " << path << "\n";
        s << "\t- expected dimension : " << mp->getOriginalWidth(camId) << "x" << mp->getOriginalHeight(camId) << "\n";
        s << "\t- real dimension : " << img.width() << "x" << img.height() << "\n";
        throw std::runtime_error(s.str());
    }

    // scale choosed by the user and apply during the process
    const int processScale = mp->getProcessDownscale();
    const int width = mp->getWidth(camId);
    const int height = mp->getHeight(camId);

    if(processScale > 1)
    {
        ALICEVISION_LOG_DEBUG("Downscale (x" << processScale << ") image: " << mp->getViewId(camId) << ".");
        Image bmpr;
        imageIO::resizeImage(processScale, img, bmpr);
        img.swap(bmpr);
    }

    if(bandType == 1)
    {
        Image smooth;
        imageIO::convolveImage(img, smooth, "gaussian", 11.0f, 11.0f);
        img.swap(smooth);
    }

    if(bandType == 2)
    {
        Image bmps;
        imageIO::convolveImage(img, bmps, "gaussian", 11.0f, 11.0f);

        for(int y = 0; y < height; y++)
        {
            for(int x = 0; x < width; x++)
            {
                img.at(x, y) -= bmps.at(x, y);
            }
        }
    }
}

bool DeleteDirectory(const std::string& sPath)
{
    boost::filesystem::remove_all(sPath);
    return true;
}

} // namespace mvsUtils
} // namespace aliceVision
