// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "KeyframeSelector.hpp"
#include <aliceVision/image/all.hpp>
#include <aliceVision/sensorDB/parseDatabase.hpp>
#include <aliceVision/feature/sift/ImageDescriber_SIFT.hpp>
#include <aliceVision/system/Logger.hpp>

#include <tuple>
#include <cassert>

namespace aliceVision {
namespace keyframe {

KeyframeSelector::KeyframeSelector(const std::vector<std::string>& mediaPaths,
                                   const std::string& sensorDbPath,
                                   const std::string& voctreeFilePath,
                                   const std::string& outputFolder)
  : _mediaPaths(mediaPaths)
  , _sensorDbPath(sensorDbPath)
  , _voctreeFilePath(voctreeFilePath)
  , _outputFolder(outputFolder)
{
  // load vocabulary tree
  _voctree.reset(new aliceVision::voctree::VocabularyTree<DescriptorFloat>(voctreeFilePath));

  {
      ALICEVISION_LOG_INFO("vocabulary tree loaded with :" << std::endl
                       << " - " << _voctree->levels() << " levels" << std::endl
                       << " - " << _voctree->splits() << " branching factor" << std::endl);
  }

  // check number of input media filePaths
  if(mediaPaths.empty())
  {
    ALICEVISION_LOG_ERROR("ERROR : can't create KeyframeSelector without a media file path !");
    throw std::invalid_argument("ERROR : can't create KeyframeSelector without a media file path !");
  }

  // resize mediasInfo container
  _mediasInfo.resize(mediaPaths.size());

  // create feeds and count minimum number of frames
  std::size_t nbFrames = std::numeric_limits<std::size_t>::max();
  for(const auto& path : _mediaPaths)
  {
    // create a feed provider per mediaPaths
    _feeds.emplace_back(new dataio::FeedProvider(path));

    const auto& feed = *_feeds.back();

    // check if feed is initialized
    if(!feed.isInit())
    {
      ALICEVISION_LOG_ERROR("ERROR : while initializing the FeedProvider with " << path);
      throw std::invalid_argument("ERROR : while initializing the FeedProvider with " + path);
    }

    // update minimum number of frames
    nbFrames = std::min(nbFrames, feed.nbFrames());
  }
  
  // check if minimum number of frame is zero
  if(nbFrames == 0)
  {
    ALICEVISION_LOG_ERROR("ERROR : one or multiple medias can't be found or empty !");
    throw std::invalid_argument("ERROR : one or multiple medias can't be found or empty !");
  }

  // resize selection data vector
  _framesData.resize(nbFrames);

  // create SIFT image describer
  _imageDescriber.reset(new feature::ImageDescriber_SIFT());
}

void KeyframeSelector::process()
{
  // feed provider variables
  image::Image< image::RGBColor> image;                 // original image
  camera::PinholeRadialK3 queryIntrinsics; // image associated camera intrinsics
  bool hasIntrinsics = false;                           // true if queryIntrinsics is valid
  std::string currentImgName;                           // current image name
  
  // process variables
  const unsigned int frameStep = _maxFrameStep - _minFrameStep;
  const unsigned int tileSharpSubset =  (_nbTileSide * _nbTileSide) / _sharpSubset;
  
  for(std::size_t mediaIndex = 0 ; mediaIndex < _feeds.size(); ++mediaIndex)
  {
    // first frame
    if(!_feeds.at(mediaIndex)->readImage(image, queryIntrinsics, currentImgName, hasIntrinsics))
    {
      ALICEVISION_LOG_ERROR("ERROR : can't read media first frame " << _mediaPaths[mediaIndex]);
      throw std::invalid_argument("ERROR : can't read media first frame " + _mediaPaths[mediaIndex]);
    }

    // define output image metadata
    if(!_cameraInfos.at(mediaIndex).focalIsMM)
    {
      convertFocalLengthInMM(_cameraInfos.at(mediaIndex), image.Width());
    }

    // define media informations variables
    auto& mediaInfo =  _mediasInfo.at(mediaIndex);
    mediaInfo.tileHeight = (image.Height() / 2) / _nbTileSide;
    mediaInfo.tileWidth = (image.Width() / 2) / _nbTileSide;
    mediaInfo.spec = oiio::ImageSpec(image.Width(), image.Height(), 3, oiio::TypeDesc::UINT8); // always jpeg
    mediaInfo.spec.attribute("CompressionQuality", 100);   // always best compression quality
    mediaInfo.spec.attribute("jpeg:subsampling", "4:4:4"); // always subsampling 4:4:4
    mediaInfo.spec.attribute("oiio:ColorSpace", "sRGB");   // always sRGB
    mediaInfo.spec.attribute("Make",  _cameraInfos[mediaIndex].brand);
    mediaInfo.spec.attribute("Model", _cameraInfos[mediaIndex].model);
    mediaInfo.spec.attribute("Exif:FocalLength", _cameraInfos[mediaIndex].focalLength);
  }

  // iteration process
  _keyframeIndexes.clear();
  std::size_t currentFrameStep = _minFrameStep; // start directly (dont skip minFrameStep first frames)
  
  for(std::size_t frameIndex = 0; frameIndex < _framesData.size(); ++frameIndex)
  {
    ALICEVISION_LOG_TRACE("frame : " << frameIndex);
    bool frameSelected = true;
    auto& frameData = _framesData.at(frameIndex);
    frameData.mediasData.resize(_feeds.size());

    for(std::size_t mediaIndex = 0; mediaIndex < _feeds.size(); ++mediaIndex)
    {
      ALICEVISION_LOG_TRACE("media : " << _mediaPaths.at(mediaIndex));
      auto& feed = *_feeds.at(mediaIndex);

      if(frameSelected) // false if a camera of a rig is not selected
      {
        if(!feed.readImage(image, queryIntrinsics, currentImgName, hasIntrinsics))
        {
          ALICEVISION_LOG_ERROR("ERROR  : can't read frame '" << currentImgName << "' !");
          throw std::invalid_argument("ERROR : can't read frame '" + currentImgName + "' !");
        }

        // compute sharpness and sparse distance
        if(!computeFrameData(image, frameIndex, mediaIndex, tileSharpSubset))
        {
          frameSelected = false;
        }
      }

      feed.goToNextFrame();
    }

    {
      if(frameSelected)
      {
        ALICEVISION_LOG_TRACE(" > selected" << std::endl);
        frameData.selected = true;
        frameData.computeAvgSharpness();
      }
      else
      {
        ALICEVISION_LOG_TRACE(" > skipped" << std::endl);
        frameData.mediasData.clear(); // remove unselected mediasData
      }
    }

    // selection process
    if(currentFrameStep >= _maxFrameStep)
    {
      currentFrameStep = _minFrameStep;
      bool hasKeyframe = false;
      std::size_t keyframeIndex = 0;
      float maxSharpness = 0;

      // find the sharpest selected frame
      for(std::size_t index = frameIndex - (frameStep - 1); index <= frameIndex; ++index)
      {
        if(_framesData[index].selected && (_framesData[index].avgSharpness > maxSharpness))
        {
          hasKeyframe = true;
          keyframeIndex = index;
          maxSharpness = _framesData[index].avgSharpness;
        }
      }

      // save keyframe
      if(hasKeyframe)
      {
        ALICEVISION_LOG_INFO("keyframe choice : " << keyframeIndex << std::endl);

        // write keyframe
        for(std::size_t mediaIndex = 0; mediaIndex < _feeds.size(); ++mediaIndex)
        {
          auto& feed = *_feeds.at(mediaIndex);

          feed.goToFrame(keyframeIndex);

          if(_maxOutFrame == 0) // no limit of keyframes (direct evaluation)
          {
            feed.readImage(image, queryIntrinsics, currentImgName, hasIntrinsics);
            writeKeyframe(image, keyframeIndex, mediaIndex);
          }
        }
        _framesData[keyframeIndex].keyframe = true;
        _keyframeIndexes.push_back(keyframeIndex);

        frameIndex = keyframeIndex + _minFrameStep - 1;
      }
      else
      {
        ALICEVISION_LOG_INFO("keyframe choice : none" << std::endl);
      }
    }
    ++currentFrameStep;
  }

  if(_maxOutFrame == 0) // no limit of keyframes (evaluation and write already done)
  {
    return;
  }

  // if limited number of keyframe select smallest sparse distance
  {
    std::vector< std::tuple<float, float, std::size_t> > keyframes;

    for(std::size_t i = 0; i < _framesData.size(); ++i)
    {
      if(_framesData[i].keyframe)
      {
        keyframes.emplace_back(_framesData[i].maxDistScore, 1 / _framesData[i].avgSharpness, i);
      }
    }
    std::sort(keyframes.begin(), keyframes.end());

    const std::size_t nbOutFrames = std::min(static_cast<std::size_t>(_maxOutFrame), keyframes.size());

    for(std::size_t i = 0; i < nbOutFrames; ++i)
    {
      const std::size_t frameIndex = std::get<2>(keyframes.at(i));
      for(std::size_t mediaIndex = 0; mediaIndex < _feeds.size(); ++mediaIndex)
      {
        auto& feed = *_feeds.at(mediaIndex);
        feed.goToFrame(frameIndex);
        feed.readImage(image, queryIntrinsics, currentImgName, hasIntrinsics);
        writeKeyframe(image, frameIndex, mediaIndex);
      }
    }
  }
}

float KeyframeSelector::computeSharpness(const image::Image<unsigned char>& imageGray,
                                         const unsigned int tileHeight,
                                         const unsigned int tileWidth,
                                         const unsigned int tileSharpSubset) const
{
  image::Image<float> image;
  image::Image<float> scharrXDer;
  image::Image<float> scharrYDer;

  image::ConvertPixelType(imageGray, &image);
  image::ImageScharrXDerivative(image, scharrXDer); // normalized
  image::ImageScharrYDerivative(image, scharrYDer); // normalized

  scharrXDer = scharrXDer.cwiseAbs(); // absolute value
  scharrYDer = scharrYDer.cwiseAbs(); // absolute value

  // image tiles
  std::vector<float> averageTileIntensity;
  const float tileSizeInv = 1 / static_cast<float>(tileHeight * tileWidth);

  for(std::size_t y =  0; y < (_nbTileSide * tileHeight); y += tileHeight)
  {
    for(std::size_t x =  0; x < (_nbTileSide * tileWidth); x += tileWidth)
    {
      const auto sum = scharrXDer.block(y, x, tileHeight, tileWidth).sum() + scharrYDer.block(y, x, tileHeight, tileWidth).sum();
      averageTileIntensity.push_back(sum * tileSizeInv);
    }
  }

  // sort tiles average pixel intensity
  std::sort(averageTileIntensity.begin(), averageTileIntensity.end());

  // return the sum of the subset average pixel intensity
  return std::accumulate(averageTileIntensity.end() - tileSharpSubset, averageTileIntensity.end(), 0.0f) / tileSharpSubset;
}


bool KeyframeSelector::computeFrameData(const image::Image<image::RGBColor>& image,
                                        std::size_t frameIndex,
                                        std::size_t mediaIndex,
                                        unsigned int tileSharpSubset)
{
  image::Image<unsigned char> imageGray;                // grayscale image
  image::Image<unsigned char> imageGrayHalfSample;      // half resolution grayscale image
  
  const auto& currMediaInfo = _mediasInfo.at(mediaIndex);
  auto& currframeData = _framesData.at(frameIndex);
  auto& currMediaData = currframeData.mediasData.at(mediaIndex);

  // get grayscale image and resize
  image::ConvertPixelType(image, &imageGray);
  image::ImageHalfSample(imageGray, imageGrayHalfSample);

  // compute sharpness
  currMediaData.sharpness = computeSharpness(imageGrayHalfSample,
                                             currMediaInfo.tileHeight,
                                             currMediaInfo.tileWidth,
                                             tileSharpSubset);

  ALICEVISION_LOG_TRACE( " - sharpness : " << currMediaData.sharpness);

  if(currMediaData.sharpness > _sharpnessThreshold)
  {
    bool noKeyframe = (_keyframeIndexes.empty());

    // compute current frame sparse histogram
    std::unique_ptr<feature::Regions> regions;
    _imageDescriber->describe(imageGrayHalfSample, regions);
    currMediaData.histogram = voctree::SparseHistogram(_voctree->quantizeToSparse(dynamic_cast<feature::SIFT_Regions*>(regions.get())->Descriptors()));

    // compute sparseDistance
    if(!noKeyframe)
    {
      unsigned int nbKeyframetoCompare = (_keyframeIndexes.size() < _nbKeyFrameDist)? _keyframeIndexes.size() : _nbKeyFrameDist;

      for(std::size_t i = _keyframeIndexes.size() - nbKeyframetoCompare; i < _keyframeIndexes.size(); ++i)
      {
        for(auto& media : _framesData.at(_keyframeIndexes.at(i)).mediasData)
        {
          currMediaData.distScore = std::max(currMediaData.distScore, std::abs(voctree::sparseDistance(media.histogram, currMediaData.histogram, "strongCommonPoints")));
        }
      }
      currframeData.maxDistScore = std::max(currframeData.maxDistScore, currMediaData.distScore);
      ALICEVISION_LOG_TRACE(" - distScore : " << currMediaData.distScore);
    }

    if(noKeyframe || (currMediaData.distScore < _distScoreMax))
    {
      return true;
    }
  } 
  return false;
}

void KeyframeSelector::writeKeyframe(const image::Image<image::RGBColor>& image, 
                                     std::size_t frameIndex,
                                     std::size_t mediaIndex)
{
  const auto& mediaInfo = _mediasInfo.at(mediaIndex);
  const auto filepath = _outputFolder + "/frame_" + std::to_string(frameIndex) + "_media_" + std::to_string(mediaIndex) + ".jpg";

  std::unique_ptr<oiio::ImageOutput> out(oiio::ImageOutput::create(filepath));
  
  if(out.get() == nullptr)
  {
    throw std::invalid_argument("ERROR : can't create image file : " + filepath);
  }
  
  if(!out->open(filepath, mediaInfo.spec))
  {
    throw std::invalid_argument("ERROR : can't open image file : " + filepath);
  }

  out->write_image(oiio::TypeDesc::UINT8, image.data()); // always jpeg
  out->close();
}

void KeyframeSelector::convertFocalLengthInMM(CameraInfo& cameraInfo, int imageWidth)
{
  assert(imageWidth > 0);
  
  sensorDB::Datasheet find;
  std::vector<sensorDB::Datasheet> vecDatabase;
  sensorDB::parseDatabase(_sensorDbPath, vecDatabase);

  if(sensorDB::getInfo(cameraInfo.brand, cameraInfo.model, vecDatabase, find))
  {
    cameraInfo.focalLength = (cameraInfo.focalLength * find._sensorSize) / imageWidth;
    cameraInfo.focalIsMM = true;
    ALICEVISION_LOG_INFO("Focal length converted in mm : " << cameraInfo.focalLength);
  }
  else
  {
    ALICEVISION_LOG_WARNING("Warning: can't convert focal length in mm  : " << cameraInfo.brand << " / " << cameraInfo.model);
  }
}

} // namespace keyframe 
} // namespace aliceVision
