#include "KeyframeSelector.hpp"
#include "openMVG/image/image.hpp"
#include "openMVG/features/sift/SIFT_describer.hpp"
#include "openMVG/exif/sensor_width_database/ParseDatabase.hpp"
#include "openMVG/logger.hpp"

#include <tuple>
#include <cassert>

namespace openMVG {
namespace keyframe {

KeyframeSelector::KeyframeSelector(const std::vector<std::string>& mediaPaths,
                                   const std::string& sensorDbPath,
                                   const std::string& voctreeFilePath,
                                   const std::string& outputDirectory)
  : _mediaPaths(mediaPaths)
  , _sensorDbPath(sensorDbPath)
  , _voctreeFilePath(voctreeFilePath)
  , _outputDirectory(outputDirectory)
{
  // load vocabulary tree
  _voctree.reset(new openMVG::voctree::VocabularyTree<DescriptorFloat>(voctreeFilePath));

  {
      OPENMVG_COUT("vocabulary tree loaded with :");
      OPENMVG_COUT(" - " << _voctree->levels() << " levels");
      OPENMVG_COUT(" - " << _voctree->splits() << " branching factor");
  }

  // check number of input media filePaths
  if(mediaPaths.empty())
  {
    OPENMVG_CERR("ERROR : can't create KeyframeSelector without a media file path !");
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
      OPENMVG_CERR("ERROR : while initializing the FeedProvider with " << path);
      throw std::invalid_argument("ERROR : while initializing the FeedProvider with " + path);
    }

    // update minimum number of frames
    nbFrames = std::min(nbFrames, feed.nbFrames());
  }
  
  // check if minimum number of frame is zero
  if(nbFrames == 0)
  {
    OPENMVG_CERR("ERROR : one or multiple medias are empty (no frames) !");
    throw std::invalid_argument("ERROR : one or multiple medias are empty (no frames) !");
  }

  // resize selection data vector
  _framesData.resize(nbFrames);

  // create SIFT image describer
  _imageDescriber.reset(new features::SIFT_ImageDescriber());
}

void KeyframeSelector::process()
{
  // feed provider variables
  image::Image< image::RGBColor> image;                 // original image
  cameras::Pinhole_Intrinsic_Radial_K3 queryIntrinsics; // image associated camera intrinsics
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
      OPENMVG_CERR("ERROR : can't read media first frame " << _mediaPaths[mediaIndex]);
      throw std::invalid_argument("ERROR : can't read media first frame " + _mediaPaths[mediaIndex]);
    }

    // define output image metadata
    if(!_cameraInfos.at(mediaIndex).focalIsMM)
    {
      convertFocalLengthInMM(_cameraInfos.at(mediaIndex), image.Width());
    }

    // define media informations variables
    auto& mediaInfo =  _mediasInfo.at(mediaIndex);
    mediaInfo.halfHeight = image.Height() / 2;
    mediaInfo.halfWidth = image.Width() / 2;
    mediaInfo.tileHeight = mediaInfo.halfHeight / _nbTileSide;
    mediaInfo.tileWidth = mediaInfo.halfWidth / _nbTileSide;
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
    OPENMVG_COUT("frame : " << frameIndex <<  std::endl);
    bool frameSelected = true;
    auto& frameData = _framesData.at(frameIndex);
    frameData.mediasData.resize(_feeds.size());

    for(std::size_t mediaIndex = 0; mediaIndex < _feeds.size(); ++mediaIndex)
    {
      OPENMVG_COUT("media : " << _mediaPaths.at(mediaIndex) << std::endl);
      auto& feed = *_feeds.at(mediaIndex);

      if(frameSelected) // false if a camera of a rig is not selected
      {
        if(!feed.readImage(image, queryIntrinsics, currentImgName, hasIntrinsics))
        {
          OPENMVG_CERR("ERROR  : can't read frame '" << currentImgName << "' !");
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
        OPENMVG_COUT(" > selected "  <<  std::endl);
        frameData.selected = true;
        frameData.computeAvgSharpness();
      }
      else
      {
        OPENMVG_COUT(" > skipped "  <<  std::endl);
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
        OPENMVG_COUT("--> keyframe choice : " << keyframeIndex <<  std::endl);
        if(_maxOutFrame == 0) // no limit of keyframes (direct evaluation)
        {
          // write keyframe
          for(std::size_t mediaIndex = 0; mediaIndex < _feeds.size(); ++mediaIndex)
          {
            auto& feed = *_feeds.at(mediaIndex);

            feed.goToFrame(keyframeIndex);
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
        OPENMVG_COUT("--> keyframe choice : none");
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

    for(std::size_t i = 0; i < _maxOutFrame; ++i)
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

  OPENMVG_COUT( " - sharpness : " << currMediaData.sharpness <<  std::endl);

  if(currMediaData.sharpness > _sharpnessThreshold)
  {
    bool noKeyframe = (_keyframeIndexes.empty());

    // compute current frame sparse histogram
    std::unique_ptr<features::Regions> regions;
    _imageDescriber->Describe(imageGrayHalfSample, regions);
    currMediaData.histogram = voctree::SparseHistogram(_voctree->quantizeToSparse(dynamic_cast<features::SIFT_Regions*>(regions.get())->Descriptors()));

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
      OPENMVG_COUT(" - distScore : " << currMediaData.distScore <<  std::endl);
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
  const auto filepath = _outputDirectory + "/frame_" + to_string(frameIndex) + "_media_" + to_string(mediaIndex) + ".jpg";

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
  
  exif::sensordb::Datasheet find;
  std::vector<exif::sensordb::Datasheet> vecDatabase;
  exif::sensordb::parseDatabase(_sensorDbPath, vecDatabase);

  if(exif::sensordb::getInfo(cameraInfo.brand, cameraInfo.model, vecDatabase, find))
  {
    cameraInfo.focalLength = (cameraInfo.focalLength * find._sensorSize) / imageWidth;
    cameraInfo.focalIsMM = true;
    OPENMVG_COUT("INFO : Focal length converted in mm : " << cameraInfo.focalLength << std::endl);
  }
  else
  {
    OPENMVG_COUT("WARNING : can't convert focal length in mm  : " << cameraInfo.brand << " / " << cameraInfo.model <<  std::endl);
  }
}

} // namespace keyframe 
} // namespace openMVG
