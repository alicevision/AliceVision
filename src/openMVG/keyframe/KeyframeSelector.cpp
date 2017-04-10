#include "KeyframeSelector.hpp"

#include <openMVG/exif/sensor_width_database/ParseDatabase.hpp>
#include <openMVG/logger.hpp>

#include <nonFree/sift/SIFT_describer.hpp>
#include <nonFree/sift/SIFT_float_describer.hpp>

#include <tuple>
#include <cassert>

namespace openMVG {
namespace keyframe {

float computeSharpness(const image::Image<unsigned char>& imageGray, 
                        const unsigned int tileHeight, 
                        const unsigned int tileWidth, 
                        const unsigned int sharpSubset)
{
  image::Image<float> image;
  image::Image<float> scharrXDer;
  image::Image<float> scharrYDer;
  
  image::ConvertPixelType(imageGray, &image);
  image::ImageScharrXDerivative(image, scharrXDer); // normalized
  image::ImageScharrYDerivative(image, scharrYDer); // normalized

  scharrXDer = scharrXDer.cwiseAbs(); //absolute value
  scharrYDer = scharrYDer.cwiseAbs(); //absolute value
  
  // image tiles 
  std::vector<float> averageTileIntensity; 
  const float tileSizeInv = 1 / static_cast<float>(tileHeight * tileWidth);
  
  for(size_t y =  0; y < imageGray.Height(); y += tileHeight)
  {
    for(size_t x =  0; x < imageGray.Width(); x += tileWidth)
    {
      const auto sum = scharrXDer.block(y, x, tileHeight, tileWidth).sum() + scharrYDer.block(y, x, tileHeight, tileWidth).sum();

      averageTileIntensity.push_back(sum * tileSizeInv);
    }
  }
  
  // sort tiles average pixel intensity 
  std::sort(averageTileIntensity.begin(), averageTileIntensity.end()); 

  // return the sum of the subset average pixel intensity
  return std::accumulate(averageTileIntensity.end() - sharpSubset, averageTileIntensity.end(), 0.0f) / sharpSubset;
}


KeyframeSelector::KeyframeSelector(const std::string& mediaFilePath,
                                    const std::string& sensorDbPath,
                                    const std::string& voctreeFilePath,
                                    const std::string& outputDirectory)
 : _mediaFilePath(mediaFilePath)
 , _sensorDbPath(sensorDbPath)
 , _voctreeFilePath(voctreeFilePath)
 , _outputDirectory(outputDirectory)
 , _feed(_mediaFilePath)
{
  // load vocabulary tree
  _voctree.reset(new openMVG::voctree::VocabularyTree<DescriptorFloat>(voctreeFilePath));
  
  {
      OPENMVG_COUT("vocabulary tree loaded with" << endl);
      OPENMVG_COUT("\t" << _voctree->levels() << " levels" << std::endl);
      OPENMVG_COUT("\t" << _voctree->splits() << " branching factor" << std::endl);
  }
  
  // load media 
  if(!_feed.isInit())
  {
    OPENMVG_CERR("ERROR while initializing the FeedProvider !");
    throw std::invalid_argument("ERROR while initializing the FeedProvider with " + _mediaFilePath);
  }
  
  // resize selection data vector
  _selectionData.resize(_feed.nbFrames());

  // create SIFT image describer
  _imageDescriber.reset(new features::SIFT_Image_describer());
}

void KeyframeSelector::process()
{
  // feed provider variables
  image::Image< image::RGBColor> image;                 // original image
  cameras::Pinhole_Intrinsic_Radial_K3 queryIntrinsics; // image associated camera intrinsics
  bool hasIntrinsics = false;                           // true if queryIntrinsics is valid
  std::string currentImgName;                           // current image name
  
  // first frame
  if(!_feed.readImage(image, queryIntrinsics, currentImgName, hasIntrinsics))
  {
    OPENMVG_CERR("ERROR can't read media first frame !");
    throw std::invalid_argument("ERROR can't read media first frame of " + _mediaFilePath);
  }
  
  // algorithm variable
  const int halfHeight = image.Height() / 2;
  const int halfWidth = image.Width() / 2;
  
  const unsigned int tileSharpSubset =  (_nbTileSide * _nbTileSide) / _sharpSubset;
  const unsigned int tileHeight = halfHeight / _nbTileSide;
  const unsigned int tileWidth = halfWidth / _nbTileSide;
  const unsigned int frameStep = _maxFrameStep - _minFrameStep;
  
  // define output image metadata
  
  if(!_focalIsMM)
  {
    convertFocalLengthInMM(image.Width());
  }
  
  oiio::ImageSpec spec(image.Width(), image.Height(), 3, oiio::TypeDesc::UINT8); //always jpeg
  spec.attribute ("oiio:ColorSpace", "sRGB");
  spec.attribute("Make", _brand);
  spec.attribute("Model", _model);
  spec.attribute("Exif:FocalLength", _focalLength);
  
  // selection
  
  _keyframeIndexes.clear();
  size_t currentFrameStep = _minFrameStep; //begin directly
  size_t currentFrameIndex = 0;

  while(_feed.readImage(image, queryIntrinsics, currentImgName, hasIntrinsics))
  {
    OPENMVG_COUT("frame : " << currentFrameIndex <<  std::endl);
    
    // compute sharpness and sparse distance
    computeFrameData(image, currentFrameIndex, tileSharpSubset, tileHeight, tileWidth);
    
    if(currentFrameStep >= _maxFrameStep) // evaluation
    {
      currentFrameStep = _minFrameStep;
      size_t maxIndex = 0; 
      float maxSharpness = 0;
      
      //find the sharpest selected frame 
      for(size_t index = currentFrameIndex - (frameStep - 1); index <= currentFrameIndex; ++index)
      {
        if(_selectionData[index].selected && (_selectionData[index].sharpness > maxSharpness))
        {
          maxIndex = index;
          maxSharpness = _selectionData[index].sharpness;
        }
      }
      
      OPENMVG_COUT("-> keyframe choice : " << maxIndex <<  std::endl);
      
      if(maxIndex != 0)
      {
        currentFrameIndex = maxIndex; 
        _feed.goToFrame(currentFrameIndex);  
        _feed.readImage(image, queryIntrinsics, currentImgName, hasIntrinsics);
        if(_maxOutFrame == 0) //no limit of keyframes
        {
          writeKeyframe(image, spec, currentFrameIndex);
          //WriteImage((_outputDirectory + "/frame_" + to_string(currentFrameIndex) + ".jpg").c_str() , image); //write without metadata
        }
        _selectionData[currentFrameIndex].keyframe = true;
        _keyframeIndexes.push_back(currentFrameIndex);
        currentFrameIndex += _minFrameStep - 1;
      }
    }
    
    ++currentFrameIndex;
    ++currentFrameStep;
    _feed.goToFrame(currentFrameIndex); 
  }
  
  if(_maxOutFrame == 0) //no limit of keyframes
  {
    return; 
  }
  
  // if limited number of keyframe select smallest sparse distance 
  {
    std::vector< std::tuple<float, float, size_t> > keyframes;
    
    for(size_t i = 0; i < _selectionData.size(); ++i)
    {
      if((_selectionData[i].keyframe) && (_selectionData[i].sharpness != 0.0f))
      {
        keyframes.emplace_back(_selectionData[i].distScore, 1 / _selectionData[i].sharpness, i);
      }
    }
    std::sort(keyframes.begin(), keyframes.end()); 
    
    for(size_t i = 0; i < _maxOutFrame; ++i)
    {
        _feed.goToFrame(i);  
        _feed.readImage(image, queryIntrinsics, currentImgName, hasIntrinsics);
        writeKeyframe(image, spec, currentFrameIndex);
        //WriteImage((_outputDirectory + "/frame_" + to_string(currentFrameIndex) + ".jpg").c_str() , image); //write without metadata
    }
  }
}

void KeyframeSelector::computeFrameData(const image::Image<image::RGBColor>& image, 
        size_t frameIndex, 
        unsigned int tileSharpSubset,
        unsigned int tileHeight,
        unsigned int tileWidth
        )
{
  image::Image<unsigned char> imageGray;                // grayscale image
  image::Image<unsigned char> imageGrayHalfSample;      // half resolution grayscale image
  
  auto& frameData = _selectionData[frameIndex];

  // get grayscale image and resize
  image::ConvertPixelType(image, &imageGray);
  image::ImageHalfSample(imageGray, imageGrayHalfSample);

  // compute sharpness
  frameData.sharpness = computeSharpness(imageGrayHalfSample, tileHeight, tileWidth, tileSharpSubset);
  OPENMVG_COUT( " - sharpness : " << frameData.sharpness <<  std::endl);

  if(frameData.sharpness > _sharpnessThreshold) 
  {
    bool noKeyframe = (_keyframeIndexes.empty());
    // compute current frame sparse histogram
    std::unique_ptr<features::Regions> regions;
    _imageDescriber->Describe(imageGrayHalfSample, regions);
    frameData.histogram.reset( new voctree::SparseHistogram(_voctree->quantizeToSparse(dynamic_cast<features::SIFT_Regions*>(regions.get())->Descriptors())));

    // compute sparseDistance
    if(!noKeyframe)
    {
      unsigned int nbKeyframetoCompare = (_keyframeIndexes.size() < _nbKeyFrameDist)? _keyframeIndexes.size() : _nbKeyFrameDist;
      
      for(size_t i = _keyframeIndexes.size() - nbKeyframetoCompare; i < _keyframeIndexes.size(); ++i)
      {
        frameData.distScore = std::min(frameData.distScore, std::abs(voctree::sparseDistance(*(_selectionData[i].histogram), *(frameData.histogram), "strongCommonPoints"))); 
      }

      OPENMVG_COUT(" - distScore : " << frameData.distScore <<  std::endl);
    }
    
    if(noKeyframe || (frameData.distScore < _distScoreMax))
    {
      OPENMVG_COUT(" - selected "  <<  std::endl);
      frameData.selected = true;
    }
    else
    {
      frameData.histogram->clear(); //clear unselected frame histogram  
    }
  }
}

void KeyframeSelector::writeKeyframe(const image::Image<image::RGBColor>& image, 
                                      const oiio::ImageSpec& spec,
                                      size_t frameIndex)
{
  const auto filepath = _outputDirectory + "/frame_" + to_string(frameIndex) + ".jpg";
  std::unique_ptr<oiio::ImageOutput> out(oiio::ImageOutput::create(filepath));
  
  if(out.get() == nullptr)
  {
      throw std::invalid_argument("ERROR can't create image file : " + filepath);
  }
  
  if(!out->open(filepath, spec))
  {
      throw std::invalid_argument("ERROR can't open image file : " + filepath);
  }

  out->write_image(oiio::TypeDesc::UINT8, image.data()); //always jpeg
  out->close();
}

void KeyframeSelector::convertFocalLengthInMM(int imageWidth)
{
  assert(imageWidth > 0);
  
  exif::sensordb::Datasheet find;
  std::vector<exif::sensordb::Datasheet> vecDatabase;
  exif::sensordb::parseDatabase(_sensorDbPath, vecDatabase);

  if(exif::sensordb::getInfo(_brand, _model, vecDatabase, find))
  {
    _focalLength = (_focalLength * find._sensorSize) / imageWidth;
    _focalIsMM = true;
    OPENMVG_COUT("Focal length converted in mm : " << _focalLength << std::endl);
  }
  else
  {
    OPENMVG_COUT("WARNING can't convert focal length in mm  : " << _brand << " / " << _model <<  std::endl);
  }
}

} // namespace keyframe 
} // namespace openMVG