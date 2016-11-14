#include "ImageFeed.hpp"
#include <openMVG/sfm/sfm_data.hpp>
#include <openMVG/sfm/sfm_data_io.hpp>
#include <openMVG/image/image_io.hpp>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string/case_conv.hpp> 
#include <boost/algorithm/string/replace.hpp>

#include <queue>
#include <iostream>
#include <fstream>
#include <exception>
#include <regex>
#include <iterator>
#include <string>

namespace openMVG{
namespace dataio{

class ImageFeed::FeederImpl
{
public:
  
  static bool isSupported(const std::string &ext);
  
  FeederImpl() : _isInit(false) {}
  
  FeederImpl(const std::string& imagePath, const std::string& calibPath);

  bool readImage(image::Image<unsigned char> &imageGray, 
                     cameras::Pinhole_Intrinsic_Radial_K3 &camIntrinsics,
                     std::string &imageName,
                     bool &hasIntrinsics);
  
  std::size_t nbFrames() const;
  
  bool goToFrame(const unsigned int frame);
  
  bool goToNextFrame();
  
  bool isInit() const {return _isInit;} 
  
private:
  
  bool feedWithJson(image::Image<unsigned char> &imageGray, 
                     cameras::Pinhole_Intrinsic_Radial_K3 &camIntrinsics,
                     std::string &imageName,
                     bool &hasIntrinsics);
  
private:
  static const std::vector<std::string> supportedExtensions;
  
private:
  bool _isInit;
  bool _withCalibration;
  // It contains the images to be fed
  std::vector<std::string> _images;
  cameras::Pinhole_Intrinsic_Radial_K3 _camIntrinsics;
  
  bool _sfmMode = false;
  sfm::SfM_Data _sfmdata;
  sfm::Views::const_iterator _viewIterator;
  unsigned int _currentImageIndex = 0;
};

const std::vector<std::string> ImageFeed::FeederImpl::supportedExtensions = { ".jpg", ".jpeg", ".png", ".ppm" };

bool ImageFeed::FeederImpl::isSupported(const std::string &ext)
{
  const auto start = FeederImpl::supportedExtensions.begin();
  const auto end = FeederImpl::supportedExtensions.end();
  return(std::find(start, end, boost::to_lower_copy(ext)) != end);
}

ImageFeed::FeederImpl::FeederImpl(const std::string& imagePath, const std::string& calibPath) 
: _isInit(false)
, _withCalibration(false)
{
  namespace bf = boost::filesystem;
//    OPENMVG_LOG_DEBUG(imagePath);
  // if it is a json, calibPath is neglected
  if(bf::is_regular_file(imagePath))
  {
    const std::string ext = bf::path(imagePath).extension().string();
    // if it is a sfmdata.json
    if(ext == ".json")
    {
      // load the json
      _isInit = sfm::Load(_sfmdata, imagePath, sfm::ESfM_Data(sfm::ESfM_Data::VIEWS | sfm::ESfM_Data::INTRINSICS));
      _viewIterator = _sfmdata.GetViews().begin();
      _sfmMode = true;
    }
    // if it is an image file
    else if(FeederImpl::isSupported(ext))
    {
      _images.push_back(imagePath);
      _withCalibration = !calibPath.empty();
      _sfmMode = false;
      _isInit = true;
    }
    // if it is an image file
    else if(ext == ".txt")
    {
      // we expect a simple txt file with a list of path to images relative to the 
      // location of the txt file itself
      std::fstream fs(imagePath, std::ios::in);
      std::string line;
      // parse each line of the text file
      while(getline(fs, line))
      {
        // compose the file name as the base path of the inputPath and
        // the filename just read
        const std::string filename = (bf::path(imagePath).parent_path() / line).string();
        _images.push_back(filename);
      }
      // Close file
      fs.close();
      _withCalibration = !calibPath.empty();
      _sfmMode = false;
      _isInit = true;
    }
    else
    {
      // no other file format are supported
      throw std::invalid_argument("File or mode not yet implemented");
    }
  }
  else if(bf::is_directory(imagePath) || bf::is_directory(bf::path(imagePath).parent_path()))
  {
    std::string folder = imagePath;
    // Recover the pattern : img.@.png (for example)
    std::string filePattern;
    std::regex re;
    if(!bf::is_directory(imagePath))
    {
      filePattern = bf::path(imagePath).filename().string();
      folder = bf::path(imagePath).parent_path().string();
      OPENMVG_LOG_DEBUG("filePattern: " << filePattern);
      std::string regexStr = filePattern;
      // escape "."
      boost::algorithm::replace_all(regexStr, ".", "\\.");
      // recognize # as a digit
      boost::algorithm::replace_all(regexStr, "#", "[0-9]");
      // recognize @ as a sequence of digits
      boost::algorithm::replace_all(regexStr, "@", "[0-9]+");
      re = regexStr;
    }
    else
    {
      OPENMVG_LOG_DEBUG("folder without expression: " << imagePath);
    }
    OPENMVG_LOG_DEBUG("directory feedImage");
    // if it is a directory, list all the images and add them to the list
    bf::directory_iterator iterator(folder);
    // since some OS will provide the files in a random order, first store them
    // in a priority queue and then fill the _image queue with the alphabetical
    // order from the priority queue
    std::priority_queue<std::string, 
                    std::vector<std::string>, 
                    std::greater<std::string> > tmpSorter;
    for(; iterator != bf::directory_iterator(); ++iterator)
    {
      // get the extension of the current file to check whether it is an image
      const std::string ext = iterator->path().extension().string();
      if(FeederImpl::isSupported(ext))
      {
        const std::string filepath = iterator->path().string();
        const std::string filename = iterator->path().filename().string();
        // If we have a filePattern (a sequence of images), we have to match the regex.
        if(filePattern.empty() || std::regex_match(filename, re))
          tmpSorter.push(filepath);
      }
    }
    // put all the retrieve files inside the queue
    while(!tmpSorter.empty())
    {
      _images.push_back(tmpSorter.top());
      tmpSorter.pop();
    }
    
    _withCalibration = !calibPath.empty();
    _sfmMode = false;
    _isInit = true;
  }
  else
  {
    throw std::invalid_argument("File or mode not yet implemented");
  }
  
  // last thing: if _withCalibration is true it means that it is not a json and
  // a path to a calibration file has been passed
  // then load the calibration
  if(_withCalibration)
  {
    // load the calibration from calibPath
    readCalibrationFromFile(calibPath, _camIntrinsics);
  }
}



bool ImageFeed::FeederImpl::readImage(image::Image<unsigned char> &imageGray, 
                   cameras::Pinhole_Intrinsic_Radial_K3 &camIntrinsics,
                   std::string &imageName,
                   bool &hasIntrinsics)
{
  if(!_isInit)
  {
    OPENMVG_LOG_WARNING("Image feed is not initialized ");
    return false;
  }

  // dealing with SFM mode
  if(_sfmMode)
  {
    return feedWithJson(imageGray, camIntrinsics, imageName, hasIntrinsics);
  }
  else
  {
    if(_images.empty())
      return false;
    if(_currentImageIndex >= _images.size())
      return false;

    if(_withCalibration)
    {
      // get the calibration
      camIntrinsics = _camIntrinsics;
      hasIntrinsics = true;
    }
    else
    {
      hasIntrinsics = false;
    }
    imageName = _images[_currentImageIndex];
    OPENMVG_LOG_DEBUG(imageName);
    if (!image::ReadImage(imageName.c_str(), &imageGray))
    {
      OPENMVG_LOG_WARNING("Error while opening image " << imageName);
      throw std::invalid_argument("Error while opening image " + imageName);
    }
    return true;
  }
  return true;
}

std::size_t ImageFeed::FeederImpl::nbFrames() const
{
  if(!_isInit)
    return 0;
  
  if(_sfmMode)
    return _sfmdata.GetViews().size();
  
  return _images.size();
}

bool ImageFeed::FeederImpl::goToFrame(const unsigned int frame)
{
  if(!_isInit)
  {
    _currentImageIndex = frame;
    OPENMVG_LOG_WARNING("Image feed is not initialized ");
    return false;
  }
  
  // Reconstruction mode
  if(_sfmMode)
  {
    if(frame >= _sfmdata.GetViews().size())
    {
      _viewIterator = _sfmdata.GetViews().end();
      OPENMVG_LOG_WARNING("The current frame is out of the range.");
      return false;
    }

    _viewIterator = _sfmdata.GetViews().begin();
    std::advance(_viewIterator, frame);
  }
  else
  {
    _currentImageIndex = frame;
    // Image list mode
    if(frame >= _images.size())
    {
      OPENMVG_LOG_WARNING("The current frame is out of the range.");
      return false;
    }
    OPENMVG_LOG_DEBUG("frame " << frame);
  }
  return true;
}

bool ImageFeed::FeederImpl::goToNextFrame()
{
  if(_sfmMode)
  {
    if(_viewIterator == _sfmdata.GetViews().end())
      return false;
    ++_viewIterator;
    if(_viewIterator == _sfmdata.GetViews().end())
      return false;
  }
  else
  {
    ++_currentImageIndex;
    OPENMVG_LOG_DEBUG("next frame " << _currentImageIndex);
    if(_currentImageIndex >= _images.size())
      return false;
  }
  return true;
}

bool ImageFeed::FeederImpl::feedWithJson(image::Image<unsigned char> &imageGray, 
                   cameras::Pinhole_Intrinsic_Radial_K3 &camIntrinsics,
                   std::string &imageName,
                   bool &hasIntrinsics)
{
  // if there are no more images to process
  if(_viewIterator == _sfmdata.GetViews().end())
  {
    return false;
  }

  namespace bf = boost::filesystem;

  // get the image
  const std::string rootPath = _sfmdata.s_root_path;
  const sfm::View *view = _viewIterator->second.get();
  imageName = (bf::path(rootPath) / bf::path(view->s_Img_path)).string();
  if (!image::ReadImage(imageName.c_str(), &imageGray))
  {
    OPENMVG_LOG_WARNING("Error while opening image " << imageName);
    return false;
  }
  // get the associated Intrinsics
  if((view->id_intrinsic == UndefinedIndexT) || (!_sfmdata.GetIntrinsics().count(view->id_intrinsic)))
  {
    OPENMVG_LOG_DEBUG("Image "<< imageName << " does not have associated intrinsics");
    hasIntrinsics = false;
  }
  else
  {
    const cameras::IntrinsicBase * cam = _sfmdata.GetIntrinsics().at(view->id_intrinsic).get();
    if(cam->getType() != cameras::EINTRINSIC::PINHOLE_CAMERA_RADIAL3)
    {
      OPENMVG_LOG_WARNING("Only Pinhole_Intrinsic_Radial_K3 is supported");
      hasIntrinsics = false;
    }
    else
    {
      const cameras::Pinhole_Intrinsic_Radial_K3 * intrinsics = dynamic_cast<const cameras::Pinhole_Intrinsic_Radial_K3*>(cam) ;

      // simply copy values
      camIntrinsics = *intrinsics;
      hasIntrinsics = true;
    }
  }
  ++_viewIterator;
  return true;
}




/*******************************************************************************/
/*                     ImageFeed                                               */
/*******************************************************************************/

ImageFeed::ImageFeed() : _imageFeed(new FeederImpl()) { }

ImageFeed::ImageFeed(const std::string& imagePath, const std::string& calibPath)  
    : _imageFeed( new FeederImpl(imagePath, calibPath) ) { }

bool ImageFeed::readImage(image::Image<unsigned char> &imageGray, 
                     cameras::Pinhole_Intrinsic_Radial_K3 &camIntrinsics,
                     std::string &mediaPath,
                     bool &hasIntrinsics)
{
  return(_imageFeed->readImage(imageGray, camIntrinsics, mediaPath, hasIntrinsics));
}

std::size_t ImageFeed::nbFrames() const
{
  return _imageFeed->nbFrames();
}

bool ImageFeed::goToFrame(const unsigned int frame)
{
  return _imageFeed->goToFrame(frame);
}

bool ImageFeed::goToNextFrame()
{
  return _imageFeed->goToNextFrame();
}

bool ImageFeed::isInit() const
{
  return(_imageFeed->isInit());
}

bool ImageFeed::isSupported(const std::string &extension)
{
  std::string ext = boost::to_lower_copy(extension);
  if(ext == ".json" || ext == ".txt")
  {
    return true;
  }
  else
  {
    return FeederImpl::isSupported(ext);
  }
}

ImageFeed::~ImageFeed() { }

}//namespace dataio 
}//namespace openMVG