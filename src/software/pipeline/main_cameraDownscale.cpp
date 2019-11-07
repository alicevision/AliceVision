#include <aliceVision/image/all.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <aliceVision/mvsData/imageAlgo.hpp>


// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 2
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

int main(int argc, char * argv[]) {
  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string sfmInputDataFilename = "";
  std::string sfmOutputDataFilename = "";
  float rescaleFactor = 1.0f;

  /*****
   * DESCRIBE COMMAND LINE PARAMETERS 
   */
  po::options_description allParams(
    "Parse external information about cameras used in a panorama.\n"
    "AliceVision PanoramaExternalInfo");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("rescalefactor,r", po::value<float>(&rescaleFactor)->default_value(rescaleFactor), "Rescale Factor ]0.0, 1.0].")
    ("input,i", po::value<std::string>(&sfmInputDataFilename)->required(), "SfMData file input.")
    ("outSfMDataFilename,o", po::value<std::string>(&sfmOutputDataFilename)->required(), "SfMData file output.")
    ;

  po::options_description optionalParams("Optional parameters");

  po::options_description logParams("Log parameters");
  logParams.add_options()
    ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
      "verbosity level (fatal, error, warning, info, debug, trace).");
  
  allParams.add(requiredParams).add(optionalParams).add(logParams);


  /**
   * READ COMMAND LINE
   */
  po::variables_map vm;
  try
  {
    po::store(po::parse_command_line(argc, argv, allParams), vm);

    if(vm.count("help") || (argc == 1))
    {
      ALICEVISION_COUT(allParams);
      return EXIT_SUCCESS;
    }
    po::notify(vm);
  }
  catch(boost::program_options::required_option& e)
  {
    ALICEVISION_CERR("ERROR: " << e.what());
    ALICEVISION_COUT("Usage:\n\n" << allParams);
    return EXIT_FAILURE;
  }
  catch(boost::program_options::error& e)
  {
    ALICEVISION_CERR("ERROR: " << e.what());
    ALICEVISION_COUT("Usage:\n\n" << allParams);
    return EXIT_FAILURE;
  }

  ALICEVISION_COUT("Program called with the following parameters:");
  ALICEVISION_COUT(vm);

  /**
   *  set verbose level
   **/
  system::Logger::get()->setLogLevel(verboseLevel);

  if (rescaleFactor < 0.0001f || rescaleFactor > 1.0f) {
    ALICEVISION_LOG_ERROR("Invalid scale factor (]0,1]");
    return EXIT_FAILURE;
  }

  
  /*Analyze path*/
  boost::filesystem::path path(sfmOutputDataFilename);
  std::string output_path = path.parent_path().string();

  /**
   * Read input
   */
  sfmData::SfMData sfmData;
  if(!sfmDataIO::Load(sfmData, sfmInputDataFilename, sfmDataIO::ESfMData(sfmDataIO::ALL)))
  {
    ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmInputDataFilename << "' cannot be read.");
    return EXIT_FAILURE;
  }
  
  for (auto & v : sfmData.getViews()) {
    
    /*Read original image*/
    image::Image<image::RGBfColor> originalImage;
    image::readImage(v.second->getImagePath(), originalImage, image::EImageColorSpace::LINEAR);

    unsigned int w = v.second->getWidth();
    unsigned int h = v.second->getHeight();
    unsigned int nw = (unsigned int)(floor(float(w) * rescaleFactor));
    unsigned int nh = (unsigned int)(floor(float(h) * rescaleFactor));

    /*Create a rotated image*/
    image::Image<image::RGBfColor> rescaled(nw, nh);

    oiio::ImageSpec imageSpecResized(nw, nh, 3, oiio::TypeDesc::FLOAT);
    oiio::ImageSpec imageSpecOrigin(w, h, 3, oiio::TypeDesc::FLOAT);

    oiio::ImageBuf bufferOrigin(imageSpecOrigin, originalImage.data());
    oiio::ImageBuf bufferResized(imageSpecResized, rescaled.data());

    oiio::ImageBufAlgo::resample(bufferResized, bufferOrigin);

    boost::filesystem::path old_path(v.second->getImagePath());
    std::string old_filename = old_path.stem().string();

    /*Save this image*/
    std::stringstream sstream;
    sstream << output_path << "/" << old_filename << "_rescaled.exr";
    image::writeImage(sstream.str(), rescaled, image::EImageColorSpace::AUTO);

    /*Update view for this modification*/
    v.second->setWidth(nw);
    v.second->setHeight(nh);
    v.second->setImagePath(sstream.str());
  }

  for (auto & i : sfmData.getIntrinsics()) {
    i.second->rescale(rescaleFactor);
  }
    
  /***
   * Save downscaled sfmData 
   */
  if (!sfmDataIO::Save(sfmData, sfmOutputDataFilename, sfmDataIO::ESfMData(sfmDataIO::ALL))) {
    ALICEVISION_LOG_ERROR("The output SfMData file '" << sfmOutputDataFilename << "' cannot be write.");
    return EXIT_FAILURE;
  }

  return 0;
}