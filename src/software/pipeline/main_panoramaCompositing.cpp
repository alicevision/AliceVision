// This file is part of the AliceVision project.
// Copyright (c) 2020 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

// Input and geometry
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

// Image stuff
#include <aliceVision/image/all.hpp>
#include <aliceVision/mvsData/imageAlgo.hpp>

// Logging stuff
#include <aliceVision/system/Logger.hpp>

// Reading command line options
#include <boost/program_options.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/main.hpp>

// IO
#include <fstream>
#include <algorithm>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/filesystem.hpp>

#include <aliceVision/panorama/graphcut.hpp>
#include <aliceVision/panorama/distance.hpp>
#include <aliceVision/panorama/feathering.hpp>
#include <aliceVision/panorama/gaussian.hpp>
#include <aliceVision/panorama/graphcut.hpp>
#include <aliceVision/panorama/imageOps.hpp>
#include <aliceVision/panorama/laplacianPyramid.hpp>
#include <aliceVision/panorama/compositer.hpp>
#include <aliceVision/panorama/alphaCompositer.hpp>
#include <aliceVision/panorama/laplacianCompositer.hpp>
#include <aliceVision/panorama/seams.hpp>


// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace bpt = boost::property_tree;
namespace fs = boost::filesystem;

int aliceVision_main(int argc, char **argv)
{
  std::string sfmDataFilepath;
  std::string warpingFolder;
  std::string outputPanorama;

  std::string compositerType = "multiband";
  std::string overlayType = "none";
  bool useGraphCut = true;
  bool showBorders = false;
  bool showSeams = false;

  image::EStorageDataType storageDataType = image::EStorageDataType::Float;

  system::EVerboseLevel verboseLevel = system::Logger::getDefaultVerboseLevel();

  // Program description
  po::options_description allParams (
    "Perform panorama stiching of cameras around a nodal point for 360° panorama creation. \n"
    "AliceVision PanoramaCompositing"
  );

  // Description of mandatory parameters
  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilepath)->required(), "Input sfmData.")
    ("warpingFolder,w", po::value<std::string>(&warpingFolder)->required(), "Folder with warped images.")
    ("output,o", po::value<std::string>(&outputPanorama)->required(), "Path of the output panorama.");
  allParams.add(requiredParams);

  // Description of optional parameters
  po::options_description optionalParams("Optional parameters");
  optionalParams.add_options()
    ("compositerType,c", po::value<std::string>(&compositerType)->required(), "Compositer Type [replace, alpha, multiband].")
    ("overlayType,c", po::value<std::string>(&overlayType)->required(), "Overlay Type [none, borders, seams, all].")
    ("useGraphCut,c", po::value<bool>(&useGraphCut)->default_value(useGraphCut), "Do we use graphcut for ghost removal ?")
    ("storageDataType", po::value<image::EStorageDataType>(&storageDataType)->default_value(storageDataType),
      ("Storage data type: " + image::EStorageDataType_informations()).c_str());
  allParams.add(optionalParams);

  // Setup log level given command line
  po::options_description logParams("Log parameters");
  logParams.add_options()
    ("verboseLevel,v", po::value<system::EVerboseLevel>(&verboseLevel)->default_value(verboseLevel), "verbosity level (fatal, error, warning, info, debug, trace).");
  allParams.add(logParams);


  // Effectively parse command line given parse options
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

  // Set verbose level given command line
  system::Logger::get()->setLogLevel(verboseLevel);

  if (overlayType == "borders" || overlayType == "all")
  {
    showBorders = true;
  }

  if (overlayType == "seams" || overlayType == "all") {
    showSeams = true;
  }

  // load input scene
  sfmData::SfMData sfmData;
  if(!sfmDataIO::Load(sfmData, sfmDataFilepath, sfmDataIO::ESfMData(sfmDataIO::VIEWS|sfmDataIO::EXTRINSICS|sfmDataIO::INTRINSICS)))
  {
    ALICEVISION_LOG_ERROR("The input file '" + sfmDataFilepath + "' cannot be read");
    return EXIT_FAILURE;
  }

  std::pair<int, int> panoramaSize;
  {
      const IndexT viewId = *sfmData.getValidViews().begin();
      const std::string viewFilepath = (fs::path(warpingFolder) / (std::to_string(viewId) + ".exr")).string();
      ALICEVISION_LOG_TRACE("Read panorama size from file: " << viewFilepath);

      oiio::ParamValueList metadata = image::readImageMetadata(viewFilepath);
      panoramaSize.first = metadata.find("AliceVision:panoramaWidth")->get_int();
      panoramaSize.second = metadata.find("AliceVision:panoramaHeight")->get_int();

      if(panoramaSize.first == 0 || panoramaSize.second == 0)
      {
          ALICEVISION_LOG_ERROR("The output panorama size is empty.");
          return EXIT_FAILURE;
      }
      ALICEVISION_LOG_INFO("Output panorama size set to " << panoramaSize.first << "x" << panoramaSize.second);
  }

  std::unique_ptr<Compositer> compositer;
  bool isMultiBand = false;
  if (compositerType == "multiband")
  {
    compositer = std::unique_ptr<Compositer>(new LaplacianCompositer(panoramaSize.first, panoramaSize.second, 1));
    isMultiBand = true;
  }
  else if (compositerType == "alpha")
  {
    compositer = std::unique_ptr<Compositer>(new AlphaCompositer(panoramaSize.first, panoramaSize.second));
  }
  else
  {
    compositer = std::unique_ptr<Compositer>(new Compositer(panoramaSize.first, panoramaSize.second));
  }


  // Compute seams
  std::vector<std::shared_ptr<sfmData::View>> viewsToDraw;

  std::unique_ptr<WTASeams> wtaSeams(new WTASeams(panoramaSize.first, panoramaSize.second));
  if (isMultiBand)
  {
    std::map<size_t, std::vector<std::shared_ptr<sfmData::View>>> indexed_by_scale;
    for (const auto& viewIt : sfmData.getViews())
    {
      if(!sfmData.isPoseAndIntrinsicDefined(viewIt.second.get()))
      {
          // skip unreconstructed views
          continue;
      }
      
      // Load mask
      const std::string maskPath = (fs::path(warpingFolder) / (std::to_string(viewIt.first) + "_mask.exr")).string();
      ALICEVISION_LOG_INFO("Load mask with path " << maskPath);
      image::Image<unsigned char> mask;
      image::readImage(maskPath, mask, image::EImageColorSpace::NO_CONVERSION);

      oiio::ParamValueList metadata = image::readImageMetadata(maskPath);
      const std::size_t offsetX = metadata.find("AliceVision:offsetX")->get_int();
      const std::size_t offsetY = metadata.find("AliceVision:offsetY")->get_int();

      // Load Weights
      const std::string weightsPath = (fs::path(warpingFolder) / (std::to_string(viewIt.first) + "_weight.exr")).string();
      ALICEVISION_LOG_INFO("Load weights with path " << weightsPath);
      image::Image<float> weights;
      image::readImage(weightsPath, weights, image::EImageColorSpace::NO_CONVERSION);

      wtaSeams->append(mask, weights, viewIt.first, offsetX, offsetY);

      /*Get smalles size*/
      size_t minsize = std::min(mask.Height(), mask.Width());
      
      /*
      minsize / 2^x = 8
      minsize / 8 = 2^x
      x = log2(minsize/8)
      */
      size_t optimal_scale = size_t(floor(std::log2(double(minsize) / 5.0)));
      indexed_by_scale[optimal_scale].push_back(viewIt.second);
    }

    for (auto item : indexed_by_scale) {
      for (auto view : item.second) {
        viewsToDraw.push_back(view);
      }
    }
  }
  else {
    for (auto& viewIt : sfmData.getViews())
    {
      if(!sfmData.isPoseAndIntrinsicDefined(viewIt.second.get()))
      {
          // skip unreconstructed views
          continue;
      }
      
      viewsToDraw.push_back(viewIt.second);
    }
  }

  /* Retrieve seams from distance tool */
  image::Image<IndexT> labels = wtaSeams->getLabels();
  wtaSeams.reset();
  wtaSeams = nullptr;

  if (isMultiBand && useGraphCut) {

    int initial_level = 0;
    int max_width_for_graphcut = 5000;
    double ratio = double(panoramaSize.first) / double(max_width_for_graphcut);
    if (ratio > 1.0) {
      initial_level = int(ceil(log2(ratio)));
    }  

    for (int l = initial_level; l>= 0; l--) {
      HierarchicalGraphcutSeams seams(panoramaSize.first, panoramaSize.second, l);
      seams.setOriginalLabels(labels);
      if (l != initial_level) {
        seams.setMaximalDistance(100);
      }

      for (const auto& viewIt : sfmData.getViews())
      {
        if(!sfmData.isPoseAndIntrinsicDefined(viewIt.second.get()))
        {
            // skip unreconstructed views
            continue;
        }
        
        // Load mask
        const std::string maskPath = (fs::path(warpingFolder) / (std::to_string(viewIt.first) + "_mask.exr")).string();
        ALICEVISION_LOG_INFO("Load mask with path " << maskPath);
        image::Image<unsigned char> mask;
        image::readImage(maskPath, mask, image::EImageColorSpace::NO_CONVERSION);

        oiio::ParamValueList metadata = image::readImageMetadata(maskPath);
        const std::size_t offsetX = metadata.find("AliceVision:offsetX")->get_int();
        const std::size_t offsetY = metadata.find("AliceVision:offsetY")->get_int();

        // Load Color
        const std::string colorsPath = (fs::path(warpingFolder) / (std::to_string(viewIt.first) + ".exr")).string();
        ALICEVISION_LOG_INFO("Load colors with path " << colorsPath);
        image::Image<image::RGBfColor> colors;
        image::readImage(colorsPath, colors, image::EImageColorSpace::NO_CONVERSION);

        seams.append(colors, mask, viewIt.first, offsetX, offsetY);
      }
      
      if (seams.process()) {
        ALICEVISION_LOG_INFO("Updating labels with graphcut");
        labels = seams.getLabels();
      }
    }
  }

  oiio::ParamValueList outputMetadata;

  // Do compositing
  for (const auto & view : viewsToDraw)
  {
    IndexT viewId = view->getViewId();

    if(!sfmData.isPoseAndIntrinsicDefined(view.get()))
    {
        // skip unreconstructed views
        continue;
    }

    // Load image and convert it to linear colorspace
    const std::string imagePath = (fs::path(warpingFolder) / (std::to_string(viewId) + ".exr")).string();
    ALICEVISION_LOG_INFO("Load image with path " << imagePath);
    image::Image<image::RGBfColor> source;
    image::readImage(imagePath, source, image::EImageColorSpace::NO_CONVERSION);

    oiio::ParamValueList metadata = image::readImageMetadata(imagePath);
    if(outputMetadata.empty())
    {
        // the first one will define the output metadata (random selection)
        outputMetadata = metadata;
    }
    const std::size_t offsetX = metadata.find("AliceVision:offsetX")->get_int();
    const std::size_t offsetY = metadata.find("AliceVision:offsetY")->get_int();

    // Load mask
    const std::string maskPath = (fs::path(warpingFolder) / (std::to_string(viewId) + "_mask.exr")).string();
    ALICEVISION_LOG_INFO("Load mask with path " << maskPath);
    image::Image<unsigned char> mask;
    image::readImage(maskPath, mask, image::EImageColorSpace::NO_CONVERSION);

    // Load Weights
    const std::string weightsPath = (fs::path(warpingFolder) / (std::to_string(viewId) + "_weight.exr")).string();
    ALICEVISION_LOG_INFO("Load weights with path " << weightsPath);
    image::Image<float> weights;
    image::readImage(weightsPath, weights, image::EImageColorSpace::NO_CONVERSION);

    // Build weight map
    if (isMultiBand)
    {
      image::Image<float> seams(weights.Width(), weights.Height());
      getMaskFromLabels(seams, labels, viewId, offsetX, offsetY);

      // Composite image into panorama
      compositer->append(source, mask, seams, offsetX, offsetY);
    }
    else
    {
      compositer->append(source, mask, weights, offsetX, offsetY);
    }
  }

  // Build image
  compositer->terminate();
  

  if (showBorders)
  {
    for (const auto& viewIt : sfmData.getViews())
    {
      if(!sfmData.isPoseAndIntrinsicDefined(viewIt.second.get()))
      {
          // skip unreconstructed views
          continue;
      }

      // Load mask
      const std::string maskPath = (fs::path(warpingFolder) / (std::to_string(viewIt.first) + "_mask.exr")).string();
      ALICEVISION_LOG_INFO("Load mask with path " << maskPath);
      image::Image<unsigned char> mask;
      image::readImage(maskPath, mask, image::EImageColorSpace::NO_CONVERSION);

      oiio::ParamValueList metadata = image::readImageMetadata(maskPath);
      const std::size_t offsetX = metadata.find("AliceVision:offsetX")->get_int();
      const std::size_t offsetY = metadata.find("AliceVision:offsetY")->get_int();

      drawBorders(compositer->getPanorama(), mask, offsetX, offsetY);
    }
  }
  
  if (showSeams)
  {
    drawSeams(compositer->getPanorama(), labels);
  }

  // Remove Warping-specific metadata
  outputMetadata.remove("AliceVision:offsetX");
  outputMetadata.remove("AliceVision:offsetY");
  outputMetadata.remove("AliceVision:panoramaWidth");
  outputMetadata.remove("AliceVision:panoramaHeight");
  // no notion of extra orientation on the output panorama
  outputMetadata.remove("Orientation");
  outputMetadata.remove("orientation");

  // Store output
  ALICEVISION_LOG_INFO("Write output panorama to file " << outputPanorama);
  const aliceVision::image::Image<image::RGBAfColor> & panorama = compositer->getPanorama();

  // Select storage data type
  outputMetadata.push_back(oiio::ParamValue("AliceVision:storageDataType", image::EStorageDataType_enumToString(storageDataType)));

  image::writeImage(outputPanorama, panorama, image::EImageColorSpace::AUTO, outputMetadata);

  return EXIT_SUCCESS;
}
