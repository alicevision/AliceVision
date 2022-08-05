// This file is part of the AliceVision project.
// Copyright (c) 2015 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/feature/FeatureExtractor.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/feature/imageDescriberCommon.hpp>
#include <aliceVision/feature/feature.hpp>
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_POPSIFT) \
 || ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CCTAG)
#define ALICEVISION_HAVE_GPU_FEATURES
#include <aliceVision/gpu/gpu.hpp>
#endif
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/config.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <string>
#include <iostream>
#include <functional>
#include <memory>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

/// - Compute view image description (feature & descriptor extraction)
/// - Export computed data
int aliceVision_main(int argc, char **argv)
{
  // command-line parameters

  system::EVerboseLevel verboseLevel = system::Logger::getDefaultVerboseLevel();
  std::string sfmDataFilename;
  std::string masksFolder;
  std::string outputFolder;

  // user optional parameters

  std::string describerTypesName = feature::EImageDescriberType_enumToString(feature::EImageDescriberType::SIFT);
  feature::ConfigurationPreset featDescConfig;
  int rangeStart = -1;
  int rangeSize = 1;
  int maxThreads = 0;
  bool forceCpuExtraction = false;

  po::options_description allParams("AliceVision featureExtraction");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
      "SfMData file.")
    ("output,o", po::value<std::string>(&outputFolder)->required(),
      "Output path for the features and descriptors files (*.feat, *.desc).");

  po::options_description optionalParams("Optional parameters");
  optionalParams.add_options()
    ("describerTypes,d", po::value<std::string>(&describerTypesName)->default_value(describerTypesName),
      feature::EImageDescriberType_informations().c_str())
    ("describerPreset,p", po::value<feature::EImageDescriberPreset>(&featDescConfig.descPreset)->default_value(featDescConfig.descPreset),
      "Control the ImageDescriber configuration (low, medium, normal, high, ultra).\n"
      "Configuration 'ultra' can take long time !")
    ("describerQuality", po::value<feature::EFeatureQuality>(&featDescConfig.quality)->default_value(featDescConfig.quality),
      feature::EFeatureQuality_information().c_str())
    ("gridFiltering", po::value<bool>(&featDescConfig.gridFiltering)->default_value(featDescConfig.gridFiltering),
      "Enable grid filtering. Highly recommended to ensure usable number of features.")
    ("maxNbFeatures", po::value<int>(&featDescConfig.maxNbFeatures)->default_value(featDescConfig.maxNbFeatures),
      "Max number of features extracted (0 means default value based on describerPreset).")
    ("contrastFiltering", po::value<feature::EFeatureConstrastFiltering>(&featDescConfig.contrastFiltering)->default_value(featDescConfig.contrastFiltering),
      feature::EFeatureConstrastFiltering_information().c_str())
    ("relativePeakThreshold", po::value<float>(&featDescConfig.relativePeakThreshold)->default_value(featDescConfig.relativePeakThreshold),
       "Peak Threshold relative to median of gradiants.")
    ("forceCpuExtraction", po::value<bool>(&forceCpuExtraction)->default_value(forceCpuExtraction),
      "Use only CPU feature extraction methods.")
    ("masksFolder", po::value<std::string>(&masksFolder),
      "Masks folder.")
    ("rangeStart", po::value<int>(&rangeStart)->default_value(rangeStart),
      "Range image index start.")
    ("rangeSize", po::value<int>(&rangeSize)->default_value(rangeSize),
      "Range size.")
    ("maxThreads", po::value<int>(&maxThreads)->default_value(maxThreads),
      "Specifies the maximum number of threads to run simultaneously (0 for automatic mode).");

  po::options_description logParams("Log parameters");
  logParams.add_options()
    ("verboseLevel,v", po::value<system::EVerboseLevel>(&verboseLevel)->default_value(verboseLevel),
      "verbosity level (fatal, error, warning, info, debug, trace).");

  allParams.add(requiredParams).add(optionalParams).add(logParams);

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

  // set verbose level
  system::Logger::get()->setLogLevel(verboseLevel);

  if(describerTypesName.empty())
  {
    ALICEVISION_LOG_ERROR("--describerTypes option is empty.");
    return EXIT_FAILURE;
  }

  // create output folder
  if(!fs::exists(outputFolder))
  {
    if(!fs::create_directory(outputFolder))
    {
      ALICEVISION_LOG_ERROR("Cannot create output folder");
      return EXIT_FAILURE;
    }
  }

#ifdef ALICEVISION_HAVE_GPU_FEATURES
  // Print GPU Information
  ALICEVISION_LOG_INFO(gpu::gpuInformationCUDA());
#endif

  // load input scene
  sfmData::SfMData sfmData;
  std::cout << sfmData.getViews().size()  << std::endl;
  if(!sfmDataIO::Load(sfmData, sfmDataFilename, sfmDataIO::ESfMData(sfmDataIO::VIEWS|sfmDataIO::INTRINSICS)))
  {
    ALICEVISION_LOG_ERROR("The input file '" + sfmDataFilename + "' cannot be read");
    return EXIT_FAILURE;
  }

  // create feature extractor
  feature::FeatureExtractor extractor(sfmData);
  extractor.setMasksFolder(masksFolder);
  extractor.setOutputFolder(outputFolder);

  // set maxThreads
  extractor.setMaxThreads(maxThreads);

  // set extraction range
  if(rangeStart != -1)
  {
    if(rangeStart < 0 || rangeSize < 0 ||
       rangeStart > sfmData.getViews().size())
    {
      ALICEVISION_LOG_ERROR("Range is incorrect");
      return EXIT_FAILURE;
    }

    if(rangeStart + rangeSize > sfmData.views.size())
      rangeSize = sfmData.views.size() - rangeStart;

    extractor.setRange(rangeStart, rangeSize);
  }

  // initialize feature extractor imageDescribers
  {
    std::vector<feature::EImageDescriberType> imageDescriberTypes = feature::EImageDescriberType_stringToEnums(describerTypesName);

    for(const auto& imageDescriberType: imageDescriberTypes)
    {
      std::shared_ptr<feature::ImageDescriber> imageDescriber = feature::createImageDescriber(imageDescriberType);
      imageDescriber->setConfigurationPreset(featDescConfig);
      if(forceCpuExtraction)
        imageDescriber->setUseCuda(false);

      extractor.addImageDescriber(imageDescriber);
    }
  }

  // feature extraction routines
  // for each View of the SfMData container:
  // - if regions file exist continue,
  // - if no file, compute features
  {
    system::Timer timer;

    extractor.process();

    ALICEVISION_LOG_INFO("Task done in (s): " + std::to_string(timer.elapsed()));
  }
  return EXIT_SUCCESS;
}
