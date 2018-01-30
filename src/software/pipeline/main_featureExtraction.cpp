﻿// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/config.hpp>
#include <aliceVision/alicevision_omp.hpp>
#include <aliceVision/image/all.hpp>
#include <aliceVision/sfm/sfm.hpp>
#include <aliceVision/feature/imageDescriberCommon.hpp>
#include <aliceVision/feature/feature.hpp>
#include <aliceVision/system/MemoryInfo.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/progress.hpp>

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <functional>
#include <memory>
#include <limits>

using namespace aliceVision;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

class FeatureExtractor
{
  struct ViewJob
  {
    const sfm::View& view;
    std::size_t memoryConsuption = 0;
    std::string outputBasename;
    std::vector<std::size_t> cpuImageDescriberIndexes;
    std::vector<std::size_t> gpuImageDescriberIndexes;

    ViewJob(const sfm::View& view,
            const std::string& outputFolder)
      : view(view)
      , outputBasename(fs::path(fs::path(outputFolder) / fs::path(std::to_string(view.getViewId()))).string())
    {}

    bool useGPU() const
    {
      return !gpuImageDescriberIndexes.empty();
    }

    bool useCPU() const
    {
      return !cpuImageDescriberIndexes.empty();
    }

    std::string getFeaturesPath(feature::EImageDescriberType imageDescriberType) const
    {
      return outputBasename + "." + feature::EImageDescriberType_enumToString(imageDescriberType) + ".feat";
    }

    std::string getDescriptorPath(feature::EImageDescriberType imageDescriberType) const
    {
      return outputBasename + "." + feature::EImageDescriberType_enumToString(imageDescriberType) + ".desc";
    }

    void setImageDescribers(const std::vector<std::shared_ptr<feature::ImageDescriber>>& imageDescribers)
    {
      for(std::size_t i = 0; i < imageDescribers.size(); ++i)
      {
        const std::shared_ptr<feature::ImageDescriber>& imageDescriber = imageDescribers.at(i);
        feature::EImageDescriberType imageDescriberType = imageDescriber->getDescriberType();

        if(fs::exists(getFeaturesPath(imageDescriberType)) &&
           fs::exists(getDescriptorPath(imageDescriberType)))
          continue;

        memoryConsuption += imageDescriber->getMemoryConsumption(view.getWidth(), view.getHeight());

        if(imageDescriber->useCuda())
          gpuImageDescriberIndexes.push_back(i);
        else
          cpuImageDescriberIndexes.push_back(i);
      }
    }
  };

public:

  explicit FeatureExtractor(const sfm::SfMData& sfmData)
    : _sfmData(sfmData)
  {}

  void setRange(int rangeStart, int rangeSize)
  {
    _rangeStart = rangeStart;
    _rangeSize = rangeSize;
  }

  void setMaxThreads(int maxThreads)
  {
    _maxThreads = maxThreads;
  }

  void setOutputFolder(const std::string& folder)
  {
    _outputFolder = folder;
  }

  void addImageDescriber(std::shared_ptr<feature::ImageDescriber>& imageDescriber)
  {
    _imageDescribers.push_back(imageDescriber);
  }

  void process()
  {
    // iteration on each view in the range in order
    // to prepare viewJob stack
    sfm::Views::const_iterator itViewBegin = _sfmData.GetViews().begin();
    sfm::Views::const_iterator itViewEnd = _sfmData.GetViews().end();

    if(_rangeStart != -1)
    {
      std::advance(itViewBegin, _rangeStart);
      itViewEnd = itViewBegin;
      std::advance(itViewEnd, _rangeSize);
    }

    std::size_t jobMaxMemoryConsuption = 0;

    for(auto it = itViewBegin; it != itViewEnd; ++it)
    {
      const sfm::View& view = *(it->second.get());
      ViewJob viewJob(view, _outputFolder);

      viewJob.setImageDescribers(_imageDescribers);
      jobMaxMemoryConsuption = std::max(jobMaxMemoryConsuption, viewJob.memoryConsuption);

      if(viewJob.useCPU())
        _cpuJobs.push_back(viewJob);

      if(viewJob.useGPU())
        _gpuJobs.push_back(viewJob);
    }

    if(!_cpuJobs.empty())
    {
      system::MemoryInfo memoryInformation = system::getMemoryInfo();

      ALICEVISION_LOG_DEBUG("Job max memory consumption: " << jobMaxMemoryConsuption << " B");
      ALICEVISION_LOG_DEBUG("Memory information: " << std::endl <<memoryInformation);

      if(jobMaxMemoryConsuption == 0)
        throw std::runtime_error("Can't compute feature extraction job max memory consuption.");

      std::size_t nbThreads =  (0.9 * memoryInformation.freeRam) / jobMaxMemoryConsuption;

      if(memoryInformation.freeRam == 0)
      {
        ALICEVISION_LOG_WARNING("Can't find available system memory, this can be due to OS limitations.\n"
                                "Use only one thread for CPU feature extraction.");
        nbThreads = 1;
      }

      // nbThreads should not be higher than user maxThreads param
      if(_maxThreads > 0)
        nbThreads = std::min(static_cast<std::size_t>(_maxThreads), nbThreads);

      // nbThreads should not be higher than the core number
      nbThreads = std::min(static_cast<std::size_t>(omp_get_num_procs()), nbThreads);

      // nbThreads should not be higher than the job number
      nbThreads = std::min(_cpuJobs.size(), nbThreads);

      ALICEVISION_LOG_DEBUG("# threads for extraction: " << nbThreads);
      omp_set_nested(1);

#pragma omp parallel for num_threads(nbThreads)
      for(int i = 0; i < _cpuJobs.size(); ++i)
        computeViewJob(_cpuJobs.at(i));
    }

    if(!_gpuJobs.empty())
    {
      for(const auto& job : _gpuJobs)
        computeViewJob(job, true);
    }
  }

private:

  void computeViewJob(const ViewJob& job, bool useGPU = false)
  {
    image::Image<float> imageGrayFloat;
    image::Image<unsigned char> imageGrayUChar;

    image::readImage(job.view.getImagePath(), imageGrayFloat);

    const auto imageDescriberIndexes = useGPU ? job.gpuImageDescriberIndexes : job.cpuImageDescriberIndexes;

    for(auto& imageDescriberIndex : imageDescriberIndexes)
    {
      const auto& imageDescriber = _imageDescribers.at(imageDescriberIndex);
      const feature::EImageDescriberType imageDescriberType = imageDescriber->getDescriberType();
      const std::string imageDescriberTypeName = feature::EImageDescriberType_enumToString(imageDescriberType);

      // Compute features and descriptors and export them to files
      ALICEVISION_LOG_INFO("Extracting " << imageDescriberTypeName  << " features from view '" << job.view.getImagePath() << "' " << (useGPU ? "[gpu]" : "[cpu]"));

      std::unique_ptr<feature::Regions> regions;
      if(imageDescriber->useFloatImage())
      {
        // image buffer use float image, use the read buffer
        imageDescriber->describe(imageGrayFloat, regions);
      }
      else
      {
        // image buffer can't use float image
        if(imageGrayUChar.Width() == 0) // the first time, convert the float buffer to uchar
          imageGrayUChar = (imageGrayFloat.GetMat() * 255.f).cast<unsigned char>();
        imageDescriber->describe(imageGrayUChar, regions);
      }
      imageDescriber->Save(regions.get(), job.getFeaturesPath(imageDescriberType), job.getDescriptorPath(imageDescriberType));
      ALICEVISION_LOG_INFO(std::left << std::setw(6) << regions->RegionCount() << " " << imageDescriberTypeName  << " features extracted from view '" << job.view.getImagePath() << "'");
    }
  }

  const sfm::SfMData& _sfmData;
  std::vector<std::shared_ptr<feature::ImageDescriber>> _imageDescribers;
  std::string _outputFolder;
  int _rangeStart = -1;
  int _rangeSize = -1;
  int _maxThreads = -1;
  std::vector<ViewJob> _cpuJobs;
  std::vector<ViewJob> _gpuJobs;
};


/// - Compute view image description (feature & descriptor extraction)
/// - Export computed data
int main(int argc, char **argv)
{
  // command-line parameters

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string sfmDataFilename;
  std::string outputFolder;

  // user optional parameters

  std::string describerTypesName = feature::EImageDescriberType_enumToString(feature::EImageDescriberType::SIFT);
  std::string describerPreset = feature::EImageDescriberPreset_enumToString(feature::EImageDescriberPreset::NORMAL);
  bool describersAreUpRight = false;
  int rangeStart = -1;
  int rangeSize = 1;
  int maxJobs = 0;
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
    ("describerPreset,p", po::value<std::string>(&describerPreset)->default_value(describerPreset),
      "Control the ImageDescriber configuration (low, medium, normal, high, ultra).\n"
      "Configuration 'ultra' can take long time !")
    ("upright,u", po::value<bool>(&describersAreUpRight)->default_value(describersAreUpRight),
      "Use Upright feature.")
    ("forceCpuExtraction", po::value<bool>(&forceCpuExtraction)->default_value(forceCpuExtraction),
      "Use only CPU feature extraction methods.")
    ("rangeStart", po::value<int>(&rangeStart)->default_value(rangeStart),
      "Range image index start.")
    ("rangeSize", po::value<int>(&rangeSize)->default_value(rangeSize),
      "Range size.")
    ("jobs", po::value<int>(&maxJobs)->default_value(maxJobs),
      "Specifies the number of jobs to run simultaneously (0 for automatic mode).");

  po::options_description logParams("Log parameters");
  logParams.add_options()
    ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
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

  // load input scene
  sfm::SfMData sfmData;
  if(!sfm::Load(sfmData, sfmDataFilename, sfm::ESfMData(sfm::VIEWS|sfm::INTRINSICS)))
  {
    ALICEVISION_LOG_ERROR("The input file '" + sfmDataFilename + "' cannot be read");
    return EXIT_FAILURE;
  }

  // create feature extractor
  FeatureExtractor extractor(sfmData);
  extractor.setOutputFolder(outputFolder);

  // set maxThreads
  extractor.setMaxThreads(maxJobs);

  // set extraction range
  if(rangeStart != -1)
  {
    if(rangeStart < 0 || rangeSize < 0 ||
       rangeStart > sfmData.GetViews().size())
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
      imageDescriber->setConfigurationPreset(describerPreset);
      imageDescriber->setUpRight(describersAreUpRight);
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
