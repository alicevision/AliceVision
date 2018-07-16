// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/sfm/sfmDataIO.hpp>
#include <aliceVision/sfm/AlembicExporter.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <cstdlib>
#include <string>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

int main(int argc, char** argv)
{
  // command-line parameters

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string sfmDataFilename;
  std::string sfmDataFilterFilename;
  std::string outFilename;

  po::options_description allParams("AliceVision exportAnimatedCamera");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
      "SfMData file containing a complete SfM.")
    ("filter", po::value<std::string>(&sfmDataFilterFilename)->required(),
      "A SfMData file use as filter.")
    ("output,o", po::value<std::string>(&outFilename)->required(),
      "Output file path for the alembic animated camera.");

  po::options_description logParams("Log parameters");
  logParams.add_options()
    ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
      "verbosity level (fatal,  error, warning, info, debug, trace).");

  allParams.add(requiredParams).add(logParams);

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

  // load SfMData files
  sfm::SfMData sfmData;
  if(!sfm::Load(sfmData, sfmDataFilename, sfm::ESfMData::ALL))
  {
    ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' cannot be read.");
    return EXIT_FAILURE;
  }

  if(sfmData.getViews().empty())
  {
    ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' is empty.");
    return EXIT_FAILURE;
  }

  sfm::SfMData sfmDataFilter;
  if(!sfm::Load(sfmDataFilter, sfmDataFilterFilename, sfm::ESfMData::VIEWS))
  {
    ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilterFilename << "' cannot be read.");
    return EXIT_FAILURE;
  }

  if(sfmDataFilter.getViews().empty())
  {
    ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilterFilename << "' is empty.");
    return EXIT_FAILURE;
  }

  const std::string cameraName = fs::path(outFilename).filename().string();

  std::map<unsigned int, IndexT> viewToFrame;

  for(const auto& viewPair : sfmDataFilter.getViews())
  {
    const std::string imagePath = fs::path(viewPair.second->getImagePath()).stem().string();

    const std::size_t lastCharIndex = imagePath.find_last_not_of("0123456789");

    if(lastCharIndex >= imagePath.size())
      continue;

    const unsigned int frame = std::stoul(imagePath.substr(lastCharIndex + 1));

    viewToFrame.emplace(frame, viewPair.first);
  }

  if(viewToFrame.empty())
  {
    ALICEVISION_LOG_ERROR("Cannot find sequence pattern in image paths.");
    return EXIT_FAILURE;
  }


  ALICEVISION_LOG_DEBUG("Begin export.");

  sfm::AlembicExporter exporter(outFilename);
  exporter.initAnimatedCamera(cameraName);

  unsigned int firstFrame = viewToFrame.begin()->first;

  for(unsigned int frame = 1; frame <= viewToFrame.rbegin()->first; ++frame)
  {
    if(frame >= firstFrame)
    {
      ALICEVISION_LOG_DEBUG("Frame: " << frame);

      const auto findFrameIt = viewToFrame.find(frame);
      if(findFrameIt != viewToFrame.end())
      {
        const IndexT viewId = findFrameIt->second;

        const auto findViewIt = sfmData.getViews().find(viewId);
        if(findViewIt != sfmData.getViews().end())
        {
          if(sfmData.isPoseAndIntrinsicDefined(findViewIt->second.get()))
          {
            ALICEVISION_LOG_DEBUG("Add Camera Keyframe");
            const IndexT intrinsicId = findViewIt->second->getIntrinsicId();
            const camera::Pinhole* cam = dynamic_cast<camera::Pinhole*>(sfmData.getIntrinsicPtr(intrinsicId));
            const geometry::Pose3 pose = sfmData.getPose(*findViewIt->second).getTransform();
            exporter.addCameraKeyframe(pose,
                                       cam,
                                       findViewIt->second->getImagePath(),
                                       viewId,
                                       intrinsicId);
            continue;
          }
        }
      }
    }
    exporter.jumpKeyframe(std::to_string(frame));
  }

  return EXIT_SUCCESS;
}


