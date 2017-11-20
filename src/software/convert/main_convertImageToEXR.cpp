// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/image/image.hpp>
#include <aliceVision/image/io.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <cstdlib>
#include <string>

using namespace aliceVision;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

int main(int argc, char** argv)
{
  // command-line parameters

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::vector<std::string> imagePaths;
  std::string outputFolder;

  po::options_description allParams("AliceVision convertImageToEXR");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::vector<std::string>>(&imagePaths)->required(),
      "Image path.")
    ("outputFolder,o", po::value<std::string>(&outputFolder)->required(),
      "The convertion output folder.");

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

  // set verbose level
  system::Logger::get()->setLogLevel(verboseLevel);

  // check output folder
  if(!fs::is_directory(outputFolder))
    fs::create_directory(outputFolder);

  for(const std::string& path : imagePaths)
  {
    // check input path
    if(!fs::is_regular_file(path))
    {
      ALICEVISION_LOG_ERROR("Error: Can't find image '" + path + "'.");
      return EXIT_FAILURE;
    }

    // genrate output filename
    std::string outputPath = (outputFolder + fs::path(path).filename().replace_extension("exr").string());

    if(fs::is_regular_file(outputPath))
    {
      ALICEVISION_LOG_ERROR("Error: Image '" + outputPath + "' already exists.");
      return EXIT_FAILURE;
    }

    // read input image
    // only read the 3 first channels
    image::Image<image::RGBColor> image;

    try
    {
      image::readImage(path, image);
    }
    catch(std::exception& e)
    {
      ALICEVISION_LOG_ERROR(std::string("Error: ") + e.what());
      return EXIT_FAILURE;
    }

    // write output image
    try
    {
      image::writeImage(outputPath, image);
    }
    catch(std::exception& e)
    {
      ALICEVISION_LOG_ERROR(std::string("Error: ") + e.what());
      return EXIT_FAILURE;
    }
  }
  return EXIT_SUCCESS;
}


