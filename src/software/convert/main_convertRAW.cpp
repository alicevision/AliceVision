// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/image/all.hpp>
#include <aliceVision/image/io.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/main.hpp>

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

int aliceVision_main(int argc, char** argv)
{
  // command-line parameters
  std::vector<std::string> imagePaths;
  std::string outputFolder;
  std::string outImageFileTypeName = image::EImageFileType_enumToString(image::EImageFileType::EXR);

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::vector<std::string>>(&imagePaths)->required()->multitoken(),
      "Image path.")
    ("outputFolder,o", po::value<std::string>(&outputFolder)->required(),
      "The convertion output folder.");

  po::options_description optionalParams("Optional parameters");
  optionalParams.add_options()
    ("outputFileType", po::value<std::string>(&outImageFileTypeName)->default_value(outImageFileTypeName),
      image::EImageFileType_informations().c_str());

  CmdLine cmdline("AliceVision convertRAW");
  cmdline.add(requiredParams);
  cmdline.add(optionalParams);
  if (!cmdline.execute(argc, argv))
  {
      return EXIT_FAILURE;
  }

  // set output file type
  image::EImageFileType outputFileType = image::EImageFileType_stringToEnum(outImageFileTypeName);

  // check output folder
  if(!fs::is_directory(outputFolder))
    fs::create_directory(outputFolder);

  int nbImages = 0;
  for(const std::string& path : imagePaths)
  {
    // check input path
    if(!fs::is_regular_file(path))
    {
      ALICEVISION_LOG_ERROR("Error: Can't find image '" + path + "'.");
      return EXIT_FAILURE;
    }

    nbImages += 1;

    // genrate output filename
    std::string outputPath = (outputFolder + "/" + fs::path(path).filename().replace_extension(image::EImageFileType_enumToString(outputFileType)).string());

    if(fs::is_regular_file(outputPath))
    {
      ALICEVISION_LOG_ERROR("Error: Image '" + outputPath + "' already exists.");
      return EXIT_FAILURE;
    }

    // read input image
    // only read the 3 first channels
    image::Image<image::RGBColor> image;
    oiio::ParamValueList metadata;

    try
    {
      ALICEVISION_LOG_INFO("Reading " << path);
      image::readImage(path, image, image::EImageColorSpace::LINEAR);
      metadata = image::readImageMetadata(path);
    }
    catch(std::exception& e)
    {
      ALICEVISION_LOG_ERROR(std::string("Error: ") + e.what());
      return EXIT_FAILURE;
    }

    // write output image
    try
    {
      image::writeImage(outputPath, image, image::ImageWriteOptions(), metadata);
    }
    catch(std::exception& e)
    {
      ALICEVISION_LOG_ERROR(std::string("Error: ") + e.what());
      return EXIT_FAILURE;
    }
  }
  ALICEVISION_LOG_INFO("Successfull conversion of " << nbImages << " image(s).");
  return EXIT_SUCCESS;
}


