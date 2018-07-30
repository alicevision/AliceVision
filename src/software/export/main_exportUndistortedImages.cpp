// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// Copyright (c) 2015 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/image/all.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/progress.hpp>

#include <stdlib.h>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

int main(int argc, char *argv[])
{
  // command-line parameters

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string sfmDataFilename;
  std::string outDirectory;
  std::string outImageFileTypeName = image::EImageFileType_enumToString(image::EImageFileType::JPEG);

  po::options_description allParams(
    "Export undistorted images related to a sfmData file.\n"
    "AliceVision exportUndistortedImages");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
      "SfMData file.")
    ("output,o", po::value<std::string>(&outDirectory)->required(),
      "Output folder.");

  po::options_description optionalParams("Optional parameters");
  optionalParams.add_options()
    ("outputFileType", po::value<std::string>(&outImageFileTypeName)->default_value(outImageFileTypeName),
      image::EImageFileType_informations().c_str());

  po::options_description logParams("Log parameters");
  logParams.add_options()
    ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
      "verbosity level (fatal,  error, warning, info, debug, trace).");

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

  // set verbose level
  system::Logger::get()->setLogLevel(verboseLevel);

  // set output file type
  image::EImageFileType outputFileType = image::EImageFileType_stringToEnum(outImageFileTypeName);

  // Create output dir
  if(!fs::exists(outDirectory))
    fs::create_directory(outDirectory);

  sfmData::SfMData sfmData;
  if(!sfmDataIO::Load(sfmData, sfmDataFilename, sfmDataIO::ESfMData(sfmDataIO::VIEWS|sfmDataIO::INTRINSICS)))
  {
    ALICEVISION_LOG_ERROR("The input SfMData file \""<< sfmDataFilename << "\" cannot be read.");
    return EXIT_FAILURE;
  }

  // export views as undistorted images (those with valid Intrinsics)
  image::Image<image::RGBfColor> image, image_ud;
  boost::progress_display progressBar(sfmData.getViews().size());
  for(sfmData::Views::const_iterator iter = sfmData.getViews().begin(); iter != sfmData.getViews().end(); ++iter, ++progressBar)
  {
    const sfmData::View* view = iter->second.get();
    //bool intrinsicDefined = view->getIntrinsicId() != UndefinedIndexT &&
    //  sfmData.getIntrinsics().find(view->getIntrinsicId()) != sfmData.getIntrinsics().end();

    sfmData::Intrinsics::const_iterator iterIntrinsic = sfmData.getIntrinsics().find(view->getIntrinsicId());

    const std::string srcImage = view->getImagePath();
    const std::string dstImage = (fs::path(outDirectory) / (fs::path(srcImage).stem().string() + "." + image::EImageFileType_enumToString(outputFileType))).string();

    const camera::IntrinsicBase * cam = iterIntrinsic->second.get();
    if (cam->isValid() && cam->have_disto())
    {
      // undistort the image and save it
      image::readImage(srcImage, image);
      camera::UndistortImage(image, cam, image_ud, image::FBLACK);
      image::writeImage(dstImage, image_ud);
    }
    else // (no distortion)
    {
      // copy the image since there is no distortion
      fs::copy_file(srcImage, dstImage);
    }
  }

  return EXIT_SUCCESS ;
}
