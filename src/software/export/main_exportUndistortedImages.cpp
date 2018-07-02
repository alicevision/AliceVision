// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// Copyright (c) 2015 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfm/sfm.hpp>
#include <aliceVision/image/all.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/progress.hpp>

#include <stdlib.h>

using namespace aliceVision;
using namespace aliceVision::camera;
using namespace aliceVision::geometry;
using namespace aliceVision::image;
using namespace aliceVision::sfm;
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
  if (!fs::exists(outDirectory))
    fs::create_directory(outDirectory);

  SfMData sfmData;
  if (!Load(sfmData, sfmDataFilename, ESfMData(VIEWS | INTRINSICS)))
  {
    ALICEVISION_LOG_ERROR("The input SfMData file \""<< sfmDataFilename << "\" cannot be read.");
    return EXIT_FAILURE;
  }

  // Export views as undistorted images (those with valid Intrinsics)
  Image<RGBfColor> image, image_ud;
  boost::progress_display my_progress_bar( sfmData.getViews().size() );
  for(Views::const_iterator iter = sfmData.getViews().begin();
    iter != sfmData.getViews().end(); ++iter, ++my_progress_bar)
  {
    const View* view = iter->second.get();
    bool bIntrinsicDefined = view->getIntrinsicId() != UndefinedIndexT &&
      sfmData.getIntrinsics().find(view->getIntrinsicId()) != sfmData.getIntrinsics().end();

    Intrinsics::const_iterator iterIntrinsic = sfmData.getIntrinsics().find(view->getIntrinsicId());

    const std::string srcImage = view->getImagePath();
    const std::string dstImage = (fs::path(outDirectory) / (fs::path(srcImage).stem().string() + "." + image::EImageFileType_enumToString(outputFileType))).string();

    const IntrinsicBase * cam = iterIntrinsic->second.get();
    if (cam->isValid() && cam->have_disto())
    {
      // undistort the image and save it
      readImage(srcImage, image);
      UndistortImage(image, cam, image_ud, FBLACK);
      writeImage(dstImage, image_ud);
    }
    else // (no distortion)
    {
      // copy the image since there is no distortion
      fs::copy_file(srcImage, dstImage);
    }
  }

  return EXIT_SUCCESS ;
}
