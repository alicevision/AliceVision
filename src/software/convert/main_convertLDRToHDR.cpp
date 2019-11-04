// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/image/all.hpp>
#include <aliceVision/image/io.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>

/*SFMData*/
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

/*HDR Related*/
#include <aliceVision/hdr/rgbCurve.hpp>
#include <aliceVision/hdr/RobertsonCalibrate.hpp>
#include <aliceVision/hdr/hdrMerge.hpp>
#include <aliceVision/hdr/DebevecCalibrate.hpp>
#include <aliceVision/hdr/GrossbergCalibrate.hpp>
#include <aliceVision/hdr/emorCurve.hpp>

/*Command line parameters*/
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <sstream>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 0
#define ALICEVISION_SOFTWARE_VERSION_MINOR 1

using namespace aliceVision;

namespace po = boost::program_options;

int main(int argc, char * argv[]) {

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string sfmInputDataFilename = "";
  std::string sfmOutputDataFilename = "";
  int groupSize = 3;
  float clampedValueCorrection = 1.0f;

  /*****
  * DESCRIBE COMMAND LINE PARAMETERS 
  */
  po::options_description allParams(
    "Parse external information about cameras used in a panorama.\n"
    "AliceVision PanoramaExternalInfo");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmInputDataFilename)->required(), "SfMData file input.")
    ("outSfMDataFilename,o", po::value<std::string>(&sfmOutputDataFilename)->required(), "SfMData file output.")
    ("groupSize,g", po::value<int>(&groupSize)->required(), "bracket count per HDR image.")
    ;

  po::options_description optionalParams("Optional parameters");
  optionalParams.add_options()
    ("expandDynamicRange,e", po::value<float>(&clampedValueCorrection)->default_value(clampedValueCorrection), "float value between 0 and 1 to correct clamped high values in dynamic range: use 0 for no correction, 0.5 for interior lighting and 1 for outdoor lighting.");

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

  if (groupSize < 0) {
    ALICEVISION_LOG_ERROR("Invalid number of brackets");
    return EXIT_FAILURE;
  }

  /*Analyze path*/
  boost::filesystem::path path(sfmOutputDataFilename);
  std::string output_path = path.parent_path().string();

  /**
   * Read sfm data
   */
  sfmData::SfMData sfmData;
  if(!sfmDataIO::Load(sfmData, sfmInputDataFilename, sfmDataIO::ESfMData(sfmDataIO::VIEWS | sfmDataIO::INTRINSICS))) {
    ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmInputDataFilename << "' cannot be read.");
    return EXIT_FAILURE;
  }
  

  size_t countImages = sfmData.getViews().size();
  if (countImages == 0) {
    ALICEVISION_LOG_ERROR("The input SfMData contains no input !");
    return EXIT_FAILURE;
  }

  if (countImages % groupSize != 0) {
    ALICEVISION_LOG_ERROR("The input SfMData file is not compatible with this bracket size");
    return EXIT_FAILURE;
  }
  size_t countGroups = countImages / groupSize;

  /*Make sure there is only one kind of image in dataset*/
  if (sfmData.getIntrinsics().size() > 2) {
    ALICEVISION_LOG_ERROR("Multiple intrinsics : Different kind of images in dataset");
    return EXIT_FAILURE;
  }

  /*If two intrinsics, may be some images are simply rotated*/
  if (sfmData.getIntrinsics().size() == 2) {
    const sfmData::Intrinsics & intrinsics = sfmData.getIntrinsics();
    
    unsigned int w = intrinsics.begin()->second->w();
    unsigned int h = intrinsics.begin()->second->h();
    unsigned int rw = intrinsics.rbegin()->second->w();
    unsigned int rh = intrinsics.rbegin()->second->h();

    if (w != rh || h != rw) {
      ALICEVISION_LOG_ERROR("Multiple intrinsics : Different kind of images in dataset");
      return EXIT_FAILURE;
    }

    IndexT firstId = intrinsics.begin()->first;
    IndexT secondId = intrinsics.rbegin()->first;

    size_t first = 0;
    size_t second = 0;
    sfmData::Views & views = sfmData.getViews();
    for (const auto & v : views) {
      if (v.second->getIntrinsicId() == firstId) {
        first++;
      }
      else {
        second++;
      }
    }

    /* Set all view with the majority intrinsics */
    if (first > second) {
      for (const auto & v : views) {
        v.second->setIntrinsicId(firstId);
      }

      sfmData.getIntrinsics().erase(secondId);
    }
    else {
      for (const auto & v : views) {
        v.second->setIntrinsicId(secondId);
      }

      sfmData.getIntrinsics().erase(firstId);
    }
  }

  /* Rotate needed images */
  {
    const sfmData::Intrinsics & intrinsics = sfmData.getIntrinsics();
    unsigned int w = intrinsics.begin()->second->w();
    unsigned int h = intrinsics.begin()->second->h();

    sfmData::Views & views = sfmData.getViews();
    for (auto & v : views) {
      if (v.second->getWidth() == h && v.second->getHeight() == w) {
        ALICEVISION_LOG_INFO("Create intermediate rotated image !");

        /*Read original image*/
        image::Image<image::RGBfColor> originalImage;
        image::readImage(v.second->getImagePath(), originalImage, image::EImageColorSpace::LINEAR);

        /*Create a rotated image*/
        image::Image<image::RGBfColor> rotated(w, h);
        for (int k = 0; k < h; k++) {
          for (int l = 0; l < w; l++) {
            rotated(k, l) = originalImage(l, rotated.Height() - 1 - k);
          }
        }

        boost::filesystem::path old_path(v.second->getImagePath());
        std::string old_filename = old_path.stem().string();

        /*Save this image*/
        std::stringstream sstream;
        sstream << output_path << "/" << old_filename << ".exr";
        oiio::ParamValueList metadata = image::readImageMetadata(v.second->getImagePath());
        image::writeImage(sstream.str(), rotated, image::EImageColorSpace::AUTO, metadata);

        /*Update view for this modification*/
        v.second->setWidth(w);
        v.second->setHeight(h);
        v.second->setImagePath(sstream.str());
      }
    }
  }

  /*Order views by their image names (without path and extension to make sure we handle rotated images) */
  std::vector<std::shared_ptr<sfmData::View>> viewsOrderedByName;
  for (auto & viewIt: sfmData.getViews()) {
    viewsOrderedByName.push_back(viewIt.second);
  }
  std::sort(viewsOrderedByName.begin(), viewsOrderedByName.end(), [](const std::shared_ptr<sfmData::View> & a, const std::shared_ptr<sfmData::View> & b) -> bool { 
    if (a == nullptr || b == nullptr) return true;
    
    boost::filesystem::path path_a(a->getImagePath());
    boost::filesystem::path path_b(b->getImagePath());

    return (path_a.stem().string() < path_b.stem().string());
  });



  /*
  Make sure all images have same aperture
  */
  float aperture = viewsOrderedByName[0]->getMetadataAperture();
  for (auto & view : viewsOrderedByName) {
    
    if (view->getMetadataAperture() != aperture) {
      ALICEVISION_LOG_ERROR("Different apertures amongst the dataset");
      return EXIT_FAILURE;
    }
  }


  /*Make groups*/
  std::vector<std::vector<std::shared_ptr<sfmData::View>>> groupedViews;
  std::vector<std::shared_ptr<sfmData::View>> group;
  for (auto & view : viewsOrderedByName) {
    
    group.push_back(view);
    if (group.size() == groupSize) {
      groupedViews.push_back(group);
      group.clear();
    }
  }

  std::vector<std::shared_ptr<sfmData::View>> targetViews;
  for (auto & group : groupedViews) {
    /*Sort all images by exposure time*/
    std::sort(group.begin(), group.end(), [](const std::shared_ptr<sfmData::View> & a, const std::shared_ptr<sfmData::View> & b) -> bool { 
      if (a == nullptr || b == nullptr) return true;
      return (a->getMetadataShutter() < b->getMetadataShutter());
    });

    /*Target views are the middle exposed views*/
    int middleIndex = group.size() / 2;

    /*If odd size, choose the more exposed view*/
    if (group.size() % 2 && group.size() > 1) {
      middleIndex++;
    }

    targetViews.push_back(group[middleIndex]);
  }

  /*Build exposure times table*/
  std::vector<std::vector<float>> groupedExposures;
  for (int i = 0; i < groupedViews.size(); i++) {
    const std::vector<std::shared_ptr<sfmData::View>> & group = groupedViews[i];
    std::vector<float> exposures;

    for (int j = 0; j < group.size(); j++) {
      float etime = group[j]->getMetadataShutter();
      exposures.push_back(etime);
    }
    groupedExposures.push_back(exposures); 
  }

  /*Build table of file names*/
  std::vector<std::vector<std::string>> groupedFilenames;
  for (int i = 0; i < groupedViews.size(); i++) {
    const std::vector<std::shared_ptr<sfmData::View>> & group = groupedViews[i];

    std::vector<std::string> filenames;

    for (int j = 0; j < group.size(); j++) {

      filenames.push_back(group[j]->getImagePath());
    }

    groupedFilenames.push_back(filenames);
  }
  

  size_t channelQuantization = std::pow(2, 10);
  hdr::rgbCurve response(channelQuantization);
  hdr::rgbCurve calibrationWeight(channelQuantization);
  hdr::rgbCurve fusionWeight(channelQuantization);

  const float lambda = channelQuantization * 1.f;
  const int nbPoints = 10000;
  bool fisheye = false;
  fusionWeight.setFunction(hdr::EFunctionType::GAUSSIAN);
  calibrationWeight.setTriangular();

  hdr::DebevecCalibrate calibration;
  if (!calibration.process(groupedFilenames, channelQuantization, groupedExposures, nbPoints, fisheye, calibrationWeight, lambda, response)) {
     ALICEVISION_LOG_ERROR("Calibration failed.");
    return EXIT_FAILURE;
  }

  ALICEVISION_LOG_INFO("Calibration done.");

  response.exponential();
  response.scale();


  sfmData::SfMData outputSfm;
  sfmData::Views & vs = outputSfm.getViews();
  outputSfm.getIntrinsics() = sfmData.getIntrinsics();

  for(int g = 0; g < groupedFilenames.size(); ++g)
  {
    std::vector<image::Image<image::RGBfColor>> images(groupSize);
    std::shared_ptr<sfmData::View> targetView = targetViews[g];
    if (targetView == nullptr) {
      ALICEVISION_LOG_ERROR("Null view");
      return EXIT_FAILURE;
    }

    /* Load all images of the group */
    for (int i = 0; i < groupSize; i++) {
      ALICEVISION_LOG_INFO("Load " << groupedFilenames[g][i]);
      image::readImage(groupedFilenames[g][i], images[i], image::EImageColorSpace::SRGB);
    }

    

    /**/
    hdr::hdrMerge merge;
    float targetTime = targetView->getMetadataShutter();
    image::Image<image::RGBfColor> HDRimage(targetView->getWidth(), targetView->getHeight(), false);
    merge.process(images, groupedExposures[g], fusionWeight, response, HDRimage, targetTime, false, clampedValueCorrection);

    /*Output image file path*/
    std::string hdr_output_path;
    std::stringstream  sstream;
    sstream << output_path << "/" << "hdr_" << std::setfill('0') << std::setw(4) << g << ".exr";

    /*Write an image with parameters from the target view*/
    oiio::ParamValueList targetMetadata = image::readImageMetadata(targetView->getImagePath());
    image::writeImage(sstream.str(), HDRimage, image::EImageColorSpace::AUTO, targetMetadata);

    targetViews[g]->setImagePath(sstream.str());
    vs[targetViews[g]->getViewId()] = targetViews[g];
  }

  /*
  Save output sfmdata
  */
  if (!sfmDataIO::Save(outputSfm, sfmOutputDataFilename, sfmDataIO::ESfMData(sfmDataIO::VIEWS|sfmDataIO::INTRINSICS)))
  {
    ALICEVISION_LOG_ERROR("Can not save output sfm file at " << sfmOutputDataFilename);
    return EXIT_FAILURE;
  }  

  return EXIT_SUCCESS;
}