// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/image/all.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/config.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/progress.hpp>

#include <stdlib.h>
#include <stdio.h>
#include <cmath>
#include <vector>
#include <set>
#include <iterator>
#include <iomanip>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 2
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;
using namespace aliceVision::camera;
using namespace aliceVision::geometry;
using namespace aliceVision::image;
using namespace aliceVision::sfmData;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

bool prepareDenseScene(const SfMData& sfmData,
                       const std::vector<std::string>& imagesFolders,
                       int beginIndex,
                       int endIndex,
                       const std::string& outFolder,
                       image::EImageFileType outputFileType,
                       bool saveMetadata,
                       bool saveMatricesFiles)
{
  // defined view Ids
  std::set<IndexT> viewIds;

  sfmData::Views::const_iterator itViewBegin = sfmData.getViews().begin();
  sfmData::Views::const_iterator itViewEnd = sfmData.getViews().end();

  if(endIndex > 0)
  {
    itViewEnd = itViewBegin;
    std::advance(itViewEnd, endIndex);
  }

  std::advance(itViewBegin, (beginIndex < 0) ? 0 : beginIndex);

  // export valid views as projective cameras
  for(auto it = itViewBegin; it != itViewEnd; ++it)
  {
    const View* view = it->second.get();
    if (!sfmData.isPoseAndIntrinsicDefined(view))
      continue;
    viewIds.insert(view->getViewId());
  }

  if((outputFileType != image::EImageFileType::EXR) && saveMetadata)
    ALICEVISION_LOG_WARNING("Cannot save informations in images metadata.\n"
                            "Choose '.exr' file type if you want AliceVision custom metadata");

  // export data
  boost::progress_display progressBar(viewIds.size(), std::cout, "Exporting Scene Undistorted Images\n");

#pragma omp parallel for num_threads(3)
  for(int i = 0; i < viewIds.size(); ++i)
  {
    auto itView = viewIds.begin();
    std::advance(itView, i);

    const IndexT viewId = *itView;
    const View* view = sfmData.getViews().at(viewId).get();

    Intrinsics::const_iterator iterIntrinsic = sfmData.getIntrinsics().find(view->getIntrinsicId());

    //we have a valid view with a corresponding camera & pose
    const std::string baseFilename = std::to_string(viewId);

    oiio::ParamValueList metadata = image::getMetadataFromMap(view->getMetadata());

    // export camera
    if(saveMetadata || saveMatricesFiles)
    {
      // get camera pose / projection
      const Pose3 pose = sfmData.getPose(*view).getTransform();
      Mat34 P = iterIntrinsic->second.get()->get_projective_equivalent(pose);

      // get camera intrinsics matrices
      const Mat3 K = dynamic_cast<const Pinhole*>(sfmData.getIntrinsicPtr(view->getIntrinsicId()))->K();
      const Mat3& R = pose.rotation();
      const Vec3& t = pose.translation();

      if(saveMatricesFiles)
      {
        std::ofstream fileP((fs::path(outFolder) / (baseFilename + "_P.txt")).string());
        fileP << std::setprecision(10)
             << P(0, 0) << " " << P(0, 1) << " " << P(0, 2) << " " << P(0, 3) << "\n"
             << P(1, 0) << " " << P(1, 1) << " " << P(1, 2) << " " << P(1, 3) << "\n"
             << P(2, 0) << " " << P(2, 1) << " " << P(2, 2) << " " << P(2, 3) << "\n";
        fileP.close();

        std::ofstream fileKRt((fs::path(outFolder) / (baseFilename + "_KRt.txt")).string());
        fileKRt << std::setprecision(10)
             << K(0, 0) << " " << K(0, 1) << " " << K(0, 2) << "\n"
             << K(1, 0) << " " << K(1, 1) << " " << K(1, 2) << "\n"
             << K(2, 0) << " " << K(2, 1) << " " << K(2, 2) << "\n"
             << "\n"
             << R(0, 0) << " " << R(0, 1) << " " << R(0, 2) << "\n"
             << R(1, 0) << " " << R(1, 1) << " " << R(1, 2) << "\n"
             << R(2, 0) << " " << R(2, 1) << " " << R(2, 2) << "\n"
             << "\n"
             << t(0) << " " << t(1) << " " << t(2) << "\n";
        fileKRt.close();
      }

      if(saveMetadata)
      {
        // convert to 44 matix
        Mat4 projectionMatrix;
        projectionMatrix << P(0, 0), P(0, 1), P(0, 2), P(0, 3),
                            P(1, 0), P(1, 1), P(1, 2), P(1, 3),
                            P(2, 0), P(2, 1), P(2, 2), P(2, 3),
                                  0,       0,       0,       1;

        // convert matrices to rowMajor
        std::vector<double> vP(projectionMatrix.size());
        std::vector<double> vK(K.size());
        std::vector<double> vR(R.size());

        typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> RowMatrixXd;
        Eigen::Map<RowMatrixXd>(vP.data(), projectionMatrix.rows(), projectionMatrix.cols()) = projectionMatrix;
        Eigen::Map<RowMatrixXd>(vK.data(), K.rows(), K.cols()) = K;
        Eigen::Map<RowMatrixXd>(vR.data(), R.rows(), R.cols()) = R;

        // add metadata
        metadata.push_back(oiio::ParamValue("AliceVision:downscale", 1));
        metadata.push_back(oiio::ParamValue("AliceVision:P", oiio::TypeDesc(oiio::TypeDesc::DOUBLE, oiio::TypeDesc::MATRIX44), 1, vP.data()));
        metadata.push_back(oiio::ParamValue("AliceVision:K", oiio::TypeDesc(oiio::TypeDesc::DOUBLE, oiio::TypeDesc::MATRIX33), 1, vK.data()));
        metadata.push_back(oiio::ParamValue("AliceVision:R", oiio::TypeDesc(oiio::TypeDesc::DOUBLE, oiio::TypeDesc::MATRIX33), 1, vR.data()));
        metadata.push_back(oiio::ParamValue("AliceVision:t", oiio::TypeDesc(oiio::TypeDesc::DOUBLE, oiio::TypeDesc::VEC3), 1, t.data()));
      }
    }

    // export undistort image
    {
      std::string srcImage = view->getImagePath();

      if(!imagesFolders.empty())
      {
        bool found = false;
        for(const std::string& folder : imagesFolders)
        {
          const fs::recursive_directory_iterator end;
          const auto findIt = std::find_if(fs::recursive_directory_iterator(folder), end,
                                   [&view](const fs::directory_entry& e) {
                                      return (e.path().stem() == std::to_string(view->getViewId()) ||
                                              e.path().stem() == fs::path(view->getImagePath()).stem());});

          if(findIt != end)
          {
            srcImage = (fs::path(folder) / (findIt->path().stem().string() + findIt->path().extension().string())).string();
            found = true;
            break;
          }
        }

        if(!found)
          throw std::runtime_error("Cannot find view " + std::to_string(view->getViewId()) + " image file in given folder(s)");
      }


      const std::string dstColorImage = (fs::path(outFolder) / (baseFilename + "." + image::EImageFileType_enumToString(outputFileType))).string();
      const IntrinsicBase* cam = iterIntrinsic->second.get();
      Image<RGBfColor> image, image_ud;

      readImage(srcImage, image, image::EImageColorSpace::LINEAR);
      
      // undistort
      if(cam->isValid() && cam->have_disto())
      {
        // undistort the image and save it
        UndistortImage(image, cam, image_ud, FBLACK);
        writeImage(dstColorImage, image_ud, image::EImageColorSpace::AUTO, metadata);
      }
      else
      {
        writeImage(dstColorImage, image, image::EImageColorSpace::AUTO, metadata);
      }
    }

    #pragma omp critical
    ++progressBar;
  }

  return true;
}

int main(int argc, char *argv[])
{
  // command-line parameters

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string sfmDataFilename;
  std::string outFolder;
  std::string outImageFileTypeName = image::EImageFileType_enumToString(image::EImageFileType::EXR);
  std::vector<std::string> imagesFolders;
  int rangeStart = -1;
  int rangeSize = 1;
  bool saveMetadata = true;
  bool saveMatricesTxtFiles = false;

  po::options_description allParams("AliceVision prepareDenseScene");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
      "SfMData file.")
    ("output,o", po::value<std::string>(&outFolder)->required(),
      "Output folder.");

  po::options_description optionalParams("Optional parameters");
  optionalParams.add_options()
    ("imagesFolders",  po::value<std::vector<std::string>>(&imagesFolders)->multitoken(),
      "Use images from specific folder(s) instead of those specify in the SfMData file.\n"
      "Filename should be the same or the image uid.")
    ("outputFileType", po::value<std::string>(&outImageFileTypeName)->default_value(outImageFileTypeName),
        image::EImageFileType_informations().c_str())
    ("saveMetadata", po::value<bool>(&saveMetadata)->default_value(saveMetadata),
      "Save projections and intrinsics information in images metadata.")
    ("saveMatricesTxtFiles", po::value<bool>(&saveMatricesTxtFiles)->default_value(saveMatricesTxtFiles),
      "Save projections and intrinsics information in text files.")
    ("rangeStart", po::value<int>(&rangeStart)->default_value(rangeStart),
      "Range image index start.")
    ("rangeSize", po::value<int>(&rangeSize)->default_value(rangeSize),
      "Range size.");

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

  // set output file type
  image::EImageFileType outputFileType = image::EImageFileType_stringToEnum(outImageFileTypeName);

  // Create output dir
  if(!fs::exists(outFolder))
    fs::create_directory(outFolder);

  // Read the input SfM scene
  SfMData sfmData;
  if(!sfmDataIO::Load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
  {
    ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' cannot be read.");
    return EXIT_FAILURE;
  }

  int rangeEnd = sfmData.getViews().size();

  // set range
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

    rangeEnd = rangeStart + rangeSize;
  }
  else
  {
    rangeStart = 0;
  }

  // export
  if(prepareDenseScene(sfmData, imagesFolders, rangeStart, rangeEnd, outFolder, outputFileType, saveMetadata, saveMatricesTxtFiles))
    return EXIT_SUCCESS;

  return EXIT_FAILURE;
}
