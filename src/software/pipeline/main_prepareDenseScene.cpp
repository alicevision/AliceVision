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
#include <aliceVision/system/main.hpp>
#include <aliceVision/config.hpp>
#include <aliceVision/sfmDataIO/viewIO.hpp>

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
using namespace aliceVision::sfmDataIO;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

template <class ImageT, class MaskFuncT>
void process(const std::string &dstColorImage, const IntrinsicBase* cam, const oiio::ParamValueList & metadata, const std::string & srcImage, bool evCorrection, float exposureCompensation, MaskFuncT && maskFunc)
{
  ImageT image, image_ud;
  readImage(srcImage, image, image::EImageColorSpace::LINEAR);

  //exposure correction
  if(evCorrection)
  {
      for(int pix = 0; pix < image.Width() * image.Height(); ++pix)
      {
          image(pix) = image(pix) * exposureCompensation;
      }
  }

  // mask
  maskFunc(image);

  // undistort
  if(cam->isValid() && cam->hasDistortion())
  {
    // undistort the image and save it
    using Pix = typename ImageT::Tpixel;
    Pix pixZero(Pix::Zero());
    UndistortImage(image, cam, image_ud, pixZero);
    writeImage(dstColorImage, image_ud, image::EImageColorSpace::AUTO, metadata);
  }
  else
  {
    writeImage(dstColorImage, image, image::EImageColorSpace::AUTO, metadata);
  }
}

bool tryLoadMask(image::Image<unsigned char>* mask, const std::vector<std::string>& masksFolders, const IndexT viewId, const std::string & srcImage)
{
  for(const auto & masksFolder_str : masksFolders)
  {
    if(!masksFolder_str.empty() && fs::exists(masksFolder_str))
    {
      const auto masksFolder = fs::path(masksFolder_str);
      const auto idMaskPath = masksFolder / fs::path(std::to_string(viewId)).replace_extension("png");
      const auto nameMaskPath = masksFolder / fs::path(srcImage).filename().replace_extension("png");

      if(fs::exists(idMaskPath))
      {
        image::readImage(idMaskPath.string(), *mask, image::EImageColorSpace::LINEAR);
        return true;
      }
      else if(fs::exists(nameMaskPath))
      {
        image::readImage(nameMaskPath.string(), *mask, image::EImageColorSpace::LINEAR);
        return true;
      }
    }
  }
  return false;
}

bool prepareDenseScene(const SfMData& sfmData,
                       const std::vector<std::string>& imagesFolders,
                       const std::vector<std::string>& masksFolders,
                       int beginIndex,
                       int endIndex,
                       const std::string& outFolder,
                       image::EImageFileType outputFileType,
                       bool saveMetadata,
                       bool saveMatricesFiles,
                       bool evCorrection)
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

  // for exposure correction
  const float medianCameraExposure = sfmData.getMedianCameraExposureSetting();
  ALICEVISION_LOG_INFO("Median Camera Exposure: " << medianCameraExposure << ", Median EV: " << std::log2(1.0f/medianCameraExposure));

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

    // get metadata from source image to be sure we get all metadata. We don't use the metadatas from the Views inside the SfMData to avoid type conversion problems with string maps.
    std::string srcImage = view->getImagePath();
    oiio::ParamValueList metadata = image::readImageMetadata(srcImage);

    // export camera
    if(saveMetadata || saveMatricesFiles)
    {
      // get camera pose / projection
      const Pose3 pose = sfmData.getPose(*view).getTransform();

      std::shared_ptr<camera::IntrinsicBase> cam = iterIntrinsic->second;
      std::shared_ptr<camera::Pinhole> camPinHole = std::dynamic_pointer_cast<camera::Pinhole>(cam);
      if (!camPinHole) {
        ALICEVISION_LOG_ERROR("Camera is not pinhole in filter");
        continue;
      }

      Mat34 P = camPinHole->getProjectiveEquivalent(pose);

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
      if(!imagesFolders.empty())
      {
          std::vector<std::string> paths = sfmDataIO::viewPathsFromFolders(*view, imagesFolders);

          // if path was not found
          if(paths.empty())
          {
              throw std::runtime_error("Cannot find view '" + std::to_string(view->getViewId()) + "' image file in given folder(s)");
          }
          else if(paths.size() > 1)
          {
              throw std::runtime_error( "Ambiguous case: Multiple source image files found in given folder(s) for the view '" + 
                  std::to_string(view->getViewId()) + "'.");
          }

          srcImage = paths[0];
      }
      const std::string dstColorImage = (fs::path(outFolder) / (baseFilename + "." + image::EImageFileType_enumToString(outputFileType))).string();
      const IntrinsicBase* cam = iterIntrinsic->second.get();

      // add exposure values to images metadata
      float cameraExposure = view->getCameraExposureSetting();
      float ev = std::log2(1.0 / cameraExposure);
      float exposureCompensation = medianCameraExposure / cameraExposure;
      metadata.push_back(oiio::ParamValue("AliceVision:EV", ev));
      metadata.push_back(oiio::ParamValue("AliceVision:EVComp", exposureCompensation));

      if(evCorrection)
      {
          ALICEVISION_LOG_INFO("image " << viewId << ", exposure: " << cameraExposure << ", Ev " << ev << " Ev compensation: " + std::to_string(exposureCompensation));
      }

      image::Image<unsigned char> mask;
      if(tryLoadMask(&mask, masksFolders, viewId, srcImage))
      {
        process<Image<RGBAfColor>>(dstColorImage, cam, metadata, srcImage, evCorrection, exposureCompensation, [&mask] (Image<RGBAfColor> & image)
        {
          if(image.Width() * image.Height() != mask.Width() * mask.Height())
          {
            ALICEVISION_LOG_WARNING("Invalid image mask size: mask is ignored.");
            return;
          }

          for(int pix = 0; pix < image.Width() * image.Height(); ++pix)
          {
            const bool masked = (mask(pix) == 0);
            image(pix).a() = masked ? 0.f : 1.f;
          }
        });
      }
      else
      {
        const auto noMaskingFunc = [] (Image<RGBAfColor> & image) {};
        process<Image<RGBAfColor>>(dstColorImage, cam, metadata, srcImage, evCorrection, exposureCompensation, noMaskingFunc);
      }
    }

    #pragma omp critical
    ++progressBar;
  }

  return true;
}

int aliceVision_main(int argc, char *argv[])
{
  // command-line parameters

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string sfmDataFilename;
  std::string outFolder;
  std::string outImageFileTypeName = image::EImageFileType_enumToString(image::EImageFileType::EXR);
  std::vector<std::string> imagesFolders;
  std::vector<std::string> masksFolders;
  int rangeStart = -1;
  int rangeSize = 1;
  bool saveMetadata = true;
  bool saveMatricesTxtFiles = false;
  bool evCorrection = false;

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
    ("masksFolders", po::value<std::vector<std::string>>(&masksFolders)->multitoken(),
      "Use masks from specific folder(s).\n"
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
      "Range size.")
    ("evCorrection", po::value<bool>(&evCorrection)->default_value(evCorrection),
      "Correct exposure value.");

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
  if(prepareDenseScene(sfmData, imagesFolders, masksFolders, rangeStart, rangeEnd, outFolder, outputFileType, saveMetadata, saveMatricesTxtFiles, evCorrection))
    return EXIT_SUCCESS;

  return EXIT_FAILURE;
}
