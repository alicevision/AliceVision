// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfm/sfm.hpp>
#include <aliceVision/sfm/viewIO.hpp>
#include <aliceVision/sfm/sfmDataIO_json.hpp>
#include <aliceVision/exif/sensorWidthDatabase/parseDatabase.hpp>
#include <aliceVision/stl/split.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>

#include <dependencies/stlplus3/filesystemSimplified/file_system.hpp>

#include <boost/program_options.hpp>

#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>
#include <string>
#include <vector>
#include <cstdlib>

using namespace aliceVision;
using namespace aliceVision::sfm;

namespace po = boost::program_options;

/**
 * @brief Check that Kmatrix is a string like "f;0;ppx;0;f;ppy;0;0;1"
 * @param[in] Kmatrix
 * @param[out] focal
 * @param[out] ppx
 * @param[out] ppy
 * @return true if the string is correct
 */
bool checkIntrinsicStringValidity(const std::string& Kmatrix,
                                  double& focal,
                                  double& ppx,
                                  double& ppy)
{
  std::vector<std::string> vec_str;
  stl::split(Kmatrix, ";", vec_str);
  if (vec_str.size() != 9)
  {
    ALICEVISION_LOG_ERROR("Error: In K matrix string, missing ';' character");
    return false;
  }

  // Check that all K matrix value are valid numbers
  for (size_t i = 0; i < vec_str.size(); ++i)
  {
    double readvalue = 0.0;
    std::stringstream ss;
    ss.str(vec_str[i]);
    if(!(ss >> readvalue))
    {
      ALICEVISION_LOG_ERROR("Error: In K matrix string, used an invalid not a number character");
      return false;
    }
    if (i==0) focal = readvalue;
    if (i==2) ppx = readvalue;
    if (i==5) ppy = readvalue;
  }
  return true;
}

/**
 * @brief Recursively list all files from a folder with a specific extension
 * @param[in] folderOrFile A file or foder path
 * @param[in] extensions An extensions filter
 * @param[out] outFiles A list of output image paths
 * @return true if folderOrFile have been load successfully
 */
bool listFiles(const std::string& folderOrFile,
               const std::vector<std::string>& extensions,
               std::vector<std::string>& resources)
{
  if(stlplus::is_file(folderOrFile))
  {
    std::string fileExtension = stlplus::extension_part(folderOrFile);
    std::transform(fileExtension.begin(), fileExtension.end(), fileExtension.begin(), ::tolower);
    for(const std::string& extension: extensions)
    {
      if(fileExtension == extension)
      {
        resources.push_back(folderOrFile);
        return true;
      }
    }
  }
  else if(stlplus::is_folder(folderOrFile))
  {
    // list all files of the folder
    const std::vector<std::string> allFiles = stlplus::folder_all(folderOrFile);
    if(allFiles.empty())
    {
      ALICEVISION_LOG_ERROR("Error: Folder '" << stlplus::filename_part(folderOrFile) <<"' is empty.");
      return false;
    }

    for(const std::string& item: allFiles)
    {
      const std::string itemPath = stlplus::create_filespec(folderOrFile, item);
      if(!listFiles(itemPath, extensions, resources))
        return false;
    }
  }
  else
  {
    ALICEVISION_LOG_ERROR("Error: '" << folderOrFile << "' is not a valid folder or file path.");
    return false;
  }
  return true;
}


/**
 * @brief Create the description of an input image dataset for AliceVision toolsuite
 * - Export a SfMData file with View & Intrinsic data
 */
int main(int argc, char **argv)
{
  // command-line parameters

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string sfmFilePath;
  std::string imageFolder;
  std::string sensorDatabasePath;
  std::string outputFilePath;

  // user optional parameters

  std::string defaultIntrinsicKMatrix;
  std::string defaultCameraModelName;
  double defaultFocalLengthPixel = -1.0;
  double defaultSensorWidth = -1.0;
  int groupCameraModel = 1;

  po::options_description allParams("AliceVision cameraInit");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmFilePath)->default_value(sfmFilePath),
      "a SfMData file (*.sfm).")
    ("imageFolder", po::value<std::string>(&imageFolder)->default_value(imageFolder),
      "Input images folder.")
    ("sensorDatabase,s", po::value<std::string>(&sensorDatabasePath)->required(),
      "Camera sensor width database path.")
    ("output,o", po::value<std::string>(&outputFilePath)->default_value("cameraInit.sfm"),
      "Output file path for the new SfMData file");

  po::options_description optionalParams("Optional parameters");
  optionalParams.add_options()
    ("defaultFocalLengthPix", po::value<double>(&defaultFocalLengthPixel)->default_value(defaultFocalLengthPixel),
      "Focal length in pixels.")
    ("defaultSensorWidth", po::value<double>(&defaultSensorWidth)->default_value(defaultSensorWidth),
      "Sensor width in mm.")
    ("defaultIntrinsic", po::value<std::string>(&defaultIntrinsicKMatrix)->default_value(defaultIntrinsicKMatrix),
      "Intrinsics Kmatrix \"f;0;ppx;0;f;ppy;0;0;1\".")
    ("defaultCameraModel", po::value<std::string>(&defaultCameraModelName)->default_value(defaultCameraModelName),
      "Camera model type (pinhole, radial1, radial3, brown, fisheye4).")
    ("groupCameraModel", po::value<int>(&groupCameraModel)->default_value(groupCameraModel),
      "* 0: each view have its own camera intrinsic parameters\n"
      "* 1: view share camera intrinsic parameters based on metadata, if no metadata each view has its own camera intrinsic parameters\n"
      "* 2: view share camera intrinsic parameters based on metadata, if no metadata they are grouped by folder\n");

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

  // set user camera model
  camera::EINTRINSIC defaultCameraModel = camera::EINTRINSIC::PINHOLE_CAMERA_START;
  if(!defaultCameraModelName.empty())
      defaultCameraModel = camera::EINTRINSIC_stringToEnum(defaultCameraModelName);

  // check user choose at least one input option
  if(imageFolder.empty() && sfmFilePath.empty())
  {
    ALICEVISION_LOG_ERROR("Error: Program need -i or -f option");
    return EXIT_FAILURE;
  }

  // check user don't choose both input options
  if(!imageFolder.empty() && !sfmFilePath.empty())
  {
    ALICEVISION_LOG_ERROR("Error: Cannot combine -i and -f options");
    return EXIT_FAILURE;
  }

  // check input folder
  if(!imageFolder.empty() && !stlplus::folder_exists(imageFolder))
  {
    ALICEVISION_LOG_ERROR("Error: The input folder doesn't exist");
    return EXIT_FAILURE;
  }

  // check sfm file
  if(!sfmFilePath.empty() && !stlplus::file_exists(sfmFilePath))
  {
    ALICEVISION_LOG_ERROR("Error: The input sfm file doesn't exist");
    return EXIT_FAILURE;
  }

  // check output  string
  if(outputFilePath.empty())
  {
    ALICEVISION_LOG_ERROR("Error: Invalid output");
    return EXIT_FAILURE;
  }

  // check if output folder exists, if no create it
  {
    const std::string outputFolderPart = stlplus::folder_part(outputFilePath);

    if(!outputFolderPart.empty() && !stlplus::folder_exists(outputFolderPart))
    {
      if(!stlplus::folder_create(outputFolderPart))
      {
        ALICEVISION_LOG_ERROR("Error: Cannot create output folder");
        return EXIT_FAILURE;
      }
    }
  }

  // check user don't combine focal and K matrix
  if(!defaultIntrinsicKMatrix.empty() && defaultFocalLengthPixel != -1.0)
  {
    ALICEVISION_LOG_ERROR("Error: Cannot combine --defaultIntrinsic and --defaultFocalLengthPix options");
    return EXIT_FAILURE;
  }

  // check if K matrix is valid
  {
    double ppx = -1.0;
    double ppy = -1.0;
    if(!defaultIntrinsicKMatrix.empty() && !checkIntrinsicStringValidity(defaultIntrinsicKMatrix, defaultFocalLengthPixel, ppx, ppy))
    {
      ALICEVISION_LOG_ERROR("Error: --defaultIntrinsic Invalid K matrix input");
      return EXIT_FAILURE;
    }
  }

  // check sensor database
  std::vector<exif::sensordb::Datasheet> sensorDatabase;
  if(!sensorDatabasePath.empty())
  {
    if(!exif::sensordb::parseDatabase(sensorDatabasePath, sensorDatabase))
    {
      ALICEVISION_LOG_ERROR("Error: Invalid input database '" << sensorDatabasePath << "', please specify a valid file.");
      return EXIT_FAILURE;
    }
  }

  // use current time as seed for random generator for intrinsic Id without metadata
  std::srand(std::time(0));

  std::vector<std::string> noMetadataImagePaths;
  std::map<std::pair<std::string, std::string>, std::string> unknownSensors; // key (make,model) value (first imagePath)

  SfMData sfmData;

  // number of views with an initialized intrinsic
  std::size_t completeViewCount = 0;

  // load known informations
  if(imageFolder.empty())
  {
    // fill SfMData from the JSON file
    sfm::loadJSON(sfmData, sfmFilePath, ESfMData(VIEWS|INTRINSICS|EXTRINSICS), true);
  }
  else
  {
    // fill SfMData with the images in the input folder
    Views& views = sfmData.GetViews();
    std::vector<std::string> imagePaths;

    if(listFiles(imageFolder, {"jpg", "jpeg"},  imagePaths))
    {
      for(const auto& imagePath : imagePaths)
      {
        View view = View(imagePath);
        sfm::updateIncompleteView(view);
        views.emplace(view.getViewId(), std::make_shared<View>(view));
      }
    }
    else
      return EXIT_FAILURE;
  }

  if(sfmData.GetViews().empty())
  {
    ALICEVISION_LOG_ERROR("Error: Can't find views in input.");
    return EXIT_FAILURE;
  }

  // create missing intrinsics
  for(auto& viewPair : sfmData.GetViews())
  {
    View& view = *(viewPair.second);
    IndexT intrinsicId = view.getIntrinsicId();

    if(intrinsicId != UndefinedIndexT)
    {
      std::shared_ptr<camera::IntrinsicBase> intrinsic = sfmData.GetIntrinsicSharedPtr(view.getIntrinsicId());
      if(intrinsic != nullptr)
      {
        if(intrinsic->initialFocalLengthPix() > 0)
          ++completeViewCount;
        continue;
      }
    }

    bool hasCameraMetadata = (view.hasMetadata("camera_make") && view.hasMetadata("camera_model"));
    double sensorWidth = -1;

    if(hasCameraMetadata)
    {
      aliceVision::exif::sensordb::Datasheet datasheet;
      if(getInfo(view.getMetadata("camera_make"), view.getMetadata("camera_model"), sensorDatabase, datasheet))
        sensorWidth = datasheet._sensorSize;
      else
      {
        unknownSensors.emplace(std::make_pair(view.getMetadata("camera_make"),view.getMetadata("camera_model")), view.getImagePath()); // will throw an error message and exit program
        continue;
      }

    }
    else
      noMetadataImagePaths.emplace_back(view.getImagePath());

    std::shared_ptr<camera::IntrinsicBase> intrinsic = getViewIntrinsic(view, sensorWidth, defaultFocalLengthPixel, defaultCameraModel);

    if(intrinsic->initialFocalLengthPix() > 0)
      ++completeViewCount;

    // override serial number if necessary
    if(!hasCameraMetadata)
    {
      if(groupCameraModel == 2)
      {
        // when we have no metadata at all, we create one intrinsic group per folder.
        // the use case is images extracted from a video without metadata and assumes fixed intrinsics in the video.
        intrinsic->setSerialNumber(stlplus::folder_part(view.getImagePath()));
      }

      if(view.isPartOfRig())
      {
        // when we have no metadata for rig images, we create an intrinsic per camera.
        intrinsic->setSerialNumber("no_metadata_rig_" + std::to_string(view.getRigId()) + "_" + std::to_string(view.getSubPoseId()));
      }
    }

    // create intrinsic id
    // group camera that share common properties (leads to more faster & stable BA).
    if(intrinsicId == UndefinedIndexT)
      intrinsicId = intrinsic->hashValue();

    // don't group camera that share common properties
    if(!groupCameraModel)
      intrinsicId = std::rand(); // random number

    view.setIntrinsicId(intrinsicId);
    sfmData.GetIntrinsics().emplace(intrinsicId, intrinsic);
  }

  if(!noMetadataImagePaths.empty())
  {
    ALICEVISION_LOG_WARNING("Warning: No metadata in image(s) :");
    for(const auto& imagePath : noMetadataImagePaths)
      ALICEVISION_LOG_WARNING("\t- '" << imagePath << "'");
  }

  if(!unknownSensors.empty())
  {
    ALICEVISION_LOG_ERROR("Error: Sensor width doesn't exist in the database for image(s) :");
    for(const auto& unknownSensor : unknownSensors)
      ALICEVISION_LOG_ERROR("image: '" << stlplus::filename_part(unknownSensor.second) << "'" << std::endl
                        << "\t- camera brand: " << unknownSensor.first.first <<  std::endl
                        << "\t- camera model: " << unknownSensor.first.second <<  std::endl);
    ALICEVISION_LOG_ERROR("Please add camera model(s) and sensor width(s) in the database." << std::endl);
    return EXIT_FAILURE;
  }

  if(completeViewCount < 2)
  {
    ALICEVISION_LOG_ERROR("Error: At least two images should have an initialized intrinsic." << std::endl
                          << "Check your input images metadata (brand, model, focal length, ...), more should be set and correct." << std::endl);
    return EXIT_FAILURE;
  }

  // store SfMData views & intrinsic data
  if (!Save(sfmData, outputFilePath, ESfMData(VIEWS|INTRINSICS|EXTRINSICS)))
  {
    return EXIT_FAILURE;
  }

  // print report
  ALICEVISION_LOG_INFO("CameraInit report:" << std::endl
                   << "\t- # view(s) listed in SfMData: " << sfmData.GetViews().size() << std::endl
                   << "\t- # view(s) with an initialized intrinsic listed in SfMData: " << completeViewCount << std::endl
                   << "\t- # intrinsic(s) listed in SfMData: " << sfmData.GetIntrinsics().size());

  return EXIT_SUCCESS;
}
