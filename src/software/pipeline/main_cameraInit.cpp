// This file is part of the AliceVision project.
// Copyright (c) 2015 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/jsonIO.hpp>
#include <aliceVision/sfmDataIO/viewIO.hpp>
#include <aliceVision/sensorDB/parseDatabase.hpp>
#include <aliceVision/lensCorrectionProfile/lcp.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/image/io.cpp>
#include <aliceVision/image/dcp.hpp>

#include <boost/atomic/atomic_ref.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>
#include <boost/foreach.hpp>

#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>
#include <string>
#include <vector>
#include <cstdlib>
#include <stdexcept>
#include <cmath>
#include <algorithm>
#include <iterator>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 2
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;
using namespace aliceVision::sfmDataIO;

namespace po = boost::program_options;
namespace fs = boost::filesystem;


/**
 * @brief Recursively list all files from a folder with a specific extension
 * @param[in] folderOrFile A file or folder path
 * @param[in] extensions An extensions filter
 * @param[out] outFiles A list of output image paths
 * @return true if folderOrFile have been load successfully
 */
bool listFiles(const fs::path& folderOrFile,
               const std::vector<std::string>& extensions,
               std::vector<fs::path>& resources)
{
  if(fs::is_regular_file(folderOrFile))
  {
    std::string fileExtension = fs::extension(folderOrFile);
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
  else if(fs::is_directory(folderOrFile))
  {
    // list all files of the folder
    std::vector<std::string> allFiles;

    fs::directory_iterator endItr;
    for(fs::directory_iterator itr(folderOrFile); itr != endItr; ++itr)
      allFiles.push_back(itr->path().string());

    bool hasFile = false;
    for(const std::string& filePath: allFiles)
    {
      if(listFiles(filePath, extensions, resources))
        hasFile = true;
    }
    return hasFile;
  }
  ALICEVISION_LOG_ERROR("'" << folderOrFile << "' is not a valid folder or file path.");
  return false;
}

enum class EGroupCameraFallback {
    GLOBAL,
    FOLDER,
    IMAGE
};

inline std::string EGroupCameraFallback_enumToString(EGroupCameraFallback strategy)
{
  switch(strategy)
  {
    case EGroupCameraFallback::GLOBAL:
      return "global";
    case EGroupCameraFallback::FOLDER:
      return "folder";
    case EGroupCameraFallback::IMAGE:
      return "image";
  }
  throw std::out_of_range("Invalid GroupCameraFallback type Enum: " + std::to_string(int(strategy)));
}

inline EGroupCameraFallback EGroupCameraFallback_stringToEnum(const std::string& strategy)
{
  if(strategy == "global")
    return EGroupCameraFallback::GLOBAL;
  if(strategy == "folder")
    return EGroupCameraFallback::FOLDER;
  if(strategy == "image")
    return EGroupCameraFallback::IMAGE;
  throw std::out_of_range("Invalid GroupCameraFallback type string " + strategy);
}

inline std::ostream& operator<<(std::ostream& os, EGroupCameraFallback s)
{
    return os << EGroupCameraFallback_enumToString(s);
}

inline std::istream& operator>>(std::istream& in, EGroupCameraFallback& s)
{
    std::string token;
    in >> token;
    s = EGroupCameraFallback_stringToEnum(token);
    return in;
}

bool rigHasUniqueFrameIds(const sfmData::SfMData & sfmData)
{
    std::set<std::tuple<IndexT, IndexT, IndexT>> unique_ids;
    for (auto vitem : sfmData.getViews())
    {
        if (!(vitem.second->isPartOfRig())) continue;

        std::tuple<IndexT, IndexT, IndexT> tuple = { vitem.second->getRigId(), vitem.second->getSubPoseId(), vitem.second->getFrameId() };

        if (unique_ids.find(tuple) != unique_ids.end())
        {
            return false;
        }

        unique_ids.insert(tuple);
    }

    return true;
}


/**
 * @brief Create the description of an input image dataset for AliceVision toolsuite
 * - Export a SfMData file with View & Intrinsic data
 */
int aliceVision_main(int argc, char **argv)
{
  // command-line parameters
  std::string sfmFilePath;
  std::string imageFolder;
  std::string sensorDatabasePath;
  std::string lensCorrectionProfileInfo;
  std::string outputFilePath;

  // user optional parameters
  std::string defaultCameraModelName;
  std::string allowedCameraModelsStr = "pinhole,radial1,radial3,brown,fisheye4,fisheye1";
  std::string colorProfileDatabaseDirPath;

  double defaultFocalLength = -1.0;
  double defaultFieldOfView = -1.0;
  double defaultFocalRatio = 1.0;
  double defaultOffsetX = 0.0;
  double defaultOffsetY = 0.0;

  EGroupCameraFallback groupCameraFallback = EGroupCameraFallback::FOLDER;
  EViewIdMethod viewIdMethod = EViewIdMethod::METADATA;
  std::string viewIdRegex = ".*?(\\d+)";

  bool allowSingleView = false;
  bool errorOnMissingColorProfile = true;
  image::ERawColorInterpretation rawColorInterpretation = image::ERawColorInterpretation::LibRawWhiteBalancing;
  bool lensCorrectionProfileSearchIgnoreCameraModel = true;


  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmFilePath)->default_value(sfmFilePath),
      "A SfMData file (*.sfm) [if specified, --imageFolder cannot be used].")
    ("imageFolder", po::value<std::string>(&imageFolder)->default_value(imageFolder),
      "Input images folder [if specified, --input cannot be used].")
    ("output,o", po::value<std::string>(&outputFilePath)->default_value("cameraInit.sfm"),
      "Output file path for the new SfMData file");

  po::options_description optionalParams("Optional parameters");
  optionalParams.add_options()
    ("sensorDatabase,s", po::value<std::string>(&sensorDatabasePath)->default_value(""),
      "Camera sensor width database path.")
    ("colorProfileDatabase,c", po::value<std::string>(&colorProfileDatabaseDirPath)->default_value(""),
      "DNG Color Profiles (DCP) database path.")
    ("lensCorrectionProfileInfo", po::value<std::string>(&lensCorrectionProfileInfo)->default_value(""),
      "Lens Correction Profile filepath or database directory path.")
    ("lensCorrectionProfileSearchIgnoreCameraModel", po::value<bool>(&lensCorrectionProfileSearchIgnoreCameraModel)->default_value(lensCorrectionProfileSearchIgnoreCameraModel),
      "Automatic LCP Search considers only the camera maker and the lens name")
    ("defaultFocalLength", po::value<double>(&defaultFocalLength)->default_value(defaultFocalLength),
      "Focal length in mm. (or '-1' to unset)")
    ("defaultFieldOfView", po::value<double>(&defaultFieldOfView)->default_value(defaultFieldOfView),
      "Empirical value for the field of view in degree. (or '-1' to unset)")
    ("defaultFocalRatio", po::value<double>(&defaultFocalRatio)->default_value(defaultFocalRatio),
      "Ratio between the pixel X size on the sensor and the Y size.")
    ("defaultOffsetX", po::value<double>(&defaultOffsetX)->default_value(defaultOffsetX),
      "default offset from the principal point X coordinate")
    ("defaultOffsetY", po::value<double>(&defaultOffsetY)->default_value(defaultOffsetY),
      "default offset from the principal point Y coordinate")
    ("defaultCameraModel", po::value<std::string>(&defaultCameraModelName)->default_value(defaultCameraModelName),
      "Default camera model type (pinhole, radial1, radial3, brown, fisheye4, fisheye1).")
    ("allowedCameraModels", po::value<std::string>(&allowedCameraModelsStr)->default_value(allowedCameraModelsStr),
      "Permitted model type (pinhole, radial1, radial3, brown, fisheye4, fisheye1).")
    ("groupCameraFallback", po::value<EGroupCameraFallback>(&groupCameraFallback)->default_value(groupCameraFallback),
      std::string("When there is no serial number in the image metadata, we cannot know if the images come from the same camera. "
      "This is problematic for grouping images sharing the same internal camera settings and we have to decide on a fallback strategy:\n"
      " * " + EGroupCameraFallback_enumToString(EGroupCameraFallback::GLOBAL) + ": all images may come from a single device (make/model/focal will still be a differentiator).\n"
      " * " + EGroupCameraFallback_enumToString(EGroupCameraFallback::FOLDER) + ": different folders will be considered as different devices\n"
      " * " + EGroupCameraFallback_enumToString(EGroupCameraFallback::IMAGE) + ": consider that each image has different internal camera parameters").c_str())
    ("viewIdMethod", po::value<EViewIdMethod>(&viewIdMethod)->default_value(viewIdMethod),
      std::string("Allows to choose the way the viewID is generated:\n"
      " * " + EViewIdMethod_enumToString(EViewIdMethod::METADATA) + ": Generate viewId from image metadata.\n"
      " * " + EViewIdMethod_enumToString(EViewIdMethod::FILENAME) + ": Generate viewId from file names using regex.") .c_str())
    ("viewIdRegex", po::value<std::string>(&viewIdRegex)->default_value(viewIdRegex),
      "Regex used to catch number used as viewId in filename.")
    ("rawColorInterpretation", po::value<image::ERawColorInterpretation>(&rawColorInterpretation)->default_value(rawColorInterpretation),
      ("RAW color interpretation: " + image::ERawColorInterpretation_informations()).c_str())
    ("errorOnMissingColorProfile", po::value<bool>(&errorOnMissingColorProfile)->default_value(errorOnMissingColorProfile),
      "Rise an error if a DCP color profiles database is specified but no DCP file matches with the camera model (maker+name) extracted from metadata (Only for raw images)")
    ("allowSingleView", po::value<bool>(&allowSingleView)->default_value(allowSingleView),
      "Allow the program to process a single view.\n"
      "Warning: if a single view is process, the output file can't be use in many other programs.");

  CmdLine cmdline("AliceVision cameraInit");
  cmdline.add(requiredParams);
  cmdline.add(optionalParams);
  if (!cmdline.execute(argc, argv))
  {
      return EXIT_FAILURE;
  }

  // set user camera model
  camera::EINTRINSIC defaultCameraModel = camera::EINTRINSIC::UNKNOWN;
  if(!defaultCameraModelName.empty())
      defaultCameraModel = camera::EINTRINSIC_stringToEnum(defaultCameraModelName);

  // check user choose at least one input option
  if(imageFolder.empty() && sfmFilePath.empty())
  {
    ALICEVISION_LOG_ERROR("Program need -i or --imageFolder option" << std::endl << "No input images.");
    return EXIT_FAILURE;
  }

  // check user don't choose both input options
  if(!imageFolder.empty() && !sfmFilePath.empty())
  {
    ALICEVISION_LOG_ERROR("Cannot combine -i and --imageFolder options");
    return EXIT_FAILURE;
  }

  // check input folder
  if(!imageFolder.empty() && !fs::exists(imageFolder) && !fs::is_directory(imageFolder))
  {
    ALICEVISION_LOG_ERROR("The input folder doesn't exist");
    return EXIT_FAILURE;
  }

  // check sfm file
  if(!sfmFilePath.empty() && !fs::exists(sfmFilePath) && !fs::is_regular_file(sfmFilePath))
  {
    ALICEVISION_LOG_ERROR("The input sfm file doesn't exist");
    return EXIT_FAILURE;
  }

  // check output  string
  if(outputFilePath.empty())
  {
    ALICEVISION_LOG_ERROR("Invalid output");
    return EXIT_FAILURE;
  }

  // ensure output folder exists
  {
    const std::string outputFolderPart = fs::path(outputFilePath).parent_path().string();

    if(!outputFolderPart.empty() && !fs::exists(outputFolderPart))
    {
      if(!fs::create_directory(outputFolderPart))
      {
        ALICEVISION_LOG_ERROR("Cannot create output folder");
        return EXIT_FAILURE;
      }
    }
  }

  if(defaultFocalLength > 0 && defaultFieldOfView > 0)
  {
    ALICEVISION_LOG_ERROR("Cannot combine --defaultFocalLength --defaultFieldOfView options");
    return EXIT_FAILURE;
  }

  if (defaultFocalRatio <= 0.0)
  {
      ALICEVISION_LOG_ERROR("Focal Ratio needs to be a positive value: " << defaultFocalRatio);
      return EXIT_FAILURE;
  }

  // check sensor database
  std::vector<sensorDB::Datasheet> sensorDatabase;
  if (sensorDatabasePath.empty())
  {
      const auto root = image::getAliceVisionRoot();
      if (root.empty())
      {
          ALICEVISION_LOG_WARNING("ALICEVISION_ROOT is not defined, default sensor database cannot be accessed.");
      }
      else
      {
          sensorDatabasePath = root + "/share/aliceVision/cameraSensors.db";
      }
  }

  if(!sensorDatabasePath.empty() && !sensorDB::parseDatabase(sensorDatabasePath, sensorDatabase))
  {
      ALICEVISION_LOG_ERROR("Invalid input sensor database '" << sensorDatabasePath << "', please specify a valid file.");
      return EXIT_FAILURE;
  }

  camera::EINTRINSIC allowedCameraModels = camera::EINTRINSIC_parseStringToBitmask(allowedCameraModelsStr);

  // use current time as seed for random generator for intrinsic Id without metadata
  std::srand(std::time(0));

  std::vector<std::string> noMetadataImagePaths; // imagePaths
  std::map<std::string, std::pair<double, double>> intrinsicsSetFromFocal35mm; // key imagePath value (sensor width, focal length)
  std::vector<std::string> missingDeviceUID;
  std::map<std::pair<std::string, std::string>, std::string> unknownSensors; // key (make,model), value (first imagePath)
  std::map<std::pair<std::string, std::string>, std::pair<std::string, aliceVision::sensorDB::Datasheet>> unsureSensors; // key (make,model), value (first imagePath,datasheet)
  std::map<IndexT, std::map<int, std::size_t>> detectedRigs; // key rigId, value (subPoseId, nbPose)

  sfmData::SfMData sfmData;

  // number of views with an initialized intrinsic
  std::size_t completeViewCount = 0;
  // number of views with LCP data used to initialize intrinsics
  std::size_t lcpGeometryViewCount = 0;
  // number of views with LCP data used to add vignetting params in metadata
  std::size_t lcpVignettingViewCount = 0;

  // load known informations
  if(imageFolder.empty())
  {
    // fill SfMData from the JSON file
    loadJSON(sfmData, sfmFilePath, ESfMData(VIEWS|INTRINSICS|EXTRINSICS), true, viewIdMethod, viewIdRegex);
  }
  else
  {
    // fill SfMData with the images in the input folder
    sfmData::Views& views = sfmData.getViews();
    std::vector<fs::path> imagePaths;

    if(listFiles(imageFolder, image::getSupportedExtensions(), imagePaths))
    {
      std::vector<sfmData::View> incompleteViews(imagePaths.size());

      #pragma omp parallel for
      for(int i = 0; i < incompleteViews.size(); ++i)
      {
        sfmData::View& view = incompleteViews.at(i);
        view.setImagePath(imagePaths.at(i).string());
        updateIncompleteView(view, viewIdMethod, viewIdRegex);
      }

      for(const auto& view : incompleteViews)
        views.emplace(view.getViewId(), std::make_shared<sfmData::View>(view));
    }
    else {
      return EXIT_FAILURE;
    }
  }

  if(sfmData.getViews().empty())
  {
    ALICEVISION_LOG_ERROR("Can't find views in input.");
    return EXIT_FAILURE;
  }

  // create missing intrinsics
  auto viewPairItBegin = sfmData.getViews().begin();

  boost::regex extractComposedNumberRegex("\\d+(?:[\\-\\:\\_\\.]\\d+)*");
  boost::regex extractNumberRegex("\\d+");

  std::map<IndexT, std::vector<IndexT>> poseGroups;

  ALICEVISION_LOG_DEBUG("List files in the DCP database: " << colorProfileDatabaseDirPath);
  char allColorProfilesFound = 1; // char type instead of bool to support usage of atomic
  image::DCPDatabase dcpDatabase(colorProfileDatabaseDirPath);
  if (!colorProfileDatabaseDirPath.empty())
  {
      ALICEVISION_LOG_INFO(dcpDatabase.size() << " profile(s) stored in the DCP database.");
  }
  int viewsWithDCPMetadata = 0;

  ALICEVISION_LOG_DEBUG("List files in the LCP database: " << lensCorrectionProfileInfo);
  LCPdatabase lcpStore(lensCorrectionProfileInfo, lensCorrectionProfileSearchIgnoreCameraModel);
  if (!lensCorrectionProfileInfo.empty())
  {
    ALICEVISION_LOG_INFO(lcpStore.size() << " profile(s) stored in the LCP database.");
  }

  #pragma omp parallel for
  for (int i = 0; i < sfmData.getViews().size(); ++i)
  {
    sfmData::View& view = *(std::next(viewPairItBegin,i)->second);

    // try to detect rig structure in the input folder
    const fs::path parentPath = fs::path(view.getImagePath()).parent_path();
    if(boost::starts_with(parentPath.parent_path().stem().string(), "rig"))
    {
      try
      {
        IndexT subPoseId;
        std::string prefix;
        std::string suffix;
        if(!sfmDataIO::extractNumberFromFileStem(parentPath.stem().string(), subPoseId, prefix, suffix))
        {
          ALICEVISION_THROW_ERROR("Cannot find sub-pose id from image path: " << parentPath);
        }

        std::hash<std::string> hash; // TODO use boost::hash_combine
        view.setRigAndSubPoseId(hash(parentPath.parent_path().string()), subPoseId);

        #pragma omp critical (rig)
        detectedRigs[view.getRigId()][view.getSubPoseId()]++;
      }
      catch(std::exception& e)
      {
        ALICEVISION_LOG_WARNING("Invalid rig structure for view: " << view.getImagePath() << std::endl << e.what() << std::endl << "Used as single image.");
      }
    }

    // try to detect image sequence
    {
        IndexT frameId;
        std::string prefix;
        std::string suffix;
        if(sfmDataIO::extractNumberFromFileStem(fs::path(view.getImagePath()).stem().string(), frameId, prefix, suffix))
        {
          view.setFrameId(frameId);
        }
    }
    
    if(boost::algorithm::starts_with(parentPath.stem().string(), "ps_") ||
       boost::algorithm::starts_with(parentPath.stem().string(), "hdr_"))
    {
        std::hash<std::string> hash;
        IndexT tmpPoseID = hash(parentPath.string()); // use a temporary pose Id to group the images

        #pragma omp critical (poseGroups)
        {
            poseGroups[tmpPoseID].push_back(view.getViewId());
        }
    }

    IndexT intrinsicId = view.getIntrinsicId();
    double sensorWidth = -1;
    double sensorHeight = -1;
    enum class ESensorWidthSource {
        FROM_DB,
        FROM_METADATA_ESTIMATION,
        UNKNOWN
    } sensorWidthSource = ESensorWidthSource::UNKNOWN;

    double focalLengthmm = view.getMetadataFocalLength();
    const std::string& make = view.getMetadataMake();
    const std::string& model = view.getMetadataModel();
    const bool hasCameraMetadata = (!make.empty() || !model.empty());

    std::unique_ptr<oiio::ImageInput> in(oiio::ImageInput::open(view.getImagePath()));

    std::string imgFormat = in->format_name();

    bool dcpError = true;

    // if a color profile is required check if a dcp database exists and if one is available inside 
    // if yes and if metadata exist and image format is raw then update metadata with DCP info
    if((rawColorInterpretation == image::ERawColorInterpretation::DcpLinearProcessing ||
        rawColorInterpretation == image::ERawColorInterpretation::DcpMetadata) &&
        hasCameraMetadata && (imgFormat.compare("raw") == 0))
    {

        if (dcpDatabase.empty() && errorOnMissingColorProfile)
        {
            ALICEVISION_LOG_ERROR("The specified DCP database for color profiles does not exist or is empty.");
            // No color profile available
            boost::atomic_ref<char>{allColorProfilesFound} = 0;
        }
        else
        {
            image::DCPProfile dcpProf;

            #pragma omp critical (dcp)
            {
                if (!dcpDatabase.empty() && dcpDatabase.retrieveDcpForCamera(make, model, dcpProf))
                {
                    view.addDCPMetadata(dcpProf);

                    viewsWithDCPMetadata++;

                    dcpError = false;
                }
                else if (allColorProfilesFound)
                {
                    // there is a missing color profile for at least one image
                    boost::atomic_ref<char>{allColorProfilesFound} = 0;
                }
            }

        }
    }

    if (imgFormat.compare("raw") == 0)
    {
        // Store the color interpretation mode choosen for raw images in metadata,
        // so all future loads of this image will be interpreted in the same way.
        if (!dcpError)
        {
            view.addMetadata("AliceVision:rawColorInterpretation", image::ERawColorInterpretation_enumToString(rawColorInterpretation));
        }
        else
        {
            view.addMetadata("AliceVision:rawColorInterpretation", image::ERawColorInterpretation_enumToString(image::ERawColorInterpretation::LibRawWhiteBalancing));
            if (!dcpDatabase.empty())
                ALICEVISION_LOG_WARNING("DCP Profile not found for image: " << view.getImagePath() << ". Use LibRawWhiteBalancing option for raw color processing.");
        }
    }

    // check if the view intrinsic is already defined
    if(intrinsicId != UndefinedIndexT)
    {
      camera::IntrinsicBase* intrinsicBase = sfmData.getIntrinsicPtr(view.getIntrinsicId());
      camera::Pinhole* intrinsic = dynamic_cast<camera::Pinhole*>(intrinsicBase);
      if(intrinsic != nullptr)
      {
        if(intrinsic->getFocalLengthPixX() > 0)
        {
          // the view intrinsic is initialized
          boost::atomic_ref<std::size_t>(completeViewCount)++;

          // don't need to build a new intrinsic
          continue;
        }
      }
    }

    camera::EInitMode intrinsicInitMode = camera::EInitMode::UNKNOWN;

    int errCode = view.getSensorSize(sensorDatabase, sensorWidth, sensorHeight, focalLengthmm, intrinsicInitMode, false);

    // Throw a warning at the end
    if (errCode == 1)
    {
      #pragma omp critical(unknownSensors)
      unknownSensors.emplace(std::make_pair(make, model), view.getImagePath());
    }
    else if (errCode == 2)
    {
      #pragma omp critical(noMetadataImagePaths)
      noMetadataImagePaths.emplace_back(view.getImagePath());
    }
    else if (errCode == 3)
    {
      sensorDB::Datasheet datasheet;
      sensorDB::getInfo(make, model, sensorDatabase, datasheet);
      #pragma omp critical(unsureSensors)
      unsureSensors.emplace(std::make_pair(make, model), std::make_pair(view.getImagePath(), datasheet));
    }
    else if (errCode == 4)
    {
      #pragma omp critical(intrinsicsSetFromFocal35mm)
      intrinsicsSetFromFocal35mm.emplace(view.getImagePath(), std::make_pair(sensorWidth, focalLengthmm));
    }

    // try to find an appropriate Lens Correction Profile
    LCPinfo* lcpData = nullptr;
    if (lcpStore.size() == 1)
    {
      lcpData = lcpStore.retrieveLCP();
    }
    else if (!lcpStore.empty())
    {
      // Find an LCP file that matches the camera model and the lens model.
      const std::string& lensModel = view.getMetadataLensModel();
      const int lensID = view.getMetadataLensID();

      if (!make.empty() && !lensModel.empty())
      {
        #pragma omp critical(lcp)
        lcpData = lcpStore.findLCP(make, model, lensModel, lensID, 1);
      }
    }

    // const  apertureValue = 2.f * std::log(view.getMetadataFNumber()) / std::log(2.0); // to be uncommented when adding lcp defringing model search
    const float focusDistance = 0.f;

    LensParam lensParam;
    if ((lcpData != nullptr) && !(lcpData->isEmpty()))
    {
      lcpData->getDistortionParams(focalLengthmm, focusDistance, lensParam);
      lcpData->getVignettingParams(focalLengthmm, focusDistance, lensParam);
    }

    if (lensParam.hasVignetteParams() && !lensParam.vignParams.isEmpty)
    {
      view.addVignettingMetadata(lensParam);
      ++lcpVignettingViewCount;
    }

    // build intrinsic
    std::shared_ptr<camera::IntrinsicBase> intrinsicBase = getViewIntrinsic(view, focalLengthmm, sensorWidth, defaultFocalLength, defaultFieldOfView, defaultFocalRatio,
                                                                            defaultOffsetX, defaultOffsetY, &lensParam, defaultCameraModel, allowedCameraModels);

    if (!lensParam.isEmpty())
      ++lcpGeometryViewCount;

    std::shared_ptr<camera::IntrinsicsScaleOffset> intrinsic = std::dynamic_pointer_cast<camera::IntrinsicsScaleOffset>(intrinsicBase);

    // set initialization mode
    intrinsic->setInitializationMode(intrinsicInitMode);

    // Set sensor size
    intrinsicBase->setSensorWidth(sensorWidth);
    intrinsicBase->setSensorHeight(sensorHeight);

    if(intrinsic && intrinsic->isValid())
    {
      // the view intrinsic is initialized
      boost::atomic_ref<std::size_t>(completeViewCount)++;
    }

    // Create serial number if not already filled
    if(intrinsic->serialNumber().empty())
    {
      // Create custom serial number
      const std::string& bodySerialNumber = view.getMetadataBodySerialNumber();
      const std::string& lensSerialNumber = view.getMetadataLensSerialNumber();

      if(!bodySerialNumber.empty() || !lensSerialNumber.empty())
      {
        // We can identify the device based on a unique ID.
        intrinsic->setSerialNumber(bodySerialNumber + "_" + lensSerialNumber);
      }
      else
      {
        // We have no way to identify a camera device correctly.
#pragma omp critical (missingDeviceUID)
        {
          missingDeviceUID.emplace_back(view.getImagePath()); // will throw a warning message at the end
        }

        // To avoid stopping the process, we fallback to a solution selected by the user:
        if(groupCameraFallback == EGroupCameraFallback::FOLDER)
        {
          // when we don't have a serial number, the folder will become part of the device ID.
          // This means that 2 images in different folder will NOT share intrinsics.
          intrinsic->setSerialNumber(fs::path(view.getImagePath()).parent_path().string());
        }
        else if(groupCameraFallback == EGroupCameraFallback::IMAGE)
        {
          // if no serial number, each view will get its own camera intrinsic parameters.
          intrinsic->setSerialNumber(view.getImagePath());
        }
        else if(groupCameraFallback == EGroupCameraFallback::GLOBAL)
        {
          // if no serial number, images with the same make/model/focal or no make/model/focal
          // will be considered as a single group of camera intrinsics.
        }

        if(!make.empty() || !model.empty())
        {
          // We have no correct way to identify a camera device, we fallback on the camera make/model.
          // If you use multiple identical devices, they will be fused together incorrectly.
          intrinsic->setSerialNumber(intrinsic->serialNumber() + "_" + make + "_" + model);
        }

        if(view.isPartOfRig())
        {
          // when we have no unique camera identifier, so for rig images, we ensure that each camera of the rig have different serial numbers.
          intrinsic->setSerialNumber(intrinsic->serialNumber() + "_rig_" + std::to_string(view.getRigId()) + "_" + std::to_string(view.getSubPoseId()));
        }
      }

      // If we have not managed to initialize the focal length, we need to use the focalLength in mm
      if(intrinsic->getScale()(0) <= 0 && focalLengthmm > 0)
      {
        intrinsic->setSerialNumber(intrinsic->serialNumber() + "_FocalLengthMM_" + std::to_string(focalLengthmm));
      }
    }
    
    // create intrinsic id
    // group camera that share common properties (leads to more faster & stable BA).
    if(intrinsicId == UndefinedIndexT)
    {
      intrinsicId = intrinsic->hashValue();
    }

    #pragma omp critical (intrinsics)
    {
      view.setIntrinsicId(intrinsicId);
      sfmData.getIntrinsics().emplace(intrinsicId, intrinsicBase);
    }
  }

  // create detected rigs structures
  if(!detectedRigs.empty())
  {
    for(const auto& subPosesPerRigId : detectedRigs)
    {
      const IndexT rigId = subPosesPerRigId.first;
      const std::size_t nbSubPose = subPosesPerRigId.second.size();
      const std::size_t nbPoses = subPosesPerRigId.second.begin()->second;

      for(const auto& nbPosePerSubPoseId : subPosesPerRigId.second)
      {
        // check subPoseId
        if(nbPosePerSubPoseId.first >= nbSubPose)
        {
            ALICEVISION_LOG_ERROR("Wrong subPoseId in rig structure: it should be an index not a random number (subPoseId: " << nbPosePerSubPoseId.first << ", number of subposes: " << nbSubPose << ").");
            return EXIT_FAILURE;
        }
        if(nbPosePerSubPoseId.first < 0)
        {
          ALICEVISION_LOG_ERROR("Wrong subPoseId in rig structure: cannot contain negative value: " << nbPosePerSubPoseId.first);
          return EXIT_FAILURE;
        }

        // check nbPoses
        if(nbPosePerSubPoseId.second != nbPoses)
        {
          ALICEVISION_LOG_WARNING("Wrong number of poses per subPose in detected rig structure (" << nbPosePerSubPoseId.second << " != " << nbPoses << ").");
        }
      }

      sfmData.getRigs().emplace(rigId, sfmData::Rig(nbSubPose));
    }
  }

  if (!allColorProfilesFound)
  {
      if (errorOnMissingColorProfile)
      {
          ALICEVISION_LOG_ERROR("Can't find color profile for at least one input image.");
          return EXIT_FAILURE;
      }
      else
      {
          ALICEVISION_LOG_WARNING("Can't find color profile for at least one input image.");
          ALICEVISION_LOG_WARNING("Images without color profiles have been decoded with the LibRawWhiteBalancing option.");
      }
  }

  // Update poseId for detected multi-exposure or multi-lighting images (multiple shots with the same camera pose)
  if(!poseGroups.empty())
  {
      for(const auto& poseGroup : poseGroups)
      {
          bool hasAmbiant = false;

          // Photometric stereo : ambiant viewId used for all pictures
          for(const IndexT vId : poseGroup.second)
          {
              const fs::path imagePath = fs::path(sfmData.getView(vId).getImagePath());
              if(boost::algorithm::icontains(imagePath.stem().string(), "ambiant"))
              {
                    hasAmbiant = true;
                    for(const auto it : poseGroup.second)
                    {
                        // Update poseId with ambiant view id
                        sfmData.getView(it).setPoseId(vId);
                    }
                    break;
              }
          }
          if(!hasAmbiant)
          {
              // Sort views of the poseGroup per timestamps
              std::vector<std::pair<int64_t, IndexT>> sortedViews;
              for(const IndexT vId : poseGroup.second)
              {
                  int64_t t = sfmData.getView(vId).getMetadataDateTimestamp();
                  sortedViews.push_back(std::make_pair(t, vId));
              }
              std::sort(sortedViews.begin(), sortedViews.end());

              // Get the view which was taken at the middle of the sequence
              int median = sortedViews.size() / 2;
              IndexT middleViewId = sortedViews[median].second;

              for(const auto it : sortedViews)
              {
                  const IndexT vId = it.second;
                  // Update poseId with middle view id
                  sfmData.getView(vId).setPoseId(middleViewId);
              }
          }
      }
  }

  if(!noMetadataImagePaths.empty())
  {
    std::stringstream ss;
    ss << "No metadata in image(s):\n";
    for(const auto& imagePath : noMetadataImagePaths)
      ss << "\t- '" << imagePath << "'\n";
    ALICEVISION_LOG_DEBUG(ss.str());
  }

  if(!missingDeviceUID.empty())
  {
      ALICEVISION_LOG_WARNING(
                  "Some image(s) have no serial number to identify the camera/lens device.\n"
                  "This makes it impossible to correctly group the images by device if you have used multiple identical (same model) camera devices.\n"
                  "The reconstruction will assume that only one device has been used, "
                  "so if 2 images share the same focal length approximation they will share the same internal camera parameters.\n"
                  << missingDeviceUID.size() << " image(s) are concerned.");
      ALICEVISION_LOG_DEBUG("The following images are concerned:\n");
      ALICEVISION_LOG_DEBUG(boost::algorithm::join(missingDeviceUID, "\n"));
  }

  if(!unsureSensors.empty())
  {
    ALICEVISION_LOG_WARNING("The camera found in the sensor database is slightly different for image(s):");
    for(const auto& unsureSensor : unsureSensors)
      ALICEVISION_LOG_WARNING("image: '" << fs::path(unsureSensor.second.first).filename().string() << "'" << std::endl
                        << "\t- image camera brand: " << unsureSensor.first.first <<  std::endl
                        << "\t- image camera model: " << unsureSensor.first.second <<  std::endl
                        << "\t- sensor database camera brand: " << unsureSensor.second.second._brand <<  std::endl
                        << "\t- sensor database camera model: " << unsureSensor.second.second._model << std::endl
                        << "\t- sensor database camera sensor width: " << unsureSensor.second.second._sensorWidth  << " mm");
    ALICEVISION_LOG_WARNING("Please check and correct camera model(s) name in the sensor database." << std::endl);
  }

  if(!unknownSensors.empty())
  {
    std::stringstream ss;
    ss << "Sensor width doesn't exist in the sensor database for image(s):\n";
    for(const auto& unknownSensor : unknownSensors)
    {
      ss << "\t- camera brand: " << unknownSensor.first.first << "\n"
         << "\t- camera model: " << unknownSensor.first.second << "\n"
         << "\t   - image: " << fs::path(unknownSensor.second).filename().string() << "\n";
    }
    ss << "Please add camera model(s) and sensor width(s) in the database.";

    ALICEVISION_LOG_WARNING(ss.str());
  }

  if(!intrinsicsSetFromFocal35mm.empty())
  {
    std::stringstream ss;
    ss << "Intrinsic(s) initialized from 'FocalLengthIn35mmFilm' exif metadata in image(s):\n";
    for(const auto& intrinsicSetFromFocal35mm : intrinsicsSetFromFocal35mm)
    {
      ss << "\t- image: " << fs::path(intrinsicSetFromFocal35mm.first).filename().string() << "\n"
         << "\t   - sensor width: " << intrinsicSetFromFocal35mm.second.first  << "\n"
         << "\t   - focal length: " << intrinsicSetFromFocal35mm.second.second << "\n";
    }
    ALICEVISION_LOG_DEBUG(ss.str());
  }

  if(completeViewCount < 1 || (completeViewCount < 2 && !allowSingleView))
  {
    ALICEVISION_LOG_ERROR("At least " << std::string(allowSingleView ? "one image" : "two images") << " should have an initialized intrinsic." << std::endl
                          << "Check your input images metadata (brand, model, focal length, ...), more should be set and correct." << std::endl);
    return EXIT_FAILURE;
  }

  //Check unique frame id per rig element
  if (!rigHasUniqueFrameIds(sfmData))
  {
    ALICEVISION_LOG_ERROR("Multiple frames have the same frame id.");
    return EXIT_FAILURE;
  }
  
  // store SfMData views & intrinsic data
  if(!Save(sfmData, outputFilePath, ESfMData(VIEWS|INTRINSICS|EXTRINSICS)))
  {
    return EXIT_FAILURE;
  }

  // print report
  ALICEVISION_LOG_INFO("CameraInit report:"
                   << "\n\t- # Views: " << sfmData.getViews().size()
                   << "\n\t   - # with focal length initialization (from metadata): " << completeViewCount
                   << "\n\t   - # without metadata: " << noMetadataImagePaths.size()
                   << "\n\t   - # with DCP color calibration (raw images only): " << viewsWithDCPMetadata
                   << "\n\t   - # with LCP lens distortion initialization: " << lcpGeometryViewCount
                   << "\n\t   - # with LCP vignetting calibration: " << lcpVignettingViewCount
                   << "\n\t- # Cameras Intrinsics: " << sfmData.getIntrinsics().size());

  return EXIT_SUCCESS;
}
