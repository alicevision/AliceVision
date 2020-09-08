// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/sfm/utils/alignment.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/config.hpp>

#include <boost/program_options.hpp>

#include <string>
#include <sstream>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;
using namespace aliceVision::sfm;

namespace po = boost::program_options;


/**
* @brief Alignment method enum
*/
enum class EAlignmentMethod : unsigned char
{
    FROM_CAMERAS_VIEWID = 0
    , FROM_CAMERAS_POSEID
    , FROM_CAMERAS_FILEPATH
    , FROM_CAMERAS_METADATA
    , FROM_MARKERS
};

/**
* @brief Convert an EAlignmentMethod enum to its corresponding string
* @param[in] alignmentMethod The given EAlignmentMethod enum
* @return string
*/
std::string EAlignmentMethod_enumToString(EAlignmentMethod alignmentMethod)
{
    switch (alignmentMethod)
    {
    case EAlignmentMethod::FROM_CAMERAS_VIEWID:   return "from_cameras_viewid";
    case EAlignmentMethod::FROM_CAMERAS_POSEID:   return "from_cameras_poseid";
    case EAlignmentMethod::FROM_CAMERAS_FILEPATH: return "from_cameras_filepath";
    case EAlignmentMethod::FROM_CAMERAS_METADATA: return "from_cameras_metadata";
    case EAlignmentMethod::FROM_MARKERS:          return "from_markers";
    }
    throw std::out_of_range("Invalid EAlignmentMethod enum");
}

/**
* @brief Convert a string to its corresponding EAlignmentMethod enum
* @param[in] alignmentMethod The given string
* @return EAlignmentMethod enum
*/
EAlignmentMethod EAlignmentMethod_stringToEnum(const std::string& alignmentMethod)
{
    std::string method = alignmentMethod;
    std::transform(method.begin(), method.end(), method.begin(), ::tolower); //tolower

    if (method == "from_cameras_viewid")   return EAlignmentMethod::FROM_CAMERAS_VIEWID;
    if (method == "from_cameras_poseid")   return EAlignmentMethod::FROM_CAMERAS_POSEID;
    if (method == "from_cameras_filepath") return EAlignmentMethod::FROM_CAMERAS_FILEPATH;
    if (method == "from_cameras_metadata") return EAlignmentMethod::FROM_CAMERAS_METADATA;
    if (method == "from_markers")          return EAlignmentMethod::FROM_MARKERS;
    throw std::out_of_range("Invalid SfM alignment method : " + alignmentMethod);
}

inline std::istream& operator>>(std::istream& in, EAlignmentMethod& alignment)
{
    std::string token;
    in >> token;
    alignment = EAlignmentMethod_stringToEnum(token);
    return in;
}

inline std::ostream& operator<<(std::ostream& os, EAlignmentMethod e)
{
    return os << EAlignmentMethod_enumToString(e);
}


int aliceVision_main(int argc, char **argv)
{
  // command-line parameters

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string sfmDataFilename;
  std::string outSfMDataFilename;
  std::string sfmDataReferenceFilename;
  bool applyScale = true;
  bool applyRotation = true;
  bool applyTranslation = true;
  EAlignmentMethod alignmentMethod = EAlignmentMethod::FROM_CAMERAS_VIEWID;
  std::string fileMatchingPattern;
  std::vector<std::string> metadataMatchingList = {"Make", "Model", "Exif:BodySerialNumber" , "Exif:LensSerialNumber" };
  std::string outputViewsAndPosesFilepath;

  po::options_description allParams("AliceVision sfmAlignment");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
      "SfMData file to align.")
    ("output,o", po::value<std::string>(&outSfMDataFilename)->required(),
      "Output SfMData scene.")
    ("reference,r", po::value<std::string>(&sfmDataReferenceFilename)->required(),
      "Path to the scene used as the reference coordinate system.");

  po::options_description optionalParams("Optional parameters");
  optionalParams.add_options()
    ("method", po::value<EAlignmentMethod>(&alignmentMethod)->default_value(alignmentMethod),
        "Alignment Method:\n"
        "\t- from_cameras_viewid: Align cameras with same view Id\n"
        "\t- from_cameras_poseid: Align cameras with same pose Id\n"
        "\t- from_cameras_filepath: Align cameras with a filepath matching, using --fileMatchingPattern\n"
        "\t- from_cameras_metadata: Align cameras with matching metadata, using --metadataMatchingList\n"
        "\t- from_markers: Align from markers with the same Id\n")
    ("fileMatchingPattern", po::value<std::string>(&fileMatchingPattern)->default_value(fileMatchingPattern),
        "Matching pattern for the from_cameras_filepath method.\n")
    ("metadataMatchingList", po::value<std::vector<std::string>>(&metadataMatchingList)->multitoken()->default_value(metadataMatchingList),
        "List of metadata that should match to create the correspondences.\n")
    ("applyScale", po::value<bool>(&applyScale)->default_value(applyScale),
      "Apply scale transformation.")
    ("applyRotation", po::value<bool>(&applyRotation)->default_value(applyRotation),
      "Apply rotation transformation.")
    ("applyTranslation", po::value<bool>(&applyTranslation)->default_value(applyTranslation),
      "Apply translation transformation.")
    ("outputViewsAndPoses", po::value<std::string>(&outputViewsAndPosesFilepath),
      "Path of the output SfMData file.")
    ;

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

  ALICEVISION_COUT("Program called with the following parameters:");
  ALICEVISION_COUT(vm);

  // set verbose level
  system::Logger::get()->setLogLevel(verboseLevel);

  std::mt19937 randomNumberGenerator;

  // Load input scene
  sfmData::SfMData sfmData;
  if(!sfmDataIO::Load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
  {
    ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' cannot be read");
    return EXIT_FAILURE;
  }

  // Load reference scene
  sfmData::SfMData sfmDataInRef;
  if(!sfmDataIO::Load(sfmDataInRef, sfmDataReferenceFilename, sfmDataIO::ESfMData::ALL))
  {
    ALICEVISION_LOG_ERROR("The reference SfMData file '" << sfmDataReferenceFilename << "' cannot be read");
    return EXIT_FAILURE;
  }

  ALICEVISION_LOG_INFO("Search similarity transformation.");

  double S;
  Mat3 R;
  Vec3 t;
  bool hasValidSimilarity = false;
  
  switch(alignmentMethod)
  {
    case EAlignmentMethod::FROM_CAMERAS_VIEWID:
    {
      hasValidSimilarity = sfm::computeSimilarityFromCommonCameras_viewId(sfmData, sfmDataInRef, randomNumberGenerator, &S, &R, &t);
      break;
    }
    case EAlignmentMethod::FROM_CAMERAS_POSEID:
    {
      hasValidSimilarity = sfm::computeSimilarityFromCommonCameras_poseId(sfmData, sfmDataInRef, randomNumberGenerator, &S, &R, &t);
      break;
    }
    case EAlignmentMethod::FROM_CAMERAS_FILEPATH:
    {
      hasValidSimilarity = sfm::computeSimilarityFromCommonCameras_imageFileMatching(sfmData, sfmDataInRef, fileMatchingPattern, randomNumberGenerator, &S, &R, &t);
      break;
    }
    case EAlignmentMethod::FROM_CAMERAS_METADATA:
    {
      hasValidSimilarity = sfm::computeSimilarityFromCommonCameras_metadataMatching(sfmData, sfmDataInRef, metadataMatchingList, randomNumberGenerator, &S, &R, &t);
      break;
    }
    case EAlignmentMethod::FROM_MARKERS:
    {
      hasValidSimilarity = sfm::computeSimilarityFromCommonMarkers(sfmData, sfmDataInRef, randomNumberGenerator, &S, &R, &t);
      break;
    }
  }

  if(!hasValidSimilarity)
  {
    std::stringstream ss;
    ss << "Failed to find similarity between the 2 SfM scenes:";
    ss << "\t- " << sfmDataFilename << std::endl;
    ss << "\t- " << sfmDataReferenceFilename << std::endl;
    ALICEVISION_LOG_ERROR(ss.str());
    return EXIT_FAILURE;
  }

  {
    std::stringstream ss;
    ss << "Transformation:" << std::endl;
    ss << "\t- Scale: " << S << std::endl;
    ss << "\t- Rotation:\n" << R << std::endl;
    ss << "\t- Translate: " << t.transpose() << std::endl;
    ALICEVISION_LOG_INFO(ss.str());
  }

  if (!applyScale)
      S = 1;
  if (!applyRotation)
      R = Mat3::Identity();
  if (!applyTranslation)
      t = Vec3::Zero();

  sfm::applyTransform(sfmData, S, R, t);

  ALICEVISION_LOG_INFO("Save into '" << outSfMDataFilename << "'");
  
  // Export the SfMData scene in the expected format
  if(!sfmDataIO::Save(sfmData, outSfMDataFilename, sfmDataIO::ESfMData::ALL))
  {
    ALICEVISION_LOG_ERROR("An error occurred while trying to save '" << outSfMDataFilename << "'");
    return EXIT_FAILURE;
  }

  if(!outputViewsAndPosesFilepath.empty())
  {
      sfmDataIO::Save(sfmData, outputViewsAndPosesFilepath,
                      sfmDataIO::ESfMData(sfmDataIO::VIEWS | sfmDataIO::EXTRINSICS | sfmDataIO::INTRINSICS));
  }

  return EXIT_SUCCESS;
}
