// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/sfm/utils/alignment.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/config.hpp>

#include <boost/program_options.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/filesystem.hpp>

#include <string>
#include <sstream>
#include <vector>


// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

/**
 * @brief Alignment method enum
 */
enum class EObject: unsigned char
{
  CAMERAS,
  LANDMARKS
};

/**
 * @brief Convert an EAlignmentMethod enum to its corresponding string
 * @param[in] alignmentMethod The given EAlignmentMethod enum
 * @return string
 */
std::string EObject_enumToString(EObject obj)
{
  switch(obj)
  {
    case EObject::CAMERAS:     return "cameras";
    case EObject::LANDMARKS:   return "landmarks";
  }
  throw std::out_of_range("Invalid EAlignmentMethod enum");
}

/**
 * @brief Convert a string to its corresponding EAlignmentMethod enum
 * @param[in] alignmentMethod The given string
 * @return EAlignmentMethod enum
 */
EObject EObject_stringToEnum(const std::string& alignmentMethod)
{
  std::string method = alignmentMethod;
  std::transform(method.begin(), method.end(), method.begin(), ::tolower);

  if(method == "landmarks")      return EObject::LANDMARKS;
  if(method == "cameras")   return EObject::CAMERAS;
  throw std::out_of_range("Invalid SfM alignment method : " + alignmentMethod);
}


inline std::istream& operator>>(std::istream& in, EObject& alignment)
{
    std::string token;
    in >> token;
    alignment = EObject_stringToEnum(token);
    return in;
}

inline std::ostream& operator<<(std::ostream& os, EObject e)
{
    return os << EObject_enumToString(e);
}


void extractLandmarksPositions(std::vector<std::pair<std::string, Vec3>>& outputPositions, const sfmData::SfMData& sfmData, const std::vector<std::string>& search, const std::vector<feature::EImageDescriberType>& landmarksDescriberTypes)
{
    const std::set<feature::EImageDescriberType> isCCTag = {
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CCTAG)
      feature::EImageDescriberType::CCTAG3,
      feature::EImageDescriberType::CCTAG4
#endif
      };
    const std::set<feature::EImageDescriberType> descTypes(landmarksDescriberTypes.begin(), landmarksDescriberTypes.end());
    std::set<IndexT> searchIdx;
    for (const std::string& s : search)
    {
        searchIdx.insert(boost::lexical_cast<IndexT>(s));
    }

    for (const auto& landmarkIt : sfmData.structure)
    {
        if (descTypes.count(landmarkIt.second.descType))
        {
            bool isMarker = isCCTag.count(landmarkIt.second.descType);
            if (searchIdx.empty() ||
                (isMarker ? searchIdx.count(landmarkIt.second.rgb.r()) : searchIdx.count(landmarkIt.first))
                )
            {
                outputPositions.push_back(std::make_pair(std::to_string(isMarker ? landmarkIt.second.rgb.r() : landmarkIt.first), landmarkIt.second.X));
            }
        }
    }
}

void extractCamerasPositions(std::vector<std::pair<std::string, Vec3>>& outputPositions, const sfmData::SfMData& sfmData, const std::vector<std::string>& search)
{
    std::set<std::string> searchSet(search.begin(), search.end());

    for (const auto& viewIt: sfmData.getViews())
    {
        if (!sfmData.isPoseAndIntrinsicDefined(viewIt.second.get()))
            continue;
        const std::string viewIdStr = std::to_string(viewIt.second->getViewId());
        if(searchSet.count(viewIdStr))
        {
            outputPositions.push_back(std::make_pair(viewIdStr, sfmData.getPose(*viewIt.second).getTransform().center()));
            continue;
        }
        std::string stem = fs::path(viewIt.second->getImagePath()).stem().string();
        if (searchSet.empty() || searchSet.count(stem))
        {
            outputPositions.push_back(std::make_pair(viewIt.second->getImagePath(), sfmData.getPose(*viewIt.second).getTransform().center()));
        }
    }
}


int main(int argc, char **argv)
{
  // command-line parameters

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string sfmDataFilename;
  EObject objectType = EObject::LANDMARKS;

  // user optional parameters
  std::string objectA;
  std::string objectB;
  std::string landmarksDescriberTypesName;

  po::options_description allParams("AliceVision sfmTransform");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
      "SfMData file to align.");

  po::options_description optionalParams("Optional parameters");
  optionalParams.add_options()
    ("objectType", po::value<EObject>(&objectType)->default_value(objectType),
        "Object Type:\n"
        "\t- cameras: Use cameras\n"
        "\t- landmarks: Use landmarks\n")
    ("A", po::value<std::string>(&objectA)->default_value(objectA),
        "Object ID:\n"
        "Landmark: ID\n"
        "Camera: camera UID or image filename")
    ("B", po::value<std::string>(&objectB)->default_value(objectB),
        "Object ID:\n"
        "Landmark: ID\n"
        "Camera: camera UID or image filename")
    ("landmarksDescriberTypes,d", po::value<std::string>(&landmarksDescriberTypesName)->default_value(landmarksDescriberTypesName),
      ("optional for 'landmarks' method:\n"
      "Image describer types used to compute the mean of the point cloud\n"
      "Use all of them if empty\n"
      + feature::EImageDescriberType_informations()).c_str())
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


  std::vector<feature::EImageDescriberType> landmarksDescriberTypes = feature::EImageDescriberType_stringToEnums(landmarksDescriberTypesName);

  // Load input scene
  sfmData::SfMData sfmDataIn;
  if(!sfmDataIO::Load(sfmDataIn, sfmDataFilename, sfmDataIO::ESfMData::ALL))
  {
    ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' cannot be read");
    return EXIT_FAILURE;
  }

  std::vector<std::pair<std::string, Vec3>> positionsA;
  std::vector<std::pair<std::string, Vec3>> positionsB;

  std::vector<std::string> inputA;
  std::vector<std::string> inputB;

  if(!objectA.empty())
      boost::split(inputA, objectA, boost::algorithm::is_any_of(","));
  if (!objectB.empty())
      boost::split(inputB, objectB, boost::algorithm::is_any_of(","));

  switch (objectType)
  {
      case EObject::CAMERAS:
      {
          std::cout << "== Cameras ==" << std::endl;
          extractCamerasPositions(positionsA, sfmDataIn, inputA);
          extractCamerasPositions(positionsB, sfmDataIn, inputB);
          break;
      }
      case EObject::LANDMARKS:
      {
          std::cout << "== Landmarks ==" << std::endl;
          extractLandmarksPositions(positionsA, sfmDataIn, inputA, landmarksDescriberTypes);
          extractLandmarksPositions(positionsB, sfmDataIn, inputB, landmarksDescriberTypes);
          break;
      }
  }

  for (const auto& a : positionsA)
  {
      for (const auto& b : positionsB)
      {
          if(a.first != b.first)
              std::cout << a.first << " <=> " << b.first << ": " << (b.second - a.second).norm() << std::endl;
      }
  }

  return EXIT_SUCCESS;
}
