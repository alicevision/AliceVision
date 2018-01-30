// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfm/sfm.hpp>
#include <aliceVision/sfm/utils/alignment.hpp>
#include <aliceVision/config.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>

#include <boost/program_options.hpp>

#include <string>
#include <sstream>
#include <vector>

using namespace aliceVision;
using namespace aliceVision::sfm;
namespace po = boost::program_options;

/**
 * @brief Alignment method enum
 */
enum class EAlignmentMethod: unsigned char
{
  TRANSFOMATION = 0
  , AUTO_FROM_CAMERAS
  , AUTO_FROM_LANDMARKS
};

/**
 * @brief Convert an EAlignmentMethod enum to its corresponding string
 * @param[in] alignmentMethod The given EAlignmentMethod enum
 * @return string
 */
std::string EAlignmentMethod_enumToString(EAlignmentMethod alignmentMethod)
{
  switch(alignmentMethod)
  {
    case EAlignmentMethod::TRANSFOMATION:       return "transformation";
    case EAlignmentMethod::AUTO_FROM_CAMERAS:   return "auto_from_cameras";
    case EAlignmentMethod::AUTO_FROM_LANDMARKS: return "auto_from_landmarks";
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

  if(method == "transformation")      return EAlignmentMethod::TRANSFOMATION;
  if(method == "auto_from_cameras")   return EAlignmentMethod::AUTO_FROM_CAMERAS;
  if(method == "auto_from_landmarks") return EAlignmentMethod::AUTO_FROM_LANDMARKS;
  throw std::out_of_range("Invalid SfM alignment method : " + alignmentMethod);
}

static bool parseAlignScale(const std::string& alignScale, double& S, Mat3& R, Vec3& t)
{
  double rx, ry, rz, rr;

  {
    char delim[4];
    std::istringstream iss(alignScale);
    if (!(iss >> rx >> delim[0] >> ry >> delim[1] >> rz >> delim[2] >> rr >> delim[3] >> S))
      return false;
    if (delim[0] != ',' || delim[1] != ',' || delim[2] != ';' || delim[3] != ';')
      return false;
  }

  auto q = Eigen::Quaterniond::FromTwoVectors(Vec3(rx, ry, rz), Vec3::UnitY());
  auto r = Eigen::AngleAxisd(rr*M_PI/180, Vec3::UnitY());

  R = r * q.toRotationMatrix();
  t = Vec3::Zero();

  return true;
}

int main(int argc, char **argv)
{
  // command-line parameters

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string sfmDataFilename;
  std::string outSfMDataFilename;
  std::string alignmentMethodName;

  // user optional parameters

  std::string transformationYAlignScale;
  std::string landmarksDescriberTypesName;
  double userScale = 1;

  po::options_description allParams("AliceVision sfmTransform");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
      "SfMData file to align.")
    ("output,o", po::value<std::string>(&outSfMDataFilename)->required(),
      "Output SfMData scene.")
    ("method", po::value<std::string>(&alignmentMethodName)->required(),
      "Transform method:\n"
      "\t- transformation: Apply a given transformation\n"
      "\t- auto_from_cameras: Use cameras\n"
      "\t- auto_from_landmarks: Use landmarks\n");

  po::options_description optionalParams("Optional parameters");
  optionalParams.add_options()
    ("transformation", po::value<std::string>(&transformationYAlignScale)->default_value(transformationYAlignScale),
      "required only for 'transformation' method:\n"
      "Align [X,Y,Z] to +Y-axis, rotate around Y by R deg, scale by S; syntax: X,Y,Z;R;S")
    ("landmarksDescriberTypes,d", po::value<std::string>(&landmarksDescriberTypesName)->default_value(landmarksDescriberTypesName),
      ("optional for 'landmarks' method:\n"
      "Image describer types used to compute the mean of the point cloud\n"
      "Use all of them if empty\n"
      + feature::EImageDescriberType_informations()).c_str())
    ("scale", po::value<double>(&userScale)->default_value(userScale),
      "Additional scale to apply.");

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

  // set alignment method
  const EAlignmentMethod alignmentMethod = EAlignmentMethod_stringToEnum(alignmentMethodName);

  if(alignmentMethod == EAlignmentMethod::TRANSFOMATION &&
     transformationYAlignScale.empty())
  {
    ALICEVISION_LOG_ERROR("Missing --transformation option");
    return EXIT_FAILURE;
  }

  // Load input scene
  SfMData sfmDataIn;
  if(!Load(sfmDataIn, sfmDataFilename, ESfMData(ALL)))
  {
    ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' cannot be read");
    return EXIT_FAILURE;
  }

  double S;
  Mat3 R;
  Vec3 t;

  switch(alignmentMethod)
  {
    case EAlignmentMethod::TRANSFOMATION:
    {
      if(!parseAlignScale(transformationYAlignScale, S, R, t))
      {
         ALICEVISION_LOG_ERROR("Failed to parse align/scale argument");
         return EXIT_FAILURE;
      }
    }
    break;

    case EAlignmentMethod::AUTO_FROM_CAMERAS:
      computeNewCoordinateSystemFromCameras(sfmDataIn, S, R, t);
    break;

    case EAlignmentMethod::AUTO_FROM_LANDMARKS:
      computeNewCoordinateSystemFromLandmarks(sfmDataIn, feature::EImageDescriberType_stringToEnums(landmarksDescriberTypesName), S, R, t);
    break;
  }

  {
    std::stringstream ss;
    ss << "Transformation:" << std::endl;
    ss << "\t- Scale: " << S << std::endl;
    ss << "\t- Rotation:\n" << R << std::endl;
    ss << "\t- Translate: " << t.transpose() << std::endl;
    ALICEVISION_LOG_INFO(ss.str());
  }

  // apply user scale
  S *= userScale;
  t *= userScale;

  applyTransform(sfmDataIn, S, R, t);

  ALICEVISION_LOG_INFO("Save into '" << outSfMDataFilename << "'");
  
  // Export the SfMData scene in the expected format
  if(!Save(sfmDataIn, outSfMDataFilename, ESfMData::ALL))
  {
    ALICEVISION_LOG_ERROR("An error occurred while trying to save '" << outSfMDataFilename << "'");
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
