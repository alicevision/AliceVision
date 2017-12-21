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

  // user optional parameters

  std::string sfmDataReferenceFilename;
  std::string sfmDataYAlignScale;

  po::options_description allParams("AliceVision sfmAlignment");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
      "SfMData file to align.")
    ("output,o", po::value<std::string>(&outSfMDataFilename)->required(),
      "Output SfMData scene.");

  po::options_description optionalParams("Optional parameters");
  optionalParams.add_options()
    ("reference,r", po::value<std::string>(&sfmDataReferenceFilename)->default_value(sfmDataReferenceFilename),
      "Path to the scene used as the reference coordinate system.")
    ("yAlignScale", po::value<std::string>(&sfmDataYAlignScale)->default_value(sfmDataYAlignScale),
      "Align [X,Y,Z] to +Y-axis, rotate around Y by R deg, scale by S; syntax: X,Y,Z;R;S");

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
  
  if (sfmDataFilename.empty() ||
      outSfMDataFilename.empty())
  {
    ALICEVISION_LOG_ERROR("Invalid input or output filename");
    return EXIT_FAILURE;
  }

  if (sfmDataReferenceFilename.empty() &&
      sfmDataYAlignScale.empty())
  {
    ALICEVISION_LOG_ERROR("At least one of -y and -r must be specified.");
    return EXIT_FAILURE;
  }
  
  if (!sfmDataReferenceFilename.empty() &&
      !sfmDataYAlignScale.empty())
  {
    ALICEVISION_LOG_ERROR("Must specify exactly one of alignment and reference scene");
    return EXIT_FAILURE;
  }

  // Load input scene
  SfMData sfm_data_in;
  if (!Load(sfm_data_in, sfmDataFilename, ESfMData(ALL)))
  {
    ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' cannot be read");
    return EXIT_FAILURE;
  }

  double S;
  Mat3 R;
  Vec3 t;

  if (!sfmDataReferenceFilename.empty())
  {
    // Load reference scene
    SfMData sfm_data_inRef;
    if (!Load(sfm_data_inRef, sfmDataReferenceFilename, ESfMData(ALL)))
    {
      ALICEVISION_LOG_ERROR("The reference SfMData file '" << sfmDataReferenceFilename << "' cannot be read");
      return EXIT_FAILURE;
    }

    std::cout << "Search similarity transformation." << std::endl;
    bool hasValidSimilarity = computeSimilarity(sfm_data_in, sfm_data_inRef, &S, &R, &t);
    if(!hasValidSimilarity)
    {
      std::stringstream ss;
      ss << "Failed to find similarity between the 2 SfM scenes:";
      ss << "\t- " << sfmDataFilename << std::endl;
      ss << "\t- " << sfmDataReferenceFilename << std::endl;
      ALICEVISION_LOG_ERROR(ss.str());
      return EXIT_FAILURE;
    }
  }
  else if (!parseAlignScale(sfmDataYAlignScale, S, R, t))
  {
    ALICEVISION_LOG_ERROR("Failed to parse align/scale argument");
    return EXIT_FAILURE;
  }

  {
    std::stringstream ss;
    ss << "Apply transformation:" << std::endl;
    ss << "\t- Scale: " << S << std::endl;
    ss << "\t- Rotation:\n" << R << std::endl;
    ss << "\t- Translate: " << t.transpose() << std::endl;
    ALICEVISION_LOG_INFO(ss);
  }

  applyTransform(sfm_data_in, S, R, t);
  
  ALICEVISION_LOG_INFO("Save into '" << outSfMDataFilename << "'");
  
  // Export the SfMData scene in the expected format
  if (!Save(sfm_data_in, outSfMDataFilename, ESfMData::ALL))
  {
    ALICEVISION_LOG_ERROR("An error occurred while trying to save '" << outSfMDataFilename << "'");
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
