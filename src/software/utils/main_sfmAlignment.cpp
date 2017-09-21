// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include "aliceVision/sfm/sfm.hpp"
#include "aliceVision/sfm/utils/alignment.hpp"
#include <aliceVision/config.hpp>

#include "dependencies/cmdLine/cmdLine.h"
#include "dependencies/stlplus3/filesystemSimplified/file_system.hpp"

#include <string>
#include <sstream>
#include <vector>

using namespace aliceVision;
using namespace aliceVision::sfm;

static bool parseAlignScale(const std::string& alignScale, double& S, Mat3& R, Vec3& t);

// Convert from a SfMData format to another
int main(int argc, char **argv)
{
  CmdLine cmd;

  std::string sSfMData_Filename_In;
  std::string sSfMData_Filename_InRef;
  std::string sSfMData_Filename_Out;
  std::string sSfm_Data_YAlignScale;

  cmd.add(make_option('i', sSfMData_Filename_In, "input_file"));
  cmd.add(make_option('r', sSfMData_Filename_InRef, "reference_file"));
  cmd.add(make_option('y', sSfm_Data_YAlignScale, "y_align_scale"));
  cmd.add(make_option('o', sSfMData_Filename_Out, "output_file"));

  try {
      if (argc == 1) throw std::string("Invalid command line parameter.");
      cmd.process(argc, argv);
  } catch(const std::string& s) {
      std::cerr << "Usage: " << argv[0] << '\n'
        << "[-i|--input_file] path to the input SfMData scene to align.\n"
        << "[-o|--output_file] path to the output SfMData scene\n"
        << "\t .json, .bin, .xml, .ply, .baf"
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_ALEMBIC)
           ", .abc"
#endif
           "\n"
        << "\n[Optional]\n"
        << "[-r|--reference_file] path to the scene used as the reference coordinate system\n"
        << "[-y|--y_align_scale] align [X,Y,Z] to +Y-axis, rotate around Y by R deg, scale by S; syntax: X,Y,Z;R;S \n"
        << std::endl;

      std::cerr << s << std::endl;
      return EXIT_FAILURE;
  }
  

  if (sSfMData_Filename_In.empty() ||
      sSfMData_Filename_Out.empty())
  {
    std::cerr << "Invalid input or output filename." << std::endl;
    return EXIT_FAILURE;
  }

  if (sSfMData_Filename_InRef.empty() &&
      sSfm_Data_YAlignScale.empty())
  {
    std::cerr << "At least one of -y and -r must be specified." << std::endl;
    return EXIT_FAILURE;
  }
  
  if (!sSfMData_Filename_InRef.empty() &&
      !sSfm_Data_YAlignScale.empty())
  {
    std::cerr << "Must specify exactly one of alignment and reference scene." << std::endl;
    return EXIT_FAILURE;
  }

  // Load input scene
  SfMData sfm_data_in;
  if (!Load(sfm_data_in, sSfMData_Filename_In, ESfMData(ALL)))
  {
    std::cerr << std::endl
      << "The input SfMData file \"" << sSfMData_Filename_In << "\" cannot be read." << std::endl;
    return EXIT_FAILURE;
  }

  double S;
  Mat3 R;
  Vec3 t;

  if (!sSfMData_Filename_InRef.empty())
  {
    // Load reference scene
    SfMData sfm_data_inRef;
    if (!Load(sfm_data_inRef, sSfMData_Filename_InRef, ESfMData(ALL)))
    {
      std::cerr << std::endl
        << "The reference SfMData file \"" << sSfMData_Filename_InRef << "\" cannot be read." << std::endl;
      return EXIT_FAILURE;
    }

    std::cout << "Search similarity transformation." << std::endl;
    bool hasValidSimilarity = computeSimilarity(sfm_data_in, sfm_data_inRef, &S, &R, &t);
    if(!hasValidSimilarity)
    {
      std::cerr << std::endl
        << "Failed to find similarity between the 2 SfM scenes:"
        << "\"" << sSfMData_Filename_In << "\", "
        << "\"" << sSfMData_Filename_InRef << "\""
        << std::endl;
      return EXIT_FAILURE;
    }
  }
  else if (!parseAlignScale(sSfm_Data_YAlignScale, S, R, t))
  {
    std::cerr << std::endl << "Failed to parse align/scale argument.";
    return EXIT_FAILURE;
  }
  
  std::cout << "Apply transformation:" << std::endl;
  std::cout << " - Scale: " << S << std::endl;
  std::cout << " - Rotation:\n" << R << std::endl;
  std::cout << " - Translate: " << t.transpose() << std::endl;

  applyTransform(sfm_data_in, S, R, t);
  
  std::cout << "Save into \"" << sSfMData_Filename_Out << "\"" << std::endl;
  
  // Export the SfMData scene in the expected format
  if (!Save(sfm_data_in, sSfMData_Filename_Out, ESfMData(ALL)))
  {
    std::cerr << std::endl
      << "An error occurred while trying to save \"" << sSfMData_Filename_Out << "\"." << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
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
  
  auto q = ::Eigen::Quaterniond::FromTwoVectors(Vec3(rx, ry, rz), Vec3::UnitY());
  auto r = ::Eigen::AngleAxisd(rr*M_PI/180, Vec3::UnitY());

  R = r * q.toRotationMatrix();

  t = Vec3::Zero();
  
  return true;
}
