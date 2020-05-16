// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfm/sfm.hpp>
#include <aliceVision/sfm/BundleAdjustmentCeres.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/config.hpp>

#include <uncertaintyTE/uncertainty.h>
#include <uncertaintyTE/IO.h>

#include <boost/program_options.hpp>

#include <string>
#include <sstream>
#include <vector>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;
using namespace aliceVision::sfm;
using namespace aliceVision::sfmData;
using namespace aliceVision::sfmDataIO;
namespace po = boost::program_options;


int aliceVision_main(int argc, char **argv)
{
  // command-line parameters

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string sfmDataFilename;
  std::string outSfMDataFilename;
  std::string outputStats;
  std::string algorithm = cov::EAlgorithm_enumToString(cov::eAlgorithmSvdTaylorExpansion);
  bool debug = false;

  po::options_description params("AliceVision Uncertainty");
  params.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
      "SfMData file to align.")
  ("output,o", po::value<std::string>(&outSfMDataFilename)->required(),
    "Output SfMData scene.")
  ("outputCov,c", po::value<std::string>(&outputStats),
    "Output covariances file.")
  ("algorithm,a", po::value<std::string>(&algorithm)->default_value(algorithm),
    "Algorithm.")
  ("debug,d", po::value<bool>(&debug)->default_value(debug),
    "Enable creation of debug files in the current folder.")
    ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
      "verbosity level (fatal,  error, warning, info, debug, trace).");

  po::variables_map vm;
  try
  {
    po::store(po::parse_command_line(argc, argv, params), vm);

    if(vm.count("help") || (argc == 1))
    {
      ALICEVISION_COUT(params);
      return EXIT_SUCCESS;
    }
    po::notify(vm);
  }
  catch(boost::program_options::required_option& e)
  {
    ALICEVISION_CERR("ERROR: " << e.what());
    ALICEVISION_COUT("Usage:\n\n" << params);
    return EXIT_FAILURE;
  }
  catch(boost::program_options::error& e)
  {
    ALICEVISION_CERR("ERROR: " << e.what());
    ALICEVISION_COUT("Usage:\n\n" << params);
    return EXIT_FAILURE;
  }
  ALICEVISION_COUT(vm);

  // set verbose level
  system::Logger::get()->setLogLevel(verboseLevel);
  
  if (sfmDataFilename.empty() ||
      outSfMDataFilename.empty())
  {
    std::cerr << "Invalid input or output filename." << std::endl;
    return EXIT_FAILURE;
  }

  // Load input scene
  SfMData sfmData;
  if (!Load(sfmData, sfmDataFilename, ESfMData(ALL)))
  {
    std::cerr << std::endl
      << "The input SfMData file \"" << sfmDataFilename << "\" cannot be read." << std::endl;
    return EXIT_FAILURE;
  }

  ceres::CRSMatrix jacobian;
  {
    BundleAdjustmentCeres bundleAdjustmentObj;
    BundleAdjustment::ERefineOptions refineOptions = BundleAdjustment::REFINE_ROTATION | BundleAdjustment::REFINE_TRANSLATION | BundleAdjustment::REFINE_STRUCTURE;
    bundleAdjustmentObj.createJacobian(sfmData, refineOptions, jacobian);
  }

  {
    cov::Options options;
    // Configure covariance engine (find the indexes of the most distatnt points etc.)
    // setPts2Fix(opt, mutable_points.size() / 3, mutable_points.data());
    options._numCams = sfmData.getValidViews().size();
    options._camParams = 6;
    options._numPoints = sfmData.structure.size();
    options._numObs = jacobian.num_rows / 2;
    options._algorithm = cov::EAlgorithm_stringToEnum(algorithm);
    options._epsilon = 1e-10;
    options._lambda = -1;
    options._svdRemoveN = -1;
    options._maxIterTE = -1;
    options._debug = debug;

    cov::Statistic statistic;
    std::vector<double> points3D;
    points3D.reserve(sfmData.structure.size() * 3);
    for(auto& landmarkIt: sfmData.structure)
    {
      double* p = landmarkIt.second.X.data();
      points3D.push_back(p[0]);
      points3D.push_back(p[1]);
      points3D.push_back(p[2]);
    }

    cov::Uncertainty uncertainty;

    getCovariances(options, statistic, jacobian, &points3D[0], uncertainty);

    if(!outputStats.empty())
      saveResults(outputStats, options, statistic, uncertainty);

    {
      const std::vector<double> posesUncertainty = uncertainty.getCamerasUncEigenValues();

      std::size_t indexPose = 0;
      for (Poses::const_iterator itPose = sfmData.getPoses().begin(); itPose != sfmData.getPoses().end(); ++itPose, ++indexPose)
      {
        const IndexT idPose = itPose->first;
        Vec6& u = sfmData._posesUncertainty[idPose]; // create uncertainty entry
        const double* uIn = &posesUncertainty[indexPose*6];
        u << uIn[0],  uIn[1],  uIn[2],  uIn[3],  uIn[4],  uIn[5];
      }
    }
    {
      const std::vector<double> landmarksUncertainty = uncertainty.getPointsUncEigenValues();

      std::size_t indexLandmark = 0;
      for (Landmarks::const_iterator itLandmark = sfmData.getLandmarks().begin(); itLandmark != sfmData.getLandmarks().end(); ++itLandmark, ++indexLandmark)
      {
        const IndexT idLandmark = itLandmark->first;
        Vec3& u = sfmData._landmarksUncertainty[idLandmark]; // create uncertainty entry
        const double* uIn = &landmarksUncertainty[indexLandmark*3];
        u << uIn[0],  uIn[1],  uIn[2];
      }
    }
  }

  std::cout << "Save into \"" << outSfMDataFilename << "\"" << std::endl;
  
  // Export the SfMData scene in the expected format
  if (!Save(sfmData, outSfMDataFilename, ESfMData(ALL)))
  {
    std::cerr << std::endl
      << "An error occurred while trying to save \"" << outSfMDataFilename << "\"." << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
