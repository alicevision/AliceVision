// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// Copyright (c) 2015 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/sfm/sfm.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/matchingImageCollection/ImagePairListIO.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/main.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <cstdlib>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;
using namespace aliceVision::sfm;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

/// Build a list of pair that share visibility content from the SfMData structure
PairSet BuildPairsFromStructureObservations(const sfmData::SfMData& sfmData)
{
  PairSet pairs;

  for (sfmData::Landmarks::const_iterator itL = sfmData.getLandmarks().begin();
    itL != sfmData.getLandmarks().end(); ++itL)
  {
    const sfmData::Landmark & landmark = itL->second;
    for(const auto& iterI : landmark.getObservations())
    {
      const IndexT id_viewI = iterI.first;
      sfmData::Observations::const_iterator iterJ = landmark.getObservations().begin();
      std::advance(iterJ, 1);
      for (; iterJ != landmark.getObservations().end(); ++iterJ)
      {
        const IndexT id_viewJ = iterJ->first;
        pairs.insert( std::make_pair(id_viewI,id_viewJ));
      }
    }
  }
  return pairs;
}

/// Build a list of pair from the camera frusta intersections
PairSet BuildPairsFromFrustumsIntersections(
  const sfmData::SfMData & sfmData,
  const double z_near = -1., // default near plane
  const double z_far = -1.,  // default far plane
  const std::string& sOutDirectory = "") // output folder to save frustums as PLY
{
  const FrustumFilter frustum_filter(sfmData, z_near, z_far);
  if (!sOutDirectory.empty())
    frustum_filter.export_Ply((fs::path(sOutDirectory) / "frustums.ply").string());
  return frustum_filter.getFrustumIntersectionPairs();
}

int aliceVision_main(int argc, char **argv)
{
  // command-line parameters
  std::string sfmDataFilename;
  std::string outputFilename;
  double zNear = -1.;
  double zFar = -1.;

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
      "SfMData file.")
    ("output,o", po::value<std::string>(&outputFilename)->required(),
      "Output pair filename.");

  po::options_description optionalParams("Optional parameters");
  optionalParams.add_options()
    ("zNear", po::value<double>(&zNear)->default_value(zNear),
      "Distance of the near camera plane.")
    ("zFar", po::value<double>(&zFar)->default_value(zFar),
      "Distance of the far camera plane.");

  CmdLine cmdline("This program computes camera cones that share some putative visual content.\n"
                  "AliceVision frustumFiltering");
  cmdline.add(requiredParams);
  cmdline.add(optionalParams);
  if (!cmdline.execute(argc, argv))
  {
      return EXIT_FAILURE;
  }

  // check that we can create the output folder
  if(!fs::exists(fs::path(outputFilename).parent_path()))
    if(!fs::exists(fs::path(outputFilename).parent_path()))
      return EXIT_FAILURE;

  // load input SfMData scene
  sfmData::SfMData sfmData;
  if(!sfmDataIO::Load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
  {
    ALICEVISION_LOG_ERROR("The input SfMData file '"<< sfmDataFilename << "' cannot be read");
    return EXIT_FAILURE;
  }

  aliceVision::system::Timer timer;

  const PairSet pairs = BuildPairsFromFrustumsIntersections(sfmData, zNear, zFar, fs::path(outputFilename).parent_path().string());
  /*const PairSet pairs = BuildPairsFromStructureObservations(sfm_data); */

  ALICEVISION_LOG_INFO("# pairs: " << pairs.size());
  ALICEVISION_LOG_INFO("Pair filtering took: " << timer.elapsed() << " s");

  // export pairs on disk
  if (matchingImageCollection::savePairsToFile(outputFilename, pairs))
    return EXIT_SUCCESS;
  else
    return EXIT_FAILURE;
}
