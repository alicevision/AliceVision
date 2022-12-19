// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// Copyright (c) 2015 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfm/FrustumFilter.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/main.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

/// Export camera frustrums as a triangle PLY file
int aliceVision_main(int argc, char **argv)
{
  // command-line parameters
  std::string sfmDataFilename;
  std::string plyOutFilename;

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
      "SfMData file.")
    ("output,o", po::value<std::string>(&plyOutFilename)->required(),
      "PLY file to store the camera frustums as triangle meshes.");

  CmdLine cmdline("Export camera frustrums as a triangle PLY file.\n"
                  "AliceVision exportCameraFrustums");
  cmdline.add(requiredParams);
  if (!cmdline.execute(argc, argv))
  {
      return EXIT_FAILURE;
  }

  // load input SfMData scene
  sfmData::SfMData sfmData;
  if(!sfmDataIO::Load(sfmData, sfmDataFilename, sfmDataIO::ESfMData(sfmDataIO::VIEWS|sfmDataIO::INTRINSICS|sfmDataIO::EXTRINSICS)))
  {
    ALICEVISION_LOG_ERROR("The input SfMData file '"<< sfmDataFilename << "' cannot be read.");
    return EXIT_FAILURE;
  }

  // check that we can create the output folder/file
  if(!fs::exists(fs::path(plyOutFilename).parent_path()))
    if(!fs::create_directory(fs::path(plyOutFilename).parent_path()))
      return EXIT_FAILURE;

  // if sfmData have not structure, cameras are displayed as tiny normalized cones
  const sfm::FrustumFilter frustumFilter(sfmData);
  if(!plyOutFilename.empty())
  {
    if(frustumFilter.export_Ply(plyOutFilename))
      return EXIT_SUCCESS;
  }

  return EXIT_FAILURE;
}
