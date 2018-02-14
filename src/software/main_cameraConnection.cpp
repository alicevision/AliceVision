// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/delaunayCut/DelaunayGraphCut.hpp>
#include <aliceVision/delaunayCut/meshPostProcessing.hpp>
#include <aliceVision/largeScale/ReconstructionPlan.hpp>
#include <aliceVision/depthMap/RefineRc.hpp>
#include <aliceVision/common/fileIO.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

namespace bfs = boost::filesystem;
namespace po = boost::program_options;

#define ALICEVISION_COUT(x) std::cout << x << std::endl
#define ALICEVISION_CERR(x) std::cerr << x << std::endl


int main(int argc, char* argv[])
{
    long startTime = clock();

    std::string iniFilepath;

    po::options_description allParams("AliceVision cameraConnection\n"
                                      "Select best neighboring cameras of each camera");

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("ini", po::value<std::string>(&iniFilepath)->required(),
            "Configuration file (mvs.ini).");

    allParams.add(requiredParams);

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
      ALICEVISION_CERR("ERROR: " << e.what() << std::endl);
      ALICEVISION_COUT("Usage:\n\n" << allParams);
      return EXIT_FAILURE;
    }
    catch(boost::program_options::error& e)
    {
      ALICEVISION_CERR("ERROR: " << e.what() << std::endl);
      ALICEVISION_COUT("Usage:\n\n" << allParams);
      return EXIT_FAILURE;
    }

    ALICEVISION_COUT("ini file: " << iniFilepath);

    // .ini parsing
    MultiViewInputParams mip(iniFilepath, "", "");
    const double simThr = mip._ini.get<double>("global.simThr", 0.0);
    MultiViewParams mp(mip.getNbCameras(), &mip, (float) simThr);
    PreMatchCams pc(&mp);

    ALICEVISION_COUT("--- compute camera pairs");
    pc.precomputeIncidentMatrixCamsFromSeeds();

    printfElapsedTime(startTime, "#");
    return EXIT_SUCCESS;
}
