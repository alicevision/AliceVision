// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/delaunaycut/mv_delaunay_GC.hpp>
#include <aliceVision/delaunaycut/mv_delaunay_meshSmooth.hpp>
#include <aliceVision/largeScale/reconstructionPlan.hpp>
#include <aliceVision/planeSweeping/ps_refine_rc.hpp>
#include <aliceVision/CUDAInterfaces/refine.hpp>
#include <aliceVision/common/fileIO.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>


namespace bfs = boost::filesystem;
namespace po = boost::program_options;

#define ALICEVISION_COUT(x) std::cout << x << std::endl
#define ALICEVISION_CERR(x) std::cerr << x << std::endl
#define EXIT_FAILURE -1;


int main(int argc, char* argv[])
{
    long startTime = clock();

    std::string iniFilepath;
    int rangeStart = -1;
    int rangeSize = -1;
    int minNumOfConsistensCams = 3;
    po::options_description inputParams("Filter depth map to remove values that are not consistent with other depth maps.");

    inputParams.add_options()
        ("ini", po::value<std::string>(&iniFilepath)->required(),
            "Configuration file (mvs.ini).")
        ("rangeStart", po::value<int>(&rangeStart)->default_value(rangeStart),
            "Compute only a sub-range of images from index rangeStart to rangeStart+rangeSize.")
        ("rangeSize", po::value<int>(&rangeSize)->default_value(rangeSize),
            "Compute only a sub-range of N images (N=rangeSize).")
        ("minNumOfConsistensCams", po::value<int>(&minNumOfConsistensCams)->default_value(minNumOfConsistensCams),
            "Minimal number of consistent cameras to consider the pixel.");
    po::variables_map vm;

    try
    {
      po::store(po::parse_command_line(argc, argv, inputParams), vm);

      if(vm.count("help") || (argc == 1))
      {
        ALICEVISION_COUT(inputParams);
        return EXIT_SUCCESS;
      }

      po::notify(vm);
    }
    catch(boost::program_options::required_option& e)
    {
      ALICEVISION_CERR("ERROR: " << e.what() << std::endl);
      ALICEVISION_COUT("Usage:\n\n" << inputParams);
      return EXIT_FAILURE;
    }
    catch(boost::program_options::error& e)
    {
      ALICEVISION_CERR("ERROR: " << e.what() << std::endl);
      ALICEVISION_COUT("Usage:\n\n" << inputParams);
      return EXIT_FAILURE;
    }

    ALICEVISION_COUT("ini file: " << iniFilepath);

    // .ini parsing
    multiviewInputParams mip(iniFilepath);
    const double simThr = mip._ini.get<double>("global.simThr", 0.0);
    multiviewParams mp(mip.getNbCameras(), &mip, (float) simThr);
    mv_prematch_cams pc(&mp);

    staticVector<int> cams(mp.ncams);
    if(rangeSize == -1)
    {
        for(int rc = 0; rc < mp.ncams; rc++) // process all cameras
            cams.push_back(rc);
    }
    else
    {
        if(rangeStart < 0)
        {
            ALICEVISION_CERR("invalid subrange of cameras to process.");
            return EXIT_FAILURE;
        }
        for(int rc = rangeStart; rc < std::min(rangeStart + rangeSize, mp.ncams); ++rc)
            cams.push_back(rc);
        if(cams.empty())
        {
            ALICEVISION_COUT("No camera to process");
            return EXIT_SUCCESS;
        }
    }

    ALICEVISION_COUT("--- filter depthmap");
    {
        mv_fuse fs(&mp, &pc);
        fs.filterGroups(cams);
        fs.filterDepthMaps(cams, minNumOfConsistensCams);
    }

    printfElapsedTime(startTime, "#");
    return EXIT_SUCCESS;
}
