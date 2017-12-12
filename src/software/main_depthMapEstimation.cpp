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


int main(int argc, char* argv[])
{
    long startTime = clock();

    std::string iniFilepath;
    std::string outputFolder;
    int rangeStart = -1;
    int rangeSize = -1;

    po::options_description allParams("AliceVision depthMapEstimation\n"
                                      "Estimate depth map for each input image");

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("ini", po::value<std::string>(&iniFilepath)->required(),
            "Configuration file (mvs.ini).")
        ("output,o", po::value<std::string>(&outputFolder)->required(),
            "Output folder for generated depth maps.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("rangeStart", po::value<int>(&rangeStart)->default_value(rangeStart),
            "Compute a sub-range of images from index rangeStart to rangeStart+rangeSize.")
        ("rangeSize", po::value<int>(&rangeSize)->default_value(rangeSize),
         "Compute a sub-range of N images (N=rangeSize).");

    allParams.add(requiredParams).add(optionalParams);

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
    multiviewInputParams mip(iniFilepath, outputFolder, "");
    const double simThr = mip._ini.get<double>("global.simThr", 0.0);

    mip._ini.put("semiGlobalMatching.maxTCams", 10);
    mip._ini.put("semiGlobalMatching.wsh", 4);
    mip._ini.put("semiGlobalMatching.gammaC", 5.5);
    mip._ini.put("semiGlobalMatching.gammaP", 8.0);

    mip._ini.put("refineRc.nSamplesHalf", 150);
    mip._ini.put("refineRc.ndepthsToRefine", 31);
    mip._ini.put("refineRc.sigma", 15.0);
    mip._ini.put("refineRc.niters", 100);
    mip._ini.put("refineRc.useTcOrRcPixSize", false);
    mip._ini.put("refineRc.wsh", 3);
    mip._ini.put("refineRc.gammaC", 15.5);
    mip._ini.put("refineRc.gammaP", 8.0);
    mip._ini.put("refineRc.maxTCams", 6);

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

    ALICEVISION_COUT("--- create depthmap");

    computeDepthMapsPSSGM(&mp, &pc, cams);
    refineDepthMaps(&mp, &pc, cams);

    printfElapsedTime(startTime, "#");
    return EXIT_SUCCESS;
}
