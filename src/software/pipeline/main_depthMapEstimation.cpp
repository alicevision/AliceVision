// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/mvsData/StaticVector.hpp>
#include <aliceVision/mvsUtils/common.hpp>
#include <aliceVision/mvsUtils/MultiViewParams.hpp>
#include <aliceVision/mvsUtils/PreMatchCams.hpp>
#include <aliceVision/depthMap/RefineRc.hpp>
#include <aliceVision/depthMap/SemiGlobalMatchingRc.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

using namespace aliceVision;
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

    // semiGlobalMatching
    int sgmMaxTCams = 10;
    int sgmWSH = 4;
    double sgmGammaC = 5.5;
    double sgmGammaP = 8.0;

    // refineRc
    int refineNSamplesHalf = 150;
    int refineNDepthsToRefine = 31;
    int refineNiters = 100;
    int refineWSH = 3;
    int refineMaxTCams = 6;
    double refineSigma = 15.0;
    double refineGammaC = 15.5;
    double refineGammaP = 8.0;
    bool refineUseTcOrRcPixSize = false;

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
            "Compute a sub-range of N images (N=rangeSize).")
        ("sgmMaxTCams", po::value<int>(&sgmMaxTCams)->default_value(sgmMaxTCams),
            "Semi Global Matching: Number of neighbour cameras.")
        ("sgmWSH", po::value<int>(&sgmWSH)->default_value(sgmWSH),
            "Semi Global Matching: Size of the patch used to compute the similarity.")
        ("sgmGammaC", po::value<double>(&sgmGammaC)->default_value(sgmGammaC),
            "Semi Global Matching: GammaC threshold.")
        ("sgmGammaP", po::value<double>(&sgmGammaP)->default_value(sgmGammaP),
            "Semi Global Matching: GammaP threshold.")
        ("refineNSamplesHalf", po::value<int>(&refineNSamplesHalf)->default_value(refineNSamplesHalf),
            "Refine: Number of samples.")
        ("refineNDepthsToRefine", po::value<int>(&refineNDepthsToRefine)->default_value(refineNDepthsToRefine),
            "Refine: Number of depths.")
        ("refineNiters", po::value<int>(&refineNiters)->default_value(refineNiters),
            "Refine: Number of iterations.")
        ("refineWSH", po::value<int>(&refineWSH)->default_value(refineWSH),
            "Refine: Size of the patch used to compute the similarity.")
        ("refineMaxTCams", po::value<int>(&refineMaxTCams)->default_value(refineMaxTCams),
            "Refine: Number of neighbour cameras.")
        ("refineSigma", po::value<double>(&refineSigma)->default_value(refineSigma),
            "Refine: Sigma threshold.")
        ("refineGammaC", po::value<double>(&refineGammaC)->default_value(refineGammaC),
            "Refine: GammaC threshold.")
        ("refineGammaP", po::value<double>(&refineGammaP)->default_value(refineGammaP),
            "Refine: GammaP threshold.")
        ("refineUseTcOrRcPixSize", po::value<bool>(&refineUseTcOrRcPixSize)->default_value(refineUseTcOrRcPixSize),
            "Refine: Use current camera pixel size or minimum pixel size of neighbour cameras.");

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
    mvsUtils::MultiViewInputParams mip(iniFilepath, outputFolder, "");
    const double simThr = mip._ini.get<double>("global.simThr", 0.0);

    // set params in bpt

    // semiGlobalMatching
    mip._ini.put("semiGlobalMatching.maxTCams", sgmMaxTCams);
    mip._ini.put("semiGlobalMatching.wsh", sgmWSH);
    mip._ini.put("semiGlobalMatching.gammaC", sgmGammaC);
    mip._ini.put("semiGlobalMatching.gammaP", sgmGammaP);

    // refineRc
    mip._ini.put("refineRc.nSamplesHalf", refineNSamplesHalf);
    mip._ini.put("refineRc.ndepthsToRefine", refineNDepthsToRefine);
    mip._ini.put("refineRc.niters", refineNiters);
    mip._ini.put("refineRc.wsh", refineWSH);
    mip._ini.put("refineRc.maxTCams", refineMaxTCams);
    mip._ini.put("refineRc.sigma", refineSigma);
    mip._ini.put("refineRc.gammaC", refineGammaC);
    mip._ini.put("refineRc.gammaP", refineGammaP);
    mip._ini.put("refineRc.useTcOrRcPixSize", refineUseTcOrRcPixSize);

    mvsUtils::MultiViewParams mp(mip.getNbCameras(), &mip, (float) simThr);
    mvsUtils::PreMatchCams pc(&mp);

    StaticVector<int> cams;
    cams.reserve(mp.ncams);
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

    depthMap::computeDepthMapsPSSGM(&mp, &pc, cams);
    depthMap::refineDepthMaps(&mp, &pc, cams);

    mvsUtils::printfElapsedTime(startTime, "#");
    return EXIT_SUCCESS;
}
