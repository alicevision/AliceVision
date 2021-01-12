// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/depthMap/RefineRc.hpp>
#include <aliceVision/depthMap/SemiGlobalMatchingRc.hpp>
#include <aliceVision/mvsData/StaticVector.hpp>
#include <aliceVision/mvsUtils/common.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/mvsUtils/MultiViewParams.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/gpu/gpu.hpp>
#include <aliceVision/depthMap/computeOnMultiGPUs.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 2
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;

int aliceVision_main(int argc, char* argv[])
{
    ALICEVISION_COMMANDLINE_START

    std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
    std::string sfmDataFilename;
    std::string outputFolder;
    std::string imagesFolder;

    // program range
    int rangeStart = -1;
    int rangeSize = -1;

    // image downscale factor during process
    int downscale = 2;

    // min / max view angle
    float minViewAngle = 2.0f;
    float maxViewAngle = 70.0f;

    // semiGlobalMatching
    int sgmScale = -1;
    int sgmStepXY = -1;
    int sgmStepZ = -1;
    int sgmMaxSideXY = 700;
    int sgmMaxTCams = 10;
    int sgmWSH = 4;
    double sgmGammaC = 5.5;
    double sgmGammaP = 8.0;
    double sgmP1 = 10;
    double sgmP2 = 20.0;
    int sgmMaxDepths = 3000;
    int sgmMaxDepthsPerTc = 1500;
    bool sgmUseSfmSeeds = true;
    std::string sgmFilteringAxes = "YX";

    // refineRc
    int refineMaxTCams = 6;
    int refineNSamplesHalf = 150;
    int refineNDepthsToRefine = 31;
    int refineNiters = 100;
    int refineWSH = 3;
    double refineSigma = 15.0;
    double refineGammaC = 15.5;
    double refineGammaP = 8.0;
    bool refineUseTcOrRcPixSize = false;

    // intermediate results
    bool exportIntermediateResults = false;

    // number of GPUs to use (0 means use all GPUs)
    int nbGPUs = 0;

    po::options_description allParams("AliceVision depthMapEstimation\n"
                                      "Estimate depth map for each input image");

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
            "SfMData file.")
        ("imagesFolder", po::value<std::string>(&imagesFolder)->required(),
            "Images folder. Filename should be the image uid.")
        ("output,o", po::value<std::string>(&outputFolder)->required(),
            "Output folder for generated depth maps.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("rangeStart", po::value<int>(&rangeStart)->default_value(rangeStart),
            "Compute a sub-range of images from index rangeStart to rangeStart+rangeSize.")
        ("rangeSize", po::value<int>(&rangeSize)->default_value(rangeSize),
            "Compute a sub-range of N images (N=rangeSize).")
        ("downscale", po::value<int>(&downscale)->default_value(downscale),
            "Image downscale factor.")
        ("minViewAngle", po::value<float>(&minViewAngle)->default_value(minViewAngle),
            "minimum angle between two views.")
        ("maxViewAngle", po::value<float>(&maxViewAngle)->default_value(maxViewAngle),
            "maximum angle between two views.")
        ("sgmScale", po::value<int>(&sgmScale)->default_value(sgmScale),
            "Semi Global Matching: Downscale factor used to compute the similarity volume.")
        ("sgmStepXY", po::value<int>(&sgmStepXY)->default_value(sgmStepXY),
            "Semi Global Matching: Step used to compute the similarity volume on the X and Y axis.")
        ("sgmStepZ", po::value<int>(&sgmStepZ)->default_value(sgmStepZ),
            "Semi Global Matching: Step used to compute the similarity volume on the Z axis.")
        ("sgmMaxSideXY", po::value<int>(&sgmMaxSideXY)->default_value(sgmMaxSideXY),
            "Semi Global Matching: Max side in pixels used to automatically decide for sgmScale/sgmStepXY if not defined.")
        ("sgmMaxTCams", po::value<int>(&sgmMaxTCams)->default_value(sgmMaxTCams),
            "Semi Global Matching: Number of neighbour cameras.")
        ("sgmWSH", po::value<int>(&sgmWSH)->default_value(sgmWSH),
            "Semi Global Matching: Size of the patch used to compute the similarity.")
        ("sgmGammaC", po::value<double>(&sgmGammaC)->default_value(sgmGammaC),
            "Semi Global Matching: GammaC threshold.")
        ("sgmGammaP", po::value<double>(&sgmGammaP)->default_value(sgmGammaP),
            "Semi Global Matching: GammaP threshold.")
        ("sgmP1", po::value<double>(&sgmP1)->default_value(sgmP1),
            "Semi Global Matching: P1.")
        ("sgmP2", po::value<double>(&sgmP2)->default_value(sgmP2),
            "Semi Global Matching: P2.")
        ("sgmMaxDepths", po::value<int>(&sgmMaxDepths)->default_value(sgmMaxDepths),
            "Semi Global Matching: Max number of depths in the overall similarity volume.")
        ("sgmMaxDepthsPerTc", po::value<int>(&sgmMaxDepthsPerTc)->default_value(sgmMaxDepthsPerTc),
            "Semi Global Matching: Max number of depths to sweep in the similarity volume per Rc/Tc cameras.")
        ("sgmUseSfmSeeds", po::value<bool>(&sgmUseSfmSeeds)->default_value(sgmUseSfmSeeds),
            "Semi Global Matching: Use landmarks from SfM to define the ranges for the plane sweeping.")
        ("sgmFilteringAxes", po::value<std::string>(&sgmFilteringAxes)->default_value(sgmFilteringAxes),
            "Semi Global Matching: Filtering axes for the 3D volume.")
        ("refineMaxTCams", po::value<int>(&refineMaxTCams)->default_value(refineMaxTCams),
            "Refine: Number of neighbour cameras.")
        ("refineNSamplesHalf", po::value<int>(&refineNSamplesHalf)->default_value(refineNSamplesHalf),
            "Refine: Number of samples.")
        ("refineNDepthsToRefine", po::value<int>(&refineNDepthsToRefine)->default_value(refineNDepthsToRefine),
            "Refine: Number of depths.")
        ("refineNiters", po::value<int>(&refineNiters)->default_value(refineNiters),
            "Refine: Number of iterations.")
        ("refineWSH", po::value<int>(&refineWSH)->default_value(refineWSH),
            "Refine: Size of the patch used to compute the similarity.")
        ("refineSigma", po::value<double>(&refineSigma)->default_value(refineSigma),
            "Refine: Sigma threshold.")
        ("refineGammaC", po::value<double>(&refineGammaC)->default_value(refineGammaC),
            "Refine: GammaC threshold.")
        ("refineGammaP", po::value<double>(&refineGammaP)->default_value(refineGammaP),
            "Refine: GammaP threshold.")
        ("refineUseTcOrRcPixSize", po::value<bool>(&refineUseTcOrRcPixSize)->default_value(refineUseTcOrRcPixSize),
            "Refine: Use current camera pixel size or minimum pixel size of neighbour cameras.")
        ("exportIntermediateResults", po::value<bool>(&exportIntermediateResults)->default_value(exportIntermediateResults),
            "Export intermediate results from the SGM and Refine steps.")
        ("nbGPUs", po::value<int>(&nbGPUs)->default_value(nbGPUs),
            "Number of GPUs to use (0 means use all GPUs).");

    po::options_description logParams("Log parameters");
    logParams.add_options()
      ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
        "verbosity level (fatal, error, warning, info, debug, trace).");

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

    ALICEVISION_COUT("Program called with the following parameters:");
    ALICEVISION_COUT(vm);

    // set verbose level
    system::Logger::get()->setLogLevel(verboseLevel);

    // print GPU Information
    ALICEVISION_LOG_INFO(gpu::gpuInformationCUDA());

    // check if the gpu suppport CUDA compute capability 2.0
    if(!gpu::gpuSupportCUDA(2,0))
    {
      ALICEVISION_LOG_ERROR("This program needs a CUDA-Enabled GPU (with at least compute capability 2.0).");
      return EXIT_FAILURE;
    }

    // check if the scale is correct
    if(downscale < 1)
    {
      ALICEVISION_LOG_ERROR("Invalid value for downscale parameter. Should be at least 1.");
      return EXIT_FAILURE;
    }

    // read the input SfM scene
    sfmData::SfMData sfmData;
    if(!sfmDataIO::Load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
    {
      ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' cannot be read.");
      return EXIT_FAILURE;
    }

    // initialization
    mvsUtils::MultiViewParams mp(sfmData, imagesFolder, outputFolder, "", false, downscale);

    mp.setMinViewAngle(minViewAngle);
    mp.setMaxViewAngle(maxViewAngle);

    // set params in bpt

    // semiGlobalMatching
    mp.userParams.put("semiGlobalMatching.maxTCams", sgmMaxTCams);
    mp.userParams.put("semiGlobalMatching.wsh", sgmWSH);
    mp.userParams.put("semiGlobalMatching.gammaC", sgmGammaC);
    mp.userParams.put("semiGlobalMatching.gammaP", sgmGammaP);
    mp.userParams.put("semiGlobalMatching.P1", sgmP1);
    mp.userParams.put("semiGlobalMatching.P2", sgmP2);

    mp.userParams.put("semiGlobalMatching.scale", sgmScale);
    mp.userParams.put("semiGlobalMatching.stepXY", sgmStepXY);
    mp.userParams.put("semiGlobalMatching.stepZ", sgmStepZ);
    mp.userParams.put("semiGlobalMatching.maxSideXY", sgmMaxSideXY);

    mp.userParams.put("semiGlobalMatching.maxDepthsToStore", sgmMaxDepths);
    mp.userParams.put("semiGlobalMatching.maxDepthsToSweep", sgmMaxDepthsPerTc);
    mp.userParams.put("semiGlobalMatching.useSeedsToCompDepthsToSweep", sgmUseSfmSeeds);

    mp.userParams.put("semiGlobalMatching.filteringAxes", sgmFilteringAxes);

    // refineRc
    mp.userParams.put("refineRc.maxTCams", refineMaxTCams);
    mp.userParams.put("refineRc.nSamplesHalf", refineNSamplesHalf);
    mp.userParams.put("refineRc.ndepthsToRefine", refineNDepthsToRefine);
    mp.userParams.put("refineRc.niters", refineNiters);
    mp.userParams.put("refineRc.wsh", refineWSH);
    mp.userParams.put("refineRc.sigma", refineSigma);
    mp.userParams.put("refineRc.gammaC", refineGammaC);
    mp.userParams.put("refineRc.gammaP", refineGammaP);
    mp.userParams.put("refineRc.useTcOrRcPixSize", refineUseTcOrRcPixSize);

    // intermediate results
    mp.userParams.put("depthMap.intermediateResults", exportIntermediateResults);

    std::vector<int> cams;
    cams.reserve(mp.ncams);
    if(rangeSize == -1)
    {
      for(int rc = 0; rc < mp.ncams; ++rc) // process all cameras
        cams.push_back(rc);
    }
    else
    {
      if(rangeStart < 0)
      {
        ALICEVISION_LOG_ERROR("invalid subrange of cameras to process.");
        return EXIT_FAILURE;
      }
      for(int rc = rangeStart; rc < std::min(rangeStart + rangeSize, mp.ncams); ++rc)
        cams.push_back(rc);
      if(cams.empty())
      {
        ALICEVISION_LOG_INFO("No camera to process.");
        return EXIT_SUCCESS;
      }
    }

    ALICEVISION_LOG_INFO("Create depth maps.");
    depthMap::computeOnMultiGPUs(mp, cams, depthMap::estimateAndRefineDepthMaps, nbGPUs);

    ALICEVISION_COMMANDLINE_END
}
