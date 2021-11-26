// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/mvsUtils/MultiViewParams.hpp>
#include <aliceVision/depthMap/computeOnMultiGPUs.hpp>
#include <aliceVision/depthMap/depthMap.hpp>
#include <aliceVision/depthMap/SgmParams.hpp>
#include <aliceVision/depthMap/RefineParams.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/gpu/gpu.hpp>

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

    // Semi Global Matching Parameters
    depthMap::SgmParams sgmParams; 

    // Refine Parameters
    depthMap::RefineParams refineParams;

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
        ("sgmScale", po::value<int>(&sgmParams.scale)->default_value(sgmParams.scale),
            "Semi Global Matching: Downscale factor used to compute the similarity volume.")
        ("sgmStepXY", po::value<int>(&sgmParams.stepXY)->default_value(sgmParams.stepXY),
            "Semi Global Matching: Step used to compute the similarity volume on the X and Y axis.")
        ("sgmStepZ", po::value<int>(&sgmParams.stepZ)->default_value(sgmParams.stepZ),
            "Semi Global Matching: Step used to compute the similarity volume on the Z axis.")
        ("sgmMaxSideXY", po::value<int>(&sgmParams.maxSideXY)->default_value(sgmParams.maxSideXY),
            "Semi Global Matching: Max side in pixels used to automatically decide for sgmScale/sgmStepXY if not defined.")
        ("sgmMaxTCams", po::value<int>(&sgmParams.maxTCams)->default_value(sgmParams.maxTCams),
            "Semi Global Matching: Number of neighbour cameras.")
        ("sgmWSH", po::value<int>(&sgmParams.wsh)->default_value(sgmParams.wsh),
            "Semi Global Matching: Size of the patch used to compute the similarity.")
        ("sgmGammaC", po::value<double>(&sgmParams.gammaC)->default_value(sgmParams.gammaC),
            "Semi Global Matching: GammaC threshold.")
        ("sgmGammaP", po::value<double>(&sgmParams.gammaP)->default_value(sgmParams.gammaP),
            "Semi Global Matching: GammaP threshold.")
        ("sgmP1", po::value<double>(&sgmParams.p1)->default_value(sgmParams.p1),
            "Semi Global Matching: P1.")
        ("sgmP2", po::value<double>(&sgmParams.p2Weighting)->default_value(sgmParams.p2Weighting),
            "Semi Global Matching: P2 Weighting.")
        ("sgmMaxDepths", po::value<int>(&sgmParams.maxDepths)->default_value(sgmParams.maxDepths),
            "Semi Global Matching: Max number of depths in the overall similarity volume.")
        ("sgmMaxDepthsPerTc", po::value<int>(&sgmParams.maxDepthsPerTc)->default_value(sgmParams.maxDepthsPerTc),
            "Semi Global Matching: Max number of depths to sweep in the similarity volume per Rc/Tc cameras.")
        ("sgmUseSfmSeeds", po::value<bool>(&sgmParams.useSfmSeeds)->default_value(sgmParams.useSfmSeeds),
            "Semi Global Matching: Use landmarks from SfM to define the ranges for the plane sweeping.")
        ("sgmFilteringAxes", po::value<std::string>(&sgmParams.filteringAxes)->default_value(sgmParams.filteringAxes),
            "Semi Global Matching: Filtering axes for the 3D volume.")
        ("refineMaxTCams", po::value<int>(&refineParams.maxTCams)->default_value(refineParams.maxTCams),
            "Refine: Number of neighbour cameras.")
        ("refineNSamplesHalf", po::value<int>(&refineParams.nSamplesHalf)->default_value(refineParams.nSamplesHalf),
            "Refine: Number of samples.")
        ("refineNDepthsToRefine", po::value<int>(&refineParams.nDepthsToRefine)->default_value(refineParams.nDepthsToRefine),
            "Refine: Number of depths.")
        ("refineNiters", po::value<int>(&refineParams.nIters)->default_value(refineParams.nIters),
            "Refine: Number of iterations.")
        ("refineWSH", po::value<int>(&refineParams.wsh)->default_value(refineParams.wsh),
            "Refine: Size of the patch used to compute the similarity.")
        ("refineSigma", po::value<double>(&refineParams.sigma)->default_value(refineParams.sigma),
            "Refine: Sigma threshold.")
        ("refineGammaC", po::value<double>(&refineParams.gammaC)->default_value(refineParams.gammaC),
            "Refine: GammaC threshold.")
        ("refineGammaP", po::value<double>(&refineParams.gammaP)->default_value(refineParams.gammaP),
            "Refine: GammaP threshold.")
        ("refineUseTcOrRcPixSize", po::value<bool>(&refineParams.useTcOrRcPixSize)->default_value(refineParams.useTcOrRcPixSize),
            "Refine: Use current camera pixel size or minimum pixel size of neighbour cameras.")
        ("exportIntermediateResults", po::value<bool>(&exportIntermediateResults)->default_value(exportIntermediateResults),
            "Export intermediate results from the SGM and Refine steps.")
        ("nbGPUs", po::value<int>(&nbGPUs)->default_value(nbGPUs),
            "Number of GPUs to use (0 means use all GPUs).");

    po::options_description logParams("Log parameters");
    logParams.add_options()
        ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
            "Verbosity level (fatal, error, warning, info, debug, trace).");

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

    // SGM Parameters
    mp.userParams.put("sgm.maxTCams", sgmParams.maxTCams);
    mp.userParams.put("sgm.wsh", sgmParams.wsh);
    mp.userParams.put("sgm.gammaC", sgmParams.gammaC);
    mp.userParams.put("sgm.gammaP", sgmParams.gammaP);
    mp.userParams.put("sgm.p1", sgmParams.p1);
    mp.userParams.put("sgm.p2Weighting", sgmParams.p2Weighting);
    mp.userParams.put("sgm.scale", sgmParams.scale);
    mp.userParams.put("sgm.stepXY", sgmParams.stepXY);
    mp.userParams.put("sgm.stepZ", sgmParams.stepZ);
    mp.userParams.put("sgm.maxSideXY", sgmParams.maxSideXY);
    mp.userParams.put("sgm.maxDepths", sgmParams.maxDepths);
    mp.userParams.put("sgm.maxDepthsPerTc", sgmParams.maxDepthsPerTc);
    mp.userParams.put("sgm.useSfmSeeds", sgmParams.useSfmSeeds);
    mp.userParams.put("sgm.filteringAxes", sgmParams.filteringAxes);
    mp.userParams.put("sgm.exportIntermediateResults", exportIntermediateResults);

    // Refine Parameters
    mp.userParams.put("refine.maxTCams", refineParams.maxTCams);
    mp.userParams.put("refine.nSamplesHalf", refineParams.nSamplesHalf);
    mp.userParams.put("refine.nDepthsToRefine", refineParams.nDepthsToRefine);
    mp.userParams.put("refine.nIters", refineParams.nIters);
    mp.userParams.put("refine.wsh", refineParams.wsh);
    mp.userParams.put("refine.sigma", refineParams.sigma);
    mp.userParams.put("refine.gammaC", refineParams.gammaC);
    mp.userParams.put("refine.gammaP", refineParams.gammaP);
    mp.userParams.put("refine.useTcOrRcPixSize", refineParams.useTcOrRcPixSize);
    mp.userParams.put("refine.exportIntermediateResults", exportIntermediateResults);

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
