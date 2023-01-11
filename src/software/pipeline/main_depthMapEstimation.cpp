// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/mvsUtils/MultiViewParams.hpp>
#include <aliceVision/depthMap/computeOnMultiGPUs.hpp>
#include <aliceVision/depthMap/depthMap.hpp>
#include <aliceVision/depthMap/DepthMapParams.hpp>
#include <aliceVision/gpu/gpu.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 3
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;

int aliceVision_main(int argc, char* argv[])
{
    ALICEVISION_COMMANDLINE_START

    std::string sfmDataFilename;
    std::string outputFolder;
    std::string imagesFolder;

    // program range
    int rangeStart = -1;
    int rangeSize = -1;

    // global image downscale factor
    int downscale = 2;

    // min / max view angle
    float minViewAngle = 2.0f;
    float maxViewAngle = 70.0f;

    // DepthMap parameters
    depthMap::DepthMapParams depthMapParams;

    // Tiling parameters
    auto& tileParams = depthMapParams.tileParams;

    // Semi Global Matching Parameters
    auto& sgmParams = depthMapParams.sgmParams;

    // Refine Parameters
    auto& refineParams = depthMapParams.refineParams;

    // intermediate results
    bool exportIntermediateDepthSimMaps = false;
    bool exportIntermediateVolumes = false;
    bool exportIntermediateCrossVolumes = false;
    bool exportIntermediateVolume9pCsv = false;

    // number of GPUs to use (0 means use all GPUs)
    int nbGPUs = 0;

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
            "Downscale the input images to compute the depth map. "
            "Full resolution (downscale=1) gives the best result, "
            "but using a larger downscale will reduce computation time at the expense of quality. "
            "If the images are noisy, blurry or if the surfaces are challenging (weakly-textured or with specularities) a larger downscale may improve.")
        ("minViewAngle", po::value<float>(&minViewAngle)->default_value(minViewAngle),
            "Minimum angle between two views (select the neighbouring cameras, select depth planes from epipolar segment point).")
        ("maxViewAngle", po::value<float>(&maxViewAngle)->default_value(maxViewAngle),
            "Maximum angle between two views (select the neighbouring cameras, select depth planes from epipolar segment point).")
        ("tileBufferWidth", po::value<int>(&tileParams.bufferWidth)->default_value(tileParams.bufferWidth),
            "Maximum tile buffer width.")
        ("tileBufferHeight", po::value<int>(&tileParams.bufferHeight)->default_value(tileParams.bufferHeight),
            "Maximum tile buffer height.")
        ("tilePadding", po::value<int>(&tileParams.padding)->default_value(tileParams.padding),
            "Buffer padding for overlapping tiles.")
        ("chooseTCamsPerTile", po::value<bool>(&depthMapParams.chooseTCamsPerTile)->default_value(depthMapParams.chooseTCamsPerTile),
            "Choose neighbour cameras per tile or globally to the image.")
        ("maxTCams", po::value<int>(&depthMapParams.maxTCams)->default_value(depthMapParams.maxTCams),
            "Maximum number of neighbour cameras per image.")
        ("sgmScale", po::value<int>(&sgmParams.scale)->default_value(sgmParams.scale),
            "Semi Global Matching: Downscale factor applied on source images for the SGM step (in addition to the global downscale).")
        ("sgmStepXY", po::value<int>(&sgmParams.stepXY)->default_value(sgmParams.stepXY),
            "Semi Global Matching: Step is used to compute the similarity volume for one pixel over N (in the XY image plane).")
        ("sgmStepZ", po::value<int>(&sgmParams.stepZ)->default_value(sgmParams.stepZ),
            "Semi Global Matching: Initial step used to compute the similarity volume on Z axis (every N pixels on the epilolar line). "
            "-1 means automatic estimation. "
            "This value will be adjusted in all case to fit in the max memory (sgmMaxDepths).")
        ("sgmMaxTCamsPerTile", po::value<int>(&sgmParams.maxTCamsPerTile)->default_value(sgmParams.maxTCamsPerTile),
            "Semi Global Matching: Maximum number of neighbour cameras used per tile.")
        ("sgmWSH", po::value<int>(&sgmParams.wsh)->default_value(sgmParams.wsh),
            "Semi Global Matching: Half-size of the patch used to compute the similarity. Patch width is wsh*2+1.")
        ("sgmUseSfmSeeds", po::value<bool>(&sgmParams.useSfmSeeds)->default_value(sgmParams.useSfmSeeds),
            "Semi Global Matching: Use landmarks from Structure-from-Motion as input seeds to define min/max depth ranges.")
        ("sgmSeedsRangeInflate", po::value<double>(&sgmParams.seedsRangeInflate)->default_value(sgmParams.seedsRangeInflate),
            "Semi Global Matching: Inflate factor to add margins around SfM seeds.")
        ("sgmGammaC", po::value<double>(&sgmParams.gammaC)->default_value(sgmParams.gammaC),
            "Semi Global Matching: GammaC threshold used for similarity computation.")
        ("sgmGammaP", po::value<double>(&sgmParams.gammaP)->default_value(sgmParams.gammaP),
            "Semi Global Matching: GammaP threshold used for similarity computation.")
        ("sgmP1", po::value<double>(&sgmParams.p1)->default_value(sgmParams.p1),
            "Semi Global Matching: P1 parameter for SGM filtering.")
        ("sgmP2Weighting", po::value<double>(&sgmParams.p2Weighting)->default_value(sgmParams.p2Weighting),
            "Semi Global Matching: P2 weighting parameter for SGM filtering.")
        ("sgmMaxDepths", po::value<int>(&sgmParams.maxDepths)->default_value(sgmParams.maxDepths),
            "Semi Global Matching: Maximum number of depths in the similarity volume.")
        ("sgmFilteringAxes", po::value<std::string>(&sgmParams.filteringAxes)->default_value(sgmParams.filteringAxes),
            "Semi Global Matching: Define axes for the filtering of the similarity volume.")
        ("sgmChooseDepthListPerTile", po::value<bool>(&sgmParams.chooseDepthListPerTile)->default_value(sgmParams.chooseDepthListPerTile),
            "Semi Global Matching: Select the list of depth planes per tile or globally to the image.")
        ("refineScale", po::value<int>(&refineParams.scale)->default_value(refineParams.scale),
            "Refine: Downscale factor applied on source images for the Refine step (in addition to the global downscale).")
        ("refineStepXY", po::value<int>(&refineParams.stepXY)->default_value(refineParams.stepXY),
            "Refine: Step is used to compute the refine volume for one pixel over N (in the XY image plane).")
        ("refineMaxTCamsPerTile", po::value<int>(&refineParams.maxTCamsPerTile)->default_value(refineParams.maxTCamsPerTile),
            "Refine: Number of neighbour cameras per tile.")
        ("refineNSamplesHalf", po::value<int>(&refineParams.nSamplesHalf)->default_value(refineParams.nSamplesHalf),
            "Refine: Number of samples.")
        ("refineNDepthsToRefine", po::value<int>(&refineParams.nDepthsToRefine)->default_value(refineParams.nDepthsToRefine),
            "Refine: Number of depths.")
        ("refineNiters", po::value<int>(&refineParams.optimizationNbIters)->default_value(refineParams.optimizationNbIters),
            "Refine: Number of optimization iterations.")
        ("refineWSH", po::value<int>(&refineParams.wsh)->default_value(refineParams.wsh),
            "Refine: Half-size of the patch used to compute the similarity. Patch width is wsh*2+1.")
        ("refineSigma", po::value<double>(&refineParams.sigma)->default_value(refineParams.sigma),
            "Refine: Sigma (2*sigma^2) of the gaussian filter used to extract the best depth from the refine volume.")
        ("refineGammaC", po::value<double>(&refineParams.gammaC)->default_value(refineParams.gammaC),
            "Refine: GammaC threshold used for similarity computation.")
        ("refineGammaP", po::value<double>(&refineParams.gammaP)->default_value(refineParams.gammaP),
            "Refine: GammaP threshold used for similarity computation.")
        ("refineEnabled", po::value<bool>(&refineParams.useRefineFuse)->default_value(refineParams.useRefineFuse),
            "Enable/Disable depth/similarity map refinement process.")
        ("colorOptimizationEnabled", po::value<bool>(&refineParams.useColorOptimization)->default_value(refineParams.useColorOptimization),
            "Enable/Disable depth/similarity map post-process color optimization.")
        ("autoAdjustSmallImage", po::value<bool>(&depthMapParams.autoAdjustSmallImage)->default_value(depthMapParams.autoAdjustSmallImage),
            "Automatically adjust depth map parameters if images are smaller than one tile (maxTCamsPerTile=maxTCams, adjust step if needed).")
        ("exportIntermediateDepthSimMaps", po::value<bool>(&exportIntermediateDepthSimMaps)->default_value(exportIntermediateDepthSimMaps),
            "Export intermediate depth/similarity maps from the SGM and Refine steps.")
        ("exportIntermediateVolumes", po::value<bool>(&exportIntermediateVolumes)->default_value(exportIntermediateVolumes),
            "Export intermediate similarity volumes from the SGM and Refine steps.")
        ("exportIntermediateCrossVolumes", po::value<bool>(&exportIntermediateCrossVolumes)->default_value(exportIntermediateCrossVolumes),
            "Export intermediate similarity cross volumes from the SGM and Refine steps.")
        ("exportIntermediateVolume9pCsv", po::value<bool>(&exportIntermediateVolume9pCsv)->default_value(exportIntermediateVolume9pCsv),
            "Export intermediate volumes 9 points CSV from the SGM and Refine steps.")
        ("exportTilePattern", po::value<bool>(&depthMapParams.exportTilePattern)->default_value(depthMapParams.exportTilePattern),
            "Export workflow tile pattern.")
        ("nbGPUs", po::value<int>(&nbGPUs)->default_value(nbGPUs),
            "Number of GPUs to use (0 means use all GPUs).");

    CmdLine cmdline("Dense Reconstruction.\n"
                    "This program estimate a depth map for each input calibrated camera using Plane Sweeping, a multi-view stereo algorithm notable for its efficiency on modern graphics hardware (GPU).\n"
                    "AliceVision depthMapEstimation");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

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

    // check that Sgm scaleStep is greater or equal to the Refine scaleStep
    if(depthMapParams.useRefine)
    {
      const int sgmScaleStep = sgmParams.scale * sgmParams.stepXY;
      const int refineScaleStep = refineParams.scale * refineParams.stepXY;

      if(sgmScaleStep < refineScaleStep)
      {
        ALICEVISION_LOG_ERROR("SGM downscale (scale & step) should be greater or equal to the Refine downscale (scale & step).");
        return EXIT_FAILURE;
      }
    }

    // check min/max view angle
    if(minViewAngle < 0.f || minViewAngle > 360.f ||
       maxViewAngle < 0.f || maxViewAngle > 360.f ||
       minViewAngle > maxViewAngle)
    {
      ALICEVISION_LOG_ERROR("Invalid value for minViewAngle/maxViewAngle parameter(s). Should be between 0 and 360.");
      return EXIT_FAILURE;
    }

    // read the input SfM scene
    sfmData::SfMData sfmData;
    if(!sfmDataIO::Load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
    {
      ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' cannot be read.");
      return EXIT_FAILURE;
    }

    // MultiViewParams initialization
    mvsUtils::MultiViewParams mp(sfmData, imagesFolder, outputFolder, "", false, downscale);

    // set MultiViewParams min/max view angle
    mp.setMinViewAngle(minViewAngle);
    mp.setMaxViewAngle(maxViewAngle);

    // set undefined tile dimensions
    if(tileParams.bufferWidth <= 0 || tileParams.bufferHeight <= 0)
    {
      tileParams.bufferWidth  = mp.getMaxImageWidth();
      tileParams.bufferHeight = mp.getMaxImageHeight();
    }

    // check if the tile padding is correct
    if(tileParams.padding < 0 &&
       tileParams.padding * 2 < tileParams.bufferWidth &&
       tileParams.padding * 2 < tileParams.bufferHeight)
    {
        ALICEVISION_LOG_ERROR("Invalid value for tilePadding parameter. Should be at least 0 and not exceed half buffer width and height.");
        return EXIT_FAILURE;
    }

    // check if tile size > max image size
    if(tileParams.bufferWidth > mp.getMaxImageWidth() || tileParams.bufferHeight > mp.getMaxImageHeight())
      ALICEVISION_LOG_WARNING("Tile buffer size (width: "  << tileParams.bufferWidth << ", height: " << tileParams.bufferHeight << ") is larger than the maximum image size (width: " << mp.getMaxImageWidth() << ", height: " << mp.getMaxImageHeight() <<  ").");

    // set params in bpt

    // Tile Parameters
    mp.userParams.put("tile.bufferWidth", tileParams.bufferWidth);
    mp.userParams.put("tile.bufferHeight", tileParams.bufferHeight);
    mp.userParams.put("tile.padding", tileParams.padding);

    // SGM Parameters
    mp.userParams.put("sgm.scale", sgmParams.scale);
    mp.userParams.put("sgm.stepXY", sgmParams.stepXY);
    mp.userParams.put("sgm.stepZ", sgmParams.stepZ);
    mp.userParams.put("sgm.wsh", sgmParams.wsh);
    mp.userParams.put("sgm.seedsRangeInflate", sgmParams.seedsRangeInflate);
    mp.userParams.put("sgm.gammaC", sgmParams.gammaC);
    mp.userParams.put("sgm.gammaP", sgmParams.gammaP);
    mp.userParams.put("sgm.p1", sgmParams.p1);
    mp.userParams.put("sgm.p2Weighting", sgmParams.p2Weighting);
    mp.userParams.put("sgm.maxTCamsPerTile", sgmParams.maxTCamsPerTile);
    mp.userParams.put("sgm.maxDepths", sgmParams.maxDepths);
    mp.userParams.put("sgm.filteringAxes", sgmParams.filteringAxes);
    mp.userParams.put("sgm.useSfmSeeds", sgmParams.useSfmSeeds);
    mp.userParams.put("sgm.chooseDepthListPerTile", sgmParams.chooseDepthListPerTile);
    mp.userParams.put("sgm.exportIntermediateDepthSimMaps", exportIntermediateDepthSimMaps);
    mp.userParams.put("sgm.exportIntermediateVolumes", exportIntermediateVolumes);
    mp.userParams.put("sgm.exportIntermediateCrossVolumes", exportIntermediateCrossVolumes);
    mp.userParams.put("sgm.exportIntermediateVolume9pCsv", exportIntermediateVolume9pCsv);

    // Refine Parameters
    mp.userParams.put("refine.scale", refineParams.scale);
    mp.userParams.put("refine.stepXY", refineParams.stepXY);
    mp.userParams.put("refine.wsh", refineParams.wsh);
    mp.userParams.put("refine.sigma", refineParams.sigma);
    mp.userParams.put("refine.gammaC", refineParams.gammaC);
    mp.userParams.put("refine.gammaP", refineParams.gammaP);
    mp.userParams.put("refine.maxTCamsPerTile", refineParams.maxTCamsPerTile);
    mp.userParams.put("refine.nSamplesHalf", refineParams.nSamplesHalf);
    mp.userParams.put("refine.nDepthsToRefine", refineParams.nDepthsToRefine);
    mp.userParams.put("refine.optimizationNbIters", refineParams.optimizationNbIters);
    mp.userParams.put("refine.useRefineFuse", refineParams.useRefineFuse);
    mp.userParams.put("refine.useColorOptimization", refineParams.useColorOptimization);
    mp.userParams.put("refine.exportIntermediateDepthSimMaps", exportIntermediateDepthSimMaps);
    mp.userParams.put("refine.exportIntermediateCrossVolumes", exportIntermediateCrossVolumes);
    mp.userParams.put("refine.exportIntermediateVolume9pCsv", exportIntermediateVolume9pCsv);

    // Workflow Parameters
    mp.userParams.put("depthMap.chooseTCamsPerTile", depthMapParams.chooseTCamsPerTile);
    mp.userParams.put("depthMap.maxTCams", depthMapParams.maxTCams);
    mp.userParams.put("depthMap.exportTilePattern", depthMapParams.exportTilePattern);
    mp.userParams.put("depthMap.autoAdjustSmallImage", depthMapParams.autoAdjustSmallImage);

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
