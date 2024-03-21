// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/mvsUtils/MultiViewParams.hpp>
#include <aliceVision/depthMap/computeOnMultiGPUs.hpp>
#include <aliceVision/depthMap/DepthMapEstimator.hpp>
#include <aliceVision/depthMap/DepthMapParams.hpp>
#include <aliceVision/depthMap/SgmParams.hpp>
#include <aliceVision/depthMap/RefineParams.hpp>
#include <aliceVision/gpu/gpu.hpp>

#include <boost/program_options.hpp>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 4
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;

int computeDownscale(const mvsUtils::MultiViewParams& mp, int scale, int maxWidth, int maxHeight)
{
    const int maxImageWidth = mp.getMaxImageWidth() / scale;
    const int maxImageHeight = mp.getMaxImageHeight() / scale;

    int downscale = 1;
    int downscaleWidth = mp.getMaxImageWidth() / scale;
    int downscaleHeight = mp.getMaxImageHeight() / scale;

    while ((downscaleWidth > maxWidth) || (downscaleHeight > maxHeight))
    {
        downscale++;
        downscaleWidth = maxImageWidth / downscale;
        downscaleHeight = maxImageHeight / downscale;
    }

    return downscale;
}

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

    // Tiling parameters
    mvsUtils::TileParams tileParams;

    // DepthMap (global) parameters
    depthMap::DepthMapParams depthMapParams;

    // Semi Global Matching Parameters
    depthMap::SgmParams sgmParams;

    // Refine Parameters
    depthMap::RefineParams refineParams;

    // intermediate results
    bool exportIntermediateDepthSimMaps = false;
    bool exportIntermediateNormalMaps = false;
    bool exportIntermediateVolumes = false;
    bool exportIntermediateCrossVolumes = false;
    bool exportIntermediateTopographicCutVolumes = false;
    bool exportIntermediateVolume9pCsv = false;

    // number of GPUs to use (0 means use all GPUs)
    int nbGPUs = 0;

    // clang-format off
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
         "If the images are noisy, blurry or if the surfaces are challenging (weakly-textured or with specularities), "
         "a larger downscale may improve.")
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
        ("sgmDepthThicknessInflate", po::value<double>(&sgmParams.depthThicknessInflate)->default_value(sgmParams.depthThicknessInflate),
         "Semi Global Matching: Inflate factor to add margins to the depth thickness.")
        ("sgmMaxSimilarity", po::value<double>(&sgmParams.maxSimilarity)->default_value(sgmParams.maxSimilarity),
         "Semi Global Matching: Maximum similarity threshold (between 0 and 1) used to filter out poorly supported depth values.")
        ("sgmGammaC", po::value<double>(&sgmParams.gammaC)->default_value(sgmParams.gammaC),
         "Semi Global Matching: GammaC threshold used for similarity computation, strength of grouping by color similarity.")
        ("sgmGammaP", po::value<double>(&sgmParams.gammaP)->default_value(sgmParams.gammaP),
         "Semi Global Matching: GammaP threshold used for similarity computation, strength of grouping by proximity.")
        ("sgmP1", po::value<double>(&sgmParams.p1)->default_value(sgmParams.p1),
         "Semi Global Matching: P1 parameter for SGM filtering.")
        ("sgmP2Weighting", po::value<double>(&sgmParams.p2Weighting)->default_value(sgmParams.p2Weighting),
         "Semi Global Matching: P2 weighting parameter for SGM filtering.")
        ("sgmMaxDepths", po::value<int>(&sgmParams.maxDepths)->default_value(sgmParams.maxDepths),
         "Semi Global Matching: Maximum number of depths in the similarity volume.")
        ("sgmFilteringAxes", po::value<std::string>(&sgmParams.filteringAxes)->default_value(sgmParams.filteringAxes),
         "Semi Global Matching: Define axes for the filtering of the similarity volume.")
        ("sgmDepthListPerTile", po::value<bool>(&sgmParams.depthListPerTile)->default_value(sgmParams.depthListPerTile),
         "Semi Global Matching: Select the list of depth planes per tile or globally to the image.")
        ("sgmUseConsistentScale", po::value<bool>(&sgmParams.useConsistentScale)->default_value(sgmParams.useConsistentScale),
         "Semi Global Matching: Compare patch with consistent scale for similarity volume computation.")
        ("sgmUseCustomPatchPattern", po::value<bool>(&sgmParams.useCustomPatchPattern)->default_value(sgmParams.useCustomPatchPattern),
         "Semi Global Matching: Use user custom patch pattern for similarity volume computation.")
        ("refineScale", po::value<int>(&refineParams.scale)->default_value(refineParams.scale),
         "Refine: Downscale factor applied on source images for the Refine step (in addition to the global downscale).")
        ("refineStepXY", po::value<int>(&refineParams.stepXY)->default_value(refineParams.stepXY),
         "Refine: Step is used to compute the refine volume for one pixel over N (in the XY image plane).")
        ("refineMaxTCamsPerTile", po::value<int>(&refineParams.maxTCamsPerTile)->default_value(refineParams.maxTCamsPerTile),
         "Refine: Maximum number of neighbour cameras used per tile.")
        ("refineHalfNbDepths", po::value<int>(&refineParams.halfNbDepths)->default_value(refineParams.halfNbDepths),
         "Refine: The thickness of the refine area around the initial depth map. "
         "This parameter defines the number of depths in front of and behind the initial value "
         "for which we evaluate the similarity with a finer z sampling.")
        ("refineSubsampling", po::value<int>(&refineParams.nbSubsamples)->default_value(refineParams.nbSubsamples),
         "Refine: Number of subsamples used to extract the best depth from the refine volume (sliding gaussian window precision).")
        ("refineWSH", po::value<int>(&refineParams.wsh)->default_value(refineParams.wsh),
         "Refine: Half-size of the patch used to compute the similarity. Patch width is wsh*2+1.")
        ("refineSigma", po::value<double>(&refineParams.sigma)->default_value(refineParams.sigma),
         "Refine: Sigma (2*sigma^2) of the gaussian filter used to extract the best depth from the refine volume.")
        ("refineGammaC", po::value<double>(&refineParams.gammaC)->default_value(refineParams.gammaC),
         "Refine: GammaC threshold used for similarity computation.")
        ("refineGammaP", po::value<double>(&refineParams.gammaP)->default_value(refineParams.gammaP),
         "Refine: GammaP threshold used for similarity computation.")
        ("refineInterpolateMiddleDepth", po::value<bool>(&refineParams.interpolateMiddleDepth)->default_value(refineParams.interpolateMiddleDepth),
         "Refine: Enable/Disable middle depth bilinear interpolation for the refinement process.")
        ("refineUseConsistentScale", po::value<bool>(&refineParams.useConsistentScale)->default_value(refineParams.useConsistentScale),
         "Refine: Compare patch with consistent scale for similarity volume computation.")
        ("refineUseCustomPatchPattern", po::value<bool>(&refineParams.useCustomPatchPattern)->default_value(refineParams.useCustomPatchPattern),
         "Refine: Use user custom patch pattern for similarity volume computation.")
        ("colorOptimizationNbIterations", po::value<int>(&refineParams.optimizationNbIterations)->default_value(refineParams.optimizationNbIterations),
         "Color Optimization: Number of iterations of the optimization.")
        ("refineEnabled", po::value<bool>(&refineParams.useRefineFuse)->default_value(refineParams.useRefineFuse),
         "Enable/Disable depth/similarity map refinement process.")
        ("colorOptimizationEnabled", po::value<bool>(&refineParams.useColorOptimization)->default_value(refineParams.useColorOptimization),
         "Enable/Disable depth/similarity map post-process color optimization.")
        ("autoAdjustSmallImage", po::value<bool>(&depthMapParams.autoAdjustSmallImage)->default_value(depthMapParams.autoAdjustSmallImage),
         "Automatically adjust depth map parameters if images are smaller than one tile (maxTCamsPerTile=maxTCams, adjust step if needed).")
        ("customPatchPatternSubparts", po::value<std::vector<depthMap::CustomPatchPatternParams::SubpartParams>>(&depthMapParams.customPatchPattern.subpartsParams)->multitoken()->default_value(depthMapParams.customPatchPattern.subpartsParams),
         "User custom patch pattern subparts for similarity volume computation.")
        ("customPatchPatternGroupSubpartsPerLevel", po::value<bool>(&depthMapParams.customPatchPattern.groupSubpartsPerLevel)->default_value(depthMapParams.customPatchPattern.groupSubpartsPerLevel),
         "Group all custom patch pattern subparts with the same image level.")
        ("exportIntermediateDepthSimMaps", po::value<bool>(&exportIntermediateDepthSimMaps)->default_value(exportIntermediateDepthSimMaps),
         "Export intermediate depth/similarity maps from the SGM and Refine steps.")
        ("exportIntermediateNormalMaps", po::value<bool>(&exportIntermediateNormalMaps)->default_value(exportIntermediateNormalMaps),
         "Export intermediate normal maps from the SGM and Refine steps.")
        ("exportIntermediateVolumes", po::value<bool>(&exportIntermediateVolumes)->default_value(exportIntermediateVolumes),
         "Export intermediate full similarity volumes from the SGM and Refine steps.")
        ("exportIntermediateCrossVolumes", po::value<bool>(&exportIntermediateCrossVolumes)->default_value(exportIntermediateCrossVolumes),
         "Export intermediate similarity cross volumes from the SGM and Refine steps.")
        ("exportIntermediateTopographicCutVolumes", po::value<bool>(&exportIntermediateTopographicCutVolumes)->default_value(exportIntermediateTopographicCutVolumes),
         "Export intermediate similarity topographic cut volumes from the SGM and Refine steps.")
        ("exportIntermediateVolume9pCsv", po::value<bool>(&exportIntermediateVolume9pCsv)->default_value(exportIntermediateVolume9pCsv),
         "Export intermediate volumes 9 points from the SGM and Refine steps in CSV files.")
        ("exportTilePattern", po::value<bool>(&depthMapParams.exportTilePattern)->default_value(depthMapParams.exportTilePattern),
         "Export workflow tile pattern.")
        ("nbGPUs", po::value<int>(&nbGPUs)->default_value(nbGPUs),
         "Number of GPUs to use (0 means use all GPUs).");
    // clang-format on

    CmdLine cmdline("Dense Reconstruction.\n"
                    "This program estimate a depth map for each input calibrated camera using Plane Sweeping, a multi-view stereo algorithm notable "
                    "for its efficiency on modern graphics hardware (GPU).\n"
                    "AliceVision depthMapEstimation");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // intermediate results
    sgmParams.exportIntermediateDepthSimMaps = exportIntermediateDepthSimMaps;
    sgmParams.exportIntermediateNormalMaps = exportIntermediateNormalMaps;
    sgmParams.exportIntermediateVolumes = exportIntermediateVolumes;
    sgmParams.exportIntermediateCrossVolumes = exportIntermediateCrossVolumes;
    sgmParams.exportIntermediateTopographicCutVolumes = exportIntermediateTopographicCutVolumes;
    sgmParams.exportIntermediateVolume9pCsv = exportIntermediateVolume9pCsv;

    refineParams.exportIntermediateDepthSimMaps = exportIntermediateDepthSimMaps;
    refineParams.exportIntermediateNormalMaps = exportIntermediateNormalMaps;
    refineParams.exportIntermediateCrossVolumes = exportIntermediateCrossVolumes;
    refineParams.exportIntermediateTopographicCutVolumes = exportIntermediateTopographicCutVolumes;
    refineParams.exportIntermediateVolume9pCsv = exportIntermediateVolume9pCsv;

    // print GPU Information
    ALICEVISION_LOG_INFO(gpu::gpuInformationCUDA());

    // check if the gpu suppport CUDA compute capability 2.0
    if (!gpu::gpuSupportCUDA(2, 0))
    {
        ALICEVISION_LOG_ERROR("This program needs a CUDA-Enabled GPU (with at least compute capability 2.0).");
        return EXIT_FAILURE;
    }

    // check if the scale is correct
    if (downscale < 1)
    {
        ALICEVISION_LOG_ERROR("Invalid value for downscale parameter. Should be at least 1.");
        return EXIT_FAILURE;
    }

    // check that Sgm scaleStep is greater or equal to the Refine scaleStep
    if (depthMapParams.useRefine)
    {
        const int sgmScaleStep = sgmParams.scale * sgmParams.stepXY;
        const int refineScaleStep = refineParams.scale * refineParams.stepXY;

        if (sgmScaleStep < refineScaleStep)
        {
            ALICEVISION_LOG_ERROR("SGM downscale (scale x step) should be greater or equal to the Refine downscale (scale x step).");
            return EXIT_FAILURE;
        }

        if (sgmScaleStep % refineScaleStep != 0)
        {
            ALICEVISION_LOG_ERROR("SGM downscale (scale x step) should be a multiple of the Refine downscale (scale x step).");
            return EXIT_FAILURE;
        }
    }

    // check min/max view angle
    if (minViewAngle < 0.f || minViewAngle > 360.f || maxViewAngle < 0.f || maxViewAngle > 360.f || minViewAngle > maxViewAngle)
    {
        ALICEVISION_LOG_ERROR("Invalid value for minViewAngle/maxViewAngle parameter(s). Should be between 0 and 360.");
        return EXIT_FAILURE;
    }

    // read the input SfM scene
    sfmData::SfMData sfmData;
    if (!sfmDataIO::load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' cannot be read.");
        return EXIT_FAILURE;
    }

    // MultiViewParams initialization
    mvsUtils::MultiViewParams mp(sfmData, imagesFolder, outputFolder, "", mvsUtils::EFileType::none, downscale);

    // set MultiViewParams min/max view angle
    mp.setMinViewAngle(minViewAngle);
    mp.setMaxViewAngle(maxViewAngle);

    // set undefined tile dimensions
    if (tileParams.bufferWidth <= 0 || tileParams.bufferHeight <= 0)
    {
        tileParams.bufferWidth = mp.getMaxImageWidth();
        tileParams.bufferHeight = mp.getMaxImageHeight();
    }

    // check if the tile padding is correct
    if (tileParams.padding < 0 && tileParams.padding * 2 < tileParams.bufferWidth && tileParams.padding * 2 < tileParams.bufferHeight)
    {
        ALICEVISION_LOG_ERROR("Invalid value for tilePadding parameter. Should be at least 0 and not exceed half buffer width and height.");
        return EXIT_FAILURE;
    }

    // check if tile size > max image size
    if (tileParams.bufferWidth > mp.getMaxImageWidth() || tileParams.bufferHeight > mp.getMaxImageHeight())
    {
        ALICEVISION_LOG_WARNING("Tile buffer size (width: " << tileParams.bufferWidth << ", height: " << tileParams.bufferHeight
                                                            << ") is larger than the maximum image size (width: " << mp.getMaxImageWidth()
                                                            << ", height: " << mp.getMaxImageHeight() << ").");
    }

    // check if SGM scale and step are set to -1
    bool autoSgmScaleStep = false;

    // compute SGM scale and step
    if (sgmParams.scale == -1 || sgmParams.stepXY == -1)
    {
        const int fileScale = 1;                               // input images scale (should be one)
        const int maxSideXY = 700 / mp.getProcessDownscale();  // max side in order to fit in device memory
        const int maxImageW = mp.getMaxImageWidth();
        const int maxImageH = mp.getMaxImageHeight();

        int maxW = maxSideXY;
        int maxH = maxSideXY * 0.8;

        if (maxImageW < maxImageH)
            std::swap(maxW, maxH);

        if (sgmParams.scale == -1)
        {
            // compute the number of scales that will be used in the plane sweeping.
            // the highest scale should have a resolution close to 700x550 (or less).
            const int scaleTmp = computeDownscale(mp, fileScale, maxW, maxH);
            sgmParams.scale = std::min(2, scaleTmp);
        }

        if (sgmParams.stepXY == -1)
        {
            sgmParams.stepXY = computeDownscale(mp, fileScale * sgmParams.scale, maxW, maxH);
        }

        autoSgmScaleStep = true;
    }

    // single tile case, update parameters
    if (depthMapParams.autoAdjustSmallImage && mvsUtils::hasOnlyOneTile(tileParams, mp.getMaxImageWidth(), mp.getMaxImageHeight()))
    {
        // update SGM maxTCamsPerTile
        if (sgmParams.maxTCamsPerTile < depthMapParams.maxTCams)
        {
            ALICEVISION_LOG_WARNING("Single tile computation, override SGM maximum number of T cameras per tile (before: "
                                    << sgmParams.maxTCamsPerTile << ", now: " << depthMapParams.maxTCams << ").");
            sgmParams.maxTCamsPerTile = depthMapParams.maxTCams;
        }

        // update Refine maxTCamsPerTile
        if (refineParams.maxTCamsPerTile < depthMapParams.maxTCams)
        {
            ALICEVISION_LOG_WARNING("Single tile computation, override Refine maximum number of T cameras per tile (before: "
                                    << refineParams.maxTCamsPerTile << ", now: " << depthMapParams.maxTCams << ").");
            refineParams.maxTCamsPerTile = depthMapParams.maxTCams;
        }

        const int maxSgmBufferWidth = divideRoundUp(mp.getMaxImageWidth(), sgmParams.scale * sgmParams.stepXY);
        const int maxSgmBufferHeight = divideRoundUp(mp.getMaxImageHeight(), sgmParams.scale * sgmParams.stepXY);

        // update SGM step XY
        if (!autoSgmScaleStep &&        // user define SGM scale & stepXY
            (sgmParams.stepXY == 2) &&  // default stepXY
            (maxSgmBufferWidth < tileParams.bufferWidth * 0.5) && (maxSgmBufferHeight < tileParams.bufferHeight * 0.5))
        {
            ALICEVISION_LOG_WARNING("Single tile computation, override SGM step XY (before: " << sgmParams.stepXY << ", now: 1).");
            sgmParams.stepXY = 1;
        }
    }

    // compute the maximum downscale factor
    const int maxDownscale = std::max(sgmParams.scale * sgmParams.stepXY, refineParams.scale * refineParams.stepXY);

    // check padding
    if (tileParams.padding % maxDownscale != 0)
    {
        const int padding = divideRoundUp(tileParams.padding, maxDownscale) * maxDownscale;
        ALICEVISION_LOG_WARNING("Override tiling padding parameter (before: " << tileParams.padding << ", now: " << padding << ").");
        tileParams.padding = padding;
    }

    // camera list
    std::vector<int> cams;
    cams.reserve(mp.ncams);

    if (rangeSize == -1)
    {
        for (int rc = 0; rc < mp.ncams; ++rc)  // process all cameras
            cams.push_back(rc);
    }
    else
    {
        if (rangeStart < 0)
        {
            ALICEVISION_LOG_ERROR("invalid subrange of cameras to process.");
            return EXIT_FAILURE;
        }
        for (int rc = rangeStart; rc < std::min(rangeStart + rangeSize, mp.ncams); ++rc)
            cams.push_back(rc);
        if (cams.empty())
        {
            ALICEVISION_LOG_INFO("No camera to process.");
            return EXIT_SUCCESS;
        }
    }

    // initialize depth map estimator
    depthMap::DepthMapEstimator depthMapEstimator(mp, tileParams, depthMapParams, sgmParams, refineParams);

    // estimate depth maps
    depthMap::computeOnMultiGPUs(cams, depthMapEstimator, nbGPUs);

    ALICEVISION_COMMANDLINE_END
}
