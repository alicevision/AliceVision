// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/system/main.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/hardwareContext.hpp>

#include <aliceVision/track/TracksHandler.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

#include <aliceVision/sfm/pipeline/expanding/SfmBundle.hpp>
#include <aliceVision/sfm/pipeline/expanding/ExpansionHistory.hpp>
#include <aliceVision/sfm/pipeline/expanding/ExpansionChunk.hpp>
#include <aliceVision/sfm/pipeline/expanding/ExpansionPolicyLegacy.hpp>
#include <aliceVision/sfm/pipeline/expanding/ExpansionIteration.hpp>
#include <aliceVision/sfm/pipeline/expanding/ExpansionProcess.hpp>
#include <aliceVision/sfm/pipeline/expanding/LbaPolicyConnexity.hpp>

#include <boost/program_options.hpp>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;

int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string sfmDataFilename;
    std::string sfmDataOutputFilename;
    std::string tracksFilename;

    std::size_t localizerEstimatorMaxIterations = 50000;
    double localizerEstimatorError = 0.0;
    bool lockScenePreviouslyReconstructed = false;
    bool useLocalBA = true;
    int lbaDistanceLimit = 1;
    std::size_t nbFirstUnstableCameras = 30;
    std::size_t maxImagesPerGroup = 30;
    int bundleAdjustmentMaxOutliers = 50;
    std::size_t minNbObservationsForTriangulation = 2;
    double minAngleForTriangulation = 3.0;
    double minAngleForLandmark = 2.0;
    double maxReprojectionError = 4.0;
    bool lockAllIntrinsics = false;
    int minNbCamerasToRefinePrincipalPoint = 3;

    int randomSeed = std::mt19937::default_seed;

     // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(), "SfMData file.")
    ("output,o", po::value<std::string>(&sfmDataOutputFilename)->required(), "SfMData output file.")
    ("tracksFilename,t", po::value<std::string>(&tracksFilename)->required(), "Tracks file.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
    ("localizerEstimatorMaxIterations", po::value<std::size_t>(&localizerEstimatorMaxIterations)->default_value(localizerEstimatorMaxIterations), "Maximum number of RANSAC iterations.")
    ("localizerEstimatorError", po::value<double>(&localizerEstimatorError)->default_value(0.0), "Reprojection error threshold (in pixels) for the localizer estimator (0 for default value according to the estimator).")
    ("lockScenePreviouslyReconstructed", po::value<bool>(&lockScenePreviouslyReconstructed)->default_value(lockScenePreviouslyReconstructed),"Lock/Unlock scene previously reconstructed.")
    ("useLocalBA,l", po::value<bool>(&useLocalBA)->default_value(useLocalBA), "Enable/Disable the Local bundle adjustment strategy.\n It reduces the reconstruction time, especially for big datasets (500+ images).")
    ("localBAGraphDistance", po::value<int>(&lbaDistanceLimit)->default_value(lbaDistanceLimit), "Graph-distance limit setting the Active region in the Local Bundle Adjustment strategy.")
    ("nbFirstUnstableCameras", po::value<std::size_t>(&nbFirstUnstableCameras)->default_value(nbFirstUnstableCameras),
         "Number of cameras for which the bundle adjustment is performed every single time a camera is added, leading to more stable "
         "results while the computations are not too expensive since there is not much data. Past this number, the bundle adjustment "
         "will only be performed once for N added cameras.")
    ("maxImagesPerGroup", po::value<std::size_t>(&maxImagesPerGroup)->default_value(maxImagesPerGroup),
         "Maximum number of cameras that can be added before the bundle adjustment is performed. "
         "This prevents adding too much data at once without performing the bundle adjustment.")
    ("bundleAdjustmentMaxOutliers", po::value<int>(&bundleAdjustmentMaxOutliers)->default_value(bundleAdjustmentMaxOutliers),
         "Threshold for the maximum number of outliers allowed at the end of a bundle adjustment iteration."
         "Using a negative value for this threshold will disable BA iterations.")
    ("minNumberOfObservationsForTriangulation", po::value<std::size_t>(&minNbObservationsForTriangulation)->default_value(minNbObservationsForTriangulation),"Minimum number of observations to triangulate a point")
    ("minAngleForTriangulation", po::value<double>(&minAngleForTriangulation)->default_value(minAngleForTriangulation),"Minimum angle for triangulation.")
    ("minAngleForLandmark", po::value<double>(&minAngleForLandmark)->default_value(minAngleForLandmark), "Minimum angle for landmark.")
    ("maxReprojectionError", po::value<double>(&maxReprojectionError)->default_value(maxReprojectionError), "Maximum reprojection error.")
    ("lockAllIntrinsics", po::value<bool>(&lockAllIntrinsics)->default_value(lockAllIntrinsics), "Force lock of all camera intrinsic parameters, so they will not be refined during Bundle Adjustment.")
    ("minNbCamerasToRefinePrincipalPoint", po::value<int>(&minNbCamerasToRefinePrincipalPoint)->default_value(minNbCamerasToRefinePrincipalPoint),
         "Minimal number of cameras to refine the principal point of the cameras (one of the intrinsic parameters of the camera). "
         "If we do not have enough cameras, the principal point in consider is considered in the center of the image. "
         "If minNbCamerasToRefinePrincipalPoint<=0, the principal point is never refined. "
         "If minNbCamerasToRefinePrincipalPoint==1, the principal point is always refined.")
    ;
     // clang-format on
   
    CmdLine cmdline("AliceVision SfM Expanding");

    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if(!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // set maxThreads
    HardwareContext hwc = cmdline.getHardwareContext();
    omp_set_num_threads(hwc.getMaxThreads());
    
    // load input SfMData scene
    sfmData::SfMData sfmData;
    if(!sfmDataIO::load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" + sfmDataFilename + "' cannot be read.");
        return EXIT_FAILURE;
    }

    if (sfmData.getValidViews().size() < 2)
    {
        ALICEVISION_LOG_INFO("Expansion requires that some views are already defined.");
        return EXIT_SUCCESS;
    }

    // lock scene previously reconstructed
    if (lockScenePreviouslyReconstructed)
    {
        // lock all reconstructed camera poses
        for (auto& cameraPosePair : sfmData.getPoses())
        {
            cameraPosePair.second.lock();
        }

        for (const auto& viewPair : sfmData.getViews())
        {
            // lock all reconstructed views intrinsics
            const sfmData::View& view = *(viewPair.second);

            if (sfmData.isPoseAndIntrinsicDefined(&view))
            {
                sfmData.getIntrinsics().at(view.getIntrinsicId())->lock();
            }
        }
    }

    // Load tracks
    ALICEVISION_LOG_INFO("Load tracks");
    track::TracksHandler tracksHandler;
    if (!tracksHandler.load(tracksFilename, sfmData.getViewsKeys()))
    {
        ALICEVISION_LOG_ERROR("The input tracks file '" + tracksFilename + "' cannot be read.");
        return EXIT_FAILURE;
    }


    sfm::ExpansionHistory::sptr expansionHistory = std::make_shared<sfm::ExpansionHistory>();

    sfm::LbaPolicy::uptr sfmPolicy;
    
    if (useLocalBA) 
    {
        sfm::LbaPolicyConnexity::uptr sfmPolicyTyped = std::make_unique<sfm::LbaPolicyConnexity>();
        sfmPolicyTyped->setExpansionHistoryHandler(expansionHistory);
        sfmPolicyTyped->setDistanceLimit(lbaDistanceLimit);
        sfmPolicy = std::move(sfmPolicyTyped);
    }
    
    sfm::SfmBundle::uptr sfmBundle = std::make_unique<sfm::SfmBundle>();
    sfmBundle->setLbaPolicyHandler(sfmPolicy);
    sfmBundle->setBundleAdjustmentMaxOutlier(bundleAdjustmentMaxOutliers);
    sfmBundle->setMinAngleLandmark(minAngleForLandmark);
    sfmBundle->setMaxReprojectionError(maxReprojectionError);
    sfmBundle->setMinNbCamerasToRefinePrincipalPoint(minNbCamerasToRefinePrincipalPoint);

    sfm::ExpansionChunk::uptr expansionChunk = std::make_unique<sfm::ExpansionChunk>();
    expansionChunk->setBundleHandler(sfmBundle);
    expansionChunk->setExpansionHistoryHandler(expansionHistory);
    expansionChunk->setResectionMaxIterations(localizerEstimatorMaxIterations);
    expansionChunk->setResectionMaxError(localizerEstimatorError);
    expansionChunk->setTriangulationMinPoints(minNbObservationsForTriangulation);
    expansionChunk->setMinAngleTriangulation(minAngleForTriangulation);
    
    sfm::ExpansionPolicy::uptr expansionPolicy;
    {
        sfm::ExpansionPolicyLegacy::uptr expansionPolicyTyped = std::make_unique<sfm::ExpansionPolicyLegacy>();
        expansionPolicyTyped->setNbFirstUnstableViews(nbFirstUnstableCameras);
        expansionPolicyTyped->setMaxViewsPerGroup(maxImagesPerGroup);
        expansionPolicy = std::move(expansionPolicyTyped);
    } 

    sfm::ExpansionIteration::uptr expansionIteration = std::make_unique<sfm::ExpansionIteration>();
    expansionIteration->setExpansionHistoryHandler(expansionHistory);
    expansionIteration->setExpansionPolicyHandler(expansionPolicy);
    expansionIteration->setExpansionChunkHandler(expansionChunk);

    sfm::ExpansionProcess::uptr expansionProcess = std::make_unique<sfm::ExpansionProcess>();
    expansionProcess->setExpansionHistoryHandler(expansionHistory);
    expansionProcess->setExpansionIterationHandler(expansionIteration);

    if (!expansionProcess->process(sfmData, tracksHandler))
    {
        ALICEVISION_LOG_INFO("Error processing sfmData");
        return EXIT_FAILURE;
    }

    sfmDataIO::save(sfmData, sfmDataOutputFilename, sfmDataIO::ESfMData::ALL);

    

    return EXIT_SUCCESS;
}