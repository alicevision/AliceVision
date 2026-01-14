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
#include <aliceVision/sfm/pipeline/expanding/ExpansionPostProcessRig.hpp>
#include <aliceVision/sfm/pipeline/expanding/LocalizationValidationPolicyLegacy.hpp>
#include <aliceVision/sfm/pipeline/expanding/SfmTriangulation.hpp>

#include <aliceVision/mesh/MeshIntersection.hpp>

#include <boost/program_options.hpp>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 2
#define ALICEVISION_SOFTWARE_VERSION_MINOR 4

using namespace aliceVision;

namespace po = boost::program_options;

//This intermediate class is used as a proxy to not link
//sfm with mesh library
class MeshPointFetcher : public sfm::PointFetcher
{
public:
    /**
     * @brief initialize object. to be called before any other method
     * @param pathToModel path to obj file to use as mesh
    */
    bool initialize(const std::string & pathToModel)
    {
        return _mi.initialize(pathToModel);
    }

    /**
     * @brief Set the pose of the camera
     * @param pose the pose of the camera wrt some global coordinates frame
    */
    void setPose(const geometry::Pose3 & pose) override
    {
        _mi.setPose(pose);
    }

    /**
     * @brief virtual method to get coordinates and normals of a pixel of an image
     * @param point result point in some global coordinates frame
     * @param normal result normal in some global coordinates frame
     * @param intrinsic the camera intrinsic object
     * @param imageCoords the input image pixel coordinates in 2D.
     * @return false on error
    */
    bool pickPointAndNormal(Vec3 & point,
                                Vec3 & normal,
                                const camera::IntrinsicBase & intrinsic,
                                const Vec2 & imageCoords) override
    {
        return _mi.pickPointAndNormal(point, normal, intrinsic, imageCoords);
    }

private:
    mesh::MeshIntersection _mi;
};

int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string sfmDataFilename;
    std::string sfmDataOutputFilename;
    std::string tracksFilename;
    std::string meshFilename;
    std::string outputSfMViewsAndPoses;

    std::size_t localizerEstimatorMaxIterations = 50000;
    double localizerEstimatorError = 0.0;
    bool lockScenePreviouslyReconstructed = false;
    bool useLocalBA = true;
    bool useTemporalConstraint = false;
    int lbaDistanceLimit = 1;
    std::size_t nbFirstUnstableCameras = 30;
    std::size_t maxImagesPerGroup = 30;
    int bundleAdjustmentMaxOutliers = 50;
    std::size_t minNbObservationsForTriangulation = 2;
    double minAngleForTriangulation = 3.0;
    double minAngleForLandmark = 2.0;
    double maxReprojectionError = 4.0;
    double maxTriangulationError = 8.0;
    bool lockAllIntrinsics = false;
    bool enableStructureRefinement = true;
    int minNbCamerasToRefinePrincipalPoint = 3;
    bool useRigConstraint = true;
    int minNbCamerasForRigCalibration = 20;
    int weakResectionSize = 100;
    bool enableDepthPrior = true;
    bool ignoreMultiviewOnPrior = false;

    int randomSeed = std::mt19937::default_seed;

    aliceVision::sfm::TemporalConstraintParams tempConstrParams;

     // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(), "SfMData file.")
    ("output,o", po::value<std::string>(&sfmDataOutputFilename)->required(), "SfMData output file.")
    ("tracksFilename,t", po::value<std::string>(&tracksFilename)->required(), "Tracks file.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
    ("outputViewsAndPoses", po::value<std::string>(&outputSfMViewsAndPoses)->default_value(outputSfMViewsAndPoses), "Path to the output SfMData file (with only views and poses).")
    ("localizerEstimatorMaxIterations", po::value<std::size_t>(&localizerEstimatorMaxIterations)->default_value(localizerEstimatorMaxIterations), "Maximum number of RANSAC iterations.")
    ("localizerEstimatorError", po::value<double>(&localizerEstimatorError)->default_value(0.0), "Reprojection error threshold (in pixels) for the localizer estimator (0 for default value according to the estimator).")
    ("enableDepthPrior", po::value<bool>(&enableDepthPrior)->default_value(enableDepthPrior),"If available in the tracks, use the depth prior.")
    ("ignoreMultiviewOnPrior", po::value<bool>(&ignoreMultiviewOnPrior)->default_value(ignoreMultiviewOnPrior),"Favour the prior based 3d reconstruction over the multiview reconstruction.")
    ("lockScenePreviouslyReconstructed", po::value<bool>(&lockScenePreviouslyReconstructed)->default_value(lockScenePreviouslyReconstructed),"Lock/Unlock scene previously reconstructed.")
    ("useLocalBA,l", po::value<bool>(&useLocalBA)->default_value(useLocalBA), "Enable/Disable the Local bundle adjustment strategy.\n It reduces the reconstruction time, especially for big datasets (500+ images).")
    ("useTemporalConstraint", po::value<bool>(&useTemporalConstraint)->default_value(useTemporalConstraint), "Enable/Disable the temporal smoothness constraint to the bundle adjustment.")
    ("tscPositionWeight", po::value<double>(&tempConstrParams.positionWeight)->default_value(tempConstrParams.positionWeight), "Temporal Constraint Position Weight. Controls the weight of the temporal constraint applied to camera positions. Higher values enforce smoother camera path.")
    ("tscOrientationWeight", po::value<double>(&tempConstrParams.orientationWeight)->default_value(tempConstrParams.orientationWeight), "Temporal Constraint Orientation Weight. Controls the weight of the temporal constraint applied to camera orientations. Higher values enforce smoother camera rotation.")
    ("tscC0positionWeight", po::value<double>(&tempConstrParams.c0positionWeight)->default_value(tempConstrParams.c0positionWeight), "Temporal Constraint C0 Position Weight. Controls the weight of the continuity constraint on camera positions in the temporal constraint. Higher values enforce smoother transitions in position, reducing abrupt changes of position.")
    ("tscC1positionWeight", po::value<double>(&tempConstrParams.c1positionWeight)->default_value(tempConstrParams.c1positionWeight), "Temporal Constraint C1 Position Weight. Controls the weight of the first derivative of camera position in the temporal constraint. Higher values enforce continuity of the camera velocity, reducing abrupt changes of velocity.")
    ("tscC2positionWeight", po::value<double>(&tempConstrParams.c2positionWeight)->default_value(tempConstrParams.c2positionWeight), "Temporal Constraint C2 Position Weight: Controls the weight of the second derivative of camera position in the temporal constraint. Higher values enforce continuity of the camera acceleration, reducing abrupt changes of acceleration.")
    ("tscC0orientationWeight", po::value<double>(&tempConstrParams.c0orientationWeight)->default_value(tempConstrParams.c0orientationWeight), "Temporal Constraint C0 Orientation Weight. Controls the weight of the continuity constraint on camera orientation in the temporal constraint. Higher values enforce smoother transitions, reducing abrupt changes of the camera orientation.")
    ("tscC1orientationWeight", po::value<double>(&tempConstrParams.c1orientationWeight)->default_value(tempConstrParams.c1orientationWeight), "Temporal Constraint C1 Orientation Weight. Controls the weight of the first derivative of camera orientation in the temporal constraint. Higher values enforce continuity of the rotation velocity, reducing abrupt changes of rotation velocity.")
    ("tscC2orientationWeight", po::value<double>(&tempConstrParams.c2orientationWeight)->default_value(tempConstrParams.c2orientationWeight), "Temporal Constraint C2 Orientation Weight. Controls the weight of the second derivative of camera orientation in the temporal constraint. Higher values enforce continuity of the rotation acceleration, reducing abrupt changes of rotation acceleration.")
    ("tscLand2ViewsRegWeight", po::value<double>(&tempConstrParams.land2ViewsRegWeight)->default_value(tempConstrParams.land2ViewsRegWeight), "Scene Scale Based Regularization Weight. Controls the strength of a regularization applied to the temporal constraint, encouraging the mean distance between the landmarks and the views to remain constant.")
    ("tscTrajLengthRegWeight", po::value<double>(&tempConstrParams.trajLengthRegWeight)->default_value(tempConstrParams.trajLengthRegWeight), "Trajectory Length Based Regularization Weight. Controls the strength of a regularization applied to the temporal constraint, encouraging the trajectory length to remain constant.")
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
    ("weakResectionSize", po::value<int>(&weakResectionSize)->default_value(weakResectionSize),
        "When adding a view during the expansion process, we compute the pose. If the inliers count"
        "Is less than this value, the resection is considered weak. If not all views in the batch"
        "are weak, then the weak views are put back in the list of views to estimate again.")
    ("minNumberOfObservationsForTriangulation", po::value<std::size_t>(&minNbObservationsForTriangulation)->default_value(minNbObservationsForTriangulation),"Minimum number of observations to triangulate a point")
    ("minAngleForTriangulation", po::value<double>(&minAngleForTriangulation)->default_value(minAngleForTriangulation),"Minimum angle for triangulation.")
    ("minAngleForLandmark", po::value<double>(&minAngleForLandmark)->default_value(minAngleForLandmark), "Minimum angle for landmark.")
    ("maxTriangulationError", po::value<double>(&maxTriangulationError)->default_value(maxTriangulationError), "Maximum reprojection error in the triangulation process.")
    ("maxReprojectionError", po::value<double>(&maxReprojectionError)->default_value(maxReprojectionError), "Maximum reprojection error in the bundle verification step.")
    ("lockAllIntrinsics", po::value<bool>(&lockAllIntrinsics)->default_value(lockAllIntrinsics), "Force lock of all camera intrinsic parameters, so they will not be refined during Bundle Adjustment.")
    ("enableStructureRefinement", po::value<bool>(&enableStructureRefinement)->default_value(enableStructureRefinement), "Bundle adjustment will try to optimize the landmarks positions.")
    ("minNbCamerasToRefinePrincipalPoint", po::value<int>(&minNbCamerasToRefinePrincipalPoint)->default_value(minNbCamerasToRefinePrincipalPoint),
         "Minimal number of cameras to refine the principal point of the cameras (one of the intrinsic parameters of the camera). "
         "If we do not have enough cameras, the principal point is considered in the center of the image. "
         "If minNbCamerasToRefinePrincipalPoint<=0, the principal point is never refined. "
         "If minNbCamerasToRefinePrincipalPoint==1, the principal point is always refined.")
    ("useRigConstraint", po::value<bool>(&useRigConstraint)->default_value(useRigConstraint), "Enable/Disable rig constraint.")
    ("rigMinNbCamerasForCalibration", po::value<int>(&minNbCamerasForRigCalibration)->default_value(minNbCamerasForRigCalibration),"Minimal number of cameras to start the calibration of the rig.")
    ("meshFilename,t", po::value<std::string>(&meshFilename)->default_value(meshFilename), "Mesh file.");
    ;
     // clang-format on

    CmdLine cmdline("AliceVision SfM Expanding");

    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if(!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // set maxThreads (Limit to 100 threads on large machines)
    HardwareContext hwc = cmdline.getHardwareContext();
    hwc.setUserCoresLimit(100);
    omp_set_num_threads(hwc.getMaxThreads());

    // load input SfMData scene
    sfmData::SfMData sfmData;
    if(!sfmDataIO::load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" + sfmDataFilename + "' cannot be read.");
        return EXIT_FAILURE;
    }

    if (sfmData.getValidViews().size() < 1)
    {
        ALICEVISION_LOG_INFO("Expansion requires that some views are already defined.");
        return EXIT_SUCCESS;
    }

    // lock scene previously reconstructed
    if (lockScenePreviouslyReconstructed)
    {
        // lock all reconstructed camera poses
        for (auto& [_, pose] : sfmData.getPoses().valueRange())
        {
            pose.lock();
        }

        for (const auto& viewPair : sfmData.getViews())
        {
            // lock all reconstructed views intrinsics
            const sfmData::View& view = *(viewPair.second);

            if (sfmData.isPoseAndIntrinsicDefined(view))
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
    sfmBundle->setIsStructureRefinementEnabled(enableStructureRefinement);
    sfmBundle->setTemporalConstraintParams(tempConstrParams, useTemporalConstraint);

    sfm::PointFetcher::uptr pointFetcherHandler;
    if (!meshFilename.empty())
    {
        ALICEVISION_LOG_INFO("Load mesh");
        std::unique_ptr<MeshPointFetcher> handler = std::make_unique<MeshPointFetcher>();

        if (!handler->initialize(meshFilename))
        {
            return EXIT_FAILURE;
        }

        pointFetcherHandler = std::move(handler);
    }

    sfm::LocalizationValidationPolicy::uptr resectionValidationPolicy = std::make_unique<sfm::LocalizationValidationPolicyLegacy>();

    sfm::SfmResection::uptr sfmResectionHandler = std::make_unique<sfm::SfmResection>();
    sfmResectionHandler->setResectionMaxError(localizerEstimatorError);
    sfmResectionHandler->setMaxIterations(localizerEstimatorMaxIterations);
    sfmResectionHandler->setValidationPolicy(resectionValidationPolicy);

    sfm::SfmTriangulation::uptr sfmTriangulationHandler = std::make_unique<sfm::SfmTriangulation>(minNbObservationsForTriangulation, maxTriangulationError);
    sfmTriangulationHandler->setPointFetcherHandler(pointFetcherHandler);

    sfm::ExpansionChunk::uptr expansionChunk = std::make_unique<sfm::ExpansionChunk>();
    expansionChunk->setBundleHandler(sfmBundle);
    expansionChunk->setExpansionHistoryHandler(expansionHistory);
    expansionChunk->setResectionHandler(sfmResectionHandler);
    expansionChunk->setTriangulationHandler(sfmTriangulationHandler);
    expansionChunk->setEnableDepthPrior(enableDepthPrior);
    expansionChunk->setIgnoreMultiviewOnPrior(ignoreMultiviewOnPrior);
    expansionChunk->setMinAngleTriangulation(minAngleForTriangulation);
    expansionChunk->setWeakResectionSize(weakResectionSize);

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

    sfm::ExpansionPostProcess::uptr expansionPostProcess;
    if (useRigConstraint)
    {
        sfm::ExpansionPostProcessRig::uptr expansionPostProcessTyped = std::make_unique<sfm::ExpansionPostProcessRig>();
        expansionPostProcessTyped->setMinimalNumberCameras(minNbCamerasForRigCalibration);
        expansionPostProcess = std::move(expansionPostProcessTyped);
    }


    sfm::ExpansionProcess::uptr expansionProcess = std::make_unique<sfm::ExpansionProcess>();
    expansionProcess->setExpansionHistoryHandler(expansionHistory);
    expansionProcess->setExpansionIterationHandler(expansionIteration);
    expansionProcess->setExpansionIterationPostProcessHandler(expansionPostProcess);

    if (!expansionProcess->process(sfmData, tracksHandler))
    {
        ALICEVISION_LOG_INFO("Error processing sfmData");
        return EXIT_FAILURE;
    }

    sfmDataIO::save(sfmData, sfmDataOutputFilename, sfmDataIO::ESfMData::ALL);

    if (!outputSfMViewsAndPoses.empty())
    {
        sfmDataIO::save(sfmData, outputSfMViewsAndPoses,
            sfmDataIO::ESfMData(sfmDataIO::VIEWS | sfmDataIO::EXTRINSICS | sfmDataIO::INTRINSICS)
        );
    }

    return EXIT_SUCCESS;
}