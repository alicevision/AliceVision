// This file is part of the AliceVision project.
// Copyright (c) 2015 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/sfm/sfm.hpp>
#include <aliceVision/sfm/pipeline/regionsIO.hpp>
#include <aliceVision/feature/imageDescriberCommon.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/types.hpp>
#include <aliceVision/config.hpp>
#include <aliceVision/track/TracksBuilder.hpp>
#include <aliceVision/sfm/bundle/BundleAdjustment.hpp>
#include <aliceVision/sfm/utils/alignment.hpp>

#include <boost/program_options.hpp>

#include <cstdlib>
#include <filesystem>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 2
#define ALICEVISION_SOFTWARE_VERSION_MINOR 4

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = std::filesystem;
using namespace aliceVision::track;
using namespace aliceVision::sfm;

/**
 * @brief Retrieve the view id in the sfmData from the image filename.
 * @param[in] sfmData the SfM scene
 * @param[in] name the image name to find (uid or filename or path)
 * @param[out] out_viewId the id found
 * @return if a view is found
 */
bool retrieveViewIdFromImageName(const sfmData::SfMData& sfmData, const std::string& name, IndexT& out_viewId)
{
    out_viewId = UndefinedIndexT;

    // list views uid / filenames and find the one that correspond to the user ones
    for (const auto& viewPair : sfmData.getViews())
    {
        const sfmData::View& v = *(viewPair.second.get());

        if (name == std::to_string(v.getViewId()) || name == fs::path(v.getImage().getImagePath()).filename().string() ||
            name == v.getImage().getImagePath())
        {
            out_viewId = v.getViewId();
            break;
        }
    }

    if (out_viewId == UndefinedIndexT)
        ALICEVISION_LOG_ERROR("Can't find the given initial pair view: " << name);

    return out_viewId != UndefinedIndexT;
}

int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string sfmDataFilename;
    std::vector<std::string> featuresFolders;
    std::vector<std::string> matchesFolders;
    std::string outputSfM;

    // user optional parameters
    std::string outputSfMViewsAndPoses;
    std::string extraInfoFolder;
    std::string describerTypesName = feature::EImageDescriberType_enumToString(feature::EImageDescriberType::SIFT);
    std::pair<std::string, std::string> initialPairString("", "");
    bool useAutoTransform = true;

    sfm::ReconstructionEngine_sequentialSfM::Params sfmParams;
    bool lockScenePreviouslyReconstructed = true;
    int maxNbMatches = 0;
    int minNbMatches = 0;
    bool useOnlyMatchesFromInputFolder = false;
    bool computeStructureColor = true;

    int randomSeed = std::mt19937::default_seed;
    bool logIntermediateSteps = false;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
         "SfMData file.")
        ("output,o", po::value<std::string>(&outputSfM)->required(),
         "Path to the output SfMData file.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("featuresFolders,f", po::value<std::vector<std::string>>(&featuresFolders)->multitoken(),
         "Path to folder(s) containing the extracted features.")
        ("matchesFolders,m", po::value<std::vector<std::string>>(&matchesFolders)->multitoken(),
         "Path to folder(s) in which computed matches are stored.")
        ("outputViewsAndPoses", po::value<std::string>(&outputSfMViewsAndPoses)->default_value(outputSfMViewsAndPoses),
         "Path to the output SfMData file (with only views and poses).")
        ("extraInfoFolder", po::value<std::string>(&extraInfoFolder)->default_value(extraInfoFolder),
         "Folder for intermediate reconstruction files and additional reconstruction information files.")
        ("describerTypes,d", po::value<std::string>(&describerTypesName)->default_value(describerTypesName),
         feature::EImageDescriberType_informations().c_str())
        ("interFileExtension", po::value<std::string>(&sfmParams.sfmStepFileExtension)->default_value(sfmParams.sfmStepFileExtension),
         "Extension of the intermediate file export.")
        ("maxNumberOfMatches", po::value<int>(&maxNbMatches)->default_value(maxNbMatches),
         "Maximum number of matches per image pair (and per feature type). "
         "This can be useful to have a quick reconstruction overview. 0 means no limit.")
        ("minNumberOfMatches", po::value<int>(&minNbMatches)->default_value(minNbMatches),
         "Minimum number of matches per image pair (and per feature type). "
         "This can be useful to have a meaningful reconstruction with accurate keypoints. 0 means no limit.")
        ("minInputTrackLength", po::value<int>(&sfmParams.minInputTrackLength)->default_value(sfmParams.minInputTrackLength),
         "Minimum track length in input of SfM.")
        ("minAngleForTriangulation", po::value<double>(&sfmParams.minAngleForTriangulation)->default_value(sfmParams.minAngleForTriangulation),
         "Minimum angle for triangulation.")
        ("minAngleForLandmark", po::value<double>(&sfmParams.minAngleForLandmark)->default_value(sfmParams.minAngleForLandmark),
         "Minimum angle for landmark.")
        ("maxReprojectionError", po::value<double>(&sfmParams.maxReprojectionError)->default_value(sfmParams.maxReprojectionError),
         "Maximum reprojection error.")
        ("minAngleInitialPair", po::value<float>(&sfmParams.minAngleInitialPair)->default_value(sfmParams.minAngleInitialPair),
         "Minimum angle for the initial pair.")
        ("maxAngleInitialPair", po::value<float>(&sfmParams.maxAngleInitialPair)->default_value(sfmParams.maxAngleInitialPair),
         "Maximum angle for the initial pair.")
        ("minNumberOfObservationsForTriangulation", po::value<std::size_t>(&sfmParams.minNbObservationsForTriangulation)->default_value(sfmParams.minNbObservationsForTriangulation),
         "Minimum number of observations to triangulate a point.\n"
         "Set it to 3 (or more) reduces drastically the noise in the point cloud, but the number of final poses is "
         "a little bit reduced (from 1.5% to 11% on the tested datasets).\n"
         "Note: set it to 0 or 1 to use the old triangulation algorithm (using 2 views only) during resection.")
        ("initialPairA", po::value<std::string>(&initialPairString.first)->default_value(initialPairString.first),
         "UID or filepath or filename of the first image.")
        ("initialPairB", po::value<std::string>(&initialPairString.second)->default_value(initialPairString.second),
         "UID or filepath or filename of the second image.")
        ("lockAllIntrinsics", po::value<bool>(&sfmParams.lockAllIntrinsics)->default_value(sfmParams.lockAllIntrinsics),
         "Force lock of all camera intrinsic parameters, so they will not be refined during Bundle Adjustment.")
        ("minNbCamerasToRefinePrincipalPoint", po::value<int>(&sfmParams.minNbCamerasToRefinePrincipalPoint)->default_value(sfmParams.minNbCamerasToRefinePrincipalPoint),
         "Minimal number of cameras to refine the principal point of the cameras (one of the intrinsic parameters of the camera). "
         "If we do not have enough cameras, the principal point in consider is considered in the center of the image. "
         "If minNbCamerasToRefinePrincipalPoint<=0, the principal point is never refined. "
         "If minNbCamerasToRefinePrincipalPoint==1, the principal point is always refined.")
        ("useLocalBA,l", po::value<bool>(&sfmParams.useLocalBundleAdjustment)->default_value(sfmParams.useLocalBundleAdjustment),
         "Enable/Disable the Local bundle adjustment strategy.\n"
         "It reduces the reconstruction time, especially for big datasets (500+ images).")
        ("localBAGraphDistance", po::value<int>(&sfmParams.localBundelAdjustementGraphDistanceLimit)->default_value(sfmParams.localBundelAdjustementGraphDistanceLimit),
         "Graph-distance limit setting the Active region in the Local Bundle Adjustment strategy.")
        ("nbFirstUnstableCameras", po::value<std::size_t>(&sfmParams.nbFirstUnstableCameras)->default_value(sfmParams.nbFirstUnstableCameras),
         "Number of cameras for which the bundle adjustment is performed every single time a camera is added, leading to more stable "
         "results while the computations are not too expensive since there is not much data. Past this number, the bundle adjustment "
         "will only be performed once for N added cameras.")
        ("maxImagesPerGroup", po::value<std::size_t>(&sfmParams.maxImagesPerGroup)->default_value(sfmParams.maxImagesPerGroup),
         "Maximum number of cameras that can be added before the bundle adjustment is performed. "
         "This prevents adding too much data at once without performing the bundle adjustment.")
        ("bundleAdjustmentMaxOutliers", po::value<int>(&sfmParams.bundleAdjustmentMaxOutliers)->default_value(sfmParams.bundleAdjustmentMaxOutliers),
         "Threshold for the maximum number of outliers allowed at the end of a bundle adjustment iteration."
         "Using a negative value for this threshold will disable BA iterations.")
        ("localizerEstimator", po::value<robustEstimation::ERobustEstimator>(&sfmParams.localizerEstimator)->default_value(sfmParams.localizerEstimator),
         "Estimator type used to localize cameras (acransac (default), ransac, lsmeds, loransac, maxconsensus).")
        ("localizerEstimatorError", po::value<double>(&sfmParams.localizerEstimatorError)->default_value(0.0),
         "Reprojection error threshold (in pixels) for the localizer estimator (0 for default value according to the estimator).")
        ("localizerEstimatorMaxIterations", po::value<std::size_t>(&sfmParams.localizerEstimatorMaxIterations)->default_value(sfmParams.localizerEstimatorMaxIterations),
         "Maximum number of RANSAC iterations.")
        ("useOnlyMatchesFromInputFolder", po::value<bool>(&useOnlyMatchesFromInputFolder)->default_value(useOnlyMatchesFromInputFolder),
         "Use only matches from the input matchesFolder parameter.\n"
         "Matches folders previously added to the SfMData file will be ignored.")
        ("filterTrackForks", po::value<bool>(&sfmParams.filterTrackForks)->default_value(sfmParams.filterTrackForks),
         "Enable/Disable the track forks removal. A track contains a fork when incoherent matches leads to multiple "
         "features in the same image for a single track.")
        ("useRigConstraint", po::value<bool>(&sfmParams.rig.useRigConstraint)->default_value(sfmParams.rig.useRigConstraint),
         "Enable/Disable rig constraint.")
        ("rigMinNbCamerasForCalibration", po::value<int>(&sfmParams.rig.minNbCamerasForCalibration)->default_value(sfmParams.rig.minNbCamerasForCalibration),
         "Minimal number of cameras to start the calibration of the rig.")
        ("lockScenePreviouslyReconstructed", po::value<bool>(&lockScenePreviouslyReconstructed)->default_value(lockScenePreviouslyReconstructed),
         "Lock/Unlock scene previously reconstructed.")
        ("observationConstraint", po::value<EFeatureConstraint>(&sfmParams.featureConstraint)->default_value(sfmParams.featureConstraint),
         "Use of an observation constraint: basic, scale the observation or use of the covariance.")
        ("computeStructureColor", po::value<bool>(&computeStructureColor)->default_value(computeStructureColor),
         "Compute each 3D point color.")
        ("useAutoTransform", po::value<bool>(&useAutoTransform)->default_value(useAutoTransform),
         "Transform the result with the alignment method 'AUTO'.")
        ("randomSeed", po::value<int>(&randomSeed)->default_value(randomSeed),
         "This seed value will generate a sequence using a linear random generator. Set -1 to use a random seed.")
        ("logIntermediateSteps", po::value<bool>(&sfmParams.logIntermediateSteps)->default_value(logIntermediateSteps),
         "If set to true, the current state of the scene will be dumped as an SfMData file every 3 resections.");
    // clang-format on

    CmdLine cmdline("Sequential/Incremental reconstruction.\n"
                    "This program performs incremental SfM (Initial Pair Essential + Resection).\n"
                    "AliceVision incrementalSfM");

    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // set maxThreads
    HardwareContext hwc = cmdline.getHardwareContext();
    omp_set_num_threads(hwc.getMaxThreads());

    const double defaultLoRansacLocalizationError = 4.0;
    if (!robustEstimation::adjustRobustEstimatorThreshold(
          sfmParams.localizerEstimator, sfmParams.localizerEstimatorError, defaultLoRansacLocalizationError))
    {
        return EXIT_FAILURE;
    }

    // load input SfMData scene
    sfmData::SfMData sfmData;
    if (!sfmDataIO::load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" + sfmDataFilename + "' cannot be read.");
        return EXIT_FAILURE;
    }

    // lock scene previously reconstructed
    if (lockScenePreviouslyReconstructed)
    {
        // lock all reconstructed camera poses
        for (auto& cameraPosePair : sfmData.getPoses())
            cameraPosePair.second.lock();

        for (const auto& viewPair : sfmData.getViews())
        {
            // lock all reconstructed views intrinsics
            const sfmData::View& view = *(viewPair.second);
            if (sfmData.isPoseAndIntrinsicDefined(&view))
                sfmData.getIntrinsics().at(view.getIntrinsicId())->lock();
        }
    }

    // get imageDescriber type
    const std::vector<feature::EImageDescriberType> describerTypes = feature::EImageDescriberType_stringToEnums(describerTypesName);

    // features reading
    feature::FeaturesPerView featuresPerView;
    if (!sfm::loadFeaturesPerView(featuresPerView, sfmData, featuresFolders, describerTypes))
    {
        ALICEVISION_LOG_ERROR("Invalid features.");
        return EXIT_FAILURE;
    }

    // matches reading
    matching::PairwiseMatches pairwiseMatches;
    if (!sfm::loadPairwiseMatches(
          pairwiseMatches, sfmData, matchesFolders, describerTypes, maxNbMatches, minNbMatches, useOnlyMatchesFromInputFolder))
    {
        ALICEVISION_LOG_ERROR("Unable to load matches.");
        return EXIT_FAILURE;
    }

    if (extraInfoFolder.empty())
        extraInfoFolder = fs::path(outputSfM).parent_path().string();

    if (!fs::exists(extraInfoFolder))
        fs::create_directory(extraInfoFolder);

    // sequential reconstruction process
    aliceVision::system::Timer timer;

    if (sfmParams.minNbObservationsForTriangulation < 2)
    {
        // allows to use to the old triangulatation algorithm (using 2 views only) during resection.
        sfmParams.minNbObservationsForTriangulation = 0;
        // ALICEVISION_LOG_ERROR("The value associated to the argument '--minNbObservationsForTriangulation' must be >= 2 ");
        // return EXIT_FAILURE;
    }

    // handle initial pair parameter
    if (!initialPairString.first.empty() || !initialPairString.second.empty())
    {
        if (initialPairString.first == initialPairString.second)
        {
            ALICEVISION_LOG_ERROR("Invalid image names. You cannot use the same image to initialize a pair.");
            return EXIT_FAILURE;
        }

        if (!initialPairString.first.empty() && !retrieveViewIdFromImageName(sfmData, initialPairString.first, sfmParams.userInitialImagePair.first))
        {
            ALICEVISION_LOG_ERROR("Could not find corresponding view in the initial pair: " + initialPairString.first);
            return EXIT_FAILURE;
        }

        if (!initialPairString.second.empty() &&
            !retrieveViewIdFromImageName(sfmData, initialPairString.second, sfmParams.userInitialImagePair.second))
        {
            ALICEVISION_LOG_ERROR("Could not find corresponding view in the initial pair: " + initialPairString.second);
            return EXIT_FAILURE;
        }
    }

    sfm::ReconstructionEngine_sequentialSfM sfmEngine(sfmData, sfmParams, extraInfoFolder, (fs::path(extraInfoFolder) / "sfm_log.html").string());

    sfmEngine.initRandomSeed(randomSeed);

    // configure the featuresPerView & the matches_provider
    sfmEngine.setFeatures(&featuresPerView);
    sfmEngine.setMatches(&pairwiseMatches);

    if (!sfmEngine.process())
        return EXIT_FAILURE;

    // Mimic sfmTransform "EAlignmentMethod::AUTO"
    if (useAutoTransform)
    {
        double S = 1.0;
        Mat3 R = Mat3::Identity();
        Vec3 t = Vec3::Zero();

        ALICEVISION_LOG_DEBUG("Align automatically");
        sfm::computeNewCoordinateSystemAuto(sfmEngine.getSfMData(), S, R, t);
        sfm::applyTransform(sfmEngine.getSfMData(), S, R, t);

        ALICEVISION_LOG_DEBUG("Align with ground");
        sfm::computeNewCoordinateSystemGroundAuto(sfmEngine.getSfMData(), t);
        sfm::applyTransform(sfmEngine.getSfMData(), 1.0, Eigen::Matrix3d::Identity(), t);
    }

    // set featuresFolders and matchesFolders relative paths
    {
        sfmEngine.getSfMData().addFeaturesFolders(featuresFolders);
        sfmEngine.getSfMData().addMatchesFolders(matchesFolders);
        sfmEngine.getSfMData().setAbsolutePath(outputSfM);
    }

    // get the color for the 3D points
    if (computeStructureColor)
        sfmEngine.colorize();

    sfmEngine.retrieveMarkersId();

    ALICEVISION_LOG_INFO("Structure from motion took (s): " + std::to_string(timer.elapsed()));
    ALICEVISION_LOG_INFO("Generating HTML report...");

    sfm::generateSfMReport(sfmEngine.getSfMData(), (fs::path(extraInfoFolder) / "sfm_report.html").string());

    // export to disk computed scene (data & visualizable results)
    ALICEVISION_LOG_INFO("Export SfMData to disk: " + outputSfM);

    sfmDataIO::save(sfmEngine.getSfMData(),
                    (fs::path(extraInfoFolder) / ("cloud_and_poses" + sfmParams.sfmStepFileExtension)).string(),
                    sfmDataIO::ESfMData(sfmDataIO::VIEWS | sfmDataIO::EXTRINSICS | sfmDataIO::INTRINSICS | sfmDataIO::STRUCTURE));
    sfmDataIO::save(sfmEngine.getSfMData(), outputSfM, sfmDataIO::ESfMData::ALL);

    if (!outputSfMViewsAndPoses.empty())
        sfmDataIO::save(
          sfmEngine.getSfMData(), outputSfMViewsAndPoses, sfmDataIO::ESfMData(sfmDataIO::VIEWS | sfmDataIO::EXTRINSICS | sfmDataIO::INTRINSICS));

    ALICEVISION_LOG_INFO("Structure from Motion results:" << std::endl
                                                          << "\t- # input images: " << sfmEngine.getSfMData().getViews().size() << std::endl
                                                          << "\t- # cameras calibrated: " << sfmEngine.getSfMData().getValidViews().size()
                                                          << std::endl
                                                          << "\t- # poses: " << sfmEngine.getSfMData().getPoses().size() << std::endl
                                                          << "\t- # landmarks: " << sfmEngine.getSfMData().getLandmarks().size());

    return EXIT_SUCCESS;
}
