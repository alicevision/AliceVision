// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/sfm/pipeline/regionsIO.hpp>
#include <aliceVision/feature/imageDescriberCommon.hpp>
#include <aliceVision/sfm/pipeline/panorama/ReconstructionEngine_panorama.hpp>
#include <aliceVision/sfm/utils/alignment.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/image/all.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <OpenImageIO/imageio.h>
#include <OpenImageIO/imagebuf.h>
#include <OpenImageIO/imagebufalgo.h>

#include <cstdlib>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string sfmDataFilename;
    std::vector<std::string> featuresFolders;
    std::vector<std::string> matchesFolders;
    std::string outputSfMDataFilepath;
    std::string outputViewsAndPosesFilepath;

    // user optional parameters
    std::string describerTypesName = feature::EImageDescriberType_enumToString(feature::EImageDescriberType::SIFT);
    bool filterMatches = false;
    bool refine = true;
    float offsetLongitude = 0.0f;
    float offsetLatitude = 0.0f;
    bool useAutomaticReferenceFrame = true;

    int randomSeed = std::mt19937::default_seed;

    sfm::ReconstructionEngine_panorama::Params params;

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()("input,i", po::value<std::string>(&sfmDataFilename)->required(), "SfMData file.")(
        "output,o", po::value<std::string>(&outputSfMDataFilepath)->required(), "Path of the output SfMData file.")(
        "featuresFolders,f", po::value<std::vector<std::string>>(&featuresFolders)->multitoken()->required(),
        "Path to folder(s) containing the extracted features.")(
        "matchesFolders,m", po::value<std::vector<std::string>>(&matchesFolders)->multitoken()->required(),
        "Path to folder(s) in which computed matches are stored.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()("describerTypes,d",
                                 po::value<std::string>(&describerTypesName)->default_value(describerTypesName),
                                 feature::EImageDescriberType_informations().c_str())(
        "rotationAveraging",
        po::value<sfm::ERotationAveragingMethod>(&params.eRotationAveragingMethod)
            ->default_value(params.eRotationAveragingMethod),
        "* 1: L1 minimization\n"
        "* 2: L2 minimization")("relativeRotation",
                                po::value<sfm::ERelativeRotationMethod>(&params.eRelativeRotationMethod)
                                    ->default_value(params.eRelativeRotationMethod),
                                "* from essential matrix"
                                "* from rotation matrix"
                                "* from homography matrix")(
        "rotationAveragingWeighting",
        po::value<bool>(&params.rotationAveragingWeighting)->default_value(params.rotationAveragingWeighting),
        "Use weighting of image links during rotation averaging.")(
        "offsetLongitude", po::value<float>(&offsetLongitude)->default_value(offsetLongitude),
        "offset to camera longitude")(
        "offsetLatitude", po::value<float>(&offsetLatitude)->default_value(offsetLatitude),
        "offset to camera latitude")("filterMatches", po::value<bool>(&filterMatches)->default_value(filterMatches),
                                     "Filter Matches before solving the Panorama.")(
        "refine", po::value<bool>(&refine)->default_value(refine), "Refine cameras with a Bundle Adjustment")(
        "lockAllIntrinsics", po::value<bool>(&params.lockAllIntrinsics)->default_value(params.lockAllIntrinsics),
        "Force lock of all camera intrinsic parameters, so they will not be refined during Bundle Adjustment.")
        ("maxAngleToPrior", po::value<double>(&params.maxAngleToPrior)->default_value(params.maxAngleToPrior),
        "Maximal angle allowed regarding the input prior.")
        ("maxAngleToPriorRefined", po::value<double>(&params.maxAngleToPriorRefined)->default_value(params.maxAngleToPriorRefined),
        "Maximal angle allowed regarding the input prior after refinement.")
        ("maxAngularError", po::value<double>(&params.maxAngularError)->default_value(params.maxAngularError),
        "Maximal angular error in global rotation averaging.")(
        "intermediateRefineWithFocal",
        po::value<bool>(&params.intermediateRefineWithFocal)->default_value(params.intermediateRefineWithFocal),
        "Add an intermediate refine with rotation+focal in the different BA steps.")(
        "intermediateRefineWithFocalDist",
        po::value<bool>(&params.intermediateRefineWithFocalDist)->default_value(params.intermediateRefineWithFocalDist),
        "Add an intermediate refine with rotation+focal+distortion in the different BA steps.")(
        "outputViewsAndPoses", po::value<std::string>(&outputViewsAndPosesFilepath),
        "Path of the output SfMData file.")(
        "randomSeed", po::value<int>(&randomSeed)->default_value(randomSeed),
        "This seed value will generate a sequence using a linear random generator. Set -1 to use a random seed.");

    CmdLine cmdline("Estimates the orientation of a camera around a nodal point for the creation of a 360° panorama.\n"
                    "AliceVision panoramaEstimation");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if(!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    if(params.eRotationAveragingMethod < sfm::ROTATION_AVERAGING_L1 ||
       params.eRotationAveragingMethod > sfm::ROTATION_AVERAGING_L2)
    {
        ALICEVISION_LOG_ERROR("Rotation averaging method is invalid");
        return EXIT_FAILURE;
    }

    if(params.eRelativeRotationMethod < sfm::RELATIVE_ROTATION_FROM_E ||
       params.eRelativeRotationMethod > sfm::RELATIVE_ROTATION_FROM_H)
    {
        ALICEVISION_LOG_ERROR("Relative rotation method is invalid");
        return EXIT_FAILURE;
    }

    // load input SfMData scene
    sfmData::SfMData inputSfmData;
    if(!sfmDataIO::Load(inputSfmData, sfmDataFilename,
                        sfmDataIO::ESfMData(sfmDataIO::VIEWS | sfmDataIO::INTRINSICS | sfmDataIO::EXTRINSICS)))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' cannot be read.");
        return EXIT_FAILURE;
    }

    if(!inputSfmData.getLandmarks().empty())
    {
        ALICEVISION_LOG_ERROR("Partially computed SfMData is not currently supported in PanoramaEstimation.");
        return EXIT_FAILURE;
    }

    if(!inputSfmData.getRigs().empty())
    {
        ALICEVISION_LOG_ERROR("Rigs are not currently supported in PanoramaEstimation.");
        return EXIT_FAILURE;
    }

    /* Store the pose c1_R_o of the prior */
    sfmData::Poses& initial_poses = inputSfmData.getPoses();
    Eigen::Matrix3d c1_R_oprior = Eigen::Matrix3d::Identity();
    if(!initial_poses.empty())
    {
        c1_R_oprior = initial_poses.begin()->second.getTransform().rotation();
    }

    // get describerTypes
    const std::vector<feature::EImageDescriberType> describerTypes =
        feature::EImageDescriberType_stringToEnums(describerTypesName);

    // features reading
    feature::FeaturesPerView featuresPerView;
    if(!sfm::loadFeaturesPerView(featuresPerView, inputSfmData, featuresFolders, describerTypes))
    {
        ALICEVISION_LOG_ERROR("Invalid features");
        return EXIT_FAILURE;
    }

    // matches reading
    // Load the match file (try to read the two matches file formats).
    matching::PairwiseMatches pairwiseMatches;
    if(!sfm::loadPairwiseMatches(pairwiseMatches, inputSfmData, matchesFolders, describerTypes))
    {
        ALICEVISION_LOG_ERROR("Unable to load matches files from: " << matchesFolders);
        return EXIT_FAILURE;
    }

    const std::string outDirectory = fs::path(outputSfMDataFilepath).parent_path().string();

    if(!fs::exists(outDirectory))
    {
        ALICEVISION_LOG_ERROR("Output folder does not exist: " << outDirectory);
        return EXIT_FAILURE;
    }

    // Panorama reconstruction process
    aliceVision::system::Timer timer;
    sfm::ReconstructionEngine_panorama sfmEngine(inputSfmData, params, outDirectory,
                                                 (fs::path(outDirectory) / "sfm_log.html").string());

    sfmEngine.initRandomSeed(randomSeed);

    // configure the featuresPerView & the matches_provider
    sfmEngine.SetFeaturesProvider(&featuresPerView);
    sfmEngine.SetMatchesProvider(&pairwiseMatches);

    if(filterMatches)
    {
        sfmEngine.filterMatches();
    }

    if(!sfmEngine.process())
    {
        return EXIT_FAILURE;
    }

    // set featuresFolders and matchesFolders relative paths
    {
        sfmEngine.getSfMData().addFeaturesFolders(featuresFolders);
        sfmEngine.getSfMData().addMatchesFolders(matchesFolders);
        sfmEngine.getSfMData().setAbsolutePath(outputSfMDataFilepath);
    }

    if(refine)
    {
        sfmDataIO::Save(sfmEngine.getSfMData(), (fs::path(outDirectory) / "BA_before.abc").string(),
                        sfmDataIO::ESfMData::ALL);
        if(!sfmEngine.Adjust())
        {
            return EXIT_FAILURE;
        }
        sfmDataIO::Save(sfmEngine.getSfMData(), (fs::path(outDirectory) / "BA_after.abc").string(),
                        sfmDataIO::ESfMData::ALL);
    }

    sfmData::SfMData& outSfmData = sfmEngine.getSfMData();

    // If an initial set of poses was available, make sure at least one pose is aligned with it
    // Otherwise take the middle view (sorted over time)
    sfmData::Poses& final_poses = outSfmData.getPoses();

    if(!final_poses.empty())
    {
        Eigen::Matrix3d ocur_R_oprior = Eigen::Matrix3d::Identity();

        if(initial_poses.empty())
        {
            if(useAutomaticReferenceFrame)
            {
                double S = 1.0;
                Eigen::Matrix3d R;
                Vec3 t;
                sfm::computeNewCoordinateSystemFromCamerasXAxis(outSfmData, S, R, t);
                ocur_R_oprior = R.transpose();
            }
            else
            {
                std::vector<std::pair<int64_t, IndexT>> sorted_views;

                // Sort views per timestamps
                for(auto v : outSfmData.getViews())
                {
                    int64_t t = v.second->getImage().getMetadataDateTimestamp();
                    sorted_views.push_back(std::make_pair(t, v.second->getPoseId()));
                }
                std::sort(sorted_views.begin(), sorted_views.end());

                // Get the view which was taken at the middle of the sequence
                int median = sorted_views.size() / 2;
                IndexT poseId = sorted_views[median].second;

                // Set as reference
                ocur_R_oprior = final_poses[poseId].getTransform().rotation().transpose();
            }
        }
        else
        {
            Eigen::Matrix3d c1_R_ocur = final_poses.begin()->second.getTransform().rotation();
            ocur_R_oprior = c1_R_ocur.transpose() * c1_R_oprior;
        }

        for(auto& pose : final_poses)
        {
            geometry::Pose3 p = pose.second.getTransform();

            Eigen::Matrix3d c_R_oprior = p.rotation() * ocur_R_oprior;

            p.setRotation(c_R_oprior);
            pose.second.setTransform(p);
        }
    }

    // Final report
    ALICEVISION_LOG_INFO("Panorama solve took (s): " << timer.elapsed());
    ALICEVISION_LOG_INFO("Generating HTML report...");
    sfm::generateSfMReport(outSfmData, (fs::path(outDirectory) / "sfm_report.html").string());

    // Add offsets to rotations
    for(auto& pose : outSfmData.getPoses())
    {
        geometry::Pose3 p = pose.second.getTransform();
        Eigen::Matrix3d matLongitude =
            Eigen::AngleAxisd(degreeToRadian(offsetLongitude), Vec3(0, 1, 0)).toRotationMatrix();
        Eigen::Matrix3d matLatitude =
            Eigen::AngleAxisd(degreeToRadian(offsetLatitude), Vec3(1, 0, 0)).toRotationMatrix();
        Eigen::Matrix3d newR = p.rotation() * matLongitude * matLatitude;
        p.setRotation(newR);
        pose.second.setTransform(p);
    }

    sfmEngine.buildLandmarks();
    sfmEngine.colorize();

    {
        std::set<IndexT> viewsWithObservations;
        for(const auto& landmarkIt : outSfmData.getLandmarks())
        {
            for(const auto& obsIt : landmarkIt.second.getObservations())
            {
                viewsWithObservations.insert(obsIt.first);
            }
        }

        ALICEVISION_LOG_INFO("Panorama results:"
                             << std::endl
                             << "\t- # input images: " << outSfmData.getViews().size() << std::endl
                             << "\t- # cameras calibrated: " << outSfmData.getValidViews().size() << std::endl
                             << "\t- # cameras with observations: " << viewsWithObservations.size() << std::endl
                             << "\t- # landmarks: " << outSfmData.getLandmarks().size());
    }

    // Export to disk computed scene (data & visualizable results)
    ALICEVISION_LOG_INFO("Export SfMData to disk");
    sfmDataIO::Save(outSfmData, outputSfMDataFilepath, sfmDataIO::ESfMData::ALL);
    sfmDataIO::Save(outSfmData, (fs::path(outDirectory) / "cloud_and_poses.ply").string(), sfmDataIO::ESfMData::ALL);

    if(!outputViewsAndPosesFilepath.empty())
    {
        sfmDataIO::Save(outSfmData, outputViewsAndPosesFilepath,
                        sfmDataIO::ESfMData(sfmDataIO::VIEWS | sfmDataIO::EXTRINSICS | sfmDataIO::INTRINSICS));
    }

    return EXIT_SUCCESS;
}
