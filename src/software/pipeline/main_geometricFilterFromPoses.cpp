// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/main.hpp>

#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/geometry/Pose3.hpp>
#include <aliceVision/matchingImageCollection/GeometricInfo.hpp>

#include <boost/program_options.hpp>

#include <algorithm>
#include <limits>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 1

using namespace aliceVision;
namespace po = boost::program_options;

int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string sfmDataFilename;
    std::string outputFiltersFolder;
    double threshold = 4.0;
    double minBaselineRatio = 1e-3;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
         "SfMData file.")
        ("output,o", po::value<std::string>(&outputFiltersFolder)->required(),
         "Path to a folder in which computed filters will be stored.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("threshold", po::value<double>(&threshold)->default_value(threshold),
         "Maximum error (in pixels) for a match to be considered as an inlier.")
        ("minBaselineRatio", po::value<double>(&minBaselineRatio)->default_value(minBaselineRatio),
         "Pairs with a baseline smaller than this ratio of the scene scale (median distance of cameras to their centroid) "
         "are filtered with a rotation model instead of an essential matrix.");
    // clang-format on

    CmdLine cmdline("AliceVision geometricFilterFromPoses");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    minBaselineRatio = std::max(minBaselineRatio, 1e-8);

    // load input SfMData scene
    sfmData::SfMData sfmData;
    if (!sfmDataIO::load(sfmData, sfmDataFilename, sfmDataIO::ESfMData(sfmDataIO::VIEWS | sfmDataIO::INTRINSICS | sfmDataIO::EXTRINSICS)))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" + sfmDataFilename + "' cannot be read.");
        return EXIT_FAILURE;
    }

    // Collect the pose of each view with a known pose and intrinsic (any camera model)
    // Views are stored in a map, so ids are sorted
    std::vector<std::pair<IndexT, geometry::Pose3>> poses;
    for (const auto& [viewId, view] : sfmData.getViews())
    {
        if (!sfmData.isPoseAndIntrinsicDefined(viewId))
        {
            continue;
        }

        poses.emplace_back(viewId, sfmData.getPose(*view).getTransform());
    }
    
    ALICEVISION_LOG_INFO(poses.size() << " views with known pose and intrinsic.");

    // Estimate the scene scale as the median distance of the camera centers to their centroid
    double sceneScale = 1.0;
    if (poses.size() >= 2)
    {
        Vec3 centroid = Vec3::Zero();
        for (const auto& [viewId, pose] : poses)
        {
            centroid += pose.center();
        }
        centroid /= static_cast<double>(poses.size());

        std::vector<double> distances;
        distances.reserve(poses.size());
        for (const auto& [viewId, pose] : poses)
        {
            distances.push_back((pose.center() - centroid).norm());
        }

        std::nth_element(distances.begin(), distances.begin() + distances.size() / 2, distances.end());
        const double median = distances[distances.size() / 2];
        if (median > std::numeric_limits<double>::epsilon())
        {
            sceneScale = median;
        }
    }

    const double minBaseline = minBaselineRatio * sceneScale;
    ALICEVISION_LOG_INFO("Scene scale: " << sceneScale << ", minimal baseline for an essential matrix: " << minBaseline);

    // Build a filter for each pair of views, expressed in camera space (on bearing vectors) so that it is
    // independent of the camera model:
    // - an essential matrix E = [t]x R (with t normalized) such that bJ^T E bI = 0
    // - a rotation matrix R such that bJ = R bI if the translation is too small for a reliable epipolar geometry
    int countRotations = 0;
    int countEssentials = 0;
    matchingImageCollection::PairwiseGeometricInfo geometricInfos;
    for (size_t i = 0; i < poses.size(); ++i)
    {
        for (size_t j = i + 1; j < poses.size(); ++j)
        {
            const auto& [idViewI, poseI] = poses[i];
            const auto& [idViewJ, poseJ] = poses[j];

            // Relative motion from camera I frame to camera J frame
            const geometry::Pose3 relativePose = poseJ * poseI.inverse();
            const Mat3 R = relativePose.rotation();
            const Vec3 t = relativePose.translation();

            matchingImageCollection::PairGeometricInfo info;
            info.inliers = 0;
            info.threshold = threshold;
            if (t.norm() < minBaseline)
            {
                info.model = R;
                info.type = matchingImageCollection::EGeometricFilterType::ROTATION_MATRIX;
                countRotations++;
            }
            else
            {
                info.model = CrossProductMatrix(t.normalized()) * R;
                info.type = matchingImageCollection::EGeometricFilterType::ESSENTIAL_MATRIX;
                countEssentials++;
            }
            geometricInfos.emplace(Pair(idViewI, idViewJ), info);
        }
    }

    ALICEVISION_LOG_INFO(geometricInfos.size() << " filters have been computed (" << countEssentials << " essential matrices, " << countRotations
                                               << " rotations for low baseline pairs)");

    // Output result to json file (named like a single chunk of geometricFilterEstimating)
    const std::string outputPath = outputFiltersFolder + "/matches_0.json";
    ALICEVISION_LOG_INFO("Saving to " << outputPath);
    if (!matchingImageCollection::saveGeometricInfos(outputPath, geometricInfos))
    {
        ALICEVISION_LOG_ERROR("Impossible to save geometricInfos");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
