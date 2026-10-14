// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/system/main.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/feature/imageDescriberCommon.hpp>
#include <aliceVision/camera/IntrinsicScaleOffset.hpp>
#include <aliceVision/camera/IntrinsicScaleOffsetDisto.hpp>

#include <boost/program_options.hpp>

#include <set>
#include <string>
#include <vector>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 2

using namespace aliceVision;

namespace po = boost::program_options;

int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string sfmDataFilename;
    std::string sfmDataOutputFilename;
    std::string selectedViewsFilename;
    bool lockIntrinsics = false;
    bool lockFocalLength = true;
    bool lockPrincipalPoint = true;
    bool lockDistortion = true;
    bool lockPoses = false;
    bool lockLandmarks = false;
    std::string lockLandmarkTypes;
    std::string landmarkSelectionMode = "fully_contained";

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
         "SfMData file.")
        ("output,o", po::value<std::string>(&sfmDataOutputFilename)->required(),
         "Output SfMData file.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("selectedViews,s", po::value<std::string>(&selectedViewsFilename)->default_value(selectedViewsFilename),
         "Optional SfMData file used to define a subset of views. "
         "When provided, locking is restricted to elements associated with those views.")
        ("lockIntrinsics", po::value<bool>(&lockIntrinsics)->default_value(lockIntrinsics),
         "Lock camera intrinsics.")
        ("lockFocalLength", po::value<bool>(&lockFocalLength)->default_value(lockFocalLength),
         "Lock the focal length of camera intrinsics. Only used when lockIntrinsics is enabled.")
        ("lockPrincipalPoint", po::value<bool>(&lockPrincipalPoint)->default_value(lockPrincipalPoint),
         "Lock the principal point of camera intrinsics. Only used when lockIntrinsics is enabled.")
        ("lockDistortion", po::value<bool>(&lockDistortion)->default_value(lockDistortion),
         "Lock the distortion parameters of camera intrinsics. Only used when lockIntrinsics is enabled.")
        ("lockPoses", po::value<bool>(&lockPoses)->default_value(lockPoses),
         "Lock all camera poses.")
        ("lockLandmarks", po::value<bool>(&lockLandmarks)->default_value(lockLandmarks),
         "Lock landmarks.")
        ("lockLandmarkTypes", po::value<std::string>(&lockLandmarkTypes)->default_value(lockLandmarkTypes),
         "Comma-separated list of landmark describer types to lock (e.g. 'sift,dspsift'). "
         "If empty, all landmark types will be locked.")
        ("landmarkSelectionMode", po::value<std::string>(&landmarkSelectionMode)->default_value(landmarkSelectionMode),
         "Landmark selection mode when selectedViews is provided: "
         "'fully_contained' to lock landmarks whose all observations belong to the selected views, "
         "'partially_contained' to lock landmarks with at least one observation in the selected views.");
    // clang-format on

    CmdLine cmdline("AliceVision lockSfmData");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // Load input SfMData scene
    sfmData::SfMData sfmData;
    if (!sfmDataIO::load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" + sfmDataFilename + "' cannot be read.");
        return EXIT_FAILURE;
    }

    // Validate landmarkSelectionMode
    if (landmarkSelectionMode != "fully_contained" && landmarkSelectionMode != "partially_contained")
    {
        ALICEVISION_LOG_ERROR("Invalid landmarkSelectionMode '" << landmarkSelectionMode
                              << "'. Expected 'fully_contained' or 'partially_contained'.");
        return EXIT_FAILURE;
    }

    // Build the set of selected view IDs (from the optional selectedViews SfMData)
    std::set<IndexT> selectedViewIds;
    const bool hasSelectedViews = !selectedViewsFilename.empty();
    if (hasSelectedViews)
    {
        sfmData::SfMData selectedViewsSfmData;
        // Only VIEWS data is needed since we only extract view IDs from this SfMData
        if (!sfmDataIO::load(selectedViewsSfmData, selectedViewsFilename, sfmDataIO::ESfMData::VIEWS))
        {
            ALICEVISION_LOG_ERROR("The selectedViews SfMData file '" + selectedViewsFilename + "' cannot be read.");
            return EXIT_FAILURE;
        }
        for (const auto& [viewId, _] : selectedViewsSfmData.getViews())
        {
            selectedViewIds.insert(viewId);
        }
        ALICEVISION_LOG_INFO("Selected views subset contains " << selectedViewIds.size() << " view(s).");
    }

    // Lock camera intrinsics
    if (lockIntrinsics)
    {
        // If selectedViews is provided, collect the intrinsic IDs referenced by those views
        std::set<IndexT> intrinsicIdsToLock;
        if (hasSelectedViews)
        {
            for (const auto& [viewId, view] : sfmData.getViews())
            {
                if (selectedViewIds.count(viewId) && view->getIntrinsicId() != UndefinedIndexT)
                {
                    intrinsicIdsToLock.insert(view->getIntrinsicId());
                }
            }
        }

        std::size_t lockedCount = 0;
        for (auto& [intrinsicId, intrinsic] : sfmData.getIntrinsics().valueRange())
        {
            if (hasSelectedViews && !intrinsicIdsToLock.count(intrinsicId))
            {
                continue;
            }

            if (lockFocalLength && lockPrincipalPoint && lockDistortion)
            {
                // Lock all intrinsic parts at once using the global lock
                intrinsic.lock();
            }
            else
            {
                // Lock only the requested parts
                auto* isoPtr = dynamic_cast<camera::IntrinsicScaleOffset*>(&intrinsic);
                if (isoPtr)
                {
                    isoPtr->setScaleLocked(lockFocalLength);
                    isoPtr->setOffsetLocked(lockPrincipalPoint);
                }

                if (lockDistortion)
                {
                    auto* isodPtr = dynamic_cast<camera::IntrinsicScaleOffsetDisto*>(&intrinsic);
                    if (isodPtr && isodPtr->getDistortion())
                    {
                        isodPtr->getDistortion()->setLocked(true);
                    }
                }
            }
            ++lockedCount;
        }
        ALICEVISION_LOG_INFO("Processed " << lockedCount << " camera intrinsic(s).");
    }

    // Lock camera poses
    if (lockPoses)
    {
        // If selectedViews is provided, collect the pose IDs referenced by those views
        std::set<IndexT> poseIdsToLock;
        if (hasSelectedViews)
        {
            for (const auto& [viewId, view] : sfmData.getViews())
            {
                if (selectedViewIds.count(viewId) && view->getPoseId() != UndefinedIndexT)
                {
                    poseIdsToLock.insert(view->getPoseId());
                }
            }
        }

        std::size_t lockedCount = 0;
        for (auto& [poseId, pose] : sfmData.getPoses().valueRange())
        {
            if (hasSelectedViews && !poseIdsToLock.count(poseId))
            {
                continue;
            }
            pose.lock();
            ++lockedCount;
        }
        ALICEVISION_LOG_INFO("Locked " << lockedCount << " camera pose(s).");
    }

    // Lock landmarks (optionally filtered by describer type and/or selected views)
    if (lockLandmarks)
    {
        std::set<feature::EImageDescriberType> typesToLock;
        if (!lockLandmarkTypes.empty())
        {
            const std::vector<feature::EImageDescriberType> typesList =
                feature::EImageDescriberType_stringToEnums(lockLandmarkTypes);
            typesToLock.insert(typesList.begin(), typesList.end());
        }

        std::size_t lockedCount = 0;
        for (auto& [_, landmark] : sfmData.getLandmarks())
        {
            if (!typesToLock.empty() && !typesToLock.count(landmark.getDescType()))
            {
                continue;
            }

            if (hasSelectedViews)
            {
                const Observations& obs = landmark.getObservations();
                bool include = false;
                if (landmarkSelectionMode == "partially_contained")
                {
                    // At least one observation in the selected views
                    for (const auto& [viewId, _] : obs)
                    {
                        if (selectedViewIds.count(viewId))
                        {
                            include = true;
                            break;
                        }
                    }
                }
                else // fully_contained
                {
                    // All observations must belong to selected views
                    include = !obs.empty();
                    for (const auto& [viewId, _] : obs)
                    {
                        if (!selectedViewIds.count(viewId))
                        {
                            include = false;
                            break;
                        }
                    }
                }

                if (!include)
                {
                    continue;
                }
            }

            landmark.setLocked(true);
            ++lockedCount;
        }
        ALICEVISION_LOG_INFO("Locked " << lockedCount << " landmark(s) out of " << sfmData.getLandmarks().size() << ".");
    }

    // Save output SfMData
    if (!sfmDataIO::save(sfmData, sfmDataOutputFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The output SfMData file '" + sfmDataOutputFilename + "' cannot be written.");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
