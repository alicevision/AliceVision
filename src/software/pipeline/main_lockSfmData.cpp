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
#define ALICEVISION_SOFTWARE_VERSION_MINOR 1

using namespace aliceVision;

namespace po = boost::program_options;

int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string sfmDataFilename;
    std::string sfmDataOutputFilename;
    bool lockIntrinsics = false;
    bool lockFocalLength = true;
    bool lockPrincipalPoint = true;
    bool lockDistortion = true;
    bool lockPoses = false;
    bool lockLandmarks = false;
    std::string lockLandmarkTypes;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
         "SfMData file.")
        ("output,o", po::value<std::string>(&sfmDataOutputFilename)->required(),
         "Output SfMData file.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
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
         "If empty, all landmark types will be locked.");
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

    // Lock camera intrinsics
    if (lockIntrinsics)
    {
        std::size_t lockedCount = 0;
        for (auto& [_, intrinsic] : sfmData.getIntrinsics().valueRange())
        {
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
        for (auto& [_, pose] : sfmData.getPoses().valueRange())
        {
            pose.lock();
        }
        ALICEVISION_LOG_INFO("Locked " << sfmData.getPoses().size() << " camera pose(s).");
    }

    // Lock landmarks (optionally filtered by describer type)
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
            if (typesToLock.empty() || typesToLock.count(landmark.getDescType()))
            {
                landmark.setLocked(true);
                ++lockedCount;
            }
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
