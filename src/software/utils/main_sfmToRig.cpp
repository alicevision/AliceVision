// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/stl/hash.hpp>
#include <aliceVision/system/main.hpp>

#include <boost/program_options.hpp>

#include <string>
#include <sstream>

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
    std::string outSfMDataFilename;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
         "SfMData file.")
        ("output,o", po::value<std::string>(&outSfMDataFilename)->required(),
         "Output SfMData scene.");
    // clang-format on

    CmdLine cmdline("AliceVision sfmToRig");
    cmdline.add(requiredParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // Load input scene
    sfmData::SfMData sfmData;
    if (!sfmDataIO::load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' cannot be read");
        return EXIT_FAILURE;
    }

    if (sfmData.getRigs().size() > 0)
    {
        ALICEVISION_LOG_INFO("Ignoring existing rig");
        sfmData.getRigs().clear();
    }

    // Remove existing landmarks
    sfmData.getLandmarks().clear();
    sfmData.getConstraints2D().clear();
    sfmData.getRotationPriors().clear();

    sfmData::Rig rig(sfmData.getPoses().size());

    // Build rig
    size_t indexRig = 0;
    int index = 0;
    std::map<IndexT, int> mapPoseToSubPose;
    for (const auto& pp : sfmData.getPoses())
    {
        const auto& pose = pp.second;

        sfmData::RigSubPose subPose(pose.getTransform(), sfmData::ERigSubPoseStatus::CONSTANT);
        rig.setSubPose(index, subPose);

        // Rig id is the combination of uid of poses
        stl::hash_combine(indexRig, pp.first);

        mapPoseToSubPose[pp.first] = index;
        index++;
    }

    // Insert rig
    sfmData.getRigs().emplace(indexRig, rig);

    // Update
    for (const auto& pv : sfmData.getViews())
    {
        std::shared_ptr<sfmData::View> view = pv.second;
        if (!sfmData.isPoseAndIntrinsicDefined(view.get()))
        {
            continue;
        }

        const IndexT poseId = view->getPoseId();
        const int subPoseId = mapPoseToSubPose[poseId];

        // New commmon pose id is the same than the rig id for convenience
        view->setPoseId(indexRig);
        view->setRigAndSubPoseId(indexRig, subPoseId);
        view->setIndependantPose(false);

        // Update intrinsicId
        size_t intrinsicId = view->getIntrinsicId();
        stl::hash_combine(intrinsicId, indexRig);
        view->setIntrinsicId(intrinsicId);
    }

    // Update intrinsics
    sfmData::Intrinsics intrinsics = sfmData.getIntrinsics();
    sfmData.getIntrinsics().clear();
    for (const auto& pi : intrinsics)
    {
        size_t intrinsicId = pi.first;
        stl::hash_combine(intrinsicId, indexRig);
        sfmData.getIntrinsics().emplace(intrinsicId, pi.second);
    }

    // Remove all poses
    sfmData.getPoses().clear();

    if (!sfmDataIO::save(sfmData, outSfMDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("An error occurred while trying to save '" << outSfMDataFilename << "'");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
