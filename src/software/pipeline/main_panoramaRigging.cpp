// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmData/Rig.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/cmdline/cmdline.hpp>

#include <boost/program_options.hpp>

#include <map>
#include <set>
#include <vector>
#include <algorithm>
#include <iterator>

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
    std::string rigDescriptionFilename;
    std::string outputSfMDataFilepath;
    std::string outputViewsAndPosesFilepath;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
         "SfMData file.")
        ("rigDescription", po::value<std::string>(&rigDescriptionFilename)->required(),
         "Rig description file.")
        ("output,o", po::value<std::string>(&outputSfMDataFilepath)->required(),
         "Path of the output SfMData file.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("outputViewsAndPoses", po::value<std::string>(&outputViewsAndPosesFilepath),
         "Path of the output SfMData file.");
    // clang-format on

    CmdLine cmdline("Apply rig structure ('rigDescription') to a fully solved panorama ('Input')");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

   
    // load panorama SfMData scene
    sfmData::SfMData inputSfmData;
    if (!sfmDataIO::load(inputSfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' cannot be read.");
        return EXIT_FAILURE;
    }

    // load rig SfMData scene
    sfmData::SfMData rigSfmData;
    if (!sfmDataIO::load(rigSfmData, rigDescriptionFilename, sfmDataIO::ESfMData(sfmDataIO::VIEWS | sfmDataIO::INTRINSICS | sfmDataIO::EXTRINSICS)))
    {
        ALICEVISION_LOG_ERROR("The rigDescription SfMData file '" << rigDescriptionFilename << "' cannot be read.");
        return EXIT_FAILURE;
    }
    
    if (rigSfmData.getRigs().size() != 1)
    {
        ALICEVISION_LOG_ERROR("One and only one rig is required.");
        return EXIT_FAILURE;
    }

    // Will we ever have panorama rigs with more than 2 views ?
    if (rigSfmData.getRigs().begin()->second.getNbSubPoses() != 2)
    {
        ALICEVISION_LOG_ERROR("At the moment, only 2 subposes supported.");
        return EXIT_FAILURE;
    }

    // Build set of landmarks per view
    std::map<IndexT, std::set<IndexT>> landmarkPerView;
    for (const auto [id, landmark] : inputSfmData.getLandmarks())
    {
        for (const auto [viewId, obs]: landmark.getObservations())
        {
            landmarkPerView[viewId].insert(id);
        }
    }
    
    std::vector<std::pair<Pair, size_t>> scores;
    for (const auto & [idViewRef, _] : inputSfmData.getViews())
    {
        const auto & setRef = landmarkPerView[idViewRef];

        for (const auto & [idViewOther, _] : inputSfmData.getViews())
        {
            if (idViewOther <= idViewRef)
            {
                continue;
            }

            const auto & setOther = landmarkPerView[idViewOther];

            std::vector<IndexT> commonSet;
            std::set_intersection(setRef.begin(), setRef.end(), setOther.begin(), setOther.end(), std::back_inserter(commonSet));

            Pair p;
            p.first = idViewRef;
            p.second = idViewOther;

            scores.push_back(std::make_pair(p, commonSet.size()));
        }
    }

    // Sort pairs per overlap
    std::sort(scores.begin(), scores.end(), [](const auto & item1, const auto & item2)
    {
        return item1.second > item2.second;
    });
    

    // loop over pairs which are sorted by scores
    bool found = false;
    geometry::Pose3 relativeRotation;
    const sfmData::Views & rigViews = rigSfmData.getViews();
    for (const auto & [pair, score] : scores)
    {
        if (rigViews.find(pair.first) == rigViews.end())
        {
            continue;
        }

        if (rigViews.find(pair.second) == rigViews.end())
        {
            continue;
        }

        const auto & rigvref = rigSfmData.getView(pair.first);
        const auto & rigvother = rigSfmData.getView(pair.second);
        const auto & invref = inputSfmData.getView(pair.first);
        const auto & invother = inputSfmData.getView(pair.second);

        // Looking for a pair with different subposeId
        if (rigvref.getSubPoseId() == rigvother.getSubPoseId())
        {
            continue;
        }

        // Looking for a pair with same poseId
        // Those are obviously not the most constrained pairs
        // But this make the computation easy and should be ok if the panorama is well constrained.
        if (rigvref.getPoseId() != rigvother.getPoseId())
        {
            continue;
        }

        

        const sfmData::CameraPose & cpref = inputSfmData.getPose(invref);
        const sfmData::CameraPose & cpother = inputSfmData.getPose(invother);

        Eigen::Matrix3d R = cpother.getTransform().rotation() * cpref.getTransform().rotation().transpose();

        // The relative rotation may be the inverse of the rig relative rotation (Pair order)
        if (rigvref.getSubPoseId() == 0)
        {
            relativeRotation.setRotation(R);
        }
        else 
        {
            relativeRotation.setRotation(R.transpose());
        }

        found = true;

        break;
    }

    if (!found)
    {
        ALICEVISION_LOG_ERROR("Can't find a suitable pair of poses.");
        return EXIT_FAILURE;
    }

    sfmData::SfMData outSfmData = rigSfmData;

    sfmData::Rig & rig = outSfmData.getRigs().begin()->second;
    rig.setSubPose(0, sfmData::RigSubPose(geometry::Pose3(), sfmData::ERigSubPoseStatus::ESTIMATED));
    rig.setSubPose(1, sfmData::RigSubPose(relativeRotation, sfmData::ERigSubPoseStatus::ESTIMATED));


    // Loop on all views which are from the first element of the rig
    for (const auto & [idView, view] : rigSfmData.getViews().valueRange())
    {
        if (view.getSubPoseId() != 0)
        {
            continue;
        }

        //Read pose from input
        const sfmData::View & iview = inputSfmData.getView(idView);
        const sfmData::CameraPose & cp = inputSfmData.getPose(iview);

        //Set absolute pose for rig
        IndexT destPoseId = view.getPoseId();
        outSfmData.getPoses().assign(destPoseId, cp);
    }

    // Export to disk computed scene (data & visualizable results)
    ALICEVISION_LOG_INFO("Export SfMData to disk");
    sfmDataIO::save(outSfmData, outputSfMDataFilepath, sfmDataIO::ESfMData::ALL);

    if (!outputViewsAndPosesFilepath.empty())
    {
        sfmDataIO::save(outSfmData, outputViewsAndPosesFilepath, sfmDataIO::ESfMData(sfmDataIO::VIEWS | sfmDataIO::EXTRINSICS | sfmDataIO::INTRINSICS));
    }

    return EXIT_SUCCESS;
}
