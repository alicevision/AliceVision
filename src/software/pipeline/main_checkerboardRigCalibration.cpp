// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <boost/program_options.hpp>

#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/system/Timer.hpp>

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/sfm/BundleAdjustmentSymbolicCeres.hpp>

// This application tries to estimate the rig extrinsics
// It is assumed that for each image we have a result of the checkerboard detector.
// It is assumed that for each image we have estimated its intrinsics and extrinsics using the required application.


// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 0
#define ALICEVISION_SOFTWARE_VERSION_MINOR 1

namespace po = boost::program_options;

using namespace aliceVision;


int aliceVision_main(int argc, char* argv[])
{
    std::string sfmInputDataFilepath;
    std::string outputFilePath;
    std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());

    // Command line parameters
    po::options_description allParams(
    "Parse external information about cameras used in a panorama.\n"
    "AliceVision PanoramaInit");

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
    ("inputSfmData,i", po::value<std::string>(&sfmInputDataFilepath)->required(), "SfMData file input.")
    ("output,o", po::value<std::string>(&outputFilePath)->required(), "calibration boards json output directory.");

    po::options_description logParams("Log parameters");
    logParams.add_options()
    ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
        "verbosity level (fatal, error, warning, info, debug, trace).");

    allParams.add(requiredParams).add(logParams);

    // Parse command line
    po::variables_map vm;
    try
    {
        po::store(po::parse_command_line(argc, argv, allParams), vm);

        if(vm.count("help") || (argc == 1))
        {
            ALICEVISION_COUT(allParams);
            return EXIT_SUCCESS;
        }
        po::notify(vm);
    }
    catch(boost::program_options::required_option& e)
    {
        ALICEVISION_CERR("ERROR: " << e.what());
        ALICEVISION_COUT("Usage:\n\n" << allParams);
        return EXIT_FAILURE;
    }
    catch(boost::program_options::error& e)
    {
        ALICEVISION_CERR("ERROR: " << e.what());
        ALICEVISION_COUT("Usage:\n\n" << allParams);
        return EXIT_FAILURE;
    }

    ALICEVISION_COUT("Program called with the following parameters:");
    ALICEVISION_COUT(vm);

    system::Logger::get()->setLogLevel(verboseLevel);

    sfmData::SfMData sfmData;
    if(!sfmDataIO::Load(sfmData, sfmInputDataFilepath, sfmDataIO::ESfMData(sfmDataIO::ALL)))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmInputDataFilepath << "' cannot be read.");
        return EXIT_FAILURE;
    }

    //group views by frameid
    std::map<int, std::vector<std::shared_ptr<sfmData::View>>> mapGroupViews;
    for (auto& pv : sfmData.getViews())
    {
        pv.second->setIndependantPose(false);
        mapGroupViews[pv.second->getFrameId()].push_back(pv.second);
    }

    bool first = true;

    sfmData.getRigs().begin()->second.getSubPose(0).status = sfmData::ERigSubPoseStatus::CONSTANT;
    sfmData.getRigs().begin()->second.getSubPose(1).status = sfmData::ERigSubPoseStatus::ESTIMATED;
    
    for (auto& pg : mapGroupViews)
    {
        if (pg.second.size() != 2)
        {
            ALICEVISION_LOG_ERROR("Found group with more or less than 2 views. This was not tested.");
            return EXIT_FAILURE;
        }

        IndexT posesId[] = {UndefinedIndexT, UndefinedIndexT};
        
        posesId[pg.second[0]->getSubPoseId()] = pg.second[0]->getPoseId();
        posesId[pg.second[1]->getSubPoseId()] = pg.second[1]->getPoseId();

        sfmData::CameraPose p1 = sfmData.getPoses()[posesId[0]];
   
        if (first)
        {
            sfmData::CameraPose p2 = sfmData.getPoses()[posesId[1]];
            auto c1To = p1.getTransform().getHomogeneous();
            auto c2To = p2.getTransform().getHomogeneous();
            Eigen::Matrix4d c2Tc1 = c2To * c1To.inverse();

            sfmData.getRigs().begin()->second.getSubPose(0).pose = geometry::Pose3();
            sfmData.getRigs().begin()->second.getSubPose(1).pose = geometry::Pose3(c2Tc1.block<3, 4>(0, 0));
            first = false;
        }
        
        //Both views share the same pose
        sfmData.getPoses()[posesId[1]] = sfmData.getPoses()[posesId[0]];
    }

    //Compute non linear refinement
    sfm::BundleAdjustmentSymbolicCeres::CeresOptions options;
    options.summary = true;
    sfm::BundleAdjustmentSymbolicCeres ba(options);
    sfm::BundleAdjustment::ERefineOptions boptions = sfm::BundleAdjustment::ERefineOptions::REFINE_ROTATION |
        sfm::BundleAdjustment::ERefineOptions::REFINE_TRANSLATION |
        sfm::BundleAdjustment::ERefineOptions::REFINE_INTRINSICS_ALL;
    if (!ba.adjust(sfmData, boptions))
    {
        ALICEVISION_LOG_ERROR("Failed to calibrate");
        return EXIT_FAILURE;
    }

    //Save sfmData to disk
    if (!sfmDataIO::Save(sfmData, outputFilePath, sfmDataIO::ESfMData(sfmDataIO::ALL)))
    {
        ALICEVISION_LOG_ERROR("The output SfMData file '" << outputFilePath << "' cannot be written.");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
