// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/system/Timer.hpp>

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <regex>



// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 0
#define ALICEVISION_SOFTWARE_VERSION_MINOR 1

namespace po = boost::program_options;

using namespace aliceVision;




int aliceVision_main(int argc, char* argv[])
{
    std::string sfmInput1DataFilepath;
    std::string sfmInput2DataFilepath;
    std::string outputFilePath;
    std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());

    // Command line parameters
    po::options_description allParams(
    "Create Rig from 2 input Sfm data files");

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
    ("inputSfmData1,i1", po::value<std::string>(&sfmInput1DataFilepath)->required(), "First SfMData file input.")
    ("inputSfmData2,i2", po::value<std::string>(&sfmInput2DataFilepath)->required(), "Second SfMData file input.")
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

    sfmData::SfMData sfmData1;
    if(!sfmDataIO::Load(sfmData1, sfmInput1DataFilepath, sfmDataIO::ESfMData(sfmDataIO::ALL)))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmInput1DataFilepath << "' cannot be read.");
        return EXIT_FAILURE;
    }

    sfmData::SfMData sfmData2;
    if (!sfmDataIO::Load(sfmData2, sfmInput2DataFilepath, sfmDataIO::ESfMData(sfmDataIO::ALL)))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmInput2DataFilepath << "' cannot be read.");
        return EXIT_FAILURE;
    }


    sfmData::SfMData output;
    sfmData::Rig rig(2);
    output.getRigs()[0] = rig;

    std::map<int, std::vector<std::shared_ptr<sfmData::View>>> paired_views;

    //Group views of both inputs using a common number in the filename

    for (auto& pv : sfmData1.getViews())
    {
        int val = pv.second->getFrameId();
        paired_views[val].push_back(pv.second);
    }

    for (auto& pv : sfmData2.getViews())
    {
        int val = pv.second->getFrameId();
        paired_views[val].push_back(pv.second);
    }

    auto & ov = output.getViews();
    for (auto p : paired_views)
    {
        if (p.second.size() != 2)
        {
            ALICEVISION_LOG_ERROR("Unmatched images to create rig");
            return EXIT_FAILURE;
        }

        p.second[0]->setRigAndSubPoseId(0, 0);
        p.second[1]->setRigAndSubPoseId(0, 1);
        ov[p.second[0]->getViewId()] = p.second[0];
        ov[p.second[1]->getViewId()] = p.second[1];
    }
        
    auto& outIntrinsics = output.getIntrinsics();
    outIntrinsics = sfmData1.getIntrinsics();
    const auto& in2Intrinsics = sfmData2.getIntrinsics();
    outIntrinsics.insert(in2Intrinsics.begin(), in2Intrinsics.end());

    auto& outLandmarks = output.getLandmarks();
    outLandmarks = sfmData1.getLandmarks();
    const auto& in2Landmarks = sfmData2.getLandmarks();
    outLandmarks.insert(in2Landmarks.begin(), in2Landmarks.end());

    auto& outPoses = output.getPoses();
    outPoses = sfmData1.getPoses();
    const auto& in2Poses = sfmData2.getPoses();
    outPoses.insert(in2Poses.begin(), in2Poses.end());

    //Save sfmData to disk
    if (!sfmDataIO::Save(output, outputFilePath, sfmDataIO::ESfMData(sfmDataIO::ALL)))
    {
        ALICEVISION_LOG_ERROR("The output SfMData file '" << outputFilePath << "' cannot be written.");
        return EXIT_FAILURE;
    }


    return EXIT_SUCCESS;
}
