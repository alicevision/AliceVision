// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

// Input and geometry
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

// Image
#include <aliceVision/image/all.hpp>
#include <aliceVision/image/imageAlgo.hpp>

// System
#include <aliceVision/system/MemoryInfo.hpp>
#include <aliceVision/system/Logger.hpp>

// Reading command line options
#include <boost/program_options.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/main.hpp>

// IO
#include <fstream>
#include <algorithm>

// ONNXRuntime
#include <onnxruntime_cxx_api.h>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;

int aliceVision_main(int argc, char** argv)
{
    std::string sfmDataFilepath;
    
    // Description of mandatory parameters
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilepath)->required(), "Input sfmData.");

    CmdLine cmdline(
        "AliceVision imageSegmentation");
    cmdline.add(requiredParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // load input scene
    sfmData::SfMData sfmData;
    if(!sfmDataIO::Load(sfmData, sfmDataFilepath, sfmDataIO::ESfMData(sfmDataIO::VIEWS)))
    {
        ALICEVISION_LOG_ERROR("The input file '" + sfmDataFilepath + "' cannot be read");
        return EXIT_FAILURE;
    }

    
    const OrtApi * ortObject = OrtGetApiBase()->GetApi(ORT_API_VERSION);
    if (!ortObject) {
        ALICEVISION_LOG_ERROR("ONNX runtime failed to initialize");
        return EXIT_FAILURE;
    }

    OrtEnv * ortEnvironment;
    ortObject->CreateEnv(ORT_LOGGING_LEVEL_WARNING, "imageSegmentation", &ortEnvironment);
    if (ortEnvironment == nullptr)
    {
        ALICEVISION_LOG_ERROR("ONNX runtime failed to create ONNX environment");
        return EXIT_FAILURE;
    }

    OrtSessionOptions * ortSessionOptions;
    OrtStatus * ortStatus = ortObject->CreateSessionOptions(&ortSessionOptions);

    if (ortStatus != nullptr)
    {
        ALICEVISION_LOG_ERROR("ONNX runtime failed to create ONNX session options");
        ortObject->ReleaseStatus(ortStatus);
        return EXIT_FAILURE;
    }

    OrtSession* ortSession;
    ortStatus = ortObject->CreateSession(ortEnvironment, "/s/apps/users/servantf/MeshroomResearch/mrrs/segmentation/semantic/fcn_resnet50.onnx", ortSessionOptions, &ortSession);
    if (ortStatus != nullptr)
    {
        ALICEVISION_LOG_ERROR("ONNX runtime failed to create ONNX session");
        ortObject->ReleaseStatus(ortStatus);
        return EXIT_FAILURE;
    }

    OrtMemoryInfo* ortMemoryInfo;
    ortStatus = ortObject->CreateCpuMemoryInfo(OrtArenaAllocator, OrtMemTypeDefault, &ortMemoryInfo);
    if (ortStatus != nullptr)
    {
        ALICEVISION_LOG_ERROR("ONNX runtime failed to create ONNX Memory info");
        ortObject->ReleaseStatus(ortStatus);
        return EXIT_FAILURE;
    }

    ortObject->ReleaseSessionOptions(ortSessionOptions);
    ortObject->ReleaseSession(ortSession);
    ortObject->ReleaseEnv(ortEnvironment);

    return EXIT_SUCCESS;
}
