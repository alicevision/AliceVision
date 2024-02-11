// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

// Command line parameters
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/system/Logger.hpp>

#include <aliceVision/sphereDetection/sphereDetection.hpp>

// SFMData
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>

#include <filesystem>
#include <iostream>
#include <numeric>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <onnxruntime_cxx_api.h>

#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

namespace fs = std::filesystem;
namespace po = boost::program_options;

using namespace aliceVision;

int aliceVision_main(int argc, char** argv)
{
    system::Timer timer;

    std::string inputSfMDataPath;
    std::string inputModelPath;
    std::string outputPath;
    float inputMinScore;

    bool autoDetect;
    Eigen::Vector2f sphereCenterOffset(0, 0);
    double sphereRadius = 1.0;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&inputSfMDataPath)->required(),
         "SfMData input path.")
        ("modelPath,m", po::value<std::string>(&inputModelPath)->required(),
         "Model input path.")
        ("autoDetect,a", po::value<bool>(&autoDetect)->required(),
         "True if the sphere is to be automatically detected, false otherwise.")
        ("output,o", po::value<std::string>(&outputPath)->required(),
         "Output path.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("minScore,s", po::value<float>(&inputMinScore)->default_value(0.0),
         "Minimum detection score.")
        ("x,x", po::value<float>(&sphereCenterOffset(0))->default_value(0.0),
         "Sphere's center offset X (pixels).")
        ("y,y", po::value<float>(&sphereCenterOffset(1))->default_value(0.0),
         "Sphere's center offset Y (pixels).")
        ("sphereRadius,r", po::value<double>(&sphereRadius)->default_value(1.0),
         "Sphere's radius (pixels).");
    // clang-format on

    CmdLine cmdline("AliceVision sphereDetection");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);

    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // Load SFMData file
    sfmData::SfMData sfmData;
    if (!sfmDataIO::load(sfmData, inputSfMDataPath, sfmDataIO::ESfMData(sfmDataIO::VIEWS | sfmDataIO::INTRINSICS)))
    {
        ALICEVISION_LOG_ERROR("The input file '" + inputSfMDataPath + "' cannot be read");
        return EXIT_FAILURE;
    }

    // Parse output_path
    fs::path fsOutputPath(outputPath);

    if (autoDetect)
    {
        // ONNXRuntime session setup
        Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "Sphere detector ONNX model environment");
        Ort::SessionOptions sessionOptions;
#if defined(_WIN32) || defined(_WIN64)
        std::wstring modelPath(inputModelPath.begin(), inputModelPath.end());
        Ort::Session session(env, modelPath.c_str(), sessionOptions);
#else
        Ort::Session session(env, inputModelPath.c_str(), sessionOptions);
#endif
        // DEBUG: print model I/O
        sphereDetection::modelExplore(session);

        // Neural network magic
        sphereDetection::sphereDetection(sfmData, session, fsOutputPath, inputMinScore);
    }
    else
    {
        std::array<float, 3> sphereParam;
        sphereParam[0] = sphereCenterOffset(0);
        sphereParam[1] = sphereCenterOffset(1);
        sphereParam[2] = sphereRadius;

        sphereDetection::writeManualSphereJSON(sfmData, sphereParam, fsOutputPath);
    }

    ALICEVISION_LOG_INFO("Task done in (s): " + std::to_string(timer.elapsed()));
    return EXIT_SUCCESS;
}
