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
#include <boost/filesystem.hpp>

#include <iostream>
#include <numeric>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <onnxruntime_cxx_api.h>

#define ALICEVISION_SOFTWARE_VERSION_MAJOR 2
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

namespace fs = boost::filesystem;
namespace po = boost::program_options;

using namespace aliceVision;

int aliceVision_main(int argc, char** argv)
{

    std::string input_sfmdata_path;
    std::string input_model_path;
    std::string output_path;
    float input_min_score;

    bool autoDetect;
    Eigen::Vector2f sphereCenterOffset(0, 0);
    double sphereRadius = 1.0;

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input_sfmdata_path,i", po::value<std::string>(&input_sfmdata_path)->required(), "SFMData input path") //
        ("input_model_path,m", po::value<std::string>(&input_model_path)->required(), "model input path")       //
        ("autoDetect,a", po::value<bool>(&autoDetect)->required(), "Is the sphere automaticaly detected ?")     //
        ("output_path,o", po::value<std::string>(&output_path)->required(), "output path")                      //
        ;                                                                                                       //

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
    ("input_min_score,s", po::value<float>(&input_min_score)->default_value(0.0), "minimum detection score")
    ("x,x", po::value<float>(&sphereCenterOffset(0))->default_value(0.0), "Sphere's center offset X (pixels).")
    ("y,y", po::value<float>(&sphereCenterOffset(1))->default_value(0.0), "Sphere's center offset Y (pixels).")
    ("sphereRadius,r", po::value<double>(&sphereRadius)->default_value(1.0), "Sphere's radius (pixels).");


    CmdLine cmdline("AliceVision sphereDetection");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);

    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // load SFMData file
    sfmData::SfMData sfmData;
    if(!sfmDataIO::Load(sfmData, input_sfmdata_path, sfmDataIO::ESfMData(sfmDataIO::VIEWS | sfmDataIO::INTRINSICS)))
    {
        ALICEVISION_LOG_ERROR("The input file '" + input_sfmdata_path + "' cannot be read");
        return EXIT_FAILURE;
    }

    // parse output_path
    fs::path fs_output_path(output_path);

    if(autoDetect)
    {
        // onnxruntime session setup
        Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "Sphere detector onnx model environment");
        Ort::SessionOptions session_options;
        Ort::Session session(env, input_model_path.c_str(), session_options);

        // DEBUG: print model I/O
        sphereDetection::model_explore(session);

        // neural network magic
        sphereDetection::sphereDetection(sfmData, session, fs_output_path, input_min_score);
    }
    else
    {
        std::array<float, 3> sphereParam;
        sphereParam[0] = sphereCenterOffset(0);
        sphereParam[1] = sphereCenterOffset(1);
        sphereParam[2] = sphereRadius;

        sphereDetection::writeManualSphereJSON(sfmData, sphereParam, fs_output_path);
    }
    return 0;
}
