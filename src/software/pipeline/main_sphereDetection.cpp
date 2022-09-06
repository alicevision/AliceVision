// Command line parameters
#include <aliceVision/system/main.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>

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

int aliceVision_main(int argc, char** argv)
{
    std::string input_sfmdata_path;
    std::string input_model_path;
    std::string output_path;

    po::options_description required_parameters("Required parameters");
    required_parameters.add_options()                                                                           //
        ("input_sfmdata_path,i", po::value<std::string>(&input_sfmdata_path)->required(), "SFMData input path") //
        ("input_model_path,m", po::value<std::string>(&input_model_path)->required(), "model input path")       //
        ("output_path,o", po::value<std::string>(&output_path)->required(), "output path")                      //
        ;                                                                                                       //

    po::options_description all_parameters("AliceVision sphereDetection");
    all_parameters.add(required_parameters);

    po::variables_map variable_map;
    try
    {
        po::store(po::parse_command_line(argc, argv, all_parameters), variable_map);
        if(variable_map.count("help") || (argc == 1))
        {
            ALICEVISION_COUT(all_parameters);
            return EXIT_SUCCESS;
        }
        po::notify(variable_map);
    }
    catch(boost::program_options::required_option& e)
    {
        ALICEVISION_CERR("ERROR: " << e.what());
        ALICEVISION_COUT("Usage:\n\n" << all_parameters);
        return EXIT_FAILURE;
    }
    catch(boost::program_options::error& e)
    {
        ALICEVISION_CERR("ERROR: " << e.what());
        ALICEVISION_COUT("Usage:\n\n" << all_parameters);
        return EXIT_FAILURE;
    }

    ALICEVISION_COUT("Program called with the following parameters:");
    ALICEVISION_COUT(variable_map);

    // onnxruntime session setup
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "Sphere detector onnx model environment");
    Ort::SessionOptions session_options;
    Ort::ThrowOnError(OrtSessionOptionsAppendExecutionProvider_CUDA(session_options, 0)); // use gpu n°0
    Ort::Session session(env, input_model_path.c_str(), session_options);

    // DEBUG: print model I/O
    model_explore(session);

    // load SFMData file
    aliceVision::sfmData::SfMData sfmData;
    if(!aliceVision::sfmDataIO::Load(
           sfmData, input_sfmdata_path,
           aliceVision::sfmDataIO::ESfMData(aliceVision::sfmDataIO::VIEWS | aliceVision::sfmDataIO::INTRINSICS)))
    {
        ALICEVISION_LOG_ERROR("The input file '" + input_sfmdata_path + "' cannot be read");
        return EXIT_FAILURE;
    }

    // parse output_path
    fs::path fs_output_path(output_path);

    // neural network magic
    sphereDetection(sfmData, session, fs_output_path);

    return EXIT_SUCCESS;
}