// Command line parameters
#include <aliceVision/system/main.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>

#include <aliceVision/sphereDetection/sphereDetection.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <iostream>
#include <numeric>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <onnxruntime_cxx_api.h>

#define ALICEVISION_SOFTWARE_VERSION_MAJOR 0
#define ALICEVISION_SOFTWARE_VERSION_MINOR 1

namespace po = boost::program_options;
namespace fs = boost::filesystem;

int aliceVision_main(int argc, char** argv)
{
    std::string input_model_path;
    std::string input_image_dir_path;
    std::string output_json_dir_path;

    po::options_description required_parameters("Required parameters");
    required_parameters.add_options()                                                                             //
        ("input_model_path,m", po::value<std::string>(&input_model_path)->required(), "model input path")         //
        ("input_image_dir_path,i", po::value<std::string>(&input_image_dir_path)->required(), "image input path") //
        ("output_json_dir_path,o", po::value<std::string>(&output_json_dir_path)->required(), "json output path") //
        ;                                                                                                         //

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
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "Sphere detector onnx model");
    Ort::SessionOptions session_options;
    Ort::Session session(env, input_model_path.c_str(), session_options);

    // verify every image in the folder are of the same resolution
    cv::Size size = resolution_verify(input_image_dir_path);

    // compute the mask
    cv::Mat mask = compute_mask_mean(session, input_image_dir_path, size);

    // extract circles from mask
    auto circles = compute_circles(mask);

    return 0;
}