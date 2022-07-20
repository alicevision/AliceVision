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

#include <fmt/core.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <onnxruntime_cxx_api.h>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 0
#define ALICEVISION_SOFTWARE_VERSION_MINOR 1

int aliceVision_main(int argc, char** argv)
{
    namespace po = boost::program_options;
    namespace fs = boost::filesystem;

    std::string modelInputPath;
    std::string imageInputPath;
    std::string imageOutputPath;

    po::options_description allParams("AliceVision sphereDetection");

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()                                                                      //
        ("modelInputPath,m", po::value<std::string>(&modelInputPath)->required(), "model input path") //
        ("imageInputPath,i", po::value<std::string>(&imageInputPath)->required(), "image input path") //
        ("imageOutputPath,o", po::value<std::string>(&imageOutputPath)->default_value(""), "image output path");

    allParams.add(requiredParams);

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

    // onnxruntime session setup
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "U-Net");
    Ort::SessionOptions session_options;
    Ort::Session session(env, modelInputPath.c_str(), session_options);

    // verify every image in the folder are of the same resolution
    cv::Size size = verifySameResolution(imageInputPath);

    // compute the mask
    cv::Mat mask = computeAverageMask(session, imageInputPath, size);

    // extract circles from mask
    auto circles = circlesFromMask(mask);

    return 0;
}