#include <aliceVision/image/all.hpp>
#include <aliceVision/image/io.hpp>

// SFMData
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

// Photometric Stereo
#include <aliceVision/photometricStereo/photometricDataIO.hpp>
#include <aliceVision/photometricStereo/photometricStereo.hpp>

// Command line parameters
#include <aliceVision/system/main.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>

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

/**
 * @brief Operator overloading for printing vectors
 * @tparam T
 * @param os
 * @param v
 * @return std::ostream&
 */
template <typename T>
std::ostream& operator<<(std::ostream& os, const std::vector<T>& v)
{
    os << "[";
    for(int i = 0; i < v.size(); ++i)
    {
        os << v[i];
        if(i != v.size() - 1)
        {
            os << ", ";
        }
    }
    os << "]";
    return os;
}

// void waitforQ()
// {
//     for(;;)
//     {
//         char c = cv::waitKey();
//         if('q' == c)
//             break;
//     }
// }

float sigmoid(float x)
{
    return 1 / (1 + exp(-x));
}

float scale = 1.0 / 255;
// std::string modelFilepath =
// "/home/laurent/Documents/Cours/ENSEEIHT/Stage-2A-REVA/Blender/src/opencv++onnx/best.onnx";

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

    // onnxruntime setup
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "U-Net");
    Ort::SessionOptions session_options;
    Ort::Session session(env, modelInputPath.c_str(), session_options);

    Ort::AllocatorWithDefaultOptions allocator;

    // print name/shape of inputs
    size_t input_count = session.GetInputCount();
    const char* input_name;
    for(size_t i = 0; i < input_count; i++)
    {
        input_name = session.GetInputName(i, allocator);
        std::cout << "Input[" << i << "]: " << input_name << std::endl;

        Ort::TypeInfo input_info = session.GetInputTypeInfo(i);
        auto input_info2 = input_info.GetTensorTypeAndShapeInfo();

        ONNXTensorElementDataType input_type = input_info2.GetElementType();
        std::cout << "  "
                  << "Type : " << input_type << std::endl;

        std::vector<int64_t> input_shape = input_info2.GetShape();
        size_t input_size = std::accumulate(begin(input_shape), end(input_shape), 1, std::multiplies<float>());
        std::cout << "  Shape: " << input_shape << std::endl << "  Size : " << input_size << std::endl;
    }

    // print name/shape of outputs
    size_t output_count = session.GetOutputCount();
    const char* output_name;
    for(size_t i = 0; i < output_count; i++)
    {
        output_name = session.GetOutputName(i, allocator);
        std::cout << "Output[" << i << "]: " << output_name << std::endl;

        Ort::TypeInfo output_info = session.GetOutputTypeInfo(i);
        auto output_info2 = output_info.GetTensorTypeAndShapeInfo();

        ONNXTensorElementDataType output_type = output_info2.GetElementType();
        std::cout << "  Type : " << output_type << std::endl;

        std::vector<int64_t> output_shape = output_info2.GetShape();
        size_t output_size = std::accumulate(begin(output_shape), end(output_shape), 1, std::multiplies<float>());
        std::cout << "  Shape: " << output_shape << std::endl << "  Size : " << output_size << std::endl;
    }

    // Assume model has 1 input node and 1 output node.
    assert(input_count == 1 && output_count == 1);

    std::vector<cv::String> fn;
    cv::glob(imageInputPath, fn, false);

    cv::Mat bigStuff(683, 1024, CV_32FC1);

    for(size_t i = 0; i < fn.size(); i++)
    {
        // read image
        cv::Mat imageBGR = cv::imread(fn[i], cv::ImreadModes::IMREAD_COLOR);
        auto imageShape = imageBGR.size();

        // resize image
        int longest_side = std::max(imageShape.height, imageShape.width);
        float factor = 1024.0 / longest_side;
        cv::Size resizedShape(imageShape.width * factor, imageShape.height * factor);

        cv::Mat resizedBGR;
        cv::resize(imageBGR, resizedBGR, resizedShape, cv::INTER_LINEAR);

        // show image
        // cv::imshow("resized image", resizedBGR);
        // waitforQ();

        // BGR -> RGB
        cv::Mat imageRGB;
        cv::cvtColor(resizedBGR, imageRGB, cv::ColorConversionCodes::COLOR_BGR2RGB);

        // show image
        // cv::imshow("test 2", imageRGB);
        // waitforQ();

        // uint8 -> float32
        cv::Mat floatImage;
        imageRGB.convertTo(floatImage, CV_32F, 1.0 / 255.0);

        // show image
        // cv::imshow("test 3", floatImage);
        // waitforQ();

        // HWC to CHW
        cv::Mat preprocessedImage;
        cv::dnn::blobFromImage(floatImage, preprocessedImage);

        // creating tensors
        std::vector<Ort::Value> inputTensors;
        std::vector<Ort::Value> outputTensors;

        // inference on cpu (?)
        Ort::MemoryInfo memoryInfo =
            Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);

        // selecting inputs and outputs (manually for now)
        std::vector<const char*> inputNames{input_name};
        std::vector<const char*> outputNames{output_name};

        std::vector<int64_t> input_shape = {1, 3, resizedShape.height, resizedShape.width};
        std::vector<int64_t> output_shape = {1, 1, resizedShape.height, resizedShape.width};

        size_t input_size = std::accumulate(begin(input_shape), end(input_shape), 1, std::multiplies<float>());
        size_t output_size = std::accumulate(begin(output_shape), end(output_shape), 1, std::multiplies<float>());

        std::vector<float> inputTensorValues(input_size);
        std::vector<float> outputTensorValues(output_size);

        inputTensorValues.assign(preprocessedImage.begin<float>(), preprocessedImage.end<float>());

        inputTensors.push_back(Ort::Value::CreateTensor<float>(memoryInfo, inputTensorValues.data(), input_size,
                                                               input_shape.data(), input_shape.size()));
        outputTensors.push_back(Ort::Value::CreateTensor<float>(memoryInfo, outputTensorValues.data(), output_size,
                                                                output_shape.data(), output_shape.size()));

        // run the inference
        session.Run(Ort::RunOptions{nullptr}, inputNames.data(), inputTensors.data(), input_count, outputNames.data(),
                    outputTensors.data(), output_count);
        std::cout << "done " << i << std::endl;

        assert(outputTensors.size() == session.GetOutputCount() && outputTensors[0].IsTensor());
        std::cout << "output_tensor_shape: " << outputTensors[0].GetTensorTypeAndShapeInfo().GetShape() << std::endl;

        std::transform(outputTensorValues.cbegin(), outputTensorValues.cend(), outputTensorValues.begin(), sigmoid);

        cv::Mat m = cv::Mat(output_shape[2], output_shape[3], CV_32FC1);
        memcpy(m.data, outputTensorValues.data(), outputTensorValues.size() * sizeof(float));

        // cv::imshow("INTERmediare", m);
        // waitforQ();

        bigStuff += m;
    }

    bigStuff /= fn.size();

    // threshold(bigStuff, bigStuff, 0.1, 1, cv::THRESH_BINARY);

    cv::Mat entier;
    bigStuff.convertTo(entier, CV_8UC1, 255);

    cv::Mat canny_output;
    Canny(entier, canny_output, 255 / fn.size() + 10, 255);

    std::vector<std::vector<cv::Point>> contours;
    findContours(canny_output, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    cv::Point2f center;
    float radius;
    cv::minEnclosingCircle(contours[0], center, radius);

    // convert to rgb for diplaying
    cvtColor(entier, entier, cv::COLOR_GRAY2RGB);

    cv::circle(entier, center, radius, cv::Scalar(0, 0, 255));
    cv::circle(entier, center, 0, cv::Scalar(0, 0, 255), -1);

    cv::imwrite(imageOutputPath, entier);

    // cv::imshow("RESULT", bigStuff);
    // waitforQ();

    return 0;
}