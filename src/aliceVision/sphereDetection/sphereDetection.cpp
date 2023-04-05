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

#define INPUT_NAME "input"
#define OUTPUT_NAME "output"
#define INPUT_COUNT 1
#define OUTPUT_COUNT 1

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

float sigmoid(float x)
{
    return 1 / (1 + exp(-x));
}

void modelExplore(const Ort::Session session)
{
    // define allocator
    Ort::AllocatorWithDefaultOptions allocator;

    // print name/shape of inputs
    size_t input_count = session.GetInputCount();
    const char* input_name;
    for(size_t i = 0; i < input_count; i++)
    {
        input_name = session.GetInputName(i, allocator);
        ALICEVISION_LOG_DEBUG("Input[" << i << "]: " << input_name);

        Ort::TypeInfo input_info = session.GetInputTypeInfo(i);
        auto input_info2 = input_info.GetTensorTypeAndShapeInfo();

        ONNXTensorElementDataType input_type = input_info2.GetElementType();
        ALICEVISION_LOG_DEBUG("  Type : " << input_type);

        std::vector<int64_t> input_shape = input_info2.GetShape();
        size_t input_size = std::accumulate(begin(input_shape), end(input_shape), 1, std::multiplies<float>());
        ALICEVISION_LOG_DEBUG("  Shape: " << input_shape);
        ALICEVISION_LOG_DEBUG("  Size : " << input_size);
    }

    // print name/shape of outputs
    size_t output_count = session.GetOutputCount();
    const char* output_name;
    for(size_t i = 0; i < output_count; i++)
    {
        output_name = session.GetOutputName(i, allocator);
        ALICEVISION_LOG_DEBUG("Output[" << i << "]: " << output_name);

        Ort::TypeInfo output_info = session.GetOutputTypeInfo(i);
        auto output_info2 = output_info.GetTensorTypeAndShapeInfo();

        ONNXTensorElementDataType output_type = output_info2.GetElementType();
        ALICEVISION_LOG_DEBUG("  Type : " << output_type);

        std::vector<int64_t> output_shape = output_info2.GetShape();
        size_t output_size = std::accumulate(begin(output_shape), end(output_shape), 1, std::multiplies<float>());
        ALICEVISION_LOG_DEBUG("  Shape: " << output_shape);
        ALICEVISION_LOG_DEBUG("  Size : " << output_size);
    }

    // Assume model has 1 input node and 1 output node.
    assert(input_count == 1 && output_count == 1);
}

cv::Size verifySameResolution(std::string imagesPath)
{
    std::vector<cv::String> files;
    std::string globString = imagesPath.append("/*.jpg");
    cv::glob(globString, files, false);

    cv::Mat image;
    cv::Size image_size;
    cv::Size tmp_size;

    for(std::string file : files)
    {
        // read image
        image = cv::imread(file, cv::ImreadModes::IMREAD_COLOR); // TODO: optimize to not open the file ?

        // get the resolution
        image_size = image.size();
        ALICEVISION_LOG_DEBUG("image resolution: " << image_size);

        // initialize width and height if necessary
        if(tmp_size.width == 0 && tmp_size.height == 0)
        {
            tmp_size.width = image_size.width;
            tmp_size.height = image_size.height;
        }

        // check if width and height are the same
        assert(tmp_size == image_size);
    }

    ALICEVISION_LOG_DEBUG("All images are the same resolutions !");

    return tmp_size;
}

cv::Mat computeAverageMask(Ort::Session& session, const std::string imagesPath, const cv::Size imageSize)
{
    int longest_side = std::max(imageSize.width, imageSize.height);
    float factor = 1024.0 / longest_side;
    cv::Size resizedShape(imageSize.width * factor, imageSize.height * factor);

    cv::Mat averagedImage(resizedShape.height, resizedShape.width, CV_32FC1);
    ALICEVISION_LOG_DEBUG("Resized image resolution: " << resizedShape);

    std::vector<cv::String> files;
    std::string globString = imagesPath + "/*.jpg";
    cv::glob(globString, files, false);

    for(size_t i = 0; i < files.size(); i++)
    {
        // read image
        cv::Mat imageBGR = cv::imread(files[i], cv::ImreadModes::IMREAD_COLOR);

        // resize image
        cv::Mat resizedBGR;
        cv::resize(imageBGR, resizedBGR, resizedShape, cv::INTER_LINEAR);

        // BGR -> RGB
        cv::Mat imageRGB;
        cv::cvtColor(resizedBGR, imageRGB, cv::ColorConversionCodes::COLOR_BGR2RGB);

        // uint8 -> float32
        cv::Mat floatImage;
        imageRGB.convertTo(floatImage, CV_32F, 1.0 / 255.0);

        // HWC to CHW
        cv::Mat preprocessedImage;
        cv::dnn::blobFromImage(floatImage, preprocessedImage);

        // creating tensors
        std::vector<Ort::Value> inputTensors;
        std::vector<Ort::Value> outputTensors;

        // inference on cpu (?)
        Ort::MemoryInfo memoryInfo =
            Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);

        // selecting inputs and outputs
        std::vector<const char*> inputNames{INPUT_NAME};
        std::vector<const char*> outputNames{OUTPUT_NAME};

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
        session.Run(Ort::RunOptions{nullptr}, inputNames.data(), inputTensors.data(), INPUT_COUNT, outputNames.data(),
                    outputTensors.data(), OUTPUT_COUNT);

        assert(outputTensors.size() == session.GetOutputCount() && outputTensors[0].IsTensor());
        ALICEVISION_LOG_DEBUG("output_tensor_shape: " << outputTensors[0].GetTensorTypeAndShapeInfo().GetShape());

        std::transform(outputTensorValues.cbegin(), outputTensorValues.cend(), outputTensorValues.begin(), sigmoid);

        cv::Mat m = cv::Mat(output_shape[2], output_shape[3], CV_32FC1);
        memcpy(m.data, outputTensorValues.data(), outputTensorValues.size() * sizeof(float));

        averagedImage += m;
    }

    averagedImage /= files.size();
    return averagedImage;
}

std::vector<std::pair<cv::Point2f, float>> circlesFromMask(const cv::Mat mask)
{
    std::vector<std::pair<cv::Point2f, float>> circles;

    // [0.0, 1.0] -> [0, 255]
    cv::Mat entier;
    mask.convertTo(entier, CV_8UC1, 255);

    // detect edges with canny filter
    cv::Mat canny_output;
    Canny(entier, canny_output, 0, 255);

    // detect contours
    std::vector<std::vector<cv::Point>> contours;
    findContours(canny_output, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for(size_t i = 0; i < contours.size(); i++)
    {
        // detect circle
        std::pair<cv::Point2f, float> circle;
        cv::minEnclosingCircle(contours[i], circle.first, circle.second);

        // add circle to vector
        circles.push_back(circle);
    }

    ALICEVISION_LOG_DEBUG(circles);

    return circles;
}