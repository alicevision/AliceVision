#include <aliceVision/sphereDetection/sphereDetection.hpp>

#include <iostream>
#include <numeric>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// ONNX Runtime
#include <onnxruntime_cxx_api.h>

// Command line parameters
#include <aliceVision/system/main.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>

#define INPUT_NAME "input"
#define OUTPUT_NAME "output"
#define INPUT_COUNT 1
#define OUTPUT_COUNT 1

#define MAX_SIZE 1024.0

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

void model_explore(const Ort::Session session)
{
    // define allocator
    Ort::AllocatorWithDefaultOptions allocator;

    // print infos of inputs
    size_t input_count = session.GetInputCount();
    for(size_t i = 0; i < input_count; i++)
    {
        const char* input_name = session.GetInputName(i, allocator);
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

    // print infos of outputs
    size_t output_count = session.GetOutputCount();
    for(size_t i = 0; i < output_count; i++)
    {
        const char* output_name = session.GetOutputName(i, allocator);
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
    assert(input_count == INPUT_COUNT && output_count == OUTPUT_COUNT);
}

cv::Size resolution_verify(std::string path_images)
{
    // list all jpg inside the folder
    std::vector<cv::String> files;
    std::string glob_images = path_images.append("/*.jpg");
    cv::glob(glob_images, files, false);

    cv::Mat image;
    cv::Size image_size;
    cv::Size tmp_size;

    for(std::string file : files)
    {
        // read image
        image = cv::imread(file, cv::ImreadModes::IMREAD_COLOR);

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

cv::Mat compute_mask(Ort::Session& session, const std::string image_path, const cv::Size image_size)
{
    // read image
    cv::Mat image_BGR = cv::imread(image_path, cv::ImreadModes::IMREAD_COLOR);

    // resize image
    cv::Mat resized_BGR;
    cv::resize(image_BGR, resized_BGR, image_size, cv::INTER_LINEAR);

    // BGR -> RGB
    cv::Mat image_RGB;
    cv::cvtColor(resized_BGR, image_RGB, cv::ColorConversionCodes::COLOR_BGR2RGB);

    // uint8 -> float32
    cv::Mat image_float;
    image_RGB.convertTo(image_float, CV_32F, 1.0 / 255.0);

    // HWC to CHW
    cv::Mat image_blob;
    cv::dnn::blobFromImage(image_float, image_blob);

    // creating tensors
    std::vector<Ort::Value> input_data;
    std::vector<Ort::Value> output_data;

    // inference on cpu (?), TODO: try to make inference on gpu
    Ort::MemoryInfo memory_info =
        Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);

    // selecting inputs and outputs
    std::vector<const char*> input_names{INPUT_NAME};
    std::vector<const char*> output_names{OUTPUT_NAME};

    std::vector<int64_t> input_shape = {1, 3, image_size.height, image_size.width};
    std::vector<int64_t> output_shape = {1, 1, image_size.height, image_size.width};

    size_t input_size = std::accumulate(begin(input_shape), end(input_shape), 1, std::multiplies<float>());
    size_t output_size = std::accumulate(begin(output_shape), end(output_shape), 1, std::multiplies<float>());

    std::vector<float> input_tensor(input_size);
    std::vector<float> output_tensor(output_size);

    input_tensor.assign(image_blob.begin<float>(), image_blob.end<float>());

    input_data.push_back(                                                                            //
        Ort::Value::CreateTensor<float>(                                                             //
            memory_info, input_tensor.data(), input_size, input_shape.data(), input_shape.size()     //
            )                                                                                        //
    );                                                                                               //
    output_data.push_back(                                                                           //
        Ort::Value::CreateTensor<float>(                                                             //
            memory_info, output_tensor.data(), output_size, output_shape.data(), output_shape.size() //
            )                                                                                        //
    );                                                                                               //

    // run the inference
    session.Run(Ort::RunOptions{nullptr}, input_names.data(), input_data.data(), INPUT_COUNT, output_names.data(),
                output_data.data(), OUTPUT_COUNT);

    assert(output_data.size() == session.GetOutputCount() && output_data[0].IsTensor());
    ALICEVISION_LOG_DEBUG("output_tensor_shape: " << output_data[0].GetTensorTypeAndShapeInfo().GetShape());

    std::transform(output_tensor.cbegin(), output_tensor.cend(), output_tensor.begin(), sigmoid);

    cv::Mat mask = cv::Mat(output_shape[2], output_shape[3], CV_32FC1);
    memcpy(mask.data, output_tensor.data(), output_tensor.size() * sizeof(float));

    return mask;
}

cv::Size resolution_shrink(cv::Size size_original)
{
    int longest_side = std::max(size_original.width, size_original.height);
    float factor = MAX_SIZE / longest_side;
    cv::Size size_shrank(size_original.width * factor, size_original.height * factor);
    ALICEVISION_LOG_DEBUG("Resized image resolution: " << size_shrank);

    return size_shrank;
}

cv::Mat compute_mask_mean(Ort::Session& session, const std::string images_path, const cv::Size image_size)
{
    cv::Size resized_size = resolution_shrink(image_size);
    cv::Mat average_mask(resized_size.height, resized_size.width, CV_32FC1);

    std::vector<cv::String> files;
    std::string images_glob = images_path + "/*.jpg";
    cv::glob(images_glob, files, false);

    for(std::string file : files)
    {
        cv::Mat mask = compute_mask(session, file, resized_size);
        average_mask += mask;
    }

    average_mask /= files.size();
    return average_mask;
}

std::vector<std::pair<cv::Point2f, float>> compute_circles(const cv::Mat mask)
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