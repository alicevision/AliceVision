#include <aliceVision/sphereDetection/sphereDetection.hpp>

// Standard libs
#include <iostream>
#include <numeric>

// AliceVision image library
#include <aliceVision/image/all.hpp>
#include <aliceVision/image/io.hpp>
#include <aliceVision/image/Image.hpp>

// AliceVision logger
#include <aliceVision/system/Logger.hpp>

// ONNX Runtime
#include <onnxruntime_cxx_api.h>

// Command line parameters
#include <aliceVision/system/main.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>

// black magic fuckery
#include <aliceVision/imageMasking/eigen2cvHelpers.hpp>

// OpenCv
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Assumptions on model structure
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

std::pair<size_t, size_t> resolution_verify(std::string path_images)
{
    // list all jpg inside the folder
    std::vector<cv::String> files;
    std::string glob_images = path_images.append("/*.jpg");
    cv::glob(glob_images, files, false);

    aliceVision::image::Image<aliceVision::image::RGBfColor> image_float;
    std::pair<size_t, size_t> image_size;
    std::pair<size_t, size_t> tmp_size;

    for(size_t i = 0; i < files.size(); i++)
    {
        // read image
        aliceVision::image::ImageReadOptions read_options;
        read_options.outputColorSpace = aliceVision::image::EImageColorSpace::NO_CONVERSION;
        aliceVision::image::readImage(files[i], image_float, read_options);

        // get the resolution
        image_size.first = image_float.cols();  // width
        image_size.second = image_float.rows(); // height
        ALICEVISION_LOG_DEBUG("image resolution: " << image_size);

        // store the first image_size for comparison
        if(i == 0)
            tmp_size = image_size;

        // check if sizes are the same
        assert(tmp_size == image_size);
    }

    ALICEVISION_LOG_DEBUG("All images are the same resolutions !");

    return tmp_size;
}

cv::Mat compute_mask(Ort::Session& session, const std::string image_path, const float downscale)
{
    // read image
    aliceVision::image::Image<aliceVision::image::RGBColor> image_alice;
    aliceVision::image::readImage(image_path, image_alice, aliceVision::image::EImageColorSpace::SRGB);

    // resize image
    aliceVision::image::downscaleImageInplace(image_alice, downscale);

    aliceVision::image::writeImage("/tmp/img.png", image_alice, aliceVision::image::EImageColorSpace::SRGB);

    // convert to opencv image
    cv::Mat image_opencv;
    cv::eigen2cv(image_alice.GetMat(), image_opencv);

    cv::imwrite("/tmp/img.png", image_opencv);

    // uint8 -> float32
    image_opencv.convertTo(image_opencv, CV_32F, 1.0 / 255.0);

    cv::imwrite("/tmp/img.png", image_opencv);

    // HWC to CHW
    cv::Mat image_blob;
    cv::dnn::blobFromImage(image_opencv, image_blob);

    // inference on cpu TODO: try to make inference on gpu
    Ort::MemoryInfo memory_info =
        Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);

    // create shapes
    std::vector<int64_t> input_shape = {1, 3, image_alice.Height(), image_alice.Width()};
    std::vector<int64_t> output_shape = {1, 1, image_alice.Height(), image_alice.Width()};

    // compute number of pixels inside tensors
    int64_t input_size = std::accumulate(begin(input_shape), end(input_shape), 1, std::multiplies<int64_t>());
    int64_t output_size = input_size / 3;

    // initialize tensors
    std::vector<float> input_tensor(input_size);
    std::vector<float> output_tensor(output_size);

    // insert blob inside input tensor
    input_tensor.assign(image_blob.begin<float>(), image_blob.end<float>());

    // create data payload (given to onnx run function)
    std::vector<Ort::Value> input_data;
    std::vector<Ort::Value> output_data;

    // insert tesnors in data
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

    // select inputs and outputs
    std::vector<const char*> input_names{INPUT_NAME};
    std::vector<const char*> output_names{OUTPUT_NAME};

    // run the inference
    session.Run(Ort::RunOptions{nullptr}, input_names.data(), input_data.data(), INPUT_COUNT, output_names.data(),
                output_data.data(), OUTPUT_COUNT);

    // apply sigmoid activation to model output
    std::transform(output_tensor.cbegin(), output_tensor.cend(), output_tensor.begin(), sigmoid);

    // convert output_tensor to opencv image
    cv::Mat mask = cv::Mat(output_shape[2], output_shape[3], CV_32FC1);
    memcpy(mask.data, output_tensor.data(), output_tensor.size() * sizeof(float));

    return mask;
}

cv::Mat compute_mask_mean(Ort::Session& session, const std::string images_path,
                          const std::pair<size_t, size_t> image_size)
{
    // compute downscaling factor
    int longest_side = std::max(image_size.first, image_size.second);
    float factor = longest_side / MAX_SIZE;
    ALICEVISION_LOG_DEBUG("Downscale factor: " << factor);

    // initialize final mask
    cv::Mat average_mask(floor(image_size.second / factor), floor(image_size.first / factor), CV_32FC1);

    // retreive all jpg paths
    std::vector<cv::String> files;
    cv::glob(images_path + "/*.jpg", files, false);

    for(std::string file : files)
    {
        cv::Mat mask = compute_mask(session, file, factor);
        average_mask += mask;
    }

    average_mask /= files.size();
    return average_mask;
}

std::vector<std::pair<cv::Point2f, float>> compute_circles(cv::Mat mask)
{
    cv::imwrite("/tmp/img.png", mask);

    // [0.0, 1.0] -> [0, 255]
    mask.convertTo(mask, CV_8UC1, 255);

    cv::imwrite("/tmp/img.png", mask);

    // detect edges with canny filter
    cv::Canny(mask, mask, 0, 255);

    cv::imwrite("/tmp/img.png", mask);

    // detect contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<std::pair<cv::Point2f, float>> circles;
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