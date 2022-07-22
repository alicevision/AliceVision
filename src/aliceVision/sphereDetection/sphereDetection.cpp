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

// Helper to convert Eigen Matrix to OpenCV image
#include <aliceVision/imageMasking/eigen2cvHelpers.hpp>

// OpenCv
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Boost JSON
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

// namespaces
namespace bpt = boost::property_tree;

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

void model_explore(Ort::Session& session)
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

    ALICEVISION_LOG_DEBUG("files: " << files);

    cv::Mat image;
    cv::Size image_size;
    cv::Size tmp_size;

    for(size_t i = 0; i < files.size(); i++)
    {
        // read image
        image = cv::imread(files[i], cv::ImreadModes::IMREAD_COLOR); // TODO: use exiv2 ?

        // get the resolution
        image_size = image.size();
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

cv::Mat compute_mask(Ort::Session& session, const std::string image_path, const cv::Size image_size)
{
    // read image
    aliceVision::image::Image<aliceVision::image::RGBColor> image_alice;
    aliceVision::image::readImage(image_path, image_alice, aliceVision::image::EImageColorSpace::SRGB);

    // Eigen -> OpenCV
    cv::Mat image_opencv;
    cv::eigen2cv(image_alice.GetMat(), image_opencv);
    ALICEVISION_LOG_TRACE("converted to opencv matrix");

    // uint8 -> float32
    image_opencv.convertTo(image_opencv, CV_32FC3, 1 / 255.0);
    ALICEVISION_LOG_TRACE("converted to floating image");

    // resize image
    cv::resize(image_opencv, image_opencv, image_size, cv::INTER_LINEAR);
    ALICEVISION_LOG_TRACE("resized the image");

    // HWC to CHW
    cv::dnn::blobFromImage(image_opencv, image_opencv);
    ALICEVISION_LOG_TRACE("converted to CHW format");

    // inference on cpu TODO: use gpu
    Ort::MemoryInfo memory_info =
        Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);

    // create shapes
    std::vector<int64_t> input_shape = {1, 3, image_size.height, image_size.width};
    std::vector<int64_t> output_shape = {1, 1, image_size.height, image_size.width};

    // compute number of pixels inside tensors
    size_t input_size = std::accumulate(begin(input_shape), end(input_shape), 1, std::multiplies<size_t>());
    size_t output_size = std::accumulate(begin(output_shape), end(output_shape), 1, std::multiplies<size_t>());

    // intialize tensors
    std::vector<float> input_tensor(input_size);
    std::vector<float> output_tensor(output_size);

    // modify input_tensor pointer to point at image_opencv
    input_tensor.assign(image_opencv.begin<float>(), image_opencv.end<float>());

    // create "data payloads" (given to onnx run function)
    std::vector<Ort::Value> input_data;
    std::vector<Ort::Value> output_data;

    // insert tensors in data
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

    ALICEVISION_LOG_TRACE("inference ready");

    // run the inference
    session.Run(Ort::RunOptions{nullptr}, input_names.data(), input_data.data(), INPUT_COUNT, output_names.data(),
                output_data.data(), OUTPUT_COUNT);

    ALICEVISION_LOG_TRACE("inference done");

    // apply sigmoid activation to output_tensor
    std::transform(output_tensor.cbegin(), output_tensor.cend(), output_tensor.begin(), sigmoid);

    // convert output_tensor to opencv image
    cv::Mat mask = cv::Mat(output_shape[2], output_shape[3], CV_32FC1);
    memcpy(mask.data, output_tensor.data(), output_tensor.size() * sizeof(float));
    ALICEVISION_LOG_TRACE("converted inference output to opencv image");

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

std::vector<circle_info> compute_circles(const cv::Mat prediction)
{
    std::vector<circle_info> circles;

    // [0.0, 1.0] -> [0, 255]
    cv::Mat mask;
    prediction.convertTo(mask, CV_8UC1, 255);

    // detect edges with canny filter
    Canny(mask, mask, 0, 255);

    // detect contours
    std::vector<std::vector<cv::Point>> contours;
    findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for(size_t i = 0; i < contours.size(); i++)
    {
        // detect circle
        circle_info circle;
        cv::minEnclosingCircle(contours[i], circle.first, circle.second);

        // add circle to vector
        circles.push_back(circle);
    }

    ALICEVISION_LOG_DEBUG(circles);

    return circles;
}

void export_json(std::string output_path, std::vector<circle_info> circles)
{
    bpt::ptree root, spheres;

    for(auto circle : circles)
    {
        bpt::ptree sphere;
        sphere.put("pos_x", circle.first.x);
        sphere.put("pos_y", circle.first.y);
        sphere.put("radius", circle.second);

        spheres.push_back(std::make_pair("", sphere));
    }

    root.add_child("Scene 1", spheres);

    bpt::write_json(output_path, root);

    ALICEVISION_LOG_DEBUG("JSON exported");
}