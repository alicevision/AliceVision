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

// Pre-process input images
#define MAX_SIZE 1024.0

/**
 * @brief Operator overloading for printing vectors
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

/**
 * @brief Sigmoid activation function
 *
 * @param x ∈ ℝ
 * @return sigmoid(x) ∈ [0, 1]
 */
float sigmoid(float x)
{
    return 1 / (1 + exp(-x));
}

/**
 * @brief Prints inputs and outputs of neural network, and checks the requirements.
 *
 * @param session the ONNXRuntime session
 */
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

/**
 * @brief Gets the image paths from the parent folder
 *
 * @param path_images the path to the folder containing all the images
 * @return std::vector<std::string>, list of all image paths
 */
std::vector<std::string> get_images_paths(std::string path_images)
{
    // list all jpg inside the folder
    std::vector<cv::String> files;
    std::string glob_images = path_images.append("/*.jpg");
    cv::glob(glob_images, files, false);

    ALICEVISION_LOG_DEBUG("files: " << files);

    return files;
}

/**
 * @brief Verify all images have the same resolution
 *
 * @param files list of all image paths
 * @return cv::Size, the common resolution
 */
cv::Size resolution_verify(std::vector<std::string> files)
{
    cv::Mat image;
    cv::Size image_size;
    cv::Size tmp_size;

    for(size_t i = 0; i < files.size(); i++)
    {
        // read image
        image = cv::imread(files[i], cv::ImreadModes::IMREAD_COLOR); // TODO: use exiv2 ?

        // get the resolution
        image_size = image.size();

        // store the first image_size for comparison
        if(i == 0)
            tmp_size = image_size;

        // check if sizes are the same
        assert(tmp_size == image_size);
    }

    ALICEVISION_LOG_DEBUG("All images are the same resolution !");

    return tmp_size;
}

/**
 * @brief Use ONNXRuntime to make a prediction
 *
 * @param session
 * @param image_path the path to the input image
 * @param image_size the size the image should be resized to
 * @return cv::Mat, the prediction
 */
cv::Mat predict(Ort::Session& session, const std::string image_path, const cv::Size image_size)
{
    // read image
    aliceVision::image::Image<aliceVision::image::RGBColor> image_alice;
    aliceVision::image::readImage(image_path, image_alice, aliceVision::image::EImageColorSpace::SRGB);

    // TODO: retry resize before cloning to opencv ?

    // Eigen -> OpenCV
    cv::Mat image_opencv;
    cv::eigen2cv(image_alice.GetMat(), image_opencv);

    // uint8 -> float32
    image_opencv.convertTo(image_opencv, CV_32FC3, 1 / 255.0);

    // resize image
    cv::resize(image_opencv, image_opencv, image_size, cv::INTER_LINEAR);

    // HWC to CHW
    cv::dnn::blobFromImage(image_opencv, image_opencv);

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

    // run the inference
    session.Run(Ort::RunOptions{nullptr}, input_names.data(), input_data.data(), INPUT_COUNT, output_names.data(),
                output_data.data(), OUTPUT_COUNT);

    // apply sigmoid activation to output_tensor
    std::transform(output_tensor.cbegin(), output_tensor.cend(), output_tensor.begin(), sigmoid);

    // convert output_tensor to opencv image
    cv::Mat mask = cv::Mat(output_shape[2], output_shape[3], CV_32FC1);
    memcpy(mask.data, output_tensor.data(), output_tensor.size() * sizeof(float));

    return mask;
}

/**
 * @brief Compute the new resolution such that the longest side of the image is 1024px
 *
 * @param original_size the original resolution of the image
 * @return cv::Size, the new resolution
 */
cv::Size resolution_shrink(cv::Size original_size)
{
    int longest_side = std::max(original_size.width, original_size.height);
    float factor = MAX_SIZE / longest_side;

    cv::Size new_size(original_size.width * factor, original_size.height * factor);

    ALICEVISION_LOG_DEBUG("Resize factor: " << factor);
    ALICEVISION_LOG_DEBUG("Resized image resolution: " << new_size);

    return new_size;
}

/**
 * @brief Compute the median mask using a list of masks
 *
 * @param predictions the list of predictions (masks)
 * @param size the size of a mask
 * @return cv::Mat, the median mask
 */
cv::Mat compute_median_mask(std::vector<cv::Mat> predictions, cv::Size size)
{
    cv::Mat median_mask = cv::Mat(size, CV_32FC1);

    for(size_t i = 0; i < size.height; i++)
    {
        for(size_t j = 0; j < size.width; j++)
        {
            // extract same pixel from all predictions
            std::vector<float> values(predictions.size());
            for(size_t k = 0; k < predictions.size(); k++)
            {
                values[k] = predictions[k].at<float>(i, j);
            }

            // get the median pixel value
            std::sort(values.begin(), values.end());
            float value = values[predictions.size() / 2];

            // store value in final matrix
            median_mask.at<float>(i, j) = value;
        }
    }

    return median_mask;
}

/**
 * @brief Compute the prediction mask of where the sphere are
 *
 * @param session the ONNXRuntime session
 * @param files list of all image paths
 * @param image_size the common resolution of every image
 * @return cv::Mat
 */
cv::Mat compute_mask(Ort::Session& session, std::vector<std::string> files, const cv::Size image_size)
{
    // initialize vector containing predictions
    cv::Size resized_size = resolution_shrink(image_size);
    std::vector<cv::Mat> predictions(files.size(), cv::Mat(resized_size, CV_32FC1));

    // use neural net to make predictions
    for(size_t i = 0; i < files.size(); i++)
    {
        predictions[i] = predict(session, files[i], resized_size);
    }

    // return the median mask of the predictions
    return compute_median_mask(predictions, resized_size);
}

/**
 * @brief Extract circles from mask
 *
 * @param prediction the mask to extract circles from
 * @return std::vector<circle_info>, list of circles
 */
std::vector<circle_info> compute_circles(const cv::Mat prediction)
{
    std::vector<circle_info> circles;
    cv::Mat mask;

    // uint8 -> float32
    prediction.convertTo(mask, CV_8UC1, 255);

    // detect edges with canny filter
    Canny(mask, mask, 250, 255);

    // detect contours
    std::vector<std::vector<cv::Point>> contours;
    findContours(mask, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    // fill holes using convex hulls
    std::vector<std::vector<cv::Point>> hulls(contours.size());
    for(size_t i = 0; i < contours.size(); i++)
    {
        convexHull(contours[i], hulls[i]);
        cv::drawContours(mask, hulls, i, cv::Scalar(255), cv::FILLED);
    }

    // redetect contours
    findContours(mask, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    for(size_t i = 0; i < contours.size(); i++)
    {
        // detect circle
        circle_info circle;
        cv::minEnclosingCircle(contours[i], circle.first, circle.second);

        if(circle.second > 5)
        {
            // add circle to vector
            circles.push_back(circle);
        }
    }

    // // distance transform
    // cv::Mat1f dt;
    // cv::distanceTransform(mask, dt, cv::DIST_L2, 5, cv::DIST_LABEL_PIXEL);

    // // Find max value
    // double max_val;
    // cv::Point max_loc;
    // cv::minMaxLoc(dt, nullptr, &max_val, nullptr, &max_loc);

    // // push inscribed circle
    // circles.push_back(std::make_pair(max_loc, max_val));

    ALICEVISION_LOG_DEBUG("circles: " << circles);

    return circles;
}

/**
 * @brief Export a circle list to a JSON file
 *
 * @param output_path the output path to write to
 * @param circles the list of circles to transform in JSON
 */
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

    ALICEVISION_LOG_DEBUG("JSON exported: " << output_path);
}