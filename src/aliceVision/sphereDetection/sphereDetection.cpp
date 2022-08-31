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
prediction predict(Ort::Session& session, const std::string image_path)
{
    // read image
    aliceVision::image::Image<aliceVision::image::RGBColor> image_alice;
    aliceVision::image::readImage(image_path, image_alice, aliceVision::image::EImageColorSpace::SRGB);

    // Eigen -> OpenCV
    cv::Mat image_opencv;
    cv::eigen2cv(image_alice.GetMat(), image_opencv);

    // uint8 -> float32
    image_opencv.convertTo(image_opencv, CV_32FC3, 1 / 255.0);

    // HWC to CHW
    cv::dnn::blobFromImage(image_opencv, image_opencv);

    // inference on cpu TODO: use gpu
    Ort::MemoryInfo memory_info =
        Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);

    // intialize input tensor
    std::vector<int64_t> input_shape = {1, 3, image_alice.Height(), image_alice.Width()};
    size_t input_size = std::accumulate(begin(input_shape), end(input_shape), 1, std::multiplies<size_t>());
    std::vector<float> input_tensor(input_size);
    input_tensor.assign(image_opencv.begin<float>(), image_opencv.end<float>());

    // create input data
    std::vector<Ort::Value> input_data;
    input_data.push_back(                                                                        //
        Ort::Value::CreateTensor<float>(                                                         //
            memory_info, input_tensor.data(), input_size, input_shape.data(), input_shape.size() //
            )                                                                                    //
    );                                                                                           //

    // select inputs and outputs
    std::vector<const char*> input_names{"input"};
    std::vector<const char*> output_names{"boxes", "scores", "masks"};

    // run the inference
    auto output = session.Run(Ort::RunOptions{nullptr}, input_names.data(), input_data.data(), input_names.size(),
                              output_names.data(), output_names.size());

    // get pointers to outputs
    float* boxes_ptr = output.at(0).GetTensorMutableData<float>();
    float* scores_ptr = output.at(1).GetTensorMutableData<float>();
    float* masks_ptr = output.at(2).GetTensorMutableData<float>();

    // get scores
    auto infos = output.at(1).GetTensorTypeAndShapeInfo();
    auto len = infos.GetElementCount();
    std::vector<float> scores = {scores_ptr, scores_ptr + len};

    // get bboxes
    infos = output.at(0).GetTensorTypeAndShapeInfo();
    auto shape = infos.GetShape();
    cv::Mat bboxes = cv::Mat(shape[0], shape[1], CV_32F, boxes_ptr);

    // get masks
    infos = output.at(2).GetTensorTypeAndShapeInfo();
    shape = infos.GetShape();
    std::vector<cv::Mat> masks;
    for(size_t i = 0; i < shape[0]; i++)
    {
        auto mask_ptr = masks_ptr + shape[2] * shape[3] * i;
        auto mask = cv::Mat(shape[2], shape[3], CV_32FC1, mask_ptr);
        masks.push_back(mask);
    }

    return prediction{bboxes, scores, masks};
}

// /**
//  * @brief Export a circle list to a JSON file
//  *
//  * @param output_path the output path to write to
//  * @param circles the list of circles to transform in JSON
//  */
// void export_json(std::string output_path, std::vector<circle_info> circles)
// {
//     bpt::ptree root, spheres;

//     for(auto circle : circles)
//     {
//         bpt::ptree sphere;
//         sphere.put("pos_x", circle.first.x);
//         sphere.put("pos_y", circle.first.y);
//         sphere.put("radius", circle.second);

//         spheres.push_back(std::make_pair("", sphere));
//     }

//     root.add_child("Scene 1", spheres);

//     bpt::write_json(output_path, root);

//     ALICEVISION_LOG_DEBUG("JSON exported: " << output_path);
// }