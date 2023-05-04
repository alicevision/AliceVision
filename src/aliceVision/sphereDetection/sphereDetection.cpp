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

// Boost
#include <boost/filesystem.hpp>

// Boost JSON
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

// SFMData
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

// namespaces
namespace fs = boost::filesystem;
namespace bpt = boost::property_tree;

namespace aliceVision {
namespace sphereDetection {

void model_explore(Ort::Session& session)
{
    // define allocator
    Ort::AllocatorWithDefaultOptions allocator;

    // print infos of inputs
    size_t input_count = session.GetInputCount();
    for(size_t i = 0; i < input_count; i++)
    {
#if ORT_API_VERSION >= 14
        const Ort::AllocatedStringPtr input_name = session.GetInputNameAllocated(i, allocator);
        ALICEVISION_LOG_DEBUG("Input[" << i << "]: " << input_name.get());
#else
        const char* input_name = session.GetInputName(i, allocator);
        ALICEVISION_LOG_DEBUG("Input[" << i << "]: " << input_name);
#endif

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
#if ORT_API_VERSION >= 14
        const Ort::AllocatedStringPtr output_name = session.GetOutputNameAllocated(i, allocator);
        ALICEVISION_LOG_DEBUG("Output[" << i << "]: " << output_name.get());
#else
        const char* output_name = session.GetOutputName(i, allocator);
        ALICEVISION_LOG_DEBUG("Output[" << i << "]: " << output_name);
#endif

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

prediction predict(Ort::Session& session, const fs::path image_path, const fs::path output_path, const float min_score)
{
    // read image
    image::Image<image::RGBColor> image_alice;
    image::readImage(image_path.string(), image_alice, image::EImageColorSpace::SRGB);

    // Eigen -> OpenCV
    cv::Mat image_opencv;
    cv::eigen2cv(image_alice.GetMat(), image_opencv);
    cv::Size image_opencv_shape = image_opencv.size();

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
    float* bboxes_ptr = output.at(0).GetTensorMutableData<float>();
    float* scores_ptr = output.at(1).GetTensorMutableData<float>();
    float* masks_ptr = output.at(2).GetTensorMutableData<float>();

    // get output shape
    auto infos = output.at(2).GetTensorTypeAndShapeInfo();
    auto shape = infos.GetShape();

    // get scores of detections
    std::vector<float> all_scores = {scores_ptr, scores_ptr + shape[0]};

    // initialize arrays
    std::vector<std::vector<float>> bboxes;
    std::vector<std::string> masks;
    std::vector<float> scores;

    // filter detections and fill arrays
    for(size_t i = 0; i < shape[0]; i++)
    {
        float score = all_scores.at(i);
        if(score > min_score)
        {
            // extract bboxe
            std::vector<float> bboxe(bboxes_ptr + 4 * i, bboxes_ptr + 4 * (i + 1));
            bboxes.push_back(bboxe);

            // extract mask
            float* mask_ptr = masks_ptr + shape[2] * shape[3] * i;
            cv::Mat mask = cv::Mat(shape[2], shape[3], CV_32FC1, mask_ptr);
            std::string filename = fmt::format("{}_mask_{}.png", image_path.stem().string(), i);
            std::string filepath = std::string(output_path.string()).append(filename);
            masks.push_back(filepath);
            cv::imwrite(filepath, mask);

            // extract score
            scores.push_back(score);
        }
    }

    return prediction{bboxes, scores, masks, image_opencv_shape};
}

void sphereDetection(const sfmData::SfMData& sfmData, Ort::Session& session, fs::path output_path,
                     const float min_score)
{
    // main tree
    bpt::ptree fileTree;

    for(auto& viewID : sfmData.getViews())
    {
        ALICEVISION_LOG_DEBUG("View Id: " << viewID);

        std::string sphereName = std::to_string(viewID.second->getViewId());
        const fs::path image_path = fs::path(sfmData.getView(viewID.second->getViewId()).getImagePath());

        if(boost::algorithm::icontains(image_path.stem().string(), "ambiant"))
            continue;

        auto pred = predict(session, image_path, output_path, min_score);

        bpt::ptree spheres_node;

        //for(size_t i = 0; i < pred.scores.size(); i++)
        //{
        //We only take the best sphere in the picture
        int i = 0;
        // compute sphere coords from bboxe coords
        auto bboxe = pred.bboxes.at(i);
        float r = std::min(bboxe.at(3) - bboxe.at(1), bboxe.at(2) - bboxe.at(0)) / 2;
        float x = bboxe.at(0) + r - pred.size.width / 2;
        float y = bboxe.at(1) + r - pred.size.height / 2;

        // create an unnamed node containing the sphere
        bpt::ptree sphere_node;
        sphere_node.put("x", x);
        sphere_node.put("y", y);
        sphere_node.put("r", r);
        sphere_node.put("score", pred.scores.at(i));
        sphere_node.put("mask", pred.masks.at(i));

        // add sphere to array
        spheres_node.push_back(std::make_pair("", sphere_node));
        //}

        // add spheres (array) to view
        //view.add_child("spheres", spheres_node);

        // add view n°i to array of views
        //views.push_back(std::make_pair("", view));

        fileTree.add_child(sphereName, spheres_node);
    }
    bpt::write_json(output_path.append("detection.json").string(), fileTree);
}

void writeManualSphereJSON(const sfmData::SfMData& sfmData, const std::array<float, 3>& sphereParam, fs::path output_path)
{
    // main tree
    bpt::ptree fileTree;

    for(auto& viewID : sfmData.getViews())
    {
        ALICEVISION_LOG_DEBUG("View Id: " << viewID);

        std::string sphereName = std::to_string(viewID.second->getViewId());

        bpt::ptree spheres_node;
        // Create an unnamed node containing the sphere
        bpt::ptree sphere_node;
        sphere_node.put("x", sphereParam[0]);
        sphere_node.put("y", sphereParam[1]);
        sphere_node.put("r", sphereParam[2]);

        // add sphere to array
        spheres_node.push_back(std::make_pair("", sphere_node));

        fileTree.add_child(sphereName, spheres_node);
    }
    bpt::write_json(output_path.append("detection.json").string(), fileTree);
}

}
}
