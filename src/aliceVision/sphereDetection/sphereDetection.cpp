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
 * @brief Use ONNXRuntime to make a prediction
 *
 * @param session
 * @param image_path the path to the input image
 * @return cv::Mat, the prediction
 */
prediction predict(Ort::Session& session, const fs::path image_path, const fs::path output_path)
{
    // read image
    aliceVision::image::Image<aliceVision::image::RGBColor> image_alice;
    aliceVision::image::readImage(image_path.string(), image_alice, aliceVision::image::EImageColorSpace::SRGB);

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
    float* bboxes_ptr = output.at(0).GetTensorMutableData<float>();
    float* scores_ptr = output.at(1).GetTensorMutableData<float>();
    float* masks_ptr = output.at(2).GetTensorMutableData<float>();

    // get bboxes
    auto infos = output.at(0).GetTensorTypeAndShapeInfo();
    auto shape = infos.GetShape();
    std::vector<std::vector<float>> bboxes;
    for(size_t i = 0; i < shape[0]; i++)
    {
        std::vector<float> bboxe(bboxes_ptr + 4 * i, bboxes_ptr + 4 * (i + 1));
        bboxes.push_back(bboxe);
    }

    // get scores
    infos = output.at(1).GetTensorTypeAndShapeInfo();
    shape = infos.GetShape();
    std::vector<float> scores = {scores_ptr, scores_ptr + shape[0]};

    // get masks
    infos = output.at(2).GetTensorTypeAndShapeInfo();
    shape = infos.GetShape();
    std::vector<std::string> masks;
    for(size_t i = 0; i < shape[0]; i++)
    {
        auto mask_ptr = masks_ptr + shape[2] * shape[3] * i;
        auto mask = cv::Mat(shape[2], shape[3], CV_32FC1, mask_ptr);
        auto filename = fmt::format("{}_mask_{}.png", image_path.stem().string(), i);
        auto filepath = std::string(output_path.string()).append(filename);
        masks.push_back(filepath);
        cv::imwrite(filepath, mask);
    }

    return prediction{bboxes, scores, masks};
}

bpt::ptree build_view_tree(prediction pred, cvflann::lsh::FeatureIndex viewID)
{
    bpt::ptree view;

    // adding viewID
    view.put("id", viewID);

    // adding bboxes
    bpt::ptree bboxes_node;
    for(auto bboxe : pred.bboxes)
    {
        // Create an unnamed node containing the bboxe
        bpt::ptree bboxe_node;
        for(auto coord : bboxe)
        {
            // Create an unnamed node containing the coordinate
            bpt::ptree coord_node;
            coord_node.put("", coord);

            // Add coordinate to array
            bboxe_node.push_back(std::make_pair("", coord_node));
        }

        // Add bboxe to array
        bboxes_node.push_back(std::make_pair("", bboxe_node));
    }
    view.add_child("bboxes", bboxes_node);

    // adding scores
    bpt::ptree scores_node;
    for(auto score : pred.scores)
    {
        // Create an unnamed node containing the score
        bpt::ptree score_node;
        score_node.put("", score);

        // Add score to the array
        scores_node.push_back(std::make_pair("", score_node));
    }
    view.add_child("scores", scores_node);

    // adding masks
    bpt::ptree masks_node;
    for(auto mask : pred.masks)
    {
        // Create an unnamed node containing the mask path
        bpt::ptree mask_node;
        mask_node.put("", mask);

        // Add path to the array
        masks_node.push_back(std::make_pair("", mask_node));
    }
    view.add_child("masks", masks_node);

    return view;
}

void sphereDetection(const aliceVision::sfmData::SfMData& sfmData, Ort::Session& session, fs::path output_path)
{
    // json init
    bpt::ptree root, poses;

    // split all views by pose
    std::map<aliceVision::IndexT, std::vector<aliceVision::IndexT>> viewPerPoseID;
    for(auto& viewIt : sfmData.getViews())
    {
        viewPerPoseID[viewIt.second->getPoseId()].push_back(viewIt.second->getViewId());
    }

    for(auto& poseID : viewPerPoseID)
    {
        ALICEVISION_LOG_DEBUG("Pose Id: " << poseID.first);
        bpt::ptree pose, views;

        std::vector<aliceVision::IndexT>& viewIDs = poseID.second;
        for(auto& viewID : viewIDs)
        {
            ALICEVISION_LOG_DEBUG("View Id: " << viewID);

            const fs::path image_path = fs::path(sfmData.getView(viewID).getImagePath());
            auto pred = predict(session, image_path, output_path);

            auto view = build_view_tree(pred, viewID);
            views.push_back(std::make_pair("", view));
        }

        pose.put("id", poseID.first);
        pose.add_child("views", views);

        poses.push_back(std::make_pair("", pose));
    }

    root.add_child("poses", poses);
    bpt::write_json(output_path.append("detection.json").string(), root);
}
