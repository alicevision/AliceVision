// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/image/all.hpp>
#include <aliceVision/keyframe/KeyframeSelector.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/main.hpp>

#include <boost/program_options.hpp> 
#include <boost/filesystem.hpp>

#include <string>
#include <vector>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 3
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

const std::string supportedExtensions = "exr, jpg, png";

int aliceVision_main(int argc, char** argv)
{
    // Command-line parameters
    std::vector<std::string> mediaPaths;    // media file path list
    std::vector<std::string> brands;        // media brand list
    std::vector<std::string> models;        // media model list
    std::vector<float> mmFocals;            // media focal (mm) list
    std::string sensorDbPath;               // camera sensor width database
    std::string outputFolder;               // output folder for keyframes

    // Algorithm variables
    bool useSmartSelection = true;          // enable the smart selection instead of the regular one
    unsigned int minFrameStep = 12;         // minimum number of frames between two keyframes (regular selection)
    unsigned int maxFrameStep = 36;         // maximum number of frames between two keyframes (regular selection)
    unsigned int minNbOutFrames = 10;       // minimum number of selected keyframes (smart selection)
    unsigned int maxNbOutFrames = 2000;     // maximum number of selected keyframes (both selections)
    float pxDisplacement = 3.0;             // percentage of pixels that have moved across frames since last keyframe (smart selection)
    std::size_t rescaledWidthSharp = 720;   // width of the rescaled frames for the sharpness; 0 if no rescale is performed (smart selection)
    std::size_t rescaledWidthFlow = 720;    // width of the rescaled frames for the flow; 0 if no rescale is performed (smart selection)
    std::size_t sharpnessWindowSize = 200;  // sliding window's size in sharpness computation (smart selection)
    std::size_t flowCellSize = 90;          // size of the cells within a frame used to compute the optical flow (smart selection)
    std::string outputExtension = "exr";    // file extension of the written keyframes
    image::EStorageDataType exrDataType =   // storage data type for EXR output files
        image::EStorageDataType::Float;
    bool renameKeyframes = false;           // name selected keyframes as consecutive frames instead of using their index as a name

    // Debug options
    bool exportScores = false;              // export the sharpness and optical flow scores to a CSV file
    std::string csvFilename = "scores.csv"; // name of the CSV file containing the scores
    bool exportSelectedFrames = false;      // export the selected frames (1 for selected, 0 for not selected)
    bool skipSelection = false;             // only compute the scores and do not proceed with the selection
    bool exportFlowVisualisation = false;   // export optical flow visualisation for all the frames
    bool flowVisualisationOnly = false;     // export optical flow visualisation for all the frames but do not compute scores
    bool skipSharpnessComputation = false;  // skip sharpness score computations

    po::options_description inputParams("Required parameters");
    inputParams.add_options()
        ("mediaPaths", po::value<std::vector<std::string>>(&mediaPaths)->required()->multitoken(),
            "Input video files or image sequence directories.")
        ("sensorDbPath", po::value<std::string>(&sensorDbPath)->required(),
            "Camera sensor width database path.")
        ("outputFolder", po::value<std::string>(&outputFolder)->required(),
            "Output folder in which the selected keyframes are written.");

    po::options_description metadataParams("Metadata parameters");
    metadataParams.add_options()
        ("brands", po::value<std::vector<std::string>>(&brands)->default_value(brands)->multitoken(),
            "Camera brands.")
        ("models", po::value<std::vector<std::string>>(&models)->default_value(models)->multitoken(),
            "Camera models.")
        ("mmFocals", po::value<std::vector<float>>(&mmFocals)->default_value(mmFocals)->multitoken(),
            "Focals in mm (ignored if equal to 0).");

    po::options_description algorithmParams("Algorithm parameters");  // Parameters common to both methods
    algorithmParams.add_options()
        ("maxNbOutFrames", po::value<unsigned int>(&maxNbOutFrames)->default_value(maxNbOutFrames),
            "Maximum number of output keyframes.\n"
            "\t- For the regular method, 0 = no limit. 'minFrameStep' and 'maxFrameStep' will always be respected, "
            "so combining them with this parameter might cause the selection to stop before reaching the end of the "
            "input sequence(s).\n"
            "\t- For the smart method, the default value is set to 2000.")
        ("renameKeyframes", po::value<bool>(&renameKeyframes)->default_value(renameKeyframes),
            "Instead of naming the keyframes according to their index in the input sequence / video, rename them as "
            "consecutive frames, starting from 0.\n"
            "If the selected keyframes should have originally be written as [00015.exr, 00294.exr, 00825.exr], they "
            "will instead be written as [00000.exr, 00001.exr, 00002.exr] if this option is enabled.")
        ("outputExtension", po::value<std::string>(&outputExtension)->default_value(outputExtension),
            "File extension of the output keyframes.")
        ("storageDataType", po::value<image::EStorageDataType>(&exrDataType)->default_value(exrDataType),
            ("Storage data type for EXR output files: " + image::EStorageDataType_informations()).c_str());

    po::options_description regularAlgorithmParams("Regular algorithm parameters");
    regularAlgorithmParams.add_options()
        ("minFrameStep", po::value<unsigned int>(&minFrameStep)->default_value(minFrameStep),
            "Minimum number of frames between two keyframes.")
        ("maxFrameStep", po::value<unsigned int>(&maxFrameStep)->default_value(maxFrameStep),
            "Maximum number of frames after which a keyframe can be taken (ignored if equal to 0).");

    po::options_description smartAlgorithmParams("Smart algorithm parameters");
    smartAlgorithmParams.add_options()
        ("useSmartSelection", po::value<bool>(&useSmartSelection)->default_value(useSmartSelection),
            "True to use the smart keyframe selection method, false to use the regular keyframe selection method.")
        ("minNbOutFrames", po::value<unsigned int>(&minNbOutFrames)->default_value(minNbOutFrames),
            "Minimum number of output keyframes.")
        ("pxDisplacement", po::value<float>(&pxDisplacement)->default_value(pxDisplacement),
            "Percentage of pixels in the image that have been displaced since the last selected frame. The absolute "
            "number of moving pixels is determined using min(imageWidth, imageHeight).")
        ("rescaledWidthSharpness", po::value<std::size_t>(&rescaledWidthSharp)->default_value(rescaledWidthSharp),
            "Width, in pixels, of the rescaled input frames used to compute the sharpness scores. The height of the "
            "rescaled frames will be automatically determined to preserve the aspect ratio. 0 = no rescale.")
        ("rescaledWidthFlow", po::value<std::size_t>(&rescaledWidthFlow)->default_value(rescaledWidthFlow),
            "Width, in pixels, of the rescaled input frames used to compute the motion scores. The height of the "
            "rescaled frames will be automatically determined to preserve the aspect ratio. 0 = no rescale.")
        ("sharpnessWindowSize", po::value<std::size_t>(&sharpnessWindowSize)->default_value(sharpnessWindowSize),
            "Size, in pixels, of the sliding window that is used to compute the sharpness score of a frame.")
        ("flowCellSize", po::value<std::size_t>(&flowCellSize)->default_value(flowCellSize),
            "Size, in pixels, of the cells within an input frame that are used to compute the optical flow scores.");

    po::options_description debugParams("Debug parameters");
    debugParams.add_options()
        ("exportScores", po::value<bool>(&exportScores)->default_value(exportScores),
            "Export the sharpness and optical flow scores to a CSV file.")
        ("csvFilename", po::value<std::string>(&csvFilename)->default_value(csvFilename),
            "Name of the CSV file containing the sharpness and optical flow scores.")
        ("exportSelectedFrames", po::value<bool>(&exportSelectedFrames)->default_value(exportSelectedFrames),
            "Add a column in the exported CSV file containing the selected frames (1 for frames that have been "
            "selected, 0 otherwise).")
        ("skipSelection", po::value<bool>(&skipSelection)->default_value(skipSelection),
            "Only compute the sharpness and optical flow scores, but do not proceed with the selection.")
        ("exportFlowVisualisation", po::value<bool>(&exportFlowVisualisation)->default_value(exportFlowVisualisation),
            "For all frames, export the optical flow visualisation in HSV as PNG images.")
        ("flowVisualisationOnly", po::value<bool>(&flowVisualisationOnly)->default_value(flowVisualisationOnly),
            "Export the optical flow visualisation in HSV as PNG images for all frames but do not compute scores.")
        ("skipSharpnessComputation", po::value<bool>(&skipSharpnessComputation)->default_value(skipSharpnessComputation),
            "Skip the computations for the sharpness score of each frame. A fixed sharpness score of 1.0 will be "
            "assigned to each frame.");

    aliceVision::CmdLine cmdline("This program is used to extract keyframes from single camera or a camera rig.\n"
                                "AliceVision keyframeSelection");
    cmdline.add(inputParams);
    cmdline.add(metadataParams);
    cmdline.add(algorithmParams);
    cmdline.add(regularAlgorithmParams);
    cmdline.add(smartAlgorithmParams);
    cmdline.add(debugParams);
    if (!cmdline.execute(argc, argv)) {
        return EXIT_FAILURE;
    }

    const std::size_t nbCameras = mediaPaths.size();

    // Check output folder and update to its absolute path
    {
        const fs::path outDir = fs::absolute(outputFolder);
        outputFolder = outDir.string();
        if (!fs::is_directory(outDir)) {
            ALICEVISION_LOG_ERROR("Cannot find folder: " << outputFolder);
            return EXIT_FAILURE;
        }
    }

    if (nbCameras < 1) {
        ALICEVISION_LOG_ERROR("Program needs at least one media path.");
        return EXIT_FAILURE;
    }

    if (maxFrameStep > 0 && minFrameStep >= maxFrameStep) {
        ALICEVISION_LOG_ERROR("Setting 'minFrameStep' should be less than setting 'maxFrameStep'.");
        return EXIT_FAILURE;
    }

    if (minNbOutFrames < 1) {
        ALICEVISION_LOG_ERROR("The minimum number of output keyframes cannot be less than 1.");
        return EXIT_FAILURE;
    }

    if (supportedExtensions.find(outputExtension) == std::string::npos) {
        ALICEVISION_LOG_ERROR("Unsupported extension for the output file. Supported extensions are: "
                              << supportedExtensions);
        return EXIT_FAILURE;
    }

    brands.resize(nbCameras);
    models.resize(nbCameras);
    mmFocals.resize(nbCameras);

    // Debugging prints, print out all the parameters
    {
        if (nbCameras == 1)
            ALICEVISION_LOG_INFO("Single camera");
        else
            ALICEVISION_LOG_INFO("Camera rig of " << nbCameras << " cameras.");

        for (std::size_t i = 0; i < nbCameras; ++i) {
            ALICEVISION_LOG_INFO("Camera: "              << mediaPaths.at(i)   << std::endl
                                << "\t - brand: "        << brands.at(i)       << std::endl
                                << "\t - model: "        << models.at(i)       << std::endl
                                << "\t - focal (mm): "   << mmFocals.at(i)     << std::endl);
        }
    }

    // Initialize KeyframeSelector
    keyframe::KeyframeSelector selector(mediaPaths, sensorDbPath, outputFolder);

    // Set frame-related algorithm parameters
    selector.setMinFrameStep(minFrameStep);
    selector.setMaxFrameStep(maxFrameStep);
    selector.setMinOutFrames(minNbOutFrames);
    selector.setMaxOutFrames(maxNbOutFrames);

    if (flowVisualisationOnly) {
        bool exported = selector.exportFlowVisualisation(rescaledWidthFlow);
        if (exported)
            return EXIT_SUCCESS;
        else
            return EXIT_FAILURE;
    }

    if (skipSelection) {
        selector.computeScores(rescaledWidthSharp, rescaledWidthFlow, sharpnessWindowSize, flowCellSize,
                               skipSharpnessComputation);
        if (exportScores)
            selector.exportScoresToFile(csvFilename);  // Frames have not been selected, ignore 'exportSelectedFrames'
        if (exportFlowVisualisation)
            selector.exportFlowVisualisation(rescaledWidthFlow);

        return EXIT_SUCCESS;
    }

    // Process media paths with regular or smart method
    if (useSmartSelection)
        selector.processSmart(pxDisplacement, rescaledWidthSharp, rescaledWidthFlow, sharpnessWindowSize, flowCellSize,
                              skipSharpnessComputation);
    else
        selector.processRegular();

    // Write selected keyframes
    selector.writeSelection(brands, models, mmFocals, renameKeyframes, outputExtension, exrDataType);

    // If debug options are set, export the scores as a CSV file and / or the motion vectors as images
    if (exportScores)
        selector.exportScoresToFile(csvFilename, exportSelectedFrames);
    if (exportFlowVisualisation)
        selector.exportFlowVisualisation(rescaledWidthFlow);

    return EXIT_SUCCESS;
}
