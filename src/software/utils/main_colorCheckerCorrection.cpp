// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/image/all.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/sfmDataIO/viewIO.hpp>
#include <aliceVision/utils/regexFilter.hpp>
#include <aliceVision/utils/filesIO.hpp>

#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/config.hpp>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/mcc.hpp>

#include <string>
#include <fstream>
#include <vector>
#include <unordered_map>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace fs = boost::filesystem;
namespace po = boost::program_options;

cv::Mat processColorCorrection(cv::Mat& image, cv::Mat& refColors) {
    cv::ccm::ColorCorrectionModel model(refColors, cv::ccm::COLORCHECKER_Macbeth);
    model.run();

    // set color space
    model.setColorSpace(cv::ccm::COLOR_SPACE_sRGB);

    cv::Mat img;
    cvtColor(image, img, cv::COLOR_BGR2RGB);
    img.convertTo(img, CV_64F); // convert to 64 bits double matrix / image
    const int inpSize = 255;
    const int outSize = 255;
    img /= inpSize;
    cv::Mat calibratedImage = model.infer(img); // make correction using ccm matrix
    cv::Mat out = calibratedImage * outSize;

    out.convertTo(out, CV_8UC3); // convert to 8 bits unsigned integer matrix / image with 3 channels
    cv::Mat imgOut = min(max(out, 0), outSize);
    cv::Mat outImg;
    cvtColor(imgOut, outImg, cv::COLOR_RGB2BGR);

    // try returning imgOut ?
    return outImg;
}

int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
    std::string inputExpression;
    std::string colorData;
    std::string outputPath;

    po::options_description allParams(
        "This program is used to perform color correction based on a color checker\n"
        "AliceVision colorCheckerCorrection");

    po::options_description inputParams("Required parameters");
    inputParams.add_options()
        ("input,i", po::value<std::string>(&inputExpression)->default_value(inputExpression),
        "SfMData file input, image filenames or regex(es) on the image file path (supported regex: '#' matches a "
        "single digit, '@' one or more digits, '?' one character and '*' zero or more).")(
        "inputColorData", po::value<std::string>(&colorData)->default_value(colorData),
        "Colorimetric data extracted from a detected color checker in the images")
        ("output,o", po::value<std::string>(&outputPath)->required(),
         "Output folder.")
        ;

    po::options_description logParams("Log parameters");
    logParams.add_options()
        ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
         "verbosity level (fatal, error, warning, info, debug, trace).")
        ;

    allParams.add(inputParams).add(logParams);

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

    // set verbose level
    system::Logger::get()->setLogLevel(verboseLevel);

    // check user choose an input
    if(inputExpression.empty())
    {
        ALICEVISION_LOG_ERROR("Program need --input option." << std::endl << "No input images here.");

        return EXIT_FAILURE;
    }

    // Map used to store paths of the views that need to be processed
    std::unordered_map<IndexT, std::string> ViewPaths;

    // Check if sfmInputDataFilename exist and is recognized as sfm data file
    const std::string inputExt = boost::to_lower_copy(fs::path(inputExpression).extension().string());
    ALICEVISION_COUT(inputExt);
    static const std::array<std::string, 3> sfmSupportedExtensions = {".sfm", ".json", ".abc"};
    if(!inputExpression.empty() && std::find(sfmSupportedExtensions.begin(), sfmSupportedExtensions.end(), inputExt) !=
                                       sfmSupportedExtensions.end())
    {
        sfmData::SfMData sfmData;
        if(!sfmDataIO::Load(sfmData, inputExpression, sfmDataIO::VIEWS))
        {
            ALICEVISION_LOG_ERROR("The input SfMData file '" << inputExpression << "' cannot be read.");
            return EXIT_FAILURE;
        }

        // Map used to store paths of the views that need to be processed
        std::unordered_map<IndexT, std::string> ViewPaths;

        // Iterate over all views
        for(const auto& viewIt : sfmData.getViews())
        {
            const sfmData::View& view = *(viewIt.second);

            ViewPaths.insert({view.getViewId(), view.getImagePath()});
        }

        const int size = ViewPaths.size();
        int i = 0;

        for(auto& viewIt : ViewPaths)
        {
            const IndexT viewId = viewIt.first;
            const std::string viewPath = viewIt.second;
            sfmData::View& view = sfmData.getView(viewId);

            ALICEVISION_LOG_INFO(++i << "/" << size << " - Process view '" << viewId << "' for color correction.");

            // Create an image with 3 channel BGR color
            cv::Mat image = cv::imread(viewPath, cv::IMREAD_COLOR);

            // Check if the image is empty 
            if(!image.data)
            {
                ALICEVISION_LOG_ERROR("Image with id '" << viewId << "'.\n" << "is empty.");
            
                return EXIT_FAILURE;
            }

            // Image color correction processing
            // TODO : get refColors mat in input and cv::Mat calibratedImage = processColorCorrection(image, ...);
            cv::Mat calibratedImage = image;

            // Save the image
            // TODO
            // Example: saveImage(...)
            cv::imwrite(outputPath + "/" + std::to_string(viewId) + ".calibrated.jpg", calibratedImage);

            // Update view for this modification
            // TODO: view.setImagePath(outputFilePath);
            view.setWidth(calibratedImage.size().width);
            view.setHeight(calibratedImage.size().height);
        }

        // Save sfmData with modified path to images
        const std::string sfmfilePath = (fs::path(outputPath) / fs::path(inputExpression).filename()).generic_string();
        if(!sfmDataIO::Save(sfmData, sfmfilePath, sfmDataIO::ESfMData(sfmDataIO::ALL)))
        {
            ALICEVISION_LOG_ERROR("The output SfMData file '" << sfmfilePath << "' cannot be written.");

            return EXIT_FAILURE;
        }
    }
    else
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << inputExpression << "' does not exist or is not recognized as a sfm data file.");

        return EXIT_FAILURE;
    }

    // const int size = ViewPaths.size();
    // int i = 0;
    // int nc = 1; // Number of charts in an image

    // for(auto& viewIt : ViewPaths)
    // {
    //     const IndexT viewId = viewIt.first;
    //     const std::string viewPath = viewIt.second;
    //     sfmData::View& view = sfmData.getView(viewId);

    //     // Create an image with 3 channel BGR color 
    //     cv::Mat image = cv::imread(viewPath, 1);

    //     if(image.cols == 0 || image.rows == 0)
    //     {
    //         ALICEVISION_LOG_ERROR("Image with id '" << viewId << "'.\n"
    //                               << "is empty.");
    //         return EXIT_FAILURE;
    //     }

    //     cv::Ptr<cv::mcc::CCheckerDetector> detector = cv::mcc::CCheckerDetector::create();

    //     ALICEVISION_LOG_INFO(++i << "/" << size << " - Process view '" << viewId << "'.");

    //     if(!detector->process(image, cv::mcc::TYPECHART(0), nc))
    //     {
    //         ALICEVISION_LOG_INFO("Checker not detected in image with id '" << viewId << "'");
    //     }
    //     else
    //     {
    //         ALICEVISION_LOG_INFO("Checker successfully detected in image with id '" << viewId << "'");
    //         // get checker
    //         std::vector<cv::Ptr<cv::mcc::CChecker>> checkers = detector->getListColorChecker();

    //         for(cv::Ptr<cv::mcc::CChecker> checker : checkers)
    //         {
    //             // current checker
    //             if(debug)
    //             {
    //                 drawCCheckerSVG(checker, outputFolder);

    //                 cv::Ptr<cv::mcc::CCheckerDraw> cdraw = cv::mcc::CCheckerDraw::create(checker, CV_RGB(250, 0, 0), 3);
    //                 cdraw->draw(image);

    //                 // save the image with the color checker drawn
    //                 cv::imwrite(outputFolder + "/" + std::to_string(viewId) + ".jpg", image);
    //             }

    //             // preparation for color calibration
    //             cv::Mat chartsRGB = checker->getChartsRGB();
    //             cv::Mat src = chartsRGB.col(1).clone().reshape(3, chartsRGB.rows / 3);
    //             src /= 255.0;

    //             // Macbeth color checker as parameter on the model to get the best effect of color correction in our case.
    //             cv::ccm::ColorCorrectionModel model(src, cv::ccm::COLORCHECKER_Macbeth);
    //             model.run();
    //             cv::Mat ccm = model.getCCM();
    //             double loss = model.getLoss();

    //             // set color space
    //             model.setColorSpace(cv::ccm::COLOR_SPACE_sRGB);

    //             cv::Mat img;
    //             cvtColor(image, img, cv::COLOR_BGR2RGB);
    //             img.convertTo(img, CV_64F);
    //             const int inpSize = 255;
    //             const int outSize = 255;
    //             img /= inpSize;
    //             cv::Mat calibratedImage = model.infer(img); // make correction using ccm matrix
    //             cv::Mat out = calibratedImage * outSize;

    //             out.convertTo(out, CV_8UC3);
    //             cv::Mat imgOut = min(max(out, 0), outSize);
    //             cv::Mat outImg;
    //             cvtColor(imgOut, outImg, cv::COLOR_RGB2BGR);

    //             // save the calibrated image
    //             cv::imwrite(outputFolder + "/" + std::to_string(viewId) + ".calibrated.jpg", outImg);
    //         }
    //     }
    // }

    return EXIT_SUCCESS;
}
