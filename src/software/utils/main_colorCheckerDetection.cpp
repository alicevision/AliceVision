// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/config.hpp>

#include <dependencies/vectorGraphics/svgDrawer.hpp>

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

namespace ccheckerSVG {
    struct Corners
    {
        std::vector<float> xCoords;
        std::vector<float> yCoords;

        Corners() = default;

        Corners(const std::vector<cv::Point2f>& ccheckerBox)
        {
            if (ccheckerBox.size() != 4)
            {
                ALICEVISION_LOG_ERROR("Invalid color checker box: size is not equal to 4");
                exit(EXIT_FAILURE);
            }
            for (const auto &point : ccheckerBox)
            {
                xCoords.push_back(point.x);
                yCoords.push_back(point.y);
            }
            // close polyline
            xCoords.push_back(ccheckerBox[0].x);
            yCoords.push_back(ccheckerBox[0].y);
        }
    };

    void drawCorners(const cv::Ptr<cv::mcc::CChecker> &checker, std::string outputFolder)
    {
        ccheckerSVG::Corners corners(checker->getBox());

        svg::svgDrawer svgSurface;
        svgSurface.drawPolyline(
            corners.xCoords.begin(), corners.xCoords.end(),
            corners.yCoords.begin(), corners.yCoords.end(),
            svg::svgStyle().stroke("red", 4));

        std::string sFileName = outputFolder + "/" + "corners.svg";
        std::ofstream svgFile(sFileName.c_str());
        svgFile << svgSurface.closeSvgFile().str();
        svgFile.close();
    }
} // namespace ccheckerSVG

void serializeColorMatrixToTextFile(const std::string &outputColorData, cv::Mat &colorData)
{
    std::ofstream f;
    f.open(outputColorData);
    for(int row = 0; row < colorData.rows; row++)
    {
        cv::Vec3d* rowPtr = colorData.ptr<cv::Vec3d>(row); // pointer which points to the first place of each row
        for(int col = 0; col < colorData.cols; col++)
        {
            const cv::Vec3d& matPixel = rowPtr[col];
            for(unsigned int i = 0; i < 3; ++i)
            {
                f << std::setprecision(std::numeric_limits<double>::digits10 + 2) << matPixel[i] << std::endl;
            }
        }
    }
    f.close();
}

void detectColorChecker(
    const fs::path &imgPath,
    const std::string &outputFolder,
    const std::string &outputColorData,
    const bool debug)
{
    const int nc = 1; // Number of charts in an image
    const std::string imgSrcPath = imgPath.string();
    const std::string imgSrcStem = imgPath.stem().string();
    const std::string imgDestStem = imgSrcStem;
    const std::string imgDestPath = outputFolder + "/" + imgDestStem + ".jpg";

    // Create an image with 3 channel BGR color
    cv::Mat image = cv::imread(imgSrcPath, 1);

    if(image.cols == 0 || image.rows == 0)
    {
        ALICEVISION_LOG_ERROR("Image at: '" << imgSrcPath << "'.\n" << "is empty.");
        exit(EXIT_FAILURE);
    }

    cv::Ptr<cv::mcc::CCheckerDetector> detector = cv::mcc::CCheckerDetector::create();

    if(!detector->process(image, cv::mcc::TYPECHART::MCC24, nc))
    {
        ALICEVISION_LOG_INFO("Checker not detected in image at: '" << imgSrcPath << "'");
        return;
    }

    ALICEVISION_LOG_INFO("Checker successfully detected in '" << imgSrcStem << "'");

    std::cout << "Image size:" << image.cols * image.rows << std::endl;

    for(cv::Ptr<cv::mcc::CChecker> checker : detector->getListColorChecker())
    {
        if(debug)
        {
            // Output debug data
            ccheckerSVG::draw(checker, outputFolder + "/" + imgDestStem + ".svg");

            cv::Ptr<cv::mcc::CCheckerDraw> cdraw = cv::mcc::CCheckerDraw::create(checker, CV_RGB(250, 0, 0), 3);
            cdraw->draw(image);

            cv::imwrite(imgDestPath, image);
        }

        // Get colors data
        cv::Mat chartsRGB = checker->getChartsRGB();

        // Extract average colors
        cv::Mat colorData = chartsRGB.col(1).clone().reshape(3, chartsRGB.rows / 3);
        colorData /= 255.0; // conversion to float

        serializeColorMatrixToTextFile(outputColorData, colorData);
    }
}


int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
    std::string inputExpression;
    std::string outputImages;
    std::string outputColorData;

    // user optional parameters
    bool debug = false;

    po::options_description allParams(
        "This program is used to perform color checker detection\n"
        "AliceVision colorCheckerDetection");

    po::options_description inputParams("Required parameters");
    inputParams.add_options()
        ("input,i", po::value<std::string>(&inputExpression)->required(),
         "SfMData file input, image filenames or regex(es) on the image file path (supported regex: '#' matches a single digit, '@' one or more digits, '?' one character and '*' zero or more).")
        ("output,o", po::value<std::string>(&outputImages)->required(),
         "Output path for the images.")
        ("outputColorData", po::value<std::string>(&outputColorData)->required(),
         "Output path for the color data file.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("debug", po::value<bool>(&debug),
         "Output debug data.");

    po::options_description logParams("Log parameters");
    logParams.add_options()
        ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
         "verbosity level (fatal, error, warning, info, debug, trace).");

    allParams.add(inputParams).add(optionalParams).add(logParams);

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

    std::string outputFolder = fs::path(outputImages).parent_path().string();

    // create output folder
    if(!fs::exists(outputFolder))
    {
        if(!fs::create_directory(outputFolder))
        {
            ALICEVISION_LOG_ERROR("Cannot create output folder");
            return EXIT_FAILURE;
        }
    }

    // Check if inputExpression is recognized as sfm data file
    const std::string inputExt = boost::to_lower_copy(fs::path(inputExpression).extension().string());
    static const std::array<std::string, 2> sfmSupportedExtensions = {".sfm", ".abc"};
    if(std::find(sfmSupportedExtensions.begin(), sfmSupportedExtensions.end(), inputExt) != sfmSupportedExtensions.end())
    {
        // load input as sfm data file
        sfmData::SfMData sfmData;
        if (!sfmDataIO::Load(sfmData, inputExpression, sfmDataIO::ESfMData(sfmDataIO::VIEWS)))
        {
            ALICEVISION_LOG_ERROR("The input SfMData file '" << inputExpression << "' cannot be read.");
            return EXIT_FAILURE;
        }

        // Map used to store paths of the views that need to be processed
        std::unordered_map<IndexT, std::string> imagePaths;

        // Store paths in map
        for(const auto& viewIt : sfmData.getViews())
        {
            const sfmData::View& view = *(viewIt.second);
            imagePaths.insert({view.getViewId(), view.getImagePath()});
        }

        const int size = imagePaths.size();
        int i = 0;
        int nc = 1; // Number of charts in an image

        for(auto& viewIt : imagePaths)
        {
            const IndexT viewId = viewIt.first;
            const std::string viewPath = viewIt.second;
            sfmData::View& view = sfmData.getView(viewId);

            // Create an image with 3 channel BGR color
            cv::Mat image = cv::imread(viewPath, 1);

            if(image.cols == 0 || image.rows == 0)
            {
                ALICEVISION_LOG_ERROR("Image with id '" << viewId << "'.\n"
                    << "is empty.");
                return EXIT_FAILURE;
            }

            cv::Ptr<cv::mcc::CCheckerDetector> detector = cv::mcc::CCheckerDetector::create();

            ALICEVISION_LOG_INFO(++i << "/" << size << " - Process view '" << viewId << "'.");

            if(!detector->process(image, cv::mcc::TYPECHART(0), nc))
            {
                ALICEVISION_LOG_INFO("Checker not detected in image with id '" << viewId << "'");
                continue;
            }

            ALICEVISION_LOG_INFO("Checker successfully detected in image with id '" << viewId << "'");
            // get checker
            std::vector<cv::Ptr<cv::mcc::CChecker>> checkers = detector->getListColorChecker();

            for(cv::Ptr<cv::mcc::CChecker> checker : checkers)
            {
                // current checker
                if(debug)
                {
                    ccheckerSVG::drawCorners(checker, outputFolder);

                    cv::Ptr<cv::mcc::CCheckerDraw> cdraw = cv::mcc::CCheckerDraw::create(checker, CV_RGB(250, 0, 0), 3);
                    cdraw->draw(image);

                    // save the image with the color checker drawn
                    cv::imwrite(outputFolder + "/" + std::to_string(viewId) + ".jpg", image);
                }

                // preparation for color calibration
                cv::Mat chartsRGB = checker->getChartsRGB();
                cv::Mat src = chartsRGB.col(1).clone().reshape(3, chartsRGB.rows / 3);
                src /= 255.0;

                std::ofstream f;
                f.open(outputColorData);
                f << src << std::endl;
                f.close();
            }
        }

        if(!sfmDataIO::Save(sfmData, outputImages, sfmDataIO::ESfMData(sfmDataIO::ALL)))
        {
            ALICEVISION_LOG_ERROR("Unable to save SfMData.");
            return EXIT_FAILURE;
        }

    }
    else
    {
        // load input as image file or image folder

    }

    return EXIT_SUCCESS;
}
