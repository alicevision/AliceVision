// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

#include <aliceVision/image/all.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/config.hpp>
#include <aliceVision/utils/filesIO.hpp>
#include <aliceVision/utils/regexFilter.hpp>

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

    const std::vector< cv::Point2f > MACBETH_CCHART_CORNERS_POS = {
        {0.00, 0.00}, {16.75, 0.00}, {16.75, 11.25}, {0.00, 11.25},
    };

    const std::vector< cv::Point2f > MACBETH_CCHART_CELLS_POS_CENTER =  {
        {1.50f, 1.50f}, {4.25f, 1.50f}, {7.00f, 1.50f}, {9.75f, 1.50f}, {12.50f, 1.50f}, {15.25f, 1.50f},
        {1.50f, 4.25f}, {4.25f, 4.25f}, {7.00f, 4.25f}, {9.75f, 4.25f}, {12.50f, 4.25f}, {15.25f, 4.25f},
        {1.50f, 7.00f}, {4.25f, 7.00f}, {7.00f, 7.00f}, {9.75f, 7.00f}, {12.50f, 7.00f}, {15.25f, 7.00f},
        {1.50f, 9.75f}, {4.25f, 9.75f}, {7.00f, 9.75f}, {9.75f, 9.75f}, {12.50f, 9.75f}, {15.25f, 9.75f}
    };

    const float MACBETH_CCHART_CELLS_SIZE = 2.50f * .5f;

    struct Quad
    {
        std::vector<float> xCoords;
        std::vector<float> yCoords;

        Quad() = default;

        Quad(const std::vector<cv::Point2f>& points)
        {
            if (points.size() != 4)
            {
                ALICEVISION_LOG_ERROR("Invalid color checker box: size is not equal to 4");
                exit(EXIT_FAILURE);
            }
            for(const auto& p : points)
            {
                xCoords.push_back(p.x);
                yCoords.push_back(p.y);
            }
            // close polyline
            xCoords.push_back(points[0].x);
            yCoords.push_back(points[0].y);
        }

        void transform(cv::Matx33f transformMatrix)
        {
            for (int i = 0; i < 5; ++i)
            {
                cv::Point3f p(xCoords[i], yCoords[i], 1.);
                p = transformMatrix * p;
                xCoords[i] = p.x / p.z;
                yCoords[i] = p.y / p.z;
            }
        }
    };

    void draw(const cv::Ptr<cv::mcc::CChecker> &checker, std::string outputPath)
    {
        std::vector< Quad > quadsToDraw;

        // Push back the quad representing the color checker
        quadsToDraw.push_back( Quad(checker->getBox()) );

        // Transform matrix from 'theoric' to 'measured'
        cv::Matx33f tMatrix = cv::getPerspectiveTransform(MACBETH_CCHART_CORNERS_POS,checker->getBox());

        // Push back quads representing color checker cells
        for (const auto& center : MACBETH_CCHART_CELLS_POS_CENTER)
        {
            Quad quad({
                cv::Point2f( center.x - MACBETH_CCHART_CELLS_SIZE * .5, center.y - MACBETH_CCHART_CELLS_SIZE * .5 ),
                cv::Point2f( center.x + MACBETH_CCHART_CELLS_SIZE * .5, center.y - MACBETH_CCHART_CELLS_SIZE * .5 ),
                cv::Point2f( center.x + MACBETH_CCHART_CELLS_SIZE * .5, center.y + MACBETH_CCHART_CELLS_SIZE * .5 ),
                cv::Point2f( center.x - MACBETH_CCHART_CELLS_SIZE * .5, center.y + MACBETH_CCHART_CELLS_SIZE * .5 ),
            });
            quad.transform(tMatrix);
            quadsToDraw.push_back(quad);
        }

        svg::svgDrawer svgSurface;
        for (const auto& quad : quadsToDraw)
        {
            svgSurface.drawPolyline(
                quad.xCoords.begin(), quad.xCoords.end(),
                quad.yCoords.begin(), quad.yCoords.end(),
                svg::svgStyle().stroke("red", 2));
        }

        std::ofstream svgFile(outputPath.c_str());
        svgFile << svgSurface.closeSvgFile().str();
        svgFile.close();
    }
} // namespace ccheckerSVG


struct CCheckerDetectionSettings {
    image::ImageReadOptions imgReadOptions;
    std::string outputColorData;
    std::string outputPositionData;
    bool debug;
};


struct CCheckerData {
    cv::Ptr<cv::mcc::CChecker> _cchecker;
    cv::Mat _colorData;

    CCheckerData() = default;

    CCheckerData(cv::Ptr<cv::mcc::CChecker> cchecker)
        : _cchecker(cchecker)
    {
        // Get colors data
        cv::Mat chartsRGB = cchecker->getChartsRGB();

        // Extract average colors
        _colorData = chartsRGB.col(1).clone().reshape(3, chartsRGB.rows / 3) / 255.f;
    }

    bool compare(const CCheckerData &c) const
    {
        return boundingBoxArea() > c.boundingBoxArea();
    }

    float boundingBoxArea() const
    {
        std::vector<cv::Point2f>& points = _cchecker->getBox();
        cv::Point2f min = points[0];
        cv::Point2f max = points[0];
        for(const auto& p : points) {
            if (p.x < min.x && p.y < min.y)
                min = p;
            if (p.x > max.x && p.y > max.y)
                max = p;
        }
        cv::Point2f diag = max - min;
        return diag.x * diag.y;
    }

    void serialize(
        const std::string &outputColorDataPath,
        const std::string& outputPositionDataPath)
    {
        std::ofstream f;
        f.open(outputColorDataPath);
        for(int i = 0; i < _colorData.rows; ++i)
            for(int j = 0; j < _colorData.cols; ++j)
                for(int k = 0; k < 3; ++k)
                    f << std::setprecision(std::numeric_limits<double>::digits10 + 2)
                      << _colorData.at<cv::Vec3d>(i, j)[k] << std::endl;
        f.close();
        f.open(outputPositionDataPath);
        for(const auto& p : _cchecker->getBox())
            f << p.x << '\t' << p.y << std::endl;
    }
};


void detectColorChecker(
    std::vector<CCheckerData> &detectedCCheckers,
    const fs::path &imgFsPath,
    CCheckerDetectionSettings &settings)
{
    const int nc = 2; // Max number of charts in an image
    const std::string outputColorFolder =    fs::path(settings.outputColorData   ).parent_path().string();
    const std::string outputPositionFolder = fs::path(settings.outputPositionData).parent_path().string();
    const std::string imgSrcPath = imgFsPath.string();
    const std::string imgSrcStem = imgFsPath.stem().string();
    const std::string imgDestStem = imgSrcStem;

    // Load image
    image::Image<image::RGBAfColor> image;
    image::readImage(imgSrcPath, image, settings.imgReadOptions);
    cv::Mat imageBGR = image::imageRGBAToCvMatBGR(image, CV_8UC3);

    if(imageBGR.cols == 0 || imageBGR.rows == 0)
    {
        ALICEVISION_LOG_ERROR("Image at: '" << imgSrcPath << "'.\n" << "is empty.");
        exit(EXIT_FAILURE);
    }

    cv::Ptr<cv::mcc::CCheckerDetector> detector = cv::mcc::CCheckerDetector::create();

    if(!detector->process(imageBGR, cv::mcc::TYPECHART::MCC24, nc))
    {
        ALICEVISION_LOG_INFO("Checker not detected in image at: '" << imgSrcPath << "'");
        return;
    }

    int counter = 0;

    for(const cv::Ptr<cv::mcc::CChecker> cchecker : detector->getListColorChecker())
    {
        const std::string counterStr = "_" + std::to_string(++counter);

        ALICEVISION_LOG_INFO("Checker #" << counter <<" successfully detected in '" << imgSrcStem << "'");

        if(settings.debug)
        {
            // Output debug data
            ccheckerSVG::draw(cchecker, outputColorFolder + "/" + imgDestStem + counterStr + ".svg");

            cv::Ptr<cv::mcc::CCheckerDraw> cdraw = cv::mcc::CCheckerDraw::create(cchecker, CV_RGB(250, 0, 0), 3);
            cdraw->draw(imageBGR);

            // Write debug image
            cv::imwrite(outputColorFolder + "/" + imgDestStem + counterStr + ".jpg", imageBGR);
        }

        CCheckerData ccheckerData(cchecker);
        ccheckerData.serialize(
            outputColorFolder    + "/" + imgDestStem + "_colorData" + counterStr,
            outputPositionFolder + "/" + imgDestStem + "_positionData" + counterStr);

        detectedCCheckers.push_back(ccheckerData);
    }
}


int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
    std::string inputExpression;
    std::string outputColorData;
    std::string outputPositionData;

    // user optional parameters
    bool debug = false;

    po::options_description allParams(
        "This program is used to perform Macbeth color checker chart detection.\n"
        "AliceVision colorCheckerDetection");

    po::options_description inputParams("Required parameters");
    inputParams.add_options()
        ("input,i", po::value<std::string>(&inputExpression)->required(),
         "SfMData file input, image filenames or regex(es) on the image file path (supported regex: '#' matches a single digit, '@' one or more digits, '?' one character and '*' zero or more).")
        ("outputColorData", po::value<std::string>(&outputColorData)->required(),
         "Output path for the color data file.")
        ("outputPositionData", po::value<std::string>(&outputPositionData)->required(),
         "Output path for the color checker position data file.");

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

    CCheckerDetectionSettings settings;
    settings.imgReadOptions.outputColorSpace = image::EImageColorSpace::NO_CONVERSION;
    settings.outputColorData = outputColorData;
    settings.outputPositionData = outputPositionData;
    settings.debug = debug;

    std::vector< CCheckerData > detectedCCheckers;

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

        int counter = 0;

        // Detect color checker for each images
        for(const auto& viewIt : sfmData.getViews())
        {
            const sfmData::View& view = *(viewIt.second);

            ALICEVISION_LOG_INFO(++counter << "/" << sfmData.getViews().size() << " - Process image at: '" << view.getImagePath() << "'.");

            settings.imgReadOptions.applyWhiteBalance = view.getApplyWhiteBalance();
            detectColorChecker(detectedCCheckers, view.getImagePath(), settings);
        }

    }
    else
    {
        // load input as image file or image folder
        const fs::path inputPath(inputExpression);
        std::vector<std::string> filesStrPaths;

        if(fs::is_regular_file(inputPath))
        {
            filesStrPaths.push_back(inputPath.string());
        }
        else
        {
            ALICEVISION_LOG_INFO("Working directory Path '" + inputPath.parent_path().generic_string() + "'.");

            const std::regex regex = utils::filterToRegex(inputExpression);
            // Get supported files in inputPath directory which matches our regex filter
            filesStrPaths = utils::getFilesPathsFromFolder(inputPath.parent_path().generic_string(),
               [&regex](const boost::filesystem::path& path) {
                 return image::isSupported(path.extension().string()) && std::regex_match(path.generic_string(), regex);
               }
            );
        }

        const int size = filesStrPaths.size();

        if(!size)
        {
            ALICEVISION_LOG_ERROR("Any images was found.");
            ALICEVISION_LOG_ERROR("Input folders or input expression '" << inputExpression << "' may be incorrect ?");
            return EXIT_FAILURE;
        }
        else
        {
            ALICEVISION_LOG_INFO(size << " images found.");
        }

        int i = 0;
        for(const std::string& imgSrcPath : filesStrPaths)
        {
            ALICEVISION_LOG_INFO(++i << "/" << size << " - Process image at: '" << imgSrcPath << "'.");
            detectColorChecker(detectedCCheckers, imgSrcPath, settings);
        }

    }

    if (detectedCCheckers.size() == 0)
    {
        ALICEVISION_LOG_INFO("Could not find any macbeth color checker in the input images.");
        return EXIT_SUCCESS;
    }
    ALICEVISION_LOG_INFO("Found color checkers count: " << detectedCCheckers.size());

    // Find and output best color checker
    CCheckerData best = detectedCCheckers[0];
    for (int i = 1; i < detectedCCheckers.size(); ++i)
    {
        if(detectedCCheckers[i].compare(best))
            best = detectedCCheckers[i];
    }
    best.serialize(settings.outputColorData, settings.outputPositionData);

    return EXIT_SUCCESS;
}
