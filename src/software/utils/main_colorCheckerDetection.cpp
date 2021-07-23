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
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/mcc.hpp>

#include <string>
#include <fstream>
#include <vector>
#include <limits>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace fs = boost::filesystem;
namespace bpt = boost::property_tree;
namespace po = boost::program_options;

// Match values used in OpenCV MCC
// See https://github.com/opencv/opencv_contrib/blob/342f8924cca88fe6ce979024b7776f6815c89978/modules/mcc/src/dictionary.hpp#L72
const std::vector< cv::Point2f > MACBETH_CCHART_CORNERS_POS = {
    {0.f, 0.f}, {1675.f, 0.f}, {1675.f, 1125.f}, {0.f, 1125.f},
};

const std::vector< cv::Point2f > MACBETH_CCHART_CELLS_POS_CENTER =  {
    {150.f, 150.f}, {425.f, 150.f}, {700.f, 150.f}, {975.f, 150.f}, {1250.f, 150.f}, {1525.f, 150.f},
    {150.f, 425.f}, {425.f, 425.f}, {700.f, 425.f}, {975.f, 425.f}, {1250.f, 425.f}, {1525.f, 425.f},
    {150.f, 700.f}, {425.f, 700.f}, {700.f, 700.f}, {975.f, 700.f}, {1250.f, 700.f}, {1525.f, 700.f},
    {150.f, 975.f}, {425.f, 975.f}, {700.f, 975.f}, {975.f, 975.f}, {1250.f, 975.f}, {1525.f, 975.f}
};

const float MACBETH_CCHART_CELLS_SIZE = 250.f * .5f;

struct QuadSVG
{
    std::vector<float> xCoords;
    std::vector<float> yCoords;

    QuadSVG() = default;

    explicit QuadSVG(const std::vector<cv::Point2f>& points)
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

    void transform(const cv::Matx33f& transformMatrix)
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

void drawSVG(const cv::Ptr<cv::mcc::CChecker> &checker, const std::string& outputPath)
{
    std::vector< QuadSVG > quadsToDraw;

    // Push back the quad representing the color checker
    quadsToDraw.push_back( QuadSVG(checker->getBox()) );

    // Transform matrix from 'theoric' to 'measured'
    cv::Matx33f tMatrix = cv::getPerspectiveTransform(MACBETH_CCHART_CORNERS_POS,checker->getBox());

    // Push back quads representing color checker cells
    for (const auto& center : MACBETH_CCHART_CELLS_POS_CENTER)
    {
        QuadSVG quad({
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


struct ImageOptions {
    fs::path imgFsPath;
    std::string viewId;
    std::string lensSerialNumber;
    std::string bodySerialNumber;
    image::ImageReadOptions readOptions;
};


struct CCheckerDetectionSettings {
    cv::mcc::TYPECHART typechart;
    unsigned int maxCountByImage;
    std::string outputData;
    bool debug;
};


struct Quad
{
    struct BoundingBox
    {
        cv::Point2f min;
        cv::Point2f max;

        BoundingBox()
            : min(cv::Point2f(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()))
            , max(cv::Point2f(0.f, 0.f))
        {}

        float sizeX() const { return max.x - min.x; }
        float sizeY() const { return max.y - min.y; }
        bool contains(float x, float y) const { return x > min.x && y > min.y && x < max.x && y < max.y; }
        bool contains(cv::Point2f p) const { return contains(p.x, p.y); }
    };

    std::vector< cv::Point > _corners;
    BoundingBox _bbox;

    Quad() = default;

    Quad(const std::vector<cv::Point2f>& corners)
    {
        if(corners.size() != 4)
        {
            ALICEVISION_LOG_ERROR("Invalid points count, expected 4.");
            exit(EXIT_FAILURE);
        }

        for(const auto& c : corners)
            _corners.push_back(c);
        updateBBox();
    }

    Quad(cv::Point2f center, float halfHeight, float halfWidth)
        : _corners( std::vector< cv::Point >(4) )
    {
        _corners[0] = cv::Point2f( center.x - halfWidth, center.y - halfHeight );
        _corners[1] = cv::Point2f( center.x + halfWidth, center.y - halfHeight );
        _corners[2] = cv::Point2f( center.x + halfWidth, center.y + halfHeight );
        _corners[3] = cv::Point2f( center.x - halfWidth, center.y + halfHeight );
        updateBBox();
    }

    void updateBBox()
    {
        for(auto const &c : _corners)
        {
            if(c.x < _bbox.min.x)
                _bbox.min.x = c.x;
            if(c.y < _bbox.min.y)
                _bbox.min.y = c.y;
            if(c.x > _bbox.max.x)
                _bbox.max.x = c.x;
            if(c.y > _bbox.max.y)
                _bbox.max.y = c.y;
        }
    }

    void transform(const cv::Matx33f& transformMatrix)
    {
        for (auto &c : _corners)
        {
            cv::Point3f p(c.x, c.y, 1.);
            p = transformMatrix * p;
            c.x = p.x / p.z;
            c.y = p.y / p.z;
        }
    }

    void translate(float dx, float dy)
    {
        for (auto &c : _corners)
        {
            c.x += dx;
            c.y += dy;
        }
    }
};


struct MacbethCCheckerQuad : Quad {
    cv::Ptr<cv::mcc::CChecker> _cchecker;
    cv::Mat _colorData;
    cv::Mat _transformMat;
    std::vector< cv::Mat > _cellMasks;
    ImageOptions _imgOpt;

    MacbethCCheckerQuad() = default;

    MacbethCCheckerQuad(
        cv::Ptr<cv::mcc::CChecker> cchecker,
        const image::Image<image::RGBAfColor> &img,
        const ImageOptions& imgOpt)
        : Quad(cchecker->getBox())
        , _cchecker(cchecker)
        , _imgOpt(imgOpt)
        , _cellMasks(std::vector< cv::Mat >(24))
    {
        // Transform matrix from 'theoric' to 'measured'
        _transformMat = cv::getPerspectiveTransform(MACBETH_CCHART_CORNERS_POS, _cchecker->getBox());

        // Create an image boolean mask for each cchecker cell
        for(int i = 0; i < _cellMasks.size(); ++i)
        {
            _cellMasks[i] = cv::Mat::zeros(_bbox.sizeY(), _bbox.sizeX(), CV_8UC1);
            Quad q(MACBETH_CCHART_CELLS_POS_CENTER[i],
                   MACBETH_CCHART_CELLS_SIZE * .5,
                   MACBETH_CCHART_CELLS_SIZE * .5);
            q.transform(_transformMat);
            q.translate(- _bbox.min.x, - _bbox.min.y);
            cv::fillConvexPoly(_cellMasks[i], q._corners.data(), 4, cv::Scalar(255));
        }

        // Extract colors from float image knowing cchecker position
        _colorData = cv::Mat::zeros(24, 1, CV_64FC3);
        cv::Mat pixelsCount = cv::Mat::zeros(24, 1, CV_32S);

        for(int y = 0; y < (int) _bbox.sizeY(); ++y)
        {
            for(int x = 0; x < (int) _bbox.sizeX(); ++x)
            {
                for(int i = 0; i < _cellMasks.size(); ++i)
                {
                    // Check current pixel for the current image mask
                    if(_cellMasks[i].at<uchar>(y,x) == 255)
                    {
                        const image::RGBAfColor& px = img(y + _bbox.min.y, x + _bbox.min.x);
                        _colorData.at< cv::Vec3d >(i,0) += cv::Vec3d(px.r(), px.g(), px.b());
                        ++pixelsCount.at<int>(i);
                    }
                }
            }
        }

        for(int i = 0; i < _colorData.rows; ++i)
            _colorData.at<cv::Vec3d>(i, 0) /= pixelsCount.at<int>(i); // average value
    }

    bpt::ptree ptree() const
    {
        bpt::ptree pt, ptColors, ptPositions, ptTransform;

        pt.put("bodySerialNumber", _imgOpt.bodySerialNumber);
        pt.put("lensSerialNumber", _imgOpt.lensSerialNumber);
        pt.put("imagePath", _imgOpt.imgFsPath.string());
        pt.put("viewId", _imgOpt.viewId);

        // Serialize checker.positions
        for (const auto& point : _cchecker->getBox())
        {
            bpt::ptree ptPoint;
            ptPoint.put("x", point.x);
            ptPoint.put("y", point.y);
            ptPositions.push_back( std::make_pair("", ptPoint) );
        }
        pt.add_child("positions", ptPositions);

        // Serialize checker.transform
        for (int i = 0; i < _transformMat.rows; ++i)
        {
            bpt::ptree row;
            for(int j = 0; j < _transformMat.cols; ++j)
            {
                bpt::ptree cell;
                cell.put_value(_transformMat.at<double>(i, j));
                row.push_back(std::make_pair("", cell));
            }
            ptTransform.push_back( std::make_pair("", row) );
        }
        pt.add_child("transform", ptTransform);

        // Serialize colors
        for (int i = 0; i < _colorData.rows; ++i)
        {
            bpt::ptree ptColor;
            ptColor.put("r", _colorData.at<cv::Vec3d>(i,0)[0]);
            ptColor.put("g", _colorData.at<cv::Vec3d>(i,0)[1]);
            ptColor.put("b", _colorData.at<cv::Vec3d>(i,0)[2]);
            ptColors.push_back( std::make_pair("", ptColor) );
        }
        pt.add_child("colors", ptColors);

        return pt;
    }
};


void detectColorChecker(
    std::vector<MacbethCCheckerQuad> &detectedCCheckers,
    ImageOptions& imgOpt,
    CCheckerDetectionSettings &settings)
{
    const std::string outputFolder = fs::path(settings.outputData).parent_path().string() + "/";
    const std::string imgSrcPath = imgOpt.imgFsPath.string();
    const std::string imgSrcStem = imgOpt.imgFsPath.stem().string();
    const std::string imgDestStem = imgSrcStem;

    // Load image
    image::Image<image::RGBAfColor> img;
    image::readImage(imgSrcPath, img, imgOpt.readOptions);
    cv::Mat imgBGR = image::imageRGBAToCvMatBGR(img, CV_8UC3);

    if(imgBGR.cols == 0 || imgBGR.rows == 0)
    {
        ALICEVISION_LOG_ERROR("Image at: '" << imgSrcPath << "'.\n" << "is empty.");
        exit(EXIT_FAILURE);
    }

    cv::Ptr<cv::mcc::CCheckerDetector> detector = cv::mcc::CCheckerDetector::create();

    if(!detector->process(imgBGR, settings.typechart, settings.maxCountByImage))
    {
        ALICEVISION_LOG_INFO("Checker not detected in image at: '" << imgSrcPath << "'");
        return;
    }

    int counter = 0;

    for(const cv::Ptr<cv::mcc::CChecker> cchecker : detector->getListColorChecker())
    {
        const std::string counterStr = "_" + std::to_string(++counter);

        ALICEVISION_LOG_INFO("Checker #" << counter <<" successfully detected in '" << imgSrcStem << "'");

        MacbethCCheckerQuad ccq(cchecker, img, imgOpt);
        detectedCCheckers.push_back(ccq);
        
        if(settings.debug)
        {
            // Output debug data
            drawSVG(cchecker, outputFolder + imgDestStem + counterStr + ".svg");

            cv::Ptr<cv::mcc::CCheckerDraw> cdraw = cv::mcc::CCheckerDraw::create(cchecker, CV_RGB(250, 0, 0), 3);
            cdraw->draw(imgBGR);
            cv::imwrite(outputFolder + imgDestStem + counterStr + ".jpg", imgBGR);

            const std::string masksFolder = outputFolder + "masks/";
            fs::create_directory(masksFolder);

            for (int i = 0; i < ccq._cellMasks.size(); ++i)
                cv::imwrite(masksFolder + imgDestStem + counterStr + "_" + std::to_string(i) + ".jpg", ccq._cellMasks[i]);
        }
    }
}


int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
    std::string inputExpression;
    std::string outputData;

    // user optional parameters
    bool debug = false;
    unsigned int maxCountByImage = 1;

    po::options_description allParams(
        "This program is used to perform Macbeth color checker chart detection.\n"
        "AliceVision colorCheckerDetection");

    po::options_description inputParams("Required parameters");
    inputParams.add_options()
        ("input,i", po::value<std::string>(&inputExpression)->required(),
         "SfMData file input, image filenames or regex(es) on the image file path (supported regex: '#' matches a single digit, '@' one or more digits, '?' one character and '*' zero or more).")
        ("outputData", po::value<std::string>(&outputData)->required(),
         "Output path for the color checker data.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("debug", po::value<bool>(&debug),
         "Output debug data.")
        ("maxCount", po::value<unsigned int>(&maxCountByImage),
         "Maximum color charts count to detect in a single image.");

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
    settings.typechart = cv::mcc::TYPECHART::MCC24;
    settings.maxCountByImage = maxCountByImage;
    settings.outputData = outputData;
    settings.debug = debug;

    std::vector< MacbethCCheckerQuad > detectedCCheckers;

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
            ImageOptions imgOpt = {
                view.getImagePath(),
                std::to_string(view.getViewId()),
                view.getMetadataBodySerialNumber(),
                view.getMetadataLensSerialNumber() };
            imgOpt.readOptions.outputColorSpace = image::EImageColorSpace::SRGB;
            imgOpt.readOptions.applyWhiteBalance = view.getApplyWhiteBalance();
            detectColorChecker(detectedCCheckers, imgOpt, settings);
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

        int counter = 0;
        for(const std::string& imgSrcPath : filesStrPaths)
        {
            ALICEVISION_LOG_INFO(++counter << "/" << size << " - Process image at: '" << imgSrcPath << "'.");
            ImageOptions imgOpt;
            imgOpt.imgFsPath = imgSrcPath;
            imgOpt.readOptions.outputColorSpace = image::EImageColorSpace::SRGB;
            detectColorChecker(detectedCCheckers, imgOpt, settings);
        }

    }

    if (detectedCCheckers.size() == 0)
    {
        ALICEVISION_LOG_INFO("Could not find any macbeth color checker in the input images.");
        return EXIT_SUCCESS;
    }
    ALICEVISION_LOG_INFO("Found color checkers count: " << detectedCCheckers.size());

    // Output data
    bpt::ptree data, ptCheckers;
    for (const auto& cchecker : detectedCCheckers)
    {
        ptCheckers.push_back(std::make_pair("", cchecker.ptree()));
    }
    data.add_child("checkers", ptCheckers);

    std::ofstream f;
    f.open(outputData);
    bpt::write_json(f, data);
    f.close();

    return EXIT_SUCCESS;
}
