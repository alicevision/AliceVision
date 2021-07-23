// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
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
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

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
namespace bpt = boost::property_tree;
namespace po = boost::program_options;

struct CChecker
{
    std::string _bodySerialNumber;
    std::string _lensSerialNumber;
    std::string _imagePath;
    std::string _viewId;
    cv::Mat _colorData;

    explicit CChecker(const bpt::ptree::value_type& ccheckerPTree)
    {
        _bodySerialNumber = ccheckerPTree.second.get_child("bodySerialNumber").get_value<std::string>();
        _lensSerialNumber = ccheckerPTree.second.get_child("lensSerialNumber").get_value<std::string>();
        _imagePath = ccheckerPTree.second.get_child("imagePath").get_value<std::string>();
        _viewId = ccheckerPTree.second.get_child("viewId").get_value<std::string>();
        _colorData = cv::Mat::zeros(24, 1, CV_64FC3);

        int i = 0;
        for(const bpt::ptree::value_type& row : ccheckerPTree.second.get_child("colors"))
        {
            cv::Vec3d* rowPtr = _colorData.ptr<cv::Vec3d>(i);
            cv::Vec3d& matPixel = rowPtr[0];

            matPixel[0] = row.second.get_child("r").get_value<double>();
            matPixel[1] = row.second.get_child("g").get_value<double>();
            matPixel[2] = row.second.get_child("b").get_value<double>();

            ++i;
        }
    }
};

void processColorCorrection(image::Image<image::RGBAfColor>& image, cv::Mat& refColors)
{
    cv::Mat imageBGR = image::imageRGBAToCvMatBGR(image, CV_32FC3);

    cv::ccm::ColorCorrectionModel model(refColors, cv::ccm::COLORCHECKER_Macbeth);
    model.run();
    
    model.setColorSpace(cv::ccm::COLOR_SPACE_sRGB);
    //model.setCCM_TYPE(cv::ccm::CCM_3x3);
    //model.setDistance(cv::ccm::DISTANCE_CIE2000);
    //model.setLinear(cv::ccm::LINEARIZATION_GAMMA);
    //model.setLinearGamma(2.2);
    //model.setLinearDegree(3); // to prevent overfitting
    
    cv::Mat img;
    cvtColor(imageBGR, img, cv::COLOR_BGR2RGB);
    img.convertTo(img, CV_64F);

    cv::Mat calibratedImage = model.infer(img, true); // make correction using cc matrix and assuming images are in linear color space (as RAW for example)

    calibratedImage.convertTo(calibratedImage, CV_32FC3);

    cv::Mat outImg;
    cvtColor(calibratedImage, outImg, cv::COLOR_RGB2BGR);

    image::cvMatBGRToImageRGBA(outImg, image);
}


void saveImage(image::Image<image::RGBAfColor>& image, const std::string& inputPath, const std::string& outputPath,
               const image::EStorageDataType storageDataType)
{
    // Read metadata path
    std::string metadataFilePath;

    const std::string outExtension = boost::to_lower_copy(fs::path(outputPath).extension().string());
    const bool isEXR = (outExtension == ".exr");
    
    // Metadata are extracted from the original images
    metadataFilePath = inputPath;

    // Read metadata based on a filepath
    oiio::ParamValueList metadata = image::readImageMetadata(metadataFilePath);

    if(isEXR)
    {
        // Select storage data type
        metadata.push_back(
            oiio::ParamValue("AliceVision:storageDataType", image::EStorageDataType_enumToString(storageDataType)));
    }

    // Save image
    ALICEVISION_LOG_TRACE("Export image: '" << outputPath << "'.");

    image::writeImage(outputPath, image, image::EImageColorSpace::AUTO, metadata);
}


int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
    std::string inputExpression;
    std::string inputData;
    std::string extension;
    image::EStorageDataType storageDataType = image::EStorageDataType::Float;
    std::string outputPath;

    po::options_description allParams(
        "This program is used to perform color correction based on a color checker\n"
        "AliceVision colorCheckerCorrection");

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&inputExpression)->default_value(inputExpression),
        "SfMData file input, image filenames or regex(es) on the image file path (supported regex: '#' matches a "
        "single digit, '@' one or more digits, '?' one character and '*' zero or more).")(
        "inputData", po::value<std::string>(&inputData)->default_value(inputData),
        "Position and colorimetric data extracted from detected color checkers in the images")
        ("output,o", po::value<std::string>(&outputPath)->required(),
         "Output folder.")
        ;

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()("storageDataType", po::value<image::EStorageDataType>(&storageDataType)->default_value(storageDataType),
        ("Storage data type: " + image::EStorageDataType_informations()).c_str())(
        "extension", po::value<std::string>(&extension)->default_value(extension),
         "Output image extension (like exr, or empty to keep the original source file format.");

    po::options_description logParams("Log parameters");
    logParams.add_options()
        ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
         "verbosity level (fatal, error, warning, info, debug, trace).")
        ;

    allParams.add(requiredParams).add(optionalParams).add(logParams);

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

    if(fs::exists(inputData))
    {
        // checkers collection
        std::vector<CChecker> ccheckers;

        // property tree from data input
        bpt::ptree data;
        bpt::read_json(inputData, data);

        for(const bpt::ptree::value_type& ccheckerPTree : data.get_child("checkers"))
            ccheckers.push_back(CChecker(ccheckerPTree));

        // for now the program behaves as if all the images to process are sharing the same properties
        cv::Mat colorData = ccheckers[0]._colorData;

        // Map used to store paths of the views that need to be processed
        std::unordered_map<IndexT, std::string> ViewPaths;

        // Check if sfmInputDataFilename exist and is recognized as sfm data file
        const std::string inputExt = boost::to_lower_copy(fs::path(inputExpression).extension().string());
        static const std::array<std::string, 3> sfmSupportedExtensions = {".sfm", ".json", ".abc"};
        if(!inputExpression.empty() && std::find(sfmSupportedExtensions.begin(), sfmSupportedExtensions.end(),
                                                 inputExt) != sfmSupportedExtensions.end())
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

                const fs::path fsPath = viewPath;
                const std::string fileExt = fsPath.extension().string();
                const std::string outputExt = extension.empty() ? fileExt : (std::string(".") + extension);
                const std::string outputfilePath =
                    (fs::path(outputPath) / (std::to_string(viewId) + outputExt)).generic_string();

                ALICEVISION_LOG_INFO(++i << "/" << size << " - Process view '" << viewId << "' for color correction.");

                // Read image options and load image
                image::ImageReadOptions options;
                options.outputColorSpace = image::EImageColorSpace::NO_CONVERSION;
                options.applyWhiteBalance = view.getApplyWhiteBalance();

                image::Image<image::RGBAfColor> image;
                image::readImage(viewPath, image, options);

                // Image color correction processing
                processColorCorrection(image, colorData);

                // Save image
                saveImage(image, viewPath, outputfilePath, storageDataType);

                // Update sfmdata view for this modification
                view.setImagePath(outputfilePath);
                view.setWidth(image.Width());
                view.setHeight(image.Height());
            }

            // Save sfmData with modified path to images
            const std::string sfmfilePath =
                (fs::path(outputPath) / fs::path(inputExpression).filename()).generic_string();
            if(!sfmDataIO::Save(sfmData, sfmfilePath, sfmDataIO::ESfMData(sfmDataIO::ALL)))
            {
                ALICEVISION_LOG_ERROR("The output SfMData file '" << sfmfilePath << "' cannot be written.");

                return EXIT_FAILURE;
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
                filesStrPaths = utils::getFilesPathsFromFolder(
                    inputPath.parent_path().generic_string(), [&regex](const boost::filesystem::path& path) {
                        return image::isSupported(path.extension().string()) &&
                               std::regex_match(path.generic_string(), regex);
                    });
            }

            const int size = filesStrPaths.size();

            if(!size)
            {
                ALICEVISION_LOG_ERROR("Any images was found.");
                ALICEVISION_LOG_ERROR("Input folders or input expression '" << inputExpression
                                                                            << "' may be incorrect ?");
                return EXIT_FAILURE;
            }
            else
            {
                ALICEVISION_LOG_INFO(size << " images found.");
            }

            int i = 0;
            for(const std::string& inputFilePath : filesStrPaths)
            {
                const fs::path path = fs::path(inputFilePath);
                const std::string filename = path.stem().string();
                const std::string fileExt = path.extension().string();
                const std::string outputExt = extension.empty() ? fileExt : (std::string(".") + extension);
                const std::string outputFilePath = (fs::path(outputPath) / (filename + outputExt)).generic_string();

                ALICEVISION_LOG_INFO(++i << "/" << size << " - Process image '" << filename << fileExt
                                         << "' for color correction.");

                // Read original image
                image::Image<image::RGBAfColor> image;
                image::readImage(inputFilePath, image, image::EImageColorSpace::NO_CONVERSION);

                // Image color correction processing
                processColorCorrection(image, colorData);

                // Save image
                saveImage(image, inputFilePath, outputFilePath, storageDataType);
            }
        }
    }

    return EXIT_SUCCESS;
}
