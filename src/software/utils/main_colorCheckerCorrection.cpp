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

#include <aliceVision/cmdline/cmdline.hpp>
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

double srgbToLin(double c)
{
    return c <= 0.04045 ? c / 12.92 : powf((c + 0.055) / 1.055, 2.4);
}
cv::Vec3d srgbToLin(cv::Vec3d pix)
{
    cv::Vec3d pixOut(srgbToLin(pix[0]), srgbToLin(pix[1]), srgbToLin(pix[2]));
    return pixOut;
}
double linToSrgb(double c)
{
    return c < 0.0031308 ? 12.92 * c : 1.055 * powf(c, 1.f / 2.4f) - 0.055;
}
cv::Vec3d linToSrgb(cv::Vec3d pix)
{
    cv::Vec3d pixOut(linToSrgb(pix[0]), linToSrgb(pix[1]), linToSrgb(pix[2]));
    return pixOut;
}

double linToLum(cv::Vec3d pix)
{
    return 0.2126 * pix[0] + 0.7152 * pix[1] + 0.0722 * pix[2];
}

enum class ECorrectionMethod
{
    luminance,
    whiteBalance,
    full
};

inline std::string ECorrectionMethod_enumToString(ECorrectionMethod correctionMethod)
{
    switch(correctionMethod)
    {
        case ECorrectionMethod::luminance:
            return "luminance";
        case ECorrectionMethod::whiteBalance:
            return "whitebalance";
        case ECorrectionMethod::full:
            return "full";
    }
    throw std::invalid_argument("Invalid ECorrectionMethod Enum");
}

inline ECorrectionMethod ECorrectionMethod_stringToEnum(std::string correctionMethod)
{
    boost::to_lower(correctionMethod);
    if(correctionMethod == "luminance")
        return ECorrectionMethod::luminance;
    if(correctionMethod == "whitebalance")
        return ECorrectionMethod::whiteBalance;
    if(correctionMethod == "full")
        return ECorrectionMethod::full;

    throw std::invalid_argument("Unrecognized correction method '" + correctionMethod + "'");
}

inline std::ostream& operator<<(std::ostream& os, ECorrectionMethod method)
{
    return os << ECorrectionMethod_enumToString(method);
}

inline std::istream& operator>>(std::istream& in, ECorrectionMethod& method)
{
    std::string token;
    in >> token;
    method = ECorrectionMethod_stringToEnum(token);
    return in;
}

struct CChecker
{
    std::string _bodySerialNumber;
    std::string _lensSerialNumber;
    std::string _imagePath;
    std::string _viewId;
    cv::Mat _colorData;
    double _weight; // In the range [0,1]. 1.0 corresponds to a Macbeth frontal view occupying the full image.

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

            matPixel = srgbToLin(matPixel);

            ++i;
        }

        i = 0;
        std::vector<cv::Vec2d> corners_img(4);
        for(const bpt::ptree::value_type& row : ccheckerPTree.second.get_child("positions"))
        {
            corners_img[i][0] = row.second.get_child("x").get_value<double>();
            corners_img[i][1] = row.second.get_child("y").get_value<double>();

            ++i;
        }

        const double area = (fabs((corners_img[0][0] - corners_img[2][0]) * (corners_img[1][1] - corners_img[3][1])) +
                             fabs((corners_img[1][0] - corners_img[3][0]) * (corners_img[0][1] - corners_img[2][1]))) * 0.5;

        double dot = 0.0;
        for (int i = 0; i < 4; i++)
        {
            const double x1 = corners_img[(i + 1) % 4][0] - corners_img[i][0];
            const double y1 = corners_img[(i + 1) % 4][1] - corners_img[i][1];
            const double x2 = corners_img[(i + 2) % 4][0] - corners_img[(i + 1) % 4][0];
            const double y2 = corners_img[(i + 2) % 4][1] - corners_img[(i + 1) % 4][1];
            const double norm1 = sqrt(x1 * x1 + y1 * y1);
            const double norm2 = sqrt(x2 * x2 + y2 * y2);

            dot += fabs(x1 * x2 + y1 * y2) / norm1 / norm2;
        }
        dot /= 4.0; // Average of the abs value of dot products between consecutive edges, null for a frontal view.

        std::unique_ptr<oiio::ImageInput> in(oiio::ImageInput::open(_imagePath));
        _weight = area * (1.0 - dot) / in->spec().width / in->spec().height;
    }
};

void processColorCorrection(image::Image<image::RGBAfColor>& image, cv::Mat& refColors, ECorrectionMethod method = ECorrectionMethod::full)
{
    cv::Mat imageBGR = image::imageRGBAToCvMatBGR(image, CV_32FC3);

    cv::ccm::ColorCorrectionModel model(refColors, cv::ccm::COLORCHECKER_Macbeth);
    
    //model.setColorSpace(cv::ccm::COLOR_SPACE_sRGB);
    //model.setCCM_TYPE(cv::ccm::CCM_3x3);
    //model.setDistance(cv::ccm::DISTANCE_CIE2000);
    model.setLinear(cv::ccm::LINEARIZATION_IDENTITY); // input and ref colors are linear
    //model.setLinearGamma(2.2);
    //model.setLinearDegree(3); // to prevent overfitting

    cv::Mat img;
    cvtColor(imageBGR, img, cv::COLOR_BGR2RGB);
    img.convertTo(img, CV_64F);

    cv::Mat calibratedImage = img;

    if(method == ECorrectionMethod::luminance)
    {
        const double lum = linToLum(refColors.at<cv::Vec3d>(21, 0));
        const double lumTarget = srgbToLin(122.0 / 255.0); // 0.1946
        const double gain = lumTarget / lum;

        for(int r = 0; r < img.rows; r++)
        {
            for(int c = 0; c < img.cols; c++)
            {
                calibratedImage.at<cv::Vec3d>(r, c) = gain * img.at<cv::Vec3d>(r, c);
            }
        }
    }
    else if (method == ECorrectionMethod::whiteBalance)
    {
        const double target = srgbToLin(122.0 / 255.0); // 0.1946
        const double gainR = target / refColors.at<cv::Vec3d>(21, 0)[0];
        const double gainG = target / refColors.at<cv::Vec3d>(21, 0)[1];
        const double gainB = target / refColors.at<cv::Vec3d>(21, 0)[2];

        for(int r = 0; r < img.rows; r++)
        {
            for(int c = 0; c < img.cols; c++)
            {
                calibratedImage.at<cv::Vec3d>(r, c)[0] = gainR * img.at<cv::Vec3d>(r, c)[0];
                calibratedImage.at<cv::Vec3d>(r, c)[1] = gainG * img.at<cv::Vec3d>(r, c)[1];
                calibratedImage.at<cv::Vec3d>(r, c)[2] = gainB * img.at<cv::Vec3d>(r, c)[2];
            }
        }
    }
    else
    {
        model.run();
        calibratedImage = model.infer(img, true); // make correction and return a linear image
    }

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

    image::ImageWriteOptions options;

    options.fromColorSpace(image::EImageColorSpace::LINEAR);
    if(isEXR)
    {
        // Select storage data type
        options.storageDataType(storageDataType);
    }

    // Save image
    ALICEVISION_LOG_TRACE("Export image: '" << outputPath << "'.");

    image::writeImage(outputPath, image, options, metadata);
}


int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string inputExpression;
    std::string inputData;
    std::string extension;
    image::EStorageDataType storageDataType = image::EStorageDataType::Float;
    std::string outputPath;
    ECorrectionMethod correctionMethod = ECorrectionMethod::luminance;
    bool useBestColorCheckerOnly = true;

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
    optionalParams.add_options()("storageDataType",
                                 po::value<image::EStorageDataType>(&storageDataType)->default_value(storageDataType),
                                 ("Storage data type: " + image::EStorageDataType_informations()).c_str())(
        "extension", po::value<std::string>(&extension)->default_value(extension),
        "Output image extension (like exr, or empty to keep the original source file format.")(
        "correctionMethod", po::value<ECorrectionMethod>(&correctionMethod)->default_value(correctionMethod),
        "Correction method to apply.")(
        "useBestColorCheckerOnly", po::value<bool>(&useBestColorCheckerOnly)->default_value(useBestColorCheckerOnly),
        "Use only the color chart with the best orientation and size to compute the color correction model.")
        ;

    CmdLine cmdline("This program is used to perform color correction based on a color checker.\n"
                    "AliceVision colorCheckerCorrection");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

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

        std::vector<double> ccheckerWeights;
        double weightMax = 0.0;
        int idxWeightMax = 0;
        for (int i = 0; i < ccheckers.size(); i++)
        {
            ccheckerWeights.push_back(ccheckers[i]._weight);
            if(ccheckers[i]._weight > weightMax)
            {
                weightMax = ccheckers[i]._weight;
                idxWeightMax = i;
            }
        }

        cv::Mat colorData;

        if (useBestColorCheckerOnly)
        {
            colorData = ccheckers[idxWeightMax]._colorData;
            ALICEVISION_LOG_INFO("Use color checker detected in image " << ccheckers[idxWeightMax]._imagePath);
        }
        else // Combine colors of all detected color checkers
        {
            colorData = ccheckerWeights[0] * ccheckers[0]._colorData;
            double sumWeight = ccheckerWeights[0];
            for (int i = 1; i < ccheckers.size(); i++)
            {
                colorData += ccheckerWeights[i] * ccheckers[i]._colorData;
                sumWeight += ccheckerWeights[i];
            }
            colorData /= sumWeight;
        }

        // Map used to store paths of the views that need to be processed
        std::unordered_map<IndexT, std::string> ViewPaths;

        // Check if sfmInputDataFilename exist and is recognized as sfm data file
        const std::string inputExt = boost::to_lower_copy(fs::path(inputExpression).extension().string());
        static const std::array<std::string, 3> sfmSupportedExtensions = {".sfm", ".json", ".abc"};
        if(!inputExpression.empty() && std::find(sfmSupportedExtensions.begin(), sfmSupportedExtensions.end(),
                                                 inputExt) != sfmSupportedExtensions.end())
        {
            sfmData::SfMData sfmData;
            if(!sfmDataIO::Load(sfmData, inputExpression, sfmDataIO::ALL))
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

                ViewPaths.insert({view.getViewId(), view.getImage().getImagePath()});
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
                options.workingColorSpace = image::EImageColorSpace::LINEAR;
                options.rawColorInterpretation = image::ERawColorInterpretation_stringToEnum(view.getImage().getRawColorInterpretation());

                image::Image<image::RGBAfColor> image;
                image::readImage(viewPath, image, options);

                // Image color correction processing
                processColorCorrection(image, colorData, correctionMethod);

                // Save image
                saveImage(image, viewPath, outputfilePath, storageDataType);

                // Update sfmdata view for this modification
                view.getImage().setImagePath(outputfilePath);
                view.getImage().setWidth(image.Width());
                view.getImage().setHeight(image.Height());
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
                image::readImage(inputFilePath, image, image::EImageColorSpace::LINEAR);

                // Image color correction processing
                processColorCorrection(image, colorData, correctionMethod);

                // Save image
                saveImage(image, inputFilePath, outputFilePath, storageDataType);
            }
        }
    }
    else
    {
        // Copy sfmdata if it exists
        // Check if sfmInputDataFilename exist and is recognized as sfm data file
        const std::string inputExt = boost::to_lower_copy(fs::path(inputExpression).extension().string());
        static const std::array<std::string, 3> sfmSupportedExtensions = {".sfm", ".json", ".abc"};
        if(!inputExpression.empty() && std::find(sfmSupportedExtensions.begin(), sfmSupportedExtensions.end(),
                                                 inputExt) != sfmSupportedExtensions.end())
        {
            sfmData::SfMData sfmData;
            if(!sfmDataIO::Load(sfmData, inputExpression, sfmDataIO::ALL))
            {
                ALICEVISION_LOG_ERROR("The input SfMData file '" << inputExpression << "' cannot be read.");
                return EXIT_FAILURE;
            }
            const std::string sfmfilePath = (fs::path(outputPath) / fs::path(inputExpression).filename()).generic_string();
            if(!sfmDataIO::Save(sfmData, sfmfilePath, sfmDataIO::ESfMData(sfmDataIO::ALL)))
            {
                ALICEVISION_LOG_ERROR("The output SfMData file '" << sfmfilePath << "' cannot be written.");

                return EXIT_FAILURE;
            }
        }
    }

    return EXIT_SUCCESS;
}
