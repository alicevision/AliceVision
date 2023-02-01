#include <aliceVision/image/all.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/config.hpp>
#include <aliceVision/utils/regexFilter.hpp>
#include <aliceVision/sfmDataIO/viewIO.hpp>
#include <aliceVision/utils/filesIO.hpp>
#include <aliceVision/stl/mapUtils.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OPENCV)
#include <opencv2/imgproc.hpp>
#include <opencv2/photo.hpp>
#endif

#include <OpenImageIO/imageio.h>
#include <OpenImageIO/imagebuf.h>
#include <OpenImageIO/imagebufalgo.h>
#include <OpenImageIO/color.h>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 3
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

struct SharpenParams
{
    bool enabled;
    int width;
    float contrast;
    float threshold;
};

std::istream& operator>>(std::istream& in, SharpenParams& sParams)
{
    std::string token;
    in >> token;
    std::vector<std::string> splitParams;
    boost::split(splitParams, token, boost::algorithm::is_any_of(":"));
    if(splitParams.size() != 4)
        throw std::invalid_argument("Failed to parse SharpenParams from: " + token);
    sParams.enabled = boost::to_lower_copy(splitParams[0]) == "true";
    sParams.width = boost::lexical_cast<int>(splitParams[1]);
    sParams.contrast = boost::lexical_cast<float>(splitParams[2]);
    sParams.threshold = boost::lexical_cast<float>(splitParams[3]);

    return in;
}

inline std::ostream& operator<<(std::ostream& os, const SharpenParams& sParams)
{
    os << sParams.enabled << ":" << sParams.width << ":" << sParams.contrast << ":"<< sParams.threshold;
    return os;
}

struct BilateralFilterParams
{
    bool enabled;
    int distance;
    float sigmaColor;
    float sigmaSpace;
};

std::istream& operator>>(std::istream& in, BilateralFilterParams& bfParams)
{
    std::string token;
    in >> token;
    std::vector<std::string> splitParams;
    boost::split(splitParams, token, boost::algorithm::is_any_of(":"));
    if(splitParams.size() != 4)
        throw std::invalid_argument("Failed to parse BilateralFilterParams from: " + token);
    bfParams.enabled = boost::to_lower_copy(splitParams[0]) == "true";
    bfParams.distance = boost::lexical_cast<int>(splitParams[1]);
    bfParams.sigmaColor = boost::lexical_cast<float>(splitParams[2]);
    bfParams.sigmaSpace = boost::lexical_cast<float>(splitParams[3]);

    return in;
}

inline std::ostream& operator<<(std::ostream& os, const BilateralFilterParams& bfParams)
{
    os << bfParams.enabled << ":" << bfParams.distance << ":" << bfParams.sigmaColor << ":" << bfParams.sigmaSpace;
    return os;
}

struct ClaheFilterParams
{
    bool enabled;
    float clipLimit;
    int tileGridSize;
};

std::istream& operator>>(std::istream& in, ClaheFilterParams& cfParams)
{
    std::string token;
    in >> token;
    std::vector<std::string> splitParams;
    boost::split(splitParams, token, boost::algorithm::is_any_of(":"));
    if(splitParams.size() != 3)
        throw std::invalid_argument("Failed to parse ClaheFilterParams from: " + token);
    cfParams.enabled = boost::to_lower_copy(splitParams[0]) == "true";
    cfParams.clipLimit = boost::lexical_cast<float>(splitParams[1]);
    cfParams.tileGridSize = boost::lexical_cast<int>(splitParams[2]);

    return in;
}

inline std::ostream& operator<<(std::ostream& os, const ClaheFilterParams& cfParams)
{
    os << cfParams.enabled << ":" << cfParams.clipLimit << ":" << cfParams.tileGridSize;
    return os;
}

enum class ENoiseMethod { uniform, gaussian, salt };

inline std::string ENoiseMethod_enumToString(ENoiseMethod noiseMethod)
{
    switch(noiseMethod)
    {
        case ENoiseMethod::uniform: return "uniform";
        case ENoiseMethod::gaussian: return "gaussian";
        case ENoiseMethod::salt: return "salt";
    }
    throw std::invalid_argument("Invalid ENoiseMethod Enum");
}

inline ENoiseMethod ENoiseMethod_stringToEnum(std::string noiseMethod)
{
    boost::to_lower(noiseMethod);
    if(noiseMethod == "uniform") return ENoiseMethod::uniform;
    if(noiseMethod == "gaussian") return ENoiseMethod::gaussian;
    if(noiseMethod == "salt") return ENoiseMethod::salt;

    throw std::invalid_argument("Unrecognized noise method '" + noiseMethod + "'");
}

struct NoiseFilterParams
{
    bool enabled;
    ENoiseMethod method;
    float A;
    float B;
    bool mono;
};

std::istream& operator>>(std::istream& in, NoiseFilterParams& nfParams)
{
    std::string token;
    in >> token;
    std::vector<std::string> splitParams;
    boost::split(splitParams, token, boost::algorithm::is_any_of(":"));
    if(splitParams.size() != 5)
        throw std::invalid_argument("Failed to parse NoiseFilterParams from: " + token);
    nfParams.enabled = boost::to_lower_copy(splitParams[0]) == "true";
    nfParams.method = ENoiseMethod_stringToEnum(splitParams[1]);
    nfParams.A = boost::lexical_cast<float>(splitParams[2]);
    nfParams.B = boost::lexical_cast<float>(splitParams[3]);
    nfParams.mono = boost::to_lower_copy(splitParams[4]) == "true";
    return in;
}

inline std::ostream& operator<<(std::ostream& os, const NoiseFilterParams& nfParams)
{
    os << nfParams.enabled << ":" << ENoiseMethod_enumToString(nfParams.method) << ":" << nfParams.A << ":" << nfParams.B
       << ":" << nfParams.mono;
    return os;
}

enum class EImageFormat { RGBA, RGB, Grayscale };

inline std::string EImageFormat_enumToString(EImageFormat imageFormat)
{
    switch(imageFormat)
    {
        case EImageFormat::RGBA: return "rgba";
        case EImageFormat::RGB: return "rgb";
        case EImageFormat::Grayscale: return "grayscale";
    }
    throw std::invalid_argument("Invalid EImageFormat Enum");
}

inline EImageFormat EImageFormat_stringToEnum(std::string imageFormat)
{
    boost::to_lower(imageFormat);
    if(imageFormat == "rgba") return EImageFormat::RGBA;
    if(imageFormat == "rgb") return EImageFormat::RGB;
    if(imageFormat == "grayscale") return EImageFormat::Grayscale;

    throw std::invalid_argument("Unrecognized image format '" + imageFormat + "'");
}

inline std::ostream& operator<<(std::ostream& os, EImageFormat e)
{
    return os << EImageFormat_enumToString(e);
}

inline std::istream& operator>>(std::istream& in, EImageFormat& e)
{
    std::string token;
    in >> token;
    e = EImageFormat_stringToEnum(token);
    return in;
}

struct NLMeansFilterParams
{
    bool enabled;
    float filterStrength;
    float filterStrengthColor;
    int templateWindowSize;
    int searchWindowSize;
};

std::istream& operator>>(std::istream& in, NLMeansFilterParams& nlmParams)
{
    std::string token;
    in >> token;
    std::vector<std::string> splitParams;
    boost::split(splitParams, token, boost::algorithm::is_any_of(":"));
    if(splitParams.size() != 5)
        throw std::invalid_argument("Failed to parse NLMeansFilterParams from: " + token);
    nlmParams.enabled = boost::to_lower_copy(splitParams[0]) == "true";
    nlmParams.filterStrength = boost::lexical_cast<float>(splitParams[1]);
    nlmParams.filterStrengthColor = boost::lexical_cast<float>(splitParams[2]);
    nlmParams.templateWindowSize = boost::lexical_cast<int>(splitParams[3]);
    nlmParams.searchWindowSize = boost::lexical_cast<int>(splitParams[4]);

    return in;
}

inline std::ostream& operator<<(std::ostream& os, const NLMeansFilterParams& nlmParams)
{
    os << nlmParams.enabled << ":" << nlmParams.filterStrength << ":" << nlmParams.filterStrengthColor << ":"
       << nlmParams.templateWindowSize << ":" << nlmParams.searchWindowSize;
    return os;
}

std::string getColorProfileDatabaseFolder()
{
    const char* value = std::getenv("ALICEVISION_COLOR_PROFILE_DB");
    return value ? value : "";
}

struct ProcessingParams
{
    bool reconstructedViewsOnly = false;
    bool keepImageFilename = false;
    bool exposureCompensation = false;
    EImageFormat outputFormat = EImageFormat::RGBA;
    float scaleFactor = 1.0f;
    float contrast = 1.0f;
    int medianFilter = 0;
    bool fillHoles = false;
    bool fixNonFinite = false;
    bool applyDcpMetadata = false;

    SharpenParams sharpen = 
    {
        false, // enable
        3,     // width
        1.0f,  // contrast
        0.0f   // threshold
    };

    BilateralFilterParams bilateralFilter = 
    {
        false, // enable
        0,     // distance
        0.0f,  // sigmaColor
        0.0f   // sigmaSpace
    };

    ClaheFilterParams claheFilter = 
    {
        false, // enable
        4.0f,  // clipLimit
        8      // tileGridSize
    };

    NoiseFilterParams noise = {
        false, // enable
        ENoiseMethod::uniform,  // method
        0.0f, // A
        1.0f, // B
        true // mono
    };

    NLMeansFilterParams nlmFilter = 
    {
        false, // enable
        5.0f,  // filterStrength
        10.0f, // filterStrengthColor
        7,     // templateWindowSize
        21     // searchWindowSize
    };
};

void processImage(image::Image<image::RGBAfColor>& image, const ProcessingParams& pParams, const std::map<std::string, std::string>& imageMetadata)
{
    const unsigned int nchannels = 4;

    // Fix non-finite pixels
    // Note: fill holes needs to fix non-finite values first
    if(pParams.fixNonFinite || pParams.fillHoles)
    {
        oiio::ImageBuf inBuf(oiio::ImageSpec(image.Width(), image.Height(), nchannels, oiio::TypeDesc::FLOAT), image.data());
        int pixelsFixed = 0;
        // Works inplace
        oiio::ImageBufAlgo::fixNonFinite(inBuf, inBuf, oiio::ImageBufAlgo::NonFiniteFixMode::NONFINITE_BOX3, &pixelsFixed);
        ALICEVISION_LOG_INFO("Fixed " << pixelsFixed << " non-finite pixels.");
    }

    if (pParams.scaleFactor != 1.0f)
    {
        const unsigned int w = image.Width();
        const unsigned int h = image.Height();
        const unsigned int nw = (unsigned int)(floor(float(w) * pParams.scaleFactor));
        const unsigned int nh = (unsigned int)(floor(float(h) * pParams.scaleFactor));

        image::Image<image::RGBAfColor> rescaled(nw, nh);

        const oiio::ImageSpec imageSpecResized(nw, nh, nchannels, oiio::TypeDesc::FLOAT);
        const oiio::ImageSpec imageSpecOrigin(w, h, nchannels, oiio::TypeDesc::FLOAT);

        const oiio::ImageBuf inBuf(imageSpecOrigin, image.data());
        oiio::ImageBuf outBuf(imageSpecResized, rescaled.data());

        oiio::ImageBufAlgo::resize(outBuf, inBuf);

        image.swap(rescaled);
    }
    
    if (pParams.contrast != 1.0f)
    {
        image::Image<image::RGBAfColor> filtered(image.Width(), image.Height());
        const oiio::ImageBuf inBuf(oiio::ImageSpec(image.Width(), image.Height(), nchannels, oiio::TypeDesc::FLOAT), image.data());
        oiio::ImageBuf outBuf(oiio::ImageSpec(image.Width(), image.Height(), nchannels, oiio::TypeDesc::FLOAT), filtered.data());
        oiio::ImageBufAlgo::contrast_remap(outBuf, inBuf, 0.0f, 1.0f, 0.0f, 1.0f, pParams.contrast);

        image.swap(filtered);
    }
    if (pParams.medianFilter >= 3)
    {
        image::Image<image::RGBAfColor> filtered(image.Width(), image.Height());
        const oiio::ImageBuf inBuf(oiio::ImageSpec(image.Width(), image.Height(), nchannels, oiio::TypeDesc::FLOAT), image.data());
        oiio::ImageBuf outBuf(oiio::ImageSpec(image.Width(), image.Height(), nchannels, oiio::TypeDesc::FLOAT), filtered.data());
        oiio::ImageBufAlgo::median_filter(outBuf, inBuf, pParams.medianFilter);

        image.swap(filtered);
    }
    if(pParams.sharpen.enabled)
    {
        image::Image<image::RGBAfColor> filtered(image.Width(), image.Height());
        const oiio::ImageBuf inBuf(oiio::ImageSpec(image.Width(), image.Height(), nchannels, oiio::TypeDesc::FLOAT), image.data());
        oiio::ImageBuf outBuf(oiio::ImageSpec(image.Width(), image.Height(), nchannels, oiio::TypeDesc::FLOAT), filtered.data());
        oiio::ImageBufAlgo::unsharp_mask(outBuf, inBuf, "gaussian", pParams.sharpen.width, pParams.sharpen.contrast, pParams.sharpen.threshold);

        image.swap(filtered);
    }
    
    if (pParams.bilateralFilter.enabled)
    {
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OPENCV)
            // Create temporary OpenCV Mat (keep only 3 Channels) to handled Eigen data of our image
            cv::Mat openCVMatIn = image::imageRGBAToCvMatBGR(image, CV_32FC3);
            cv::Mat openCVMatOut(image.Width(), image.Height(), CV_32FC3);

            cv::bilateralFilter(openCVMatIn, openCVMatOut, pParams.bilateralFilter.distance, pParams.bilateralFilter.sigmaColor, pParams.bilateralFilter.sigmaSpace);

            // Copy filtered data from openCV Mat(3 channels) to our image(keep the alpha channel unfiltered)
            image::cvMatBGRToImageRGBA(openCVMatOut, image);
            
#else
            throw std::invalid_argument("Unsupported mode! If you intended to use a bilateral filter, please add OpenCV support.");
#endif
    }

    // Contrast Limited Adaptive Histogram Equalization
    if(pParams.claheFilter.enabled)
    {
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OPENCV)
        // Convert alicevision::image to BGR openCV Mat
        cv::Mat BGRMat = image::imageRGBAToCvMatBGR(image);

        // Convert BGR format to Lab format
        cv::Mat labImg;
        cv::cvtColor(BGRMat, labImg, cv::COLOR_LBGR2Lab);

        // Extract the L channel 
        cv::Mat L;
        cv::extractChannel(labImg, L, 0);

        // normalise L channel from [0, 100] to [0, 1]
        std::for_each(L.begin<float>(), L.end<float>(), [](float& pixel) { pixel /= 100.0; });

        // Convert float image to 16bit 
        L.convertTo(L, CV_16U, 65535.0);

        // apply Clahe algorithm to the L channel
        {
            const cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(pParams.claheFilter.clipLimit, cv::Size(pParams.claheFilter.tileGridSize, pParams.claheFilter.tileGridSize));
            clahe->apply(L, L);
        }

        // Convert 16bit image to float 
        L.convertTo(L, CV_32F, 1.0 / 65535.0);

        // normalise back L channel from [0, 1] to [0, 100]
        std::for_each(L.begin<float>(), L.end<float>(), [](float& pixel) { pixel *= 100.0; });
        
        // Merge back Lab colors channels
        cv::insertChannel(L, labImg, 0);

        // Convert Lab format to BGR format
        cv::cvtColor(labImg, BGRMat, cv::COLOR_Lab2LBGR);

        // Copy filtered data from openCV Mat to our alicevision image(keep the alpha channel unfiltered) 
        image::cvMatBGRToImageRGBA(BGRMat, image);
#else
        throw std::invalid_argument( "Unsupported mode! If you intended to use a Clahe filter, please add OpenCV support.");
#endif
    }
    if(pParams.fillHoles)
    {
        image::Image<image::RGBAfColor> filtered(image.Width(), image.Height());
        oiio::ImageBuf inBuf(oiio::ImageSpec(image.Width(), image.Height(), nchannels, oiio::TypeDesc::FLOAT), image.data());
        oiio::ImageBuf outBuf(oiio::ImageSpec(image.Width(), image.Height(), nchannels, oiio::TypeDesc::FLOAT), filtered.data());

        // Premult necessary to ensure that the fill holes works as expected
        oiio::ImageBufAlgo::premult(inBuf, inBuf);
        oiio::ImageBufAlgo::fillholes_pushpull(outBuf, inBuf);

        image.swap(filtered);
    }

    if(pParams.noise.enabled)
    {   
        oiio::ImageBuf inBuf(oiio::ImageSpec(image.Width(), image.Height(), nchannels, oiio::TypeDesc::FLOAT), image.data());
        oiio::ImageBufAlgo::noise(inBuf, ENoiseMethod_enumToString(pParams.noise.method), pParams.noise.A, pParams.noise.B, pParams.noise.mono);
    }

    if(pParams.nlmFilter.enabled)
    {
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OPENCV)
        // Create temporary OpenCV Mat (keep only 3 channels) to handle Eigen data of our image
        cv::Mat openCVMatIn = image::imageRGBAToCvMatBGR(image, CV_8UC3);
        cv::Mat openCVMatOut(image.Width(), image.Height(), CV_8UC3);

        cv::fastNlMeansDenoisingColored(openCVMatIn, openCVMatOut, pParams.nlmFilter.filterStrength,
                                        pParams.nlmFilter.filterStrengthColor, pParams.nlmFilter.templateWindowSize,
                                        pParams.nlmFilter.searchWindowSize);

        // Copy filtered data from OpenCV Mat(3 channels) to our image (keep the alpha channel unfiltered)
        image::cvMatBGRToImageRGBA(openCVMatOut, image);

#else
        throw std::invalid_argument(
            "Unsupported mode! If you intended to use a non-local means filter, please add OpenCV support.");
#endif
    }


    if (pParams.applyDcpMetadata)
    {
        bool dcpMetadataOK = map_has_non_empty_value(imageMetadata, "AliceVision:DCP:Temp1") &&
                             map_has_non_empty_value(imageMetadata, "AliceVision:DCP:Temp2") &&
                             map_has_non_empty_value(imageMetadata, "AliceVision:DCP:ForwardMatrixNumber") &&
                             map_has_non_empty_value(imageMetadata, "AliceVision:DCP:ColorMatrixNumber");

        int colorMatrixNb;
        int fwdMatrixNb;

        if (dcpMetadataOK)
        {
            colorMatrixNb = std::stoi(imageMetadata.at("AliceVision:DCP:ColorMatrixNumber"));
            fwdMatrixNb = std::stoi(imageMetadata.at("AliceVision:DCP:ForwardMatrixNumber"));

            ALICEVISION_LOG_INFO("Matrix Number : " << colorMatrixNb << " ; " << fwdMatrixNb);

            dcpMetadataOK = !((colorMatrixNb == 0) ||
                              ((colorMatrixNb > 0) && map_has_non_empty_value(imageMetadata, "AliceVision:DCP:ColorMat1")) ||
                              ((colorMatrixNb > 1) && map_has_non_empty_value(imageMetadata, "AliceVision:DCP:ColorMat2")) ||
                              ((fwdMatrixNb > 0) && map_has_non_empty_value(imageMetadata, "AliceVision:DCP:ForwardMat1")) ||
                              ((fwdMatrixNb > 1) && map_has_non_empty_value(imageMetadata, "AliceVision:DCP:ForwardMat2")));
        }

        if (!dcpMetadataOK)
        {
            ALICEVISION_THROW_ERROR("Image Processing: All required DCP metadata cannot be found.\n" << imageMetadata);
        }

        image::DCPProfile dcpProf;

        dcpProf.info.temperature_1 = std::stof(imageMetadata.at("AliceVision:DCP:Temp1"));
        dcpProf.info.temperature_2 = std::stof(imageMetadata.at("AliceVision:DCP:Temp2"));
        dcpProf.info.has_color_matrix_1 = colorMatrixNb > 0;
        dcpProf.info.has_color_matrix_2 = colorMatrixNb > 1;
        dcpProf.info.has_forward_matrix_1 = fwdMatrixNb > 0;
        dcpProf.info.has_forward_matrix_2 = fwdMatrixNb > 1;

        std::vector<std::string> v_str;

        v_str.push_back(imageMetadata.at("AliceVision:DCP:ColorMat1"));
        if (colorMatrixNb > 1)
        {
            v_str.push_back(imageMetadata.at("AliceVision:DCP:ColorMat2"));
        }
        dcpProf.setMatricesFromStrings("color", v_str);

        v_str.clear();
        if (fwdMatrixNb > 0)
        {
            v_str.push_back(imageMetadata.at("AliceVision:DCP:ForwardMat1"));
            if (fwdMatrixNb > 1)
            {
                v_str.push_back(imageMetadata.at("AliceVision:DCP:ForwardMat2"));
            }
            dcpProf.setMatricesFromStrings("forward", v_str);
        }

        std::string cam_mul = imageMetadata.at("raw:cam_mul");
        std::vector<float> v_mult;
        size_t last = 0;
        size_t next = 1;
        while ((next = cam_mul.find(",", last)) != std::string::npos)
        {
            v_mult.push_back(std::stof(cam_mul.substr(last, next - last)));
            last = next + 1;
        }
        v_mult.push_back(std::stof(cam_mul.substr(last, cam_mul.find("}", last) - last)));

        image::DCPProfile::Triple neutral;
        for (int i = 0; i < 3; i++)
        {
            neutral[i] = v_mult[1] / v_mult[i];
        }

        dcpProf.applyLinear(image, neutral, true);
    }
}

void saveImage(image::Image<image::RGBAfColor>& image, const std::string& inputPath, const std::string& outputPath, std::map<std::string, std::string> inputMetadata,
               const std::vector<std::string>& metadataFolders, const image::EImageColorSpace workingColorSpace, const EImageFormat outputFormat,
               const image::EImageColorSpace outputColorSpace, const image::EStorageDataType storageDataType)
{
    // Read metadata path
    std::string metadataFilePath;
    
    const std::string filename = fs::path(inputPath).filename().string();
    const std::string outExtension = boost::to_lower_copy(fs::path(outputPath).extension().string());
    const bool isEXR = (outExtension == ".exr");
    oiio::ParamValueList metadata;
    
    if (!inputMetadata.empty()) // If metadata are provided as input
    {
        // metadata name in "raw" domain must be updated otherwise values are discarded by oiio when writing exr format
        // we want to propagate them so we replace the domain "raw" with "AliceVision:raw"
        for (const auto & meta : inputMetadata)
        {
            if (meta.first.compare(0, 3, "raw") == 0)
            {
                metadata.add_or_replace(oiio::ParamValue("AliceVision:"+meta.first, meta.second));
            }
            else
            {
                metadata.add_or_replace(oiio::ParamValue(meta.first, meta.second));
            }
        }
    }
    else if (!metadataFolders.empty()) // If metadataFolders is specified
    {
        // The file must match the file name and extension to be used as a metadata replacement.
        const std::vector<std::string> metadataFilePaths = utils::getFilesPathsFromFolders(
            metadataFolders, [&filename](const boost::filesystem::path& path)
            {
                return path.filename().string() == filename;
            }
        );

        if(metadataFilePaths.size() > 1)
        {
            ALICEVISION_LOG_ERROR("Ambiguous case: Multiple path corresponding to this file was found for metadata replacement.");
            throw std::invalid_argument("Ambiguous case: Multiple path corresponding to this file was found for metadata replacement");
        }

        if(metadataFilePaths.empty())
        {
            ALICEVISION_LOG_WARNING("Metadata folders was specified but there is no matching for this image: "<< filename
                                    << ". The default metadata will be used instead for this image.");
            metadataFilePath = inputPath;
        }
        else
        {
            ALICEVISION_LOG_TRACE("Metadata path found for the current image: " << filename);
            metadataFilePath = metadataFilePaths[0];
        }
        metadata = image::readImageMetadata(metadataFilePath);
    }
    else
    {
        // Metadata are extracted from the original images
        metadata = image::readImageMetadata(inputPath);
    }

    image::ImageWriteOptions options;
    options.fromColorSpace(workingColorSpace);
    options.toColorSpace(outputColorSpace);

    if(isEXR)
    {
        // Select storage data type
        options.storageDataType(storageDataType);
    }

    // Save image
    ALICEVISION_LOG_TRACE("Export image: '" << outputPath << "'.");
    
    if(outputFormat == EImageFormat::Grayscale)
    {
        image::Image<float> outputImage;
        image::ConvertPixelType(image, &outputImage);
        image::writeImage(outputPath, outputImage, options, metadata);
    }
    else if(outputFormat == EImageFormat::RGB)
    {
        image::Image<image::RGBfColor> outputImage;
        image::ConvertPixelType(image, &outputImage);
        image::writeImage(outputPath, outputImage, options, metadata);
    }
    else 
    {
        // Already in RGBAf
        image::writeImage(outputPath, image, options, metadata);
    }
}

int aliceVision_main(int argc, char * argv[])
{
    std::string inputExpression;
    std::vector<std::string> inputFolders;
    std::vector<std::string> metadataFolders;
    std::string outputPath;
    EImageFormat outputFormat = EImageFormat::RGBA;
    image::EImageColorSpace workingColorSpace = image::EImageColorSpace::LINEAR;
    image::EImageColorSpace outputColorSpace = image::EImageColorSpace::LINEAR;
    image::EStorageDataType storageDataType = image::EStorageDataType::Float;
    std::string extension;
    image::ERawColorInterpretation rawColorInterpretation = image::ERawColorInterpretation::LibRawNoWhiteBalancing;
    std::string colorProfileDatabaseDirPath = "";
    bool errorOnMissingColorProfile = true;
    bool useDCPColorMatrixOnly = false;
    bool doWBAfterDemosaicing = false;
    std::string demosaicingAlgo = "AHD";
    int highlightMode = 0;

    ProcessingParams pParams;

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&inputExpression)->default_value(inputExpression),
         "SfMData file input, image filenames or regex(es) on the image file path (supported regex: '#' matches a single digit, '@' one or more digits, '?' one character and '*' zero or more).")
        ("inputFolders", po::value<std::vector<std::string>>(&inputFolders)->multitoken(),
        "Use images from specific folder(s) instead of those specify in the SfMData file.")
        ("output,o", po::value<std::string>(&outputPath)->required(),
         "Output folder or output image if a single image is given as input.")
        ;

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("metadataFolders", po::value<std::vector<std::string>>(&metadataFolders)->multitoken(),
         "Use images metadata from specific folder(s) instead of those specified in the input images.")

        ("keepImageFilename", po::value<bool>(&pParams.keepImageFilename)->default_value(pParams.keepImageFilename),
         "Use original image names instead of view names when saving.")

        ("reconstructedViewsOnly", po::value<bool>(&pParams.reconstructedViewsOnly)->default_value(pParams.reconstructedViewsOnly),
         "Process only recontructed views or all views.")

        ("fixNonFinite", po::value<bool>(&pParams.fixNonFinite)->default_value(pParams.fixNonFinite),
         "Fill non-finite pixels.")

        ("scaleFactor", po::value<float>(&pParams.scaleFactor)->default_value(pParams.scaleFactor),
         "Scale Factor (1.0: no change).")

        ("exposureCompensation", po::value<bool>(& pParams.exposureCompensation)->default_value(pParams.exposureCompensation),
         "Exposure Compensation.")

        ("contrast", po::value<float>(&pParams.contrast)->default_value(pParams.contrast),
         "Contrast Factor (1.0: no change).")
        ("medianFilter", po::value<int>(&pParams.medianFilter)->default_value(pParams.medianFilter),
         "Median Filter (0: no filter).")

        ("sharpenFilter", po::value<SharpenParams>(&pParams.sharpen)->default_value(pParams.sharpen),
            "Sharpen Filter parameters:\n"
            " * Enabled: Use Sharpen.\n"
            " * Width: Sharpen kernel width.\n"
            " * Contrast: Sharpen contrast value.\n "
            " * Threshold: Threshold for minimal variation for contrast to avoid sharpening of small noise (0.0: no noise threshold).")

        ("fillHoles", po::value<bool>(&pParams.fillHoles)->default_value(pParams.fillHoles),
         "Fill Holes.")

        ("bilateralFilter", po::value<BilateralFilterParams>(&pParams.bilateralFilter)->default_value(pParams.bilateralFilter),
            "Bilateral Filter parameters:\n"
            " * Enabled: Use bilateral Filter.\n"
            " * Distance: Diameter of each pixel neighborhood that is used during filtering (if <=0 is computed proportionaly from sigmaSpace).\n"
            " * SigmaSpace: Filter sigma in the coordinate space.\n "
            " * SigmaColor: Filter sigma in the color space.")

        ("claheFilter", po::value<ClaheFilterParams>(&pParams.claheFilter)->default_value(pParams.claheFilter),
            "Sharpen Filter parameters:\n"
            " * Enabled: Use Contrast Limited Adaptive Histogram Equalization (CLAHE).\n"
            " * ClipLimit: Sets Threshold For Contrast Limiting.\n"
            " * TileGridSize: Sets Size Of Grid For Histogram Equalization. Input Image Will Be Divided Into Equally Sized Rectangular Tiles.")

        ("noiseFilter", po::value<NoiseFilterParams>(&pParams.noise)->default_value(pParams.noise),
            "Noise Filter parameters:\n"
            " * Enabled: Add Noise.\n"
            " * method: There are several noise types to choose from:\n"
            "    - uniform: adds noise values uninformly distributed on range [A,B).\n"
            "    - gaussian: adds Gaussian (normal distribution) noise values with mean value A and standard deviation B.\n"
            "    - salt: changes to value A a portion of pixels given by B.\n"
            " * A, B: parameters that have a different interpretation depending on the method chosen.\n"
            " * mono: If is true, a single noise value will be applied to all channels otherwise a separate noise value will be computed for each channel.")

        ("nlmFilter", po::value<NLMeansFilterParams>(&pParams.nlmFilter)->default_value(pParams.nlmFilter),
            "Non local means Filter parameters:\n"
            " * Enabled: Use non local means Filter.\n"
            " * H: Parameter regulating filter strength. Bigger H value perfectly removes noise but also removes image details, smaller H value preserves details but also preserves some noise.\n"
            " * HColor: Parameter regulating filter strength for color images only. Normally same as Filtering Parameter H. Not necessary for grayscale images\n "
            " * templateWindowSize: Size in pixels of the template patch that is used to compute weights. Should be odd. \n"
            " * searchWindowSize:Size in pixels of the window that is used to compute weighted average for given pixel. Should be odd. Affect performance linearly: greater searchWindowsSize - greater denoising time.")

        ("workingColorSpace", po::value<image::EImageColorSpace>(&workingColorSpace)->default_value(workingColorSpace),
         ("Working color space: " + image::EImageColorSpace_informations()).c_str())

        ("outputFormat", po::value<EImageFormat>(&outputFormat)->default_value(outputFormat),
         "Output image format (rgba, rgb, grayscale)")

        ("outputColorSpace", po::value<image::EImageColorSpace>(&outputColorSpace)->default_value(outputColorSpace),
            ("Output color space: " + image::EImageColorSpace_informations()).c_str())

        ("rawColorInterpretation", po::value<image::ERawColorInterpretation>(&rawColorInterpretation)->default_value(rawColorInterpretation),
            ("RAW color interpretation: " + image::ERawColorInterpretation_informations() + "\ndefault : librawnowhitebalancing").c_str())

        ("applyDcpMetadata", po::value<bool>(&pParams.applyDcpMetadata)->default_value(pParams.applyDcpMetadata),
         "Apply after all processings a linear dcp profile generated from the image DCP metadata if any")

        ("colorProfileDatabase,c", po::value<std::string>(&colorProfileDatabaseDirPath)->default_value(""),
         "DNG Color Profiles (DCP) database path.")

        ("errorOnMissingColorProfile", po::value<bool>(&errorOnMissingColorProfile)->default_value(errorOnMissingColorProfile),
         "Rise an error if a DCP color profiles database is specified but no DCP file matches with the camera model (maker+name) extracted from metadata (Only for raw images)")

        ("useDCPColorMatrixOnly", po::value<bool>(&useDCPColorMatrixOnly)->default_value(useDCPColorMatrixOnly),
         "Use only Color matrices of DCP profile, ignoring Forward matrices if any.")

        ("doWBAfterDemosaicing", po::value<bool>(&doWBAfterDemosaicing)->default_value(doWBAfterDemosaicing),
         "Do not use libRaw white balancing. White balance applied just before DCP profile")

        ("demosaicingAlgo", po::value<std::string>(&demosaicingAlgo)->default_value(demosaicingAlgo),
         "Demosaicing algorithm (see libRaw documentation).")

        ("highlightMode", po::value<int>(&highlightMode)->default_value(highlightMode),
         "Highlight management (see libRaw documentation).")

        ("storageDataType", po::value<image::EStorageDataType>(&storageDataType)->default_value(storageDataType),
         ("Storage data type: " + image::EStorageDataType_informations()).c_str())

        ("extension", po::value<std::string>(&extension)->default_value(extension),
         "Output image extension (like exr, or empty to keep the source file format.")
        ;

    CmdLine cmdline("AliceVision imageProcessing");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // check user choose at least one input option
    if(inputExpression.empty() && inputFolders.empty())
    {
        ALICEVISION_LOG_ERROR("Program need at least --input or --inputFolders option." << std::endl << "No input images here.");
        return EXIT_FAILURE;
    }

#if !ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OPENCV)
    if(pParams.bilateralFilter.enabled || pParams.claheFilter.enabled || pParams.nlmFilter.enabled)
    {
        ALICEVISION_LOG_ERROR(
            "Invalid option: BilateralFilter, claheFilter and nlmFilter can't be used without openCV !");
        return EXIT_FAILURE;
    }
#endif

    if (pParams.scaleFactor < 0.0001f || pParams.scaleFactor > 1.0f)
    {
        ALICEVISION_LOG_ERROR("Invalid scale factor, it should be in range [0.0001, 1].");
        return EXIT_FAILURE;
    }

    // Check if sfmInputDataFilename exist and is recognized as sfm data file
    const std::string inputExt = boost::to_lower_copy(fs::path(inputExpression).extension().string());
    static const std::array<std::string, 2> sfmSupportedExtensions = {".sfm", ".abc"};
    if(!inputExpression.empty() && std::find(sfmSupportedExtensions.begin(), sfmSupportedExtensions.end(), inputExt) != sfmSupportedExtensions.end())
    {
        sfmData::SfMData sfmData;
        if (!sfmDataIO::Load(sfmData, inputExpression, sfmDataIO::ALL))
        {
            ALICEVISION_LOG_ERROR("The input SfMData file '" << inputExpression << "' cannot be read.");
            return EXIT_FAILURE;
        }

        // Map used to store paths of the views that need to be processed
        std::unordered_map<IndexT, std::string> ViewPaths;

        const bool checkInputFolders = !inputFolders.empty();
        // Iterate over all views
        for(const auto& viewIt : sfmData.getViews())
        {
            const sfmData::View& view = *(viewIt.second);
            // Only valid views if needed
            if(pParams.reconstructedViewsOnly && !sfmData.isPoseAndIntrinsicDefined(&view))
            {
                continue;
            }
            // if inputFolders are used
            if(checkInputFolders)
            {
                const std::vector<std::string> foundViewPaths = sfmDataIO::viewPathsFromFolders(view, inputFolders);

                // Checks if a file associated with a given view is found in the inputfolders
                if(foundViewPaths.empty())
                {
                    ALICEVISION_LOG_ERROR("Some views from SfmData cannot be found in folders passed in the parameters.\n"
                        << "Use only SfmData input, use reconstructedViewsOnly or specify the correct inputFolders.");
                    return EXIT_FAILURE;
                }
                else if(foundViewPaths.size() > 1)
                {
                    ALICEVISION_LOG_ERROR("Ambiguous case: Multiple paths found in input folders for the corresponding view '" << view.getViewId() << "'.\n"
                        << "Make sure that only one match can be found in input folders.");
                    return EXIT_FAILURE;
                }

                // Add to ViewPaths the new associated path
                ALICEVISION_LOG_TRACE("New path found for the view " << view.getViewId() << " '" << foundViewPaths[0] << "'");
                ViewPaths.insert({view.getViewId(), foundViewPaths[0]});
            }
            else
            {
                // Otherwise use the existing path
                ViewPaths.insert({view.getViewId(), view.getImagePath()});
            }
        }

        const int size = ViewPaths.size();
        int i = 0;

        for(auto& viewIt : ViewPaths)
        {
            const IndexT viewId = viewIt.first;
            const std::string viewPath = viewIt.second;
            sfmData::View& view = sfmData.getView(viewId);

            const fs::path fsPath = viewPath;
            const std::string fileName = fsPath.stem().string();
            const std::string fileExt = fsPath.extension().string();
            const std::string outputExt = extension.empty() ? fileExt : (std::string(".") + extension);
            const std::string outputfilePath = (fs::path(outputPath) / ((pParams.keepImageFilename ? fileName : std::to_string(viewId)) + outputExt)).generic_string();

            ALICEVISION_LOG_INFO(++i << "/" << size << " - Process view '" << viewId << "'.");

            image::ImageReadOptions options;
            options.workingColorSpace = workingColorSpace;           
            if (rawColorInterpretation == image::ERawColorInterpretation::Auto)
            {
                options.rawColorInterpretation = image::ERawColorInterpretation_stringToEnum(view.getRawColorInterpretation());
            }
            else
            {
                options.rawColorInterpretation = rawColorInterpretation;
            }
            options.colorProfileFileName = view.getColorProfileFileName();
            options.useDCPColorMatrixOnly = useDCPColorMatrixOnly;
            options.doWBAfterDemosaicing = doWBAfterDemosaicing;
            options.demosaicingAlgo = demosaicingAlgo;
            options.highlightMode = highlightMode;

            // Read original image
            image::Image<image::RGBAfColor> image;
            image::readImage(viewPath, image, options);

            // If exposureCompensation is needed for sfmData files
            if (pParams.exposureCompensation)
            {
                const double medianCameraExposure = sfmData.getMedianCameraExposureSetting().getExposure();
                const double cameraExposure = view.getCameraExposureSetting().getExposure();
                const double ev = std::log2(1.0 / cameraExposure);
                const float exposureCompensation = float(medianCameraExposure / cameraExposure);

                ALICEVISION_LOG_INFO("View: " << viewId << ", Ev: " << ev << ", Ev compensation: " << exposureCompensation);

                for (int i = 0; i < image.Width() * image.Height(); ++i)
                    image(i) = image(i) * exposureCompensation;
            }

            // Image processing
            processImage(image, pParams, view.getMetadata());

            // Save the image
            saveImage(image, viewPath, outputfilePath, view.getMetadata(), metadataFolders, workingColorSpace, outputFormat, outputColorSpace, storageDataType);

            // Update view for this modification
            view.setImagePath(outputfilePath);
            view.setWidth(image.Width());
            view.setHeight(image.Height());
        }

        if (pParams.scaleFactor != 1.0f)
        {
            for (auto & i : sfmData.getIntrinsics())
            {
                i.second->rescale(pParams.scaleFactor);
            }
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
        const fs::path inputPath(inputExpression);
        std::vector<std::string> filesStrPaths;

        // If sfmInputDataFilename is empty use imageFolders instead
        if(inputExpression.empty())
        {
            // Get supported files
            filesStrPaths = utils::getFilesPathsFromFolders(inputFolders, [](const boost::filesystem::path& path) {
                return image::isSupported(path.extension().string());
            });
        }
        else
        {
            // If you try to use both a regex-like filter expression and folders as input
            if(!inputFolders.empty())
            {
                ALICEVISION_LOG_WARNING("InputFolders and filter expression cannot be used at the same time, InputFolders are ignored here.");
            }

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

        image::DCPDatabase dcpDatabase;
        int i = 0;
        for (const std::string& inputFilePath : filesStrPaths)
        {
            const fs::path path = fs::path(inputFilePath);
            const std::string filename = path.stem().string();
            const std::string fileExt = path.extension().string();
            const std::string outputExt = extension.empty() ? fileExt : (std::string(".") + extension);

            ALICEVISION_LOG_INFO(++i << "/" << size << " - Process image '" << filename << fileExt << "'.");

            const std::string userExt = fs::path(outputPath).extension().string();
            std::string outputFilePath;

            if ((size == 1) && !userExt.empty())
            {
                if (image::isSupported(userExt))
                {
                    outputFilePath = fs::path(outputPath).generic_string();
                }
                else
                {
                    outputFilePath = (fs::path(outputPath).parent_path() / (filename + outputExt)).generic_string();
                    ALICEVISION_LOG_WARNING("Extension " << userExt << " is not supported! Output image saved in " << outputFilePath);
                }    
            }
            else
            {
                outputFilePath = (fs::path(outputPath) / (filename + outputExt)).generic_string();
            }

            image::DCPProfile dcpProf;
            sfmData::View view; // used to extract and complete metadata

            if (rawColorInterpretation == image::ERawColorInterpretation::DcpLinearProcessing ||
                rawColorInterpretation == image::ERawColorInterpretation::DcpMetadata)
            {
                // Load DCP color profiles database if not already loaded
                dcpDatabase.load(colorProfileDatabaseDirPath.empty() ? getColorProfileDatabaseFolder() : colorProfileDatabaseDirPath, false);

                // Get DSLR maker and model by creating a view and picking values up in it.
                view.setImagePath(inputFilePath);
                int width, height;
                const auto metadata = image::readImageMetadata(inputFilePath, width, height);
                view.setMetadata(image::getMapFromMetadata(metadata));

                const std::string& make = view.getMetadataMake();
                const std::string& model = view.getMetadataModel();

                // Get DCP profile
                if (!dcpDatabase.retrieveDcpForCamera(make, model, dcpProf))
                {
                    if (errorOnMissingColorProfile)
                    {
                        ALICEVISION_LOG_ERROR("The specified DCP database does not contain an appropriate profil for DSLR " << make << " " << model);
                        return EXIT_FAILURE;
                    }
                    else
                    {
                        ALICEVISION_LOG_WARNING("Can't find color profile for input image " << inputFilePath);
                    }
                }

                // Add color profile info in metadata
                view.addDCPMetadata(dcpProf);
            }

            // set readOptions
            image::ImageReadOptions readOptions;
            readOptions.colorProfileFileName = dcpProf.info.filename;
            if (dcpProf.info.filename.empty() &&
                ((rawColorInterpretation == image::ERawColorInterpretation::DcpLinearProcessing) ||
                 (rawColorInterpretation == image::ERawColorInterpretation::DcpMetadata)))
            {
                // Fallback case of missing profile but no error requested
                readOptions.rawColorInterpretation = image::ERawColorInterpretation::LibRawNoWhiteBalancing;
            }
            else
            {
                readOptions.rawColorInterpretation = rawColorInterpretation;
            }
            readOptions.workingColorSpace = workingColorSpace;
            readOptions.useDCPColorMatrixOnly = useDCPColorMatrixOnly;
            readOptions.doWBAfterDemosaicing = doWBAfterDemosaicing;
            readOptions.demosaicingAlgo = demosaicingAlgo;
            readOptions.highlightMode = highlightMode;

            // Read original image
            image::Image<image::RGBAfColor> image;
            image::readImage(inputFilePath, image, readOptions);

            std::map<std::string,std::string> metadata = view.getMetadata();

            // Image processing
            processImage(image, pParams,metadata);

            // Save the image
            saveImage(image, inputFilePath, outputFilePath, metadata, metadataFolders, workingColorSpace, outputFormat, outputColorSpace, storageDataType);
        }
    }

    return 0;
}
