// This file is part of the AliceVision project.
// Copyright (c) 2020 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/image/all.hpp>
#include <aliceVision/alicevision_omp.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/sfmDataIO/viewIO.hpp>
#include <aliceVision/sensorDB/parseDatabase.hpp>
#include <aliceVision/config.hpp>
#include <aliceVision/utils/regexFilter.hpp>
#include <aliceVision/utils/filesIO.hpp>
#include <aliceVision/stl/mapUtils.hpp>
#include <aliceVision/lensCorrectionProfile/lcp.hpp>
#include <aliceVision/camera/cameraUndistortImage.hpp>
#include <aliceVision/camera/IntrinsicScaleOffset.hpp>
#include <aliceVision/system/Parallelization.hpp>

#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>

#include <aliceVision/imageProcessing/imageProcessing.hpp>
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OPENCV)
    #include <aliceVision/imageProcessing/imageProcessing_OpenCV.hpp>
#endif

#include <OpenImageIO/imageio.h>
#include <OpenImageIO/imagebuf.h>
#include <OpenImageIO/imagebufalgo.h>
#include <OpenImageIO/color.h>

#include <filesystem>
#include <string>
#include <cmath>
#include <vector>
#include <map>
#include <iostream>
#include <algorithm>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 4
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;
using namespace aliceVision::imageProcessing;

namespace po = boost::program_options;
namespace fs = std::filesystem;


// Command line parameters configuration

struct LensCorrectionParams
{
    bool enabled = false;
    bool geometry = false;
    bool vignetting = false;
    bool chromaticAberration = false;

    std::vector<float> gParams;
    std::vector<float> vParams;
    std::vector<float> caGParams;
    std::vector<float> caRGParams;
    std::vector<float> caBGParams;

    RectilinearModel geometryModel = RectilinearModel();

    RectilinearModel caGModel = RectilinearModel();
    RectilinearModel caBGModel = RectilinearModel();
    RectilinearModel caRGModel = RectilinearModel();
};

std::istream& operator>>(std::istream& in, LensCorrectionParams& lcParams)
{
    std::string token;
    in >> token;
    std::vector<std::string> splitParams;
    boost::split(splitParams, token, boost::algorithm::is_any_of(":"));
    if (splitParams.size() != 4)
    {
        throw std::invalid_argument("Failed to parse LensCorrectionParams from: " + token);
    }

    lcParams.enabled = boost::to_lower_copy(splitParams[0]) == "true";
    lcParams.geometry = boost::to_lower_copy(splitParams[1]) == "true";
    lcParams.vignetting = boost::to_lower_copy(splitParams[2]) == "true";
    lcParams.chromaticAberration = boost::to_lower_copy(splitParams[3]) == "true";

    return in;
}

inline std::ostream& operator<<(std::ostream& os, const LensCorrectionParams& lcParams)
{
    os << lcParams.enabled << ":" << lcParams.geometry << ":" << lcParams.vignetting << ":" << lcParams.chromaticAberration;
    return os;
}

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
    if (splitParams.size() != 4)
    {
        throw std::invalid_argument("Failed to parse SharpenParams from: " + token);
    }
    
    sParams.enabled = boost::to_lower_copy(splitParams[0]) == "true";
    sParams.width = boost::lexical_cast<int>(splitParams[1]);
    sParams.contrast = boost::lexical_cast<float>(splitParams[2]);
    sParams.threshold = boost::lexical_cast<float>(splitParams[3]);

    return in;
}

inline std::ostream& operator<<(std::ostream& os, const SharpenParams& sParams)
{
    os << sParams.enabled << ":" << sParams.width << ":" << sParams.contrast << ":" << sParams.threshold;
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
    if (splitParams.size() != 4)
    {
        throw std::invalid_argument("Failed to parse BilateralFilterParams from: " + token);
    }

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
    if (splitParams.size() != 3)
    {
        throw std::invalid_argument("Failed to parse ClaheFilterParams from: " + token);
    }

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

enum class ENoiseMethod
{
    uniform,
    gaussian,
    salt
};


inline std::string ENoiseMethod_enumToString(ENoiseMethod noiseMethod)
{
    switch (noiseMethod)
    {
        case ENoiseMethod::uniform:
            return "uniform";
        case ENoiseMethod::gaussian:
            return "gaussian";
        case ENoiseMethod::salt:
            return "salt";
    }
    throw std::invalid_argument("Invalid ENoiseMethod Enum");
}

inline ENoiseMethod ENoiseMethod_stringToEnum(std::string noiseMethod)
{
    boost::to_lower(noiseMethod);
    if (noiseMethod == "uniform")
        return ENoiseMethod::uniform;
    if (noiseMethod == "gaussian")
        return ENoiseMethod::gaussian;
    if (noiseMethod == "salt")
        return ENoiseMethod::salt;

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
    if (splitParams.size() != 5)
    {
        throw std::invalid_argument("Failed to parse NoiseFilterParams from: " + token);
    }

    nfParams.enabled = boost::to_lower_copy(splitParams[0]) == "true";
    nfParams.method = ENoiseMethod_stringToEnum(splitParams[1]);
    nfParams.A = boost::lexical_cast<float>(splitParams[2]);
    nfParams.B = boost::lexical_cast<float>(splitParams[3]);
    nfParams.mono = boost::to_lower_copy(splitParams[4]) == "true";
    
    return in;
}

inline std::ostream& operator<<(std::ostream& os, const NoiseFilterParams& nfParams)
{
    os << nfParams.enabled << ":" << ENoiseMethod_enumToString(nfParams.method) << ":" << nfParams.A << ":" << nfParams.B << ":" << nfParams.mono;
    return os;
}

enum class EImageFormat
{
    RGBA,
    RGB,
    Grayscale
};

inline std::string EImageFormat_enumToString(EImageFormat imageFormat)
{
    switch (imageFormat)
    {
        case EImageFormat::RGBA:
            return "rgba";
        case EImageFormat::RGB:
            return "rgb";
        case EImageFormat::Grayscale:
            return "grayscale";
    }
    throw std::invalid_argument("Invalid EImageFormat Enum");
}

inline EImageFormat EImageFormat_stringToEnum(std::string imageFormat)
{
    boost::to_lower(imageFormat);
    if (imageFormat == "rgba")
        return EImageFormat::RGBA;
    if (imageFormat == "rgb")
        return EImageFormat::RGB;
    if (imageFormat == "grayscale")
        return EImageFormat::Grayscale;

    throw std::invalid_argument("Unrecognized image format '" + imageFormat + "'");
}

inline std::ostream& operator<<(std::ostream& os, EImageFormat e) { return os << EImageFormat_enumToString(e); }

inline std::istream& operator>>(std::istream& in, EImageFormat& e)
{
    std::string token(std::istreambuf_iterator<char>(in), {});
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
    if (splitParams.size() != 5)
    {
        throw std::invalid_argument("Failed to parse NLMeansFilterParams from: " + token);
    }

    nlmParams.enabled = boost::to_lower_copy(splitParams[0]) == "true";
    nlmParams.filterStrength = boost::lexical_cast<float>(splitParams[1]);
    nlmParams.filterStrengthColor = boost::lexical_cast<float>(splitParams[2]);
    nlmParams.templateWindowSize = boost::lexical_cast<int>(splitParams[3]);
    nlmParams.searchWindowSize = boost::lexical_cast<int>(splitParams[4]);

    return in;
}

inline std::ostream& operator<<(std::ostream& os, const NLMeansFilterParams& nlmParams)
{
    os << nlmParams.enabled << ":" 
        << nlmParams.filterStrength << ":" 
        << nlmParams.filterStrengthColor << ":" 
        << nlmParams.templateWindowSize << ":"
        << nlmParams.searchWindowSize;

    return os;
}

struct pixelAspectRatioParams
{
    bool enabled;
    bool rowDecimation;
    float value;
};

std::istream& operator>>(std::istream& in, pixelAspectRatioParams& parParams)
{
    std::string token;
    in >> token;
    std::vector<std::string> splitParams;
    boost::split(splitParams, token, boost::algorithm::is_any_of(":"));

    if (splitParams.size() != 2)
    {
        throw std::invalid_argument("Failed to parse pixelAspectRatioParams from: " + token);
    }

    parParams.enabled = boost::to_lower_copy(splitParams[0]) == "true";
    parParams.rowDecimation = boost::to_lower_copy(splitParams[1]) == "true";

    return in;
}

inline std::ostream& operator<<(std::ostream& os, const pixelAspectRatioParams& parParams)
{
    os << parParams.enabled << ":" << parParams.rowDecimation;
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
    bool rawAutoBright = false;
    float rawExposureAdjust = 0.0;
    EImageFormat outputFormat = EImageFormat::RGBA;
    float scaleFactor = 1.0f;
    unsigned int maxWidth = 0;
    unsigned int maxHeight = 0;
    float contrast = 1.0f;
    int medianFilter = 0;
    bool fillHoles = false;
    bool fixNonFinite = false;
    bool applyDcpMetadata = false;
    bool useDCPColorMatrixOnly = false;
    bool enableColorTempProcessing = false;
    double correlatedColorTemperature = -1.0;
    bool reorient = false;

    LensCorrectionParams lensCorrection = {
      false,  // enable
      false,  // geometry
      false,  // vignetting
      false   // chromatic aberration
    };

    SharpenParams sharpen = {
      false,  // enable
      3,      // width
      1.0f,   // contrast
      0.0f    // threshold
    };

    BilateralFilterParams bilateralFilter = {
      false,  // enable
      0,      // distance
      0.0f,   // sigmaColor
      0.0f    // sigmaSpace
    };

    ClaheFilterParams claheFilter = {
      false,  // enable
      4.0f,   // clipLimit
      8       // tileGridSize
    };

    NoiseFilterParams noise = {
      false,                  // enable
      ENoiseMethod::uniform,  // method
      0.0f,                   // A
      1.0f,                   // B
      true                    // mono
    };

    NLMeansFilterParams nlmFilter = {
      false,  // enable
      5.0f,   // filterStrength
      10.0f,  // filterStrengthColor
      7,      // templateWindowSize
      21      // searchWindowSize
    };

    pixelAspectRatioParams par = {
      false,  // enable
      false  // rowDecimation
    };
};


void setupSteps(std::list<std::unique_ptr<ImageProcess>> & steps, const ProcessingParams& pParams)
{
    steps.clear();

    if (pParams.exposureCompensation)
    {
        steps.push_back(std::make_unique<ExposureProcess>());
    }

    if (pParams.fixNonFinite || pParams.fillHoles)
    {
        steps.push_back(std::make_unique<FixHolesProcess>());
    }

    if (pParams.lensCorrection.enabled)
    {
        if (pParams.lensCorrection.vignetting)
        {
            steps.push_back(std::make_unique<RemoveVignettingProcess>());
        }

        if (pParams.lensCorrection.chromaticAberration)
        {
            steps.push_back(std::make_unique<UndistortChromaticAberrationsProcess>(false));
        }

        if (pParams.lensCorrection.geometry)
        {
            steps.push_back(std::make_unique<UndistortProcess>());
        }
    }

    if (pParams.maxWidth != 0 || pParams.maxHeight != 0 ||  pParams.scaleFactor != 1.0f || pParams.par.enabled)
    {
        steps.push_back(std::make_unique<ResizeProcess>(pParams.maxWidth, pParams.maxHeight, pParams.scaleFactor, pParams.par.enabled, pParams.par.rowDecimation));
    }

    if (pParams.reorient)
    {
        steps.push_back(std::make_unique<ReorientProcess>());
    }
    if (pParams.contrast != 1.0f)
    {
        steps.push_back(std::make_unique<ContrastProcess>(pParams.contrast));
    }
    if (pParams.medianFilter >= 3)
    {
        steps.push_back(std::make_unique<MedianFilterProcess>(pParams.medianFilter));
    }
    if (pParams.sharpen.enabled)
    {
        steps.push_back(std::make_unique<SharpenProcess>(pParams.sharpen.width, pParams.sharpen.contrast, pParams.sharpen.threshold));
    }
    if (pParams.bilateralFilter.enabled)
    {
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OPENCV)
        steps.push_back(std::make_unique<BilateralFilterProcess>(pParams.bilateralFilter.distance, pParams.bilateralFilter.sigmaColor, pParams.bilateralFilter.sigmaSpace));
#else
        ALICEVISION_LOG_ERROR("OpenCV support is not enabled in this AliceVision build. Bilateral filter processing is unavailable and will be bypassed.");
#endif
    }
    if (pParams.claheFilter.enabled)
    {
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OPENCV)
        steps.push_back(std::make_unique<ClaheFilterProcess>(pParams.claheFilter.tileGridSize, pParams.claheFilter.clipLimit));
#else
        ALICEVISION_LOG_ERROR("OpenCV support is not enabled in this AliceVision build. Clahe filter processing is unavailable and will be bypassed.");
#endif
    }
    if (pParams.fillHoles)
    {
        steps.push_back(std::make_unique<FillHolesProcess>());
    }
    if (pParams.noise.enabled)
    {
        steps.push_back(std::make_unique<NoiseProcess>(ENoiseMethod_enumToString(pParams.noise.method), pParams.noise.A, pParams.noise.B, pParams.noise.mono));
    }
    if (pParams.nlmFilter.enabled)
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OPENCV)
    {
        steps.push_back(std::make_unique<NlmFilterProcess>(pParams.nlmFilter.filterStrength, pParams.nlmFilter.filterStrengthColor, pParams.nlmFilter.templateWindowSize, pParams.nlmFilter.searchWindowSize));
    }
#else
        ALICEVISION_LOG_ERROR("OpenCV support is not enabled in this AliceVision build. Nlm filter processing is unavailable and will be bypassed.");
#endif
    if (pParams.applyDcpMetadata || pParams.enableColorTempProcessing)
    {
        steps.push_back(std::make_unique<ColorTemperatureProcess>(pParams.applyDcpMetadata, pParams.useDCPColorMatrixOnly, pParams.enableColorTempProcessing, pParams.correlatedColorTemperature));
    }
}



void saveImage(image::Image<image::RGBAfColor>& image,
               const std::string& inputPath,
               const std::string& outputPath,
               std::map<std::string, std::string> inputMetadata,
               const std::vector<std::string>& metadataFolders,
               const EImageFormat outputFormat,
               const image::ImageWriteOptions options)
{
    // Read metadata path
    std::string metadataFilePath;

    const std::string filename = fs::path(inputPath).filename().string();
    const std::string outExtension = boost::to_lower_copy(fs::path(outputPath).extension().string());
    const bool isEXR = (outExtension == ".exr");
    oiio::ParamValueList metadata;

    if (!inputMetadata.empty())  // If metadata are provided as input
    {
        // metadata name in "raw" domain must be updated otherwise values are discarded by oiio when writing exr format
        // we want to propagate them so we replace the domain "raw" with "AliceVision:raw"
        for (const auto& meta : inputMetadata)
        {
            if (meta.first.compare(0, 3, "raw") == 0)
            {
                metadata.add_or_replace(oiio::ParamValue("AliceVision:" + meta.first, meta.second));
            }
            else
            {
                metadata.add_or_replace(oiio::ParamValue(meta.first, meta.second));
            }
        }
    }
    else if (!metadataFolders.empty())  // If metadataFolders is specified
    {
        // The file must match the file name and extension to be used as a metadata replacement.
        const std::vector<std::string> metadataFilePaths =
          utils::getFilesPathsFromFolders(metadataFolders, [&filename](const fs::path& path) { return path.filename().string() == filename; });

        if (metadataFilePaths.size() > 1)
        {
            ALICEVISION_LOG_ERROR("Ambiguous case: Multiple path corresponding to this file was found for metadata replacement.");
            throw std::invalid_argument("Ambiguous case: Multiple path corresponding to this file was found for metadata replacement");
        }

        if (metadataFilePaths.empty())
        {
            ALICEVISION_LOG_WARNING("Metadata folders was specified but there is no matching for this image: "
                                    << filename << ". The default metadata will be used instead for this image.");
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

    // Save image
    ALICEVISION_LOG_TRACE("Export image: '" << outputPath << "'.");

    if (outputFormat == EImageFormat::Grayscale)
    {
        image::Image<float> outputImage;
        image::ConvertPixelType(image, &outputImage);
        image::writeImage(outputPath, outputImage, options, metadata);
    }
    else if (outputFormat == EImageFormat::RGB)
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

int aliceVision_main(int argc, char* argv[])
{
    std::string inputSfmDataPath;
    std::vector<std::string> metadataFolders;
    std::string outputPath;
    EImageFormat outputFormat = EImageFormat::RGBA;
    image::EImageColorSpace inputColorSpace = image::EImageColorSpace::AUTO;
    image::EImageColorSpace workingColorSpace = image::EImageColorSpace::LINEAR;
    image::EImageColorSpace outputColorSpace = image::EImageColorSpace::LINEAR;
    image::EStorageDataType storageDataType = image::EStorageDataType::Float;
    image::EImageExrCompression exrCompressionMethod = image::EImageExrCompression::Auto;
    int exrCompressionLevel = 0;
    bool jpegCompress = true;
    int jpegQuality = 90;
    std::string extension;
    image::ERawColorInterpretation rawColorInterpretation = image::ERawColorInterpretation::DcpLinearProcessing;
    std::string colorProfileDatabaseDirPath = "";
    bool errorOnMissingColorProfile = true;
    bool useDCPColorMatrixOnly = true;
    bool doWBAfterDemosaicing = false;
    std::string demosaicingAlgo = "AHD";
    int highlightMode = 0;
    double correlatedColorTemperature = -1;
    std::string lensCorrectionProfileInfo;
    bool lensCorrectionProfileSearchIgnoreCameraModel = true;
    std::string sensorDatabasePath;
    int rangeIteration = 0;
    int rangeBlocksCount = 1;

    ProcessingParams pParams;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&inputSfmDataPath)->default_value(inputSfmDataPath), "SfMData file input")
        ("output,o", po::value<std::string>(&outputPath)->required(), "Output folder or output image if a single image is given as input.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("metadataFolders", po::value<std::vector<std::string>>(&metadataFolders)->multitoken(),
         "Use images metadata from specific folder(s) instead of those specified in the input images.")

        ("keepImageFilename", po::value<bool>(&pParams.keepImageFilename)->default_value(pParams.keepImageFilename),
         "Use original image names instead of view names when saving.")

        ("reconstructedViewsOnly", po::value<bool>(&pParams.reconstructedViewsOnly)->default_value(pParams.reconstructedViewsOnly),
         "Process only reconstructed views or all views.")

        ("fixNonFinite", po::value<bool>(&pParams.fixNonFinite)->default_value(pParams.fixNonFinite),
         "Fill non-finite pixels.")

        ("scaleFactor", po::value<float>(&pParams.scaleFactor)->default_value(pParams.scaleFactor),
         "Scale Factor (1.0: no change).")

        ("maxWidth", po::value<unsigned int>(&pParams.maxWidth)->default_value(pParams.maxWidth),
         "Max width (0: no change).")

        ("maxHeight", po::value<unsigned int>(&pParams.maxHeight)->default_value(pParams.maxHeight),
         "Max height (0: no change).")

        ("exposureCompensation", po::value<bool>(&pParams.exposureCompensation)->default_value(pParams.exposureCompensation),
         "Exposure Compensation. Valid only if a sfmdata is set as input.")

        ("rawExposureAdjust", po::value<float>(&pParams.rawExposureAdjust)->default_value(pParams.rawExposureAdjust),
         "Exposure Adjustment in fstops limited to the range from -2 to +3 fstops.")

        ("rawAutoBright", po::value<bool>(&pParams.rawAutoBright)->default_value(pParams.rawAutoBright),
         "Enable automatic exposure adjustment for RAW images.")

        ("lensCorrection", po::value<LensCorrectionParams>(&pParams.lensCorrection)->default_value(pParams.lensCorrection),
         "Lens Correction parameters:\n"
         " * Enabled: Use automatic lens correction.\n"
         " * Geometry: For geometry if a model is available in SfM data.\n"
         " * Vignetting: For vignetting if model parameters is available in metadata.\n "
         " * Chromatic Aberration: For chromatic aberration (fringing) if model parameters is available in metadata.")

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
         " * Enabled: Use bilateral filter.\n"
         " * Distance: Diameter of each pixel neighborhood that is used during filtering (if <= 0, it is computed "
         "proportionally from sigmaSpace).\n"
         " * SigmaSpace: Filter sigma in the coordinate space.\n "
         " * SigmaColor: Filter sigma in the color space.")

        ("claheFilter", po::value<ClaheFilterParams>(&pParams.claheFilter)->default_value(pParams.claheFilter),
         "Sharpen Filter parameters:\n"
         " * Enabled: Use Contrast Limited Adaptive Histogram Equalization (CLAHE).\n"
         " * ClipLimit: Sets threshold for contrast limiting.\n"
         " * TileGridSize: Sets size of grid for histogram equalization. Input image will be divided into equally "
         "sized rectangular tiles.")

        ("noiseFilter", po::value<NoiseFilterParams>(&pParams.noise)->default_value(pParams.noise),
         "Noise Filter parameters:\n"
         " * Enabled: Add noise.\n"
         " * method: There are several noise types to choose from:\n"
         "    - uniform: adds noise values uninformly distributed on range [A,B).\n"
         "    - gaussian: adds Gaussian (normal distribution) noise values with mean value A and standard deviation B.\n"
         "    - salt: changes to value A a portion of pixels given by B.\n"
         " * A, B: parameters that have a different interpretation depending on the method chosen.\n"
         " * mono: If is true, a single noise value will be applied to all channels otherwise a separate noise value "
         "will be computed for each channel.")

        ("nlmFilter", po::value<NLMeansFilterParams>(&pParams.nlmFilter)->default_value(pParams.nlmFilter),
         "Non-local means filter parameters:\n"
         " * Enabled: Use non-local means filter.\n"
         " * H: Parameter regulating filter strength. Bigger H value perfectly removes noise but also removes image "
         "details, smaller H value preserves details but also preserves some noise.\n"
         " * HColor: Parameter regulating filter strength for color images only. Normally same as Filtering "
         "Parameter H. Not necessary for grayscale images.\n "
         " * templateWindowSize: Size in pixels of the template patch that is used to compute weights. Should be odd.\n"
         " * searchWindowSize: Size in pixels of the window that is used to compute weighted average for a given pixel. "
         "Should be odd. Affects performance linearly: greater searchWindowsSize - greater denoising time.")

        ("parFilter", po::value<pixelAspectRatioParams>(&pParams.par)->default_value(pParams.par),
         "Pixel Aspect Ratio parameters:\n"
         " * Enabled: Apply pixel aspect ratio.\n"
         " * RowDecimation: Decimate rows (reduce image height) instead of upsampling columns (increase image width).")

        ("inputColorSpace", po::value<image::EImageColorSpace>(&inputColorSpace)->default_value(inputColorSpace),
         ("Input image color space: " + image::EImageColorSpace_informations()).c_str())

        ("workingColorSpace", po::value<image::EImageColorSpace>(&workingColorSpace)->default_value(workingColorSpace),
         ("Working color space: " + image::EImageColorSpace_informations()).c_str())

        ("outputFormat", po::value<EImageFormat>(&outputFormat)->default_value(outputFormat),
         "Output image format (rgba, rgb, grayscale).")

        ("outputColorSpace", po::value<image::EImageColorSpace>(&outputColorSpace)->default_value(outputColorSpace),
         ("Output color space: " + image::EImageColorSpace_informations()).c_str())

        ("rawColorInterpretation", po::value<image::ERawColorInterpretation>(&rawColorInterpretation)->default_value(rawColorInterpretation),
         ("RAW color interpretation: " + image::ERawColorInterpretation_informations() + "\n"
         "Default: DcpLinearProcessing").c_str())

        ("applyDcpMetadata", po::value<bool>(&pParams.applyDcpMetadata)->default_value(pParams.applyDcpMetadata),
         "Apply after all processings a linear DCP profile generated from the image DCP metadata if any.")

        ("colorProfileDatabase,c", po::value<std::string>(&colorProfileDatabaseDirPath)->default_value(""),
         "DNG Color Profiles (DCP) database path.")

        ("errorOnMissingColorProfile", po::value<bool>(&errorOnMissingColorProfile)->default_value(errorOnMissingColorProfile),
         "Rise an error if a DCP color profiles database is specified but no DCP file matches with the camera model "
         "(maker + name) extracted from metadata (only for RAW images).")

        ("useDCPColorMatrixOnly", po::value<bool>(&useDCPColorMatrixOnly)->default_value(useDCPColorMatrixOnly),
         "Use only color matrices of DCP profile, ignoring forward matrices if any. Default: False.\n"
         "In case white balancing has been done before demosaicing, the reverse operation is done before applying "
         "the color matrix.")

        ("doWBAfterDemosaicing", po::value<bool>(&doWBAfterDemosaicing)->default_value(doWBAfterDemosaicing),
         "Do not use libRaw white balancing. White balancing is applied just before DCP profile if "
         "useDCPColorMatrixOnly is set to False. Default: False.")

        ("demosaicingAlgo", po::value<std::string>(&demosaicingAlgo)->default_value(demosaicingAlgo),
         "Demosaicing algorithm (see libRaw documentation).\n"
         "Possible algos are: linear, VNG, PPG, AHD (default), DCB, AHD-Mod, AFD, VCD, Mixed, LMMSE, AMaZE, DHT, AAHD, none.")

        ("highlightMode", po::value<int>(&highlightMode)->default_value(highlightMode),
         "Highlight management (see libRaw documentation).\n"
         "0 = clip (default), 1 = unclip, 2 = blend, 3+ = rebuild.")
            
        ("lensCorrectionProfileInfo", po::value<std::string>(&lensCorrectionProfileInfo)->default_value(""),
         "Lens Correction Profile filepath or database directory path.")
            
        ("lensCorrectionProfileSearchIgnoreCameraModel", po::value<bool>(&lensCorrectionProfileSearchIgnoreCameraModel)->default_value(lensCorrectionProfileSearchIgnoreCameraModel),
         "Automatic LCP Search considers only the camera maker and the lens name.")

        ("correlatedColorTemperature", po::value<double>(&correlatedColorTemperature)->default_value(correlatedColorTemperature),
         "Correlated Color Temperature in Kelvin of scene illuminant.\n"
         "If less than or equal to 0.0, the value extracted from the metadata will be used.")

        ("sensorDatabase,s", po::value<std::string>(&sensorDatabasePath)->default_value(""),
         "Camera sensor width database path.")

        ("reorient", po::value<bool>(&pParams.reorient)->default_value(pParams.reorient),
         "Enable automatic reorientation of images.")

        ("storageDataType", po::value<image::EStorageDataType>(&storageDataType)->default_value(storageDataType),
         ("Storage data type: " + image::EStorageDataType_informations()).c_str())

        ("exrCompressionMethod", po::value<image::EImageExrCompression>(&exrCompressionMethod)->default_value(exrCompressionMethod),
         ("Compression method for EXR images: " + image::EImageExrCompression_informations()).c_str())

        ("exrCompressionLevel", po::value<int>(&exrCompressionLevel)->default_value(exrCompressionLevel),
         "Compression Level for EXR images.\n"
         "Only dwaa, dwab, zip and zips compression methods are concerned.\n"
         "dwaa/dwab: value must be strictly positive.\n"
         "zip/zips: value must be between 1 and 9.")
        
        ("jpegCompress", po::value<bool>(&jpegCompress)->default_value(jpegCompress),
         "Compress JPEG images.")
        
        ("jpegQuality", po::value<int>(&jpegQuality)->default_value(jpegQuality),
         "JPEG quality after compression (between 0 and 100).")

        ("extension", po::value<std::string>(&extension)->default_value(extension),
         "Output image extension (like exr, or empty to keep the source file format.")
         
        ("rangeIteration", po::value<int>(&rangeIteration)->default_value(rangeIteration), "Range current iteration id.")
        ("rangeBlocksCount", po::value<int>(&rangeBlocksCount)->default_value(rangeBlocksCount), "Range blocks count.");
    // clang-format on

    CmdLine cmdline("AliceVision imageProcessing");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);

    // Retrieve command line parameters
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }


    // Check that the user does not ask for unavailable features
    #if !ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OPENCV)
    if (pParams.bilateralFilter.enabled || pParams.claheFilter.enabled || pParams.nlmFilter.enabled)
    {
        ALICEVISION_LOG_ERROR("Invalid option: BilateralFilter, claheFilter and nlmFilter can't be used without openCV !");
        return EXIT_FAILURE;
    }
    #endif

    // Check parameters numeric validity
    if (pParams.scaleFactor < 0.0001f || pParams.scaleFactor > 1.0f)
    {
        ALICEVISION_LOG_ERROR("Invalid scale factor, it should be in range [0.0001, 1].");
        return EXIT_FAILURE;
    }

    // Open input sfmData
    sfmData::SfMData sfmData;
    if (!sfmDataIO::load(sfmData, inputSfmDataPath, sfmDataIO::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << inputSfmDataPath << "' cannot be read.");
        return EXIT_FAILURE;
    }

    // Prepare processing steps
    std::list<std::unique_ptr<ImageProcess>> steps;
    setupSteps(steps, pParams);

    // Map used to store paths of the views that need to be processed
    std::vector<IndexT> selectedViews;

    // Build list of valid views
    for (const auto& [viewId, view] : sfmData.getViews().valueRange())
    {   
        // Only valid views if needed
        if (pParams.reconstructedViewsOnly && !sfmData.isPoseAndIntrinsicDefined(view))
        {
            continue;
        }

        selectedViews.push_back(view.getViewId());
    }
    

    // Estimate current items to process
    // For the given [rangeIteration, rangeBlocksCount]
    int chunkStart, chunkEnd;
    if (!rangeComputation(chunkStart, chunkEnd, rangeIteration, rangeBlocksCount, selectedViews.size()))
    {
        ALICEVISION_LOG_INFO("Nothing to compute in this chunk");
        return EXIT_SUCCESS;
    }

    ALICEVISION_LOG_INFO("Processing items between " << chunkStart << " and " << chunkEnd << ".");

    // The first iteration will do at least a dry run of all the views
    // to compute the necessary information to build the sfmData once for all.
    int extChunkStart = chunkStart;
    int extChunkEnd = chunkEnd;
    if (rangeIteration == 0)
    {
        extChunkStart = 0;
        extChunkEnd = selectedViews.size();
    }

    if (steps.size() == 0)
    {
        extChunkStart = 0;
        extChunkEnd = 0;
    }

    // Loop over selected views for this chunk
    for (int pos = extChunkStart; pos < extChunkEnd; pos++)
    {
        // Retrieve view information
        const IndexT viewId = selectedViews[pos];
        sfmData::View & view = sfmData.getView(viewId);
        camera::IntrinsicBase * existingCamera = sfmData.getIntrinsicPtr(view.getIntrinsicId());
        std::shared_ptr<camera::IntrinsicBase> camera((existingCamera)?existingCamera->clone():nullptr);
        const std::string & viewPath = view.getImage().getImagePath();
        

        // Process image filename
        const bool isRAW = image::isRawFormat(viewPath);
        const fs::path fsPath = viewPath;
        const std::string fileName = fsPath.stem().string();
        const std::string fileExt = fsPath.extension().string();
        const std::string outputExt = extension.empty() ? (isRAW ? ".exr" : fileExt) : (std::string(".") + extension);
        const std::string outputfilePath = (fs::path(outputPath) / ((pParams.keepImageFilename ? fileName : std::to_string(viewId)) + outputExt)).generic_string();

        ALICEVISION_LOG_INFO(pos + 1 << "/" << selectedViews.size() << " - Process view '" << viewId << (isRAW ? "' (RAW)." : "'."));

        // Prepare options to read images
        image::ImageReadOptions options;
        options.workingColorSpace = pParams.applyDcpMetadata ? image::EImageColorSpace::NO_CONVERSION : workingColorSpace;

        if (isRAW)
        {
            if (rawColorInterpretation == image::ERawColorInterpretation::Auto)
            {
                options.rawColorInterpretation = image::ERawColorInterpretation_stringToEnum(view.getImage().getRawColorInterpretation());
                if (options.rawColorInterpretation == image::ERawColorInterpretation::DcpMetadata)
                {
                    options.useDCPColorMatrixOnly = false;
                    options.doWBAfterDemosaicing = true;
                }
                else
                {
                    options.useDCPColorMatrixOnly = useDCPColorMatrixOnly;
                    options.doWBAfterDemosaicing = doWBAfterDemosaicing;
                }
            }
            else
            {
                options.rawColorInterpretation = rawColorInterpretation;
                options.useDCPColorMatrixOnly = useDCPColorMatrixOnly;
                options.doWBAfterDemosaicing = doWBAfterDemosaicing;
            }

            options.colorProfileFileName = view.getImage().getColorProfileFileName();
            options.demosaicingAlgo = demosaicingAlgo;
            options.highlightMode = highlightMode;
            options.rawExposureAdjustment = std::pow(2.f, pParams.rawExposureAdjust);
            options.rawAutoBright = pParams.rawAutoBright;
            options.correlatedColorTemperature = correlatedColorTemperature;
            pParams.correlatedColorTemperature = correlatedColorTemperature;
            pParams.enableColorTempProcessing = options.rawColorInterpretation == image::ERawColorInterpretation::DcpLinearProcessing;
        }
        else
        {
            options.inputColorSpace = inputColorSpace;
        }

        

        // For all items related to this chunk
        image::Image<image::RGBAfColor> image;
        if (pos >= chunkStart && pos < chunkEnd)
        {
            image::readImage(viewPath, image, options);

            for (const auto & step : steps)
            {
                step->processInPlace(sfmData, view, camera.get(), image, false);
            }
            

            if (pParams.applyDcpMetadata)
            {
                workingColorSpace = image::EImageColorSpace::ACES2065_1;
            }

            image::ImageWriteOptions writeOptions;

            writeOptions.fromColorSpace(workingColorSpace);
            writeOptions.toColorSpace(outputColorSpace);
            writeOptions.exrCompressionMethod(exrCompressionMethod);
            writeOptions.exrCompressionLevel(exrCompressionLevel);
            writeOptions.jpegCompress(jpegCompress);
            writeOptions.jpegQuality(jpegQuality);

            if (boost::to_lower_copy(fs::path(outputPath).extension().string()) == ".exr")
            {
                // Select storage data type
                writeOptions.storageDataType(storageDataType);
            }

            // Save the image
            const std::map<std::string, std::string> & viewMetadata = view.getImage().getMetadata(); //Keep a copy
            saveImage(image, viewPath, outputfilePath, viewMetadata, metadataFolders, outputFormat, writeOptions);
        }
        else 
        {
            int width = 0;
            int height = 0;
            image::readImageSize(viewPath, width, height);
            image.resize(width, height);

            for (const auto & step : steps)
            {
                step->processInPlace(sfmData, view, camera.get(), image, true);
            }
        }

        // Add a new intrinsic if the properties changed
        if (rangeIteration == 0)
        {
            view.getImage().setImagePath(outputfilePath);
            if (camera->h() != existingCamera->h() || camera->w() != existingCamera->w()) 
            {
                IndexT intrinsicId = camera->hashValue();
                view.setIntrinsicId(intrinsicId);
                sfmData.getIntrinsics().emplace(intrinsicId, camera);
            }
        }
    }

    // Save updated sfmData.
    // Only saved on first chunk
    if (rangeIteration == 0)
    {
        sfmData.repair();
        
        const std::string sfmfilePath = (fs::path(outputPath) / fs::path(inputSfmDataPath).filename()).generic_string();
        if (!sfmDataIO::save(sfmData, sfmfilePath, sfmDataIO::ESfMData(sfmDataIO::ALL)))
        {
            ALICEVISION_LOG_ERROR("The output SfMData file '" << sfmfilePath << "' cannot be written.");
            return EXIT_FAILURE;
        }
    }

    return 0;
}
