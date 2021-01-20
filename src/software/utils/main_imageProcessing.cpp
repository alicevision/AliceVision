
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

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OPENCV)
#include <opencv2/imgproc.hpp>
#endif

#include <OpenImageIO/imageio.h>
#include <OpenImageIO/imagebuf.h>
#include <OpenImageIO/imagebufalgo.h>


// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 2
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

};


void processImage(image::Image<image::RGBAfColor>& image, const ProcessingParams& pParams)
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
    
    #if OIIO_VERSION >= (10000 * 2 + 100 * 0 + 0) // OIIO_VERSION >= 2.0.0
    if (pParams.contrast != 1.0f)
    {
        image::Image<image::RGBAfColor> filtered(image.Width(), image.Height());
        const oiio::ImageBuf inBuf(oiio::ImageSpec(image.Width(), image.Height(), nchannels, oiio::TypeDesc::FLOAT), image.data());
        oiio::ImageBuf outBuf(oiio::ImageSpec(image.Width(), image.Height(), nchannels, oiio::TypeDesc::FLOAT), filtered.data());
        oiio::ImageBufAlgo::contrast_remap(outBuf, inBuf, 0.0f, 1.0f, 0.0f, 1.0f, pParams.contrast);

        image.swap(filtered);
    }
    #endif
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
}

void saveImage(image::Image<image::RGBAfColor>& image, const std::string& inputPath, const std::string& outputPath,
               const std::vector<std::string>& metadataFolders,
               const EImageFormat outputFormat, const image::EStorageDataType storageDataType)
{
    // Read metadata path
    std::string metadataFilePath;
    
    const std::string filename = fs::path(inputPath).filename().string();
    const std::string outExtension = boost::to_lower_copy(fs::path(outputPath).extension().string());
    const bool isEXR = (outExtension == ".exr");
    // If metadataFolders is specified
    if(!metadataFolders.empty())
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
    }
    else
    {
        // Metadata are extracted from the original images
        metadataFilePath = inputPath;
    }

    oiio::ParamValueList metadata = image::readImageMetadata(metadataFilePath);

    if(isEXR)
    {
        // Select storage data type
        metadata.push_back(oiio::ParamValue("AliceVision:storageDataType", image::EStorageDataType_enumToString(storageDataType)));
    }

    // Save image
    ALICEVISION_LOG_TRACE("Export image: '" << outputPath << "'.");
    
    if(outputFormat == EImageFormat::Grayscale)
    {
        image::Image<float> outputImage;
        image::ConvertPixelType(image, &outputImage);
        image::writeImage(outputPath, outputImage, image::EImageColorSpace::AUTO, metadata);
    }
    else if(outputFormat == EImageFormat::RGB)
    {
        image::Image<image::RGBfColor> outputImage;
        image::ConvertPixelType(image, &outputImage);
        image::writeImage(outputPath, outputImage, image::EImageColorSpace::AUTO, metadata);
    }
    else 
    {
        // Already in RGBAf
        image::writeImage(outputPath, image, image::EImageColorSpace::AUTO, metadata);
    }
}

int aliceVision_main(int argc, char * argv[])
{
    std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
    std::string inputExpression;
    std::vector<std::string> inputFolders;
    std::vector<std::string> metadataFolders;
    std::string outputPath;
    EImageFormat outputFormat = EImageFormat::RGBA;
    image::EStorageDataType storageDataType = image::EStorageDataType::Float;
    std::string extension;

    ProcessingParams pParams;

    // Command line parameters
    po::options_description allParams(
        "Parse external information about cameras used in a panorama.\n"
        "AliceVision PanoramaExternalInfo");

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&inputExpression)->default_value(inputExpression),
         "SfMData file input, image filenames or regex(es) on the image file path (supported regex: '#' matches a single digit, '@' one or more digits, '?' one character and '*' zero or more).")
        ("inputFolders", po::value<std::vector<std::string>>(&inputFolders)->multitoken(),
        "Use images from specific folder(s) instead of those specify in the SfMData file.")
        ("output,o", po::value<std::string>(&outputPath)->required(),
         "Output folder.")
        ;

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("metadataFolders", po::value<std::vector<std::string>>(&metadataFolders)->multitoken(),
        "Use images metadata from specific folder(s) instead of those specified in the input images.")
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

        ("outputFormat", po::value<EImageFormat>(&outputFormat)->default_value(outputFormat),
         "Output image format (rgba, rgb, grayscale)")
    
        ("storageDataType", po::value<image::EStorageDataType>(&storageDataType)->default_value(storageDataType),
         ("Storage data type: " + image::EStorageDataType_informations()).c_str())

        ("extension", po::value<std::string>(&extension)->default_value(extension),
         "Output image extension (like exr, or empty to keep the source file format.")
        ;

    po::options_description logParams("Log parameters");
    logParams.add_options()
        ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
          "verbosity level (fatal, error, warning, info, debug, trace).");

    allParams.add(requiredParams).add(optionalParams).add(logParams);

    po::variables_map vm;
    try
    {
        po::store(po::parse_command_line(argc, argv, allParams), vm);

        if (vm.count("help") || (argc == 1))
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

      // Set verbose level
    system::Logger::get()->setLogLevel(verboseLevel);

    // check user choose at least one input option
    if(inputExpression.empty() && inputFolders.empty())
    {
        ALICEVISION_LOG_ERROR("Program need at least --input or --inputFolders option." << std::endl << "No input images here.");
        return EXIT_FAILURE;
    }

#if !ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OPENCV)
    if(pParams.bilateralFilter.enabled || pParams.claheFilter.enabled)
    {
        ALICEVISION_LOG_ERROR("Invalid option: BilateralFilter and claheFilter can't be used without openCV !");
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
            const std::string fileExt = fsPath.extension().string();
            const std::string outputExt = extension.empty() ? fileExt : (std::string(".") + extension);
            const std::string outputfilePath = (fs::path(outputPath) / (std::to_string(viewId) + outputExt)).generic_string();

            ALICEVISION_LOG_INFO(++i << "/" << size << " - Process view '" << viewId << "'.");


            image::ImageReadOptions options;
            options.outputColorSpace = image::EImageColorSpace::LINEAR;
            options.applyWhiteBalance = view.getApplyWhiteBalance();

            // Read original image
            image::Image<image::RGBAfColor> image;
            image::readImage(viewPath, image, options);

            // If exposureCompensation is needed for sfmData files
            if (pParams.exposureCompensation)
            {
                const float medianCameraExposure = sfmData.getMedianCameraExposureSetting();
                const float cameraExposure = view.getCameraExposureSetting();
                const float ev = std::log2(1.0 / cameraExposure);
                const float exposureCompensation = medianCameraExposure / cameraExposure;

                ALICEVISION_LOG_INFO("View: " << viewId << ", Ev: " << ev << ", Ev compensation: " << exposureCompensation);

                for (int i = 0; i < image.Width() * image.Height(); ++i)
                    image(i) = image(i) * exposureCompensation;
            }

            // Image processing
            processImage(image, pParams);

            // Save the image
            saveImage(image, viewPath, outputfilePath, metadataFolders, outputFormat, storageDataType);

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

        int i = 0;
        for(const std::string& inputFilePath : filesStrPaths)
        {
            const fs::path path = fs::path(inputFilePath);
            const std::string filename = path.stem().string();
            const std::string fileExt = path.extension().string();
            const std::string outputExt = extension.empty() ? fileExt : (std::string(".") + extension);
            const std::string outputFilePath = (fs::path(outputPath) / (filename + outputExt)).generic_string();

            ALICEVISION_LOG_INFO(++i << "/" << size << " - Process image '" << filename << fileExt << "'.");

            // Read original image
            image::Image<image::RGBAfColor> image;
            image::readImage(inputFilePath, image, image::EImageColorSpace::LINEAR);

            // Image processing
            processImage(image, pParams);

            // Save the image
            saveImage(image, inputFilePath, outputFilePath, metadataFolders, outputFormat, storageDataType);
        }
    }

    return 0;
}
