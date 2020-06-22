
#include <aliceVision/image/all.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/config.hpp>
#include <aliceVision/utils/regexFilter.hpp>

#include <OpenImageIO/imageio.h>
#include <OpenImageIO/imagebuf.h>
#include <OpenImageIO/imagebufalgo.h>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OPENCV)
#include <opencv2/imgproc.hpp>
#endif

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 2
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

struct ProcessingParams
{
    bool reconstructedViewsOnly = false;
    bool exposureCompensation = false;
    float downscale = 1.0f;
    float contrast = 1.0f;
    int medianFilter = 0;
    bool fillHoles = false;

    int sharpenWidth = 1;
    float sharpenContrast = 1.f;
    float sharpenThreshold = 0.f;

    bool bilateralFilter = false;
    int BilateralFilterDistance = 0;
    float BilateralFilterSigmaColor = 0;
    float BilateralFilterSigmaSpace = 0;
};

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OPENCV)
// Conversion functions used for bilateral filter

/**
 * @brief allows to convert an aliceVision image to an openCv image (cv::Mat) in BGR
 * Ignores the alpha channel of the source image
 * @param[in] matIn - Input RGBA aliceVision image
 * @return the resulting openCV image
 */
cv::Mat imageRGBAToCvMatBGR(const image::Image<image::RGBAfColor>& img)
{  
    cv::Mat mat(img.Height(), img.Width(), CV_32FC3);
    for(int row = 0; row < img.Height(); row++)
    {
        cv::Vec3f* rowPtr = mat.ptr<cv::Vec3f>(row);
        for(int col = 0; col < img.Width(); col++)
        {
            cv::Vec3f& matPixel = rowPtr[col];
            const image::RGBAfColor& imgPixel = img(row, col);
            matPixel = cv::Vec3f(imgPixel.b(), imgPixel.g(), imgPixel.r());
        }
    }
    return mat;
}

/**
 * @brief allows to convert an openCv image (cv::Mat) in BGR to an aliceVision image
 * Keeps the alpha channel of the output image unchanged
 * @param[in] matIn - Input openCV image (cv::Mat)
 * @param[out] matIn - output RGBA aliceVision image
 * @return the resulting regex
 */
void cvMatBGRToImageRGBA(const cv::Mat& matIn, image::Image<image::RGBAfColor>& imageOut)
{
    for(int row = 0; row < imageOut.Height(); row++)
    {
        const cv::Vec3f* rowPtr = matIn.ptr<cv::Vec3f>(row);
        for(int col = 0; col < imageOut.Width(); col++)
        {
            const cv::Vec3f& matPixel = rowPtr[col];
            imageOut(row, col) = image::RGBAfColor(matPixel[2], matPixel[1], matPixel[0], imageOut(row, col).a());
        }
    }
}
#endif

void processImage(image::Image<image::RGBAfColor>& image, const ProcessingParams& pParams)
{
    unsigned int nchannels = 3;

    if (pParams.downscale != 1.0f)
    {
        const unsigned int w = image.Width();
        const unsigned int h = image.Height();
        const unsigned int nw = (unsigned int)(floor(float(w) / pParams.downscale));
        const unsigned int nh = (unsigned int)(floor(float(h) / pParams.downscale));

        image::Image<image::RGBAfColor> rescaled(nw, nh);

        const oiio::ImageSpec imageSpecResized(nw, nh, 3, oiio::TypeDesc::FLOAT);
        const oiio::ImageSpec imageSpecOrigin(w, h, 3, oiio::TypeDesc::FLOAT);
        oiio::ImageBuf bufferOrigin(imageSpecOrigin, image.data());
        oiio::ImageBuf bufferResized(imageSpecResized, rescaled.data());
        oiio::ImageBufAlgo::resample(bufferResized, bufferOrigin);

        const oiio::ImageBuf inBuf(oiio::ImageSpec(w, h, nchannels, oiio::TypeDesc::FLOAT), image.data());
        oiio::ImageBuf outBuf(oiio::ImageSpec(nw, nh, nchannels, oiio::TypeDesc::FLOAT), rescaled.data());

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
    if (pParams.sharpenWidth >= 3.f && pParams.sharpenContrast > 0.f)
    {
        image::Image<image::RGBAfColor> filtered(image.Width(), image.Height());
        const oiio::ImageBuf inBuf(oiio::ImageSpec(image.Width(), image.Height(), nchannels, oiio::TypeDesc::FLOAT), image.data());
        oiio::ImageBuf outBuf(oiio::ImageSpec(image.Width(), image.Height(), nchannels, oiio::TypeDesc::FLOAT), filtered.data());
        oiio::ImageBufAlgo::unsharp_mask(outBuf, inBuf, "gaussian", pParams.sharpenWidth, pParams.sharpenContrast, pParams.sharpenThreshold);

        image.swap(filtered);
    }
    
    if (pParams.bilateralFilter)
    {
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OPENCV)
            // Create temporary OpenCV Mat (keep only 3 Channels) to handled Eigen data of our image
            cv::Mat openCVMatIn = imageRGBAToCvMatBGR(image);
            cv::Mat openCVMatOut(image.Width(), image.Height(), CV_32FC3);

            cv::bilateralFilter(openCVMatIn, openCVMatOut, pParams.BilateralFilterDistance, pParams.BilateralFilterSigmaColor, pParams.BilateralFilterSigmaSpace);

            // Copy filtered data from openCV Mat(3 channels) to our image(keep the alpha channel unfiltered)
            cvMatBGRToImageRGBA(openCVMatOut, image);
            
#else
            throw std::invalid_argument("Unsupported mode! If you intended to use a bilateral filter, please add OpenCV support.");
#endif
    }

    if(pParams.fillHoles)
    {
        image::Image<image::RGBAfColor> filtered(image.Width(), image.Height());
        const oiio::ImageBuf inBuf(oiio::ImageSpec(image.Width(), image.Height(), nchannels+1, oiio::TypeDesc::FLOAT), image.data());
        oiio::ImageBuf outBuf(oiio::ImageSpec(image.Width(), image.Height(), nchannels+1, oiio::TypeDesc::FLOAT), filtered.data());
        oiio::ImageBufAlgo::fillholes_pushpull(outBuf, inBuf);

        image.swap(filtered);
    }
}

int aliceVision_main(int argc, char * argv[])
{
    std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
    std::string sfmInputDataFilename = "";
    std::string sfmOutputDataFilepath = "";
    std::string extension;

    ProcessingParams pParams;

    // Command line parameters
    po::options_description allParams(
        "Parse external information about cameras used in a panorama.\n"
        "AliceVision PanoramaExternalInfo");

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmInputDataFilename)->required(),
         "SfMData file input, image filenames or regex(es) on the image file path (supported regex: '#' matches a single digit, '@' one or more digits, '?' one character and '*' zero or more).")
        ("outSfMData,o", po::value<std::string>(&sfmOutputDataFilepath)->required(),
         "SfMData file output.")
        ;

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("reconstructedViewsOnly", po::value<bool>(&pParams.reconstructedViewsOnly)->default_value(pParams.reconstructedViewsOnly),
         "Process only recontructed views or all views.")
        ("downscale", po::value<float>(&pParams.downscale)->default_value(pParams.downscale),
         "Downscale Factor (1.0: no change).")

        ("exposureCompensation", po::value<bool>(& pParams.exposureCompensation)->default_value(pParams.exposureCompensation),
         "Exposure Compensation.")

        ("contrast", po::value<float>(&pParams.contrast)->default_value(pParams.contrast),
         "Contrast Factor (1.0: no change).")
        ("medianFilter", po::value<int>(&pParams.medianFilter)->default_value(pParams.medianFilter),
         "Median Filter (0: no filter).")

        ("sharpenWidth", po::value<int>(&pParams.sharpenWidth)->default_value(pParams.sharpenWidth),
         "Sharpen kernel width (<3: no sharpening).")
        ("sharpenContrast", po::value<float>(&pParams.sharpenContrast)->default_value(pParams.sharpenContrast),
         "Sharpen contrast value (0.0: no sharpening).")
        ("sharpenThreshold", po::value<float>(&pParams.sharpenThreshold)->default_value(pParams.sharpenThreshold),
         "Threshold for minimal variation for contrast to avoid sharpening of small noise (0.0: no noise threshold).")

        ("fillHoles", po::value<bool>(&pParams.fillHoles)->default_value(pParams.fillHoles),
         "Fill Holes.")

        ("bilateralFilter", po::value<bool>(&pParams.bilateralFilter)->default_value(pParams.bilateralFilter),
            "use bilateral Filter")
        ("BilateralFilterDistance", po::value<int>(&pParams.BilateralFilterDistance)->default_value(pParams.BilateralFilterDistance),
            "Diameter of each pixel neighborhood that is used during filtering (if <=0 is computed proportionaly from sigmaSpace).")
        ("BilateralFilterSigmaSpace",po::value<float>(&pParams.BilateralFilterSigmaSpace)->default_value(pParams.BilateralFilterSigmaSpace),
            "Filter sigma in the coordinate space.")
        ("BilateralFilterSigmaColor",po::value<float>(&pParams.BilateralFilterSigmaColor)->default_value(pParams.BilateralFilterSigmaColor),
            "Filter sigma in the color space.")


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


#if !ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OPENCV)
    if(pParams.bilateralFilter)
    {
        ALICEVISION_LOG_ERROR("Invalid option: BilateralFilter can't be used without openCV !");
        return EXIT_FAILURE;
    }
#endif

    if (pParams.downscale < 0.0001f || pParams.downscale > 1.0f)
    {
        ALICEVISION_LOG_ERROR("Invalid scale factor, downscale should be in range [0.0001, 1].");
        return EXIT_FAILURE;
    }

    // Check if is sfm data file
    const std::string inputExt = boost::to_lower_copy(fs::path(sfmInputDataFilename).extension().string());
    static const std::array<std::string, 2> SFMSupportedExtensions = {".sfm", ".abc"};
    if(std::find(SFMSupportedExtensions.begin(), SFMSupportedExtensions.end(), inputExt) != SFMSupportedExtensions.end() )
    {
        sfmData::SfMData sfmData;
        if (!sfmDataIO::Load(sfmData, sfmInputDataFilename, sfmDataIO::ESfMData(sfmDataIO::ALL)))
        {
            ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmInputDataFilename << "' cannot be read.");
            return EXIT_FAILURE;
        }

        const int size = sfmData.getViews().size();
        int i = 0;
        for (auto & viewIt : sfmData.getViews())
        {
            auto& view = *viewIt.second;
            if (pParams.reconstructedViewsOnly && !sfmData.isPoseAndIntrinsicDefined(&view))
                continue;

            ALICEVISION_LOG_INFO(++i << "/" << size << " - Process view '" << view.getViewId() << "'");

            // Read original image
            image::Image<image::RGBAfColor> image;
            image::readImage(view.getImagePath(), image, image::EImageColorSpace::LINEAR);
            oiio::ParamValueList metadata = image::readImageMetadata(view.getImagePath());

            // If exposureCompensation is needed for sfmData files
            if (pParams.exposureCompensation)
            {
                const float medianCameraExposure = sfmData.getMedianCameraExposureSetting();
                const float cameraExposure = view.getCameraExposureSetting();
                const float ev = std::log2(1.0 / cameraExposure);
                const float exposureCompensation = medianCameraExposure / cameraExposure;

                ALICEVISION_LOG_INFO("View: " << view.getViewId() << ", Ev: " << ev << ", Ev compensation: " << exposureCompensation);

                for (int i = 0; i < image.Width() * image.Height(); ++i)
                    image(i) = image(i) * exposureCompensation;
            }

            // Image processing
            processImage(image, pParams);

            // Save the image
            const std::string ext = extension.empty() ? fs::path(view.getImagePath()).extension().string() : (std::string(".") + extension);

            // Analyze output path
            const std::string outputImagePath = (fs::path(sfmOutputDataFilepath).parent_path() / (std::to_string(view.getViewId()) + ext)).string();

            ALICEVISION_LOG_TRACE("Export image: '" << outputImagePath << "'.");
            image::writeImage(outputImagePath, image, image::EImageColorSpace::AUTO, metadata);

            // Update view for this modification
            view.setImagePath(outputImagePath);
            view.setWidth(image.Width());
            view.setHeight(image.Height());
        }

        if (pParams.downscale != 1.0f)
        {
            for (auto & i : sfmData.getIntrinsics())
            {
                i.second->rescale(pParams.downscale);
            }
        }

        // Save sfmData with modified path to images
        if (!sfmDataIO::Save(sfmData, sfmOutputDataFilepath, sfmDataIO::ESfMData(sfmDataIO::ALL)))
        {
            ALICEVISION_LOG_ERROR("The output SfMData file '" << sfmOutputDataFilepath << "' cannot be written.");
            return EXIT_FAILURE;
        }
    }
    else
    {
        const fs::path inputPath(sfmInputDataFilename);
        std::vector<std::string> filesStrPaths;

        if (fs::is_regular_file(inputPath))
        {
            filesStrPaths.push_back(inputPath.string());
        }
        else
        {
            ALICEVISION_LOG_INFO("Working directory Path '" + inputPath.parent_path().string() + "'.");
            // Iterate over files in directory
            for(fs::directory_entry& entry : fs::directory_iterator(inputPath.parent_path()))
            {
                const std::string entryPath = entry.path().generic_string();
                const std::string ext = entry.path().extension().string();
                if(image::isSupported(ext))
                {
                    filesStrPaths.push_back(entryPath);
                }
            }

            // regex filtering files paths 
            filterStrings(filesStrPaths, sfmInputDataFilename);

            if(!filesStrPaths.size())
            {
                ALICEVISION_LOG_INFO("Any images was found in this directory");
                ALICEVISION_LOG_INFO("Filter expression '" << sfmInputDataFilename << "' may be incorrect ?");
                return EXIT_FAILURE;
            }
            else
            {
                ALICEVISION_LOG_INFO(filesStrPaths.size() << " images found.");
            }
        }
        const int size = filesStrPaths.size();
        int i = 0;
        for(const std::string& inputFilePath : filesStrPaths)
        {
            const fs::path path = fs::path(inputFilePath);
            const std::string fileName = path.stem().string();
            const std::string fileExt = path.extension().string();
            const std::string outputExt = extension.empty() ? fileExt : (std::string(".") + extension);
            const std::string outputFilePath = (fs::path(sfmOutputDataFilepath).parent_path() / (fileName + outputExt)).string();

            ALICEVISION_LOG_INFO(++i << "/" << size << " - Process image '" << fileName << fileExt << "'.");

            // Read original image
            image::Image<image::RGBAfColor> image;
            image::readImage(inputFilePath, image, image::EImageColorSpace::LINEAR);
            const oiio::ParamValueList metadata = image::readImageMetadata(inputFilePath);

            // Image processing
            processImage(image, pParams);

            // Save the image
            ALICEVISION_LOG_TRACE("Export image: '" << outputFilePath << "'.");
            image::writeImage(outputFilePath, image, image::EImageColorSpace::AUTO, metadata);
        }
    }

    return 0;
}
