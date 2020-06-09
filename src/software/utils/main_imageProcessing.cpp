
#include <aliceVision/image/all.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/config.hpp>

#include <OpenImageIO/imageio.h>
#include <OpenImageIO/imagebuf.h>
#include <OpenImageIO/imagebufalgo.h>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>



// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 2
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

void processImage(image::Image<image::RGBfColor>& image, float downscale, float contrast, int medianFilter, int sharpenWidth, float sharpenContrast, float sharpenThreshold)
{
    unsigned int nchannels = 3;

    if(downscale != 1.0f)
    {
        const unsigned int w = image.Width();
        const unsigned int h = image.Height();
        const unsigned int nw = (unsigned int)(floor(float(w) / downscale));
        const unsigned int nh = (unsigned int)(floor(float(h) / downscale));

        image::Image<image::RGBfColor> rescaled(nw, nh);

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
    if(contrast != 1.0f)
    {
        image::Image<image::RGBfColor> filtered(image.Width(), image.Height());
        const oiio::ImageBuf inBuf(oiio::ImageSpec(image.Width(), image.Height(), nchannels, oiio::TypeDesc::FLOAT), image.data());
        oiio::ImageBuf outBuf(oiio::ImageSpec(image.Width(), image.Height(), nchannels, oiio::TypeDesc::FLOAT), filtered.data());
        oiio::ImageBufAlgo::contrast_remap(outBuf, inBuf, 0.0f, 1.0f, 0.0f, 1.0f, contrast);

        image.swap(filtered);
    }
    #endif
    if(medianFilter >= 3)
    {
        image::Image<image::RGBfColor> filtered(image.Width(), image.Height());
        const oiio::ImageBuf inBuf(oiio::ImageSpec(image.Width(), image.Height(), nchannels, oiio::TypeDesc::FLOAT), image.data());
        oiio::ImageBuf outBuf(oiio::ImageSpec(image.Width(), image.Height(), nchannels, oiio::TypeDesc::FLOAT), filtered.data());
        oiio::ImageBufAlgo::median_filter(outBuf, inBuf, medianFilter);

        image.swap(filtered);
    }
    if(sharpenWidth >= 3.f && sharpenContrast > 0.f)
    {
        image::Image<image::RGBfColor> filtered(image.Width(), image.Height());
        const oiio::ImageBuf inBuf(oiio::ImageSpec(image.Width(), image.Height(), nchannels, oiio::TypeDesc::FLOAT), image.data());
        oiio::ImageBuf outBuf(oiio::ImageSpec(image.Width(), image.Height(), nchannels, oiio::TypeDesc::FLOAT), filtered.data());
        oiio::ImageBufAlgo::unsharp_mask(outBuf, inBuf, "gaussian", sharpenWidth, sharpenContrast, sharpenThreshold);

        image.swap(filtered);
    }
}

int aliceVision_main(int argc, char * argv[])
{
    std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
    std::string sfmInputDataFilename = "";
    std::string sfmOutputDataFilepath = "";
    bool reconstructedViewsOnly = false;
    bool exposureCompensation = false;
    float downscale = 1.0f;
    float contrast = 1.0f;
    int medianFilter = 0;
    std::string extension;

    int sharpenWidth = 1;
    float sharpenContrast = 1.f;
    float sharpenThreshold = 0.f;

    // Command line parameters
    po::options_description allParams(
        "Parse external information about cameras used in a panorama.\n"
        "AliceVision PanoramaExternalInfo");

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmInputDataFilename)->required(),
         "SfMData file input.")
        ("outSfMData,o", po::value<std::string>(&sfmOutputDataFilepath)->required(),
         "SfMData file output.")
        ;

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("reconstructedViewsOnly", po::value<bool>(&reconstructedViewsOnly)->default_value(reconstructedViewsOnly),
         "Process only recontructed views or all views.")
        ("downscale", po::value<float>(&downscale)->default_value(downscale),
         "Downscale Factor (1.0: no change).")

        ("exposureCompensation", po::value<bool>(&exposureCompensation)->default_value(exposureCompensation),
         "Exposure Compensation.")

        ("contrast", po::value<float>(&contrast)->default_value(contrast),
         "Contrast Factor (1.0: no change).")
        ("medianFilter", po::value<int>(&medianFilter)->default_value(medianFilter),
         "Median Filter (0: no filter).")

        ("sharpenWidth", po::value<int>(&sharpenWidth)->default_value(sharpenWidth),
         "Sharpen kernel width (<3: no sharpening).")
        ("sharpenContrast", po::value<float>(&sharpenContrast)->default_value(sharpenContrast),
         "Sharpen contrast value (0.0: no sharpening).")
        ("sharpenThreshold", po::value<float>(&sharpenThreshold)->default_value(sharpenThreshold),
         "Threshold for minimal variation for contrast to avoid sharpening of small noise (0.0: no noise threshold).")

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

      // Set verbose level
    system::Logger::get()->setLogLevel(verboseLevel);

    if (downscale < 0.0001f || downscale > 1.0f)
    {
        ALICEVISION_LOG_ERROR("Invalid scale factor, downscale should be in range [0.0001, 1].");
        return EXIT_FAILURE;
    }

      // Analyze output path
    const boost::filesystem::path path(sfmOutputDataFilepath);
    const std::string outputPath = path.parent_path().string();

      // Read input
    sfmData::SfMData sfmData;
    if(!sfmDataIO::Load(sfmData, sfmInputDataFilename, sfmDataIO::ESfMData(sfmDataIO::ALL)))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmInputDataFilename << "' cannot be read.");
        return EXIT_FAILURE;
    }

    for (auto & viewIt : sfmData.getViews())
    {
        auto& view = *viewIt.second;
        if(reconstructedViewsOnly && !sfmData.isPoseAndIntrinsicDefined(&view))
            continue;

        ALICEVISION_LOG_INFO("Process view '" << view.getViewId() << "', url: '" << view.getImagePath() << "'");

        // Read original image
        image::Image<image::RGBfColor> image;
        image::readImage(view.getImagePath(), image, image::EImageColorSpace::LINEAR);
        oiio::ParamValueList metadata = image::readImageMetadata(view.getImagePath());

        // if exposureCompensation is needed for sfmData fIles
        if(exposureCompensation)
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
        processImage(image, downscale, contrast, medianFilter, sharpenWidth, sharpenContrast, sharpenThreshold);

        // Save the image
        const std::string ext = extension.empty() ? fs::path(view.getImagePath()).extension().string() : (std::string(".") + extension);
        const std::string outputImagePath = (fs::path(outputPath) / (std::to_string(view.getViewId()) + ext)).string();

        ALICEVISION_LOG_INFO("Export image: '" << outputImagePath << "'.");
        image::writeImage(outputImagePath, image, image::EImageColorSpace::AUTO, metadata);

        // Update view for this modification
        view.setImagePath(outputImagePath);
        view.setWidth(image.Width());
        view.setHeight(image.Height());
    }

    if (downscale != 1.0f)
    {
        for (auto & i : sfmData.getIntrinsics())
        {
          i.second->rescale(downscale);
      }
    }

      // Save sfmData with modified path to images
    if (!sfmDataIO::Save(sfmData, sfmOutputDataFilepath, sfmDataIO::ESfMData(sfmDataIO::ALL)))
    {
        ALICEVISION_LOG_ERROR("The output SfMData file '" << sfmOutputDataFilepath << "' cannot be written.");
        return EXIT_FAILURE;
    }

    return 0;
}
