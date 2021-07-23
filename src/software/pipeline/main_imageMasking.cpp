// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/jsonIO.hpp>
#include <aliceVision/image/io.hpp>
#include <aliceVision/imageMasking/imageMasking.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/system/cmdline.hpp>

#include <OpenImageIO/imagebuf.h>
#include <OpenImageIO/imagebufalgo.h>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/case_conv.hpp> 

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

enum class EAlgorithm {
    HSV,
    GrabCut,
    AutoGrayscaleThreshold,
};

inline std::string EAlgorithm_enumToString(EAlgorithm value)
{
    switch(value)
    {
        case EAlgorithm::HSV:
            return "hsv";
        case EAlgorithm::GrabCut:
            return "grabcut";
        case EAlgorithm::AutoGrayscaleThreshold:
            return "autograyscalethreshold";
    }
    throw std::out_of_range("Invalid Algorithm type Enum: " + std::to_string(int(value)));
}

inline EAlgorithm EAlgorithm_stringToEnum(std::string value)
{
    boost::to_lower(value);
    if(value == "hsv")
        return EAlgorithm::HSV;
    if(value == "grabcut")
        return EAlgorithm::GrabCut;
    if(value == "autograyscalethreshold")
        return EAlgorithm::AutoGrayscaleThreshold;
    throw std::out_of_range("Invalid Algorithm type string " + value);
}

inline std::ostream& operator<<(std::ostream& os, EAlgorithm s)
{
    return os << EAlgorithm_enumToString(s);
}

inline std::istream& operator>>(std::istream& in, EAlgorithm& s)
{
    std::string token;
    in >> token;
    s = EAlgorithm_stringToEnum(token);
    return in;
}

/**
 * @brief Write mask images from input images based on chosen algorithm.
 */
int main(int argc, char **argv)
{
    // command-line parameters
    std::string sfmFilePath;
    std::string outputFilePath;
    std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());

    // misc.
    bool invert = false;
    int growRadius = 0;
    int shrinkRadius = 0;

    // program range
    int rangeStart = 0;
    int rangeSize = -1;

    EAlgorithm algorithm = EAlgorithm::HSV;
    struct
    {
        float hue = 0.33f;
        float hueRange = 0.1f;
        float minSaturation = 0.3f;
        float maxSaturation = 1.f;
        float minValue = 0.3f;
        float maxValue = 1.f;
    } hsv;

    std::string depthMapFolder;
    std::string depthMapExp;

    po::options_description allParams("AliceVision masking");

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmFilePath)->required(),
            "A SfMData file (*.sfm) [if specified, --imageFolder cannot be used].")
        ("output,o", po::value<std::string>(&outputFilePath)->required(),
            "Output file path for the new SfMData file");

    po::options_description hsvParams("HSV parameters");
    hsvParams.add_options()
        ("algorithm,a", po::value<EAlgorithm>(&algorithm)->default_value(algorithm),
            std::string("Masking algorithm:\n"
            " * " + EAlgorithm_enumToString(EAlgorithm::HSV) + ": selected range in the Hue Saturation Value color space.\n"
            " * " + EAlgorithm_enumToString(EAlgorithm::GrabCut) + ": not implemented"
            ).c_str())
        ("hsvHue", po::value<float>(&hsv.hue)->default_value(hsv.hue)->notifier(optInRange(0.f, 1.f, "hsvHue")),
            "Hue value to isolate in [0,1] range. 0 = red, 0.33 = green, 0.66 = blue, 1 = red.")
        ("hsvHueRange", po::value<float>(&hsv.hueRange)->default_value(hsv.hueRange)->notifier(optInRange(0.f, 1.f, "hsvHueRange")),
            "Tolerance around the hue value to isolate.")
        ("hsvMinSaturation", po::value<float>(&hsv.minSaturation)->default_value(hsv.minSaturation)->notifier(optInRange(0.f, 1.f, "hsvMinSaturation")),
            "Hue is meaningless if saturation is low. Do not mask pixels below this threshold.")
        ("hsvMaxSaturation", po::value<float>(&hsv.maxSaturation)->default_value(hsv.maxSaturation)->notifier(optInRange(0.f, 1.f, "hsvMaxSaturation")),
            "Do not mask pixels above this threshold. It might be useful to mask white/black pixels.")
        ("hsvMinValue", po::value<float>(&hsv.minValue)->default_value(hsv.minValue)->notifier(optInRange(0.f, 1.f, "hsvMinValue")),
            "Hue is meaningless if value is low. Do not mask pixels below this threshold.")
        ("hsvMaxValue", po::value<float>(&hsv.maxValue)->default_value(hsv.maxValue)->notifier(optInRange(0.f, 1.f, "hsvMaxValue")),
            "Do not mask pixels above this threshold. It might be useful to mask white/black pixels.")
        ("depthMapFolder", po::value<std::string>(&depthMapFolder)->default_value(depthMapFolder),
            "Optional input depth map folder to use instead of the color image to generate the mask.")
        ("depthMapExp", po::value<std::string>(&depthMapExp)->default_value(depthMapExp),
            "Optional expression to find and use the input depth map instead of the color image to generate the mask.")
        ;

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("invert", po::value<bool>(&invert)->default_value(invert),
            "Invert the mask.")
        ("growRadius", po::value<int>(&growRadius)->default_value(growRadius)->notifier(optInRange(0, INT_MAX, "growRadius")),
            "Grow the selected area. It might be used to fill the holes: then use shrinkRadius to restore the initial coutours.")
        ("shrinkRadius", po::value<int>(&shrinkRadius)->default_value(shrinkRadius)->notifier(optInRange(0, INT_MAX, "shrinkRadius")),
            "Shrink the selected area.")
        ("rangeStart", po::value<int>(&rangeStart)->default_value(rangeStart),
            "Compute a sub-range of images from index rangeStart to rangeStart+rangeSize.")
        ("rangeSize", po::value<int>(&rangeSize)->default_value(rangeSize),
            "Compute a sub-range of N images (N=rangeSize).")
        ;

    po::options_description logParams("Log parameters");
    logParams.add_options()
        ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
            "verbosity level (fatal, error, warning, info, debug, trace).");

    allParams.add(requiredParams).add(hsvParams).add(optionalParams).add(logParams);

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

    // check user choose at least one input option
    if(sfmFilePath.empty())
    {
        ALICEVISION_LOG_ERROR("Program need -i option" << std::endl << "No input images.");
        return EXIT_FAILURE;
    }

    // check sfm file
    if(!sfmFilePath.empty() && !fs::exists(sfmFilePath) && !fs::is_regular_file(sfmFilePath))
    {
        ALICEVISION_LOG_ERROR("The input sfm file doesn't exist");
        return EXIT_FAILURE;
    }

    sfmData::SfMData sfmData;
    if(!sfmDataIO::Load(sfmData, sfmFilePath, sfmDataIO::ESfMData::VIEWS))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" + sfmFilePath + "' cannot be read.");
        return EXIT_FAILURE;
    }

    // check output string
    if(outputFilePath.empty())
    {
        ALICEVISION_LOG_ERROR("Invalid output");
        return EXIT_FAILURE;
    }

    // ensure output folder exists
    if(!outputFilePath.empty() && !fs::exists(outputFilePath))
    {
        if(!fs::create_directory(outputFilePath))
        {
            ALICEVISION_LOG_ERROR("Cannot create output folder");
            return EXIT_FAILURE;
        }
    }

    // check program range
    if(rangeStart < 0 || rangeStart >= sfmData.views.size())
    {
        ALICEVISION_LOG_ERROR("invalid subrange of views to process.");
        return EXIT_FAILURE;
    }

    // algorithms
    using imageMasking::OutImage;
    std::function<void(OutImage&, const std::string&)> process;
    switch(algorithm)
    {
        case EAlgorithm::HSV:
        {
            // check options
            if(hsv.minSaturation > hsv.maxSaturation)
            {
                ALICEVISION_LOG_ERROR("hsv-minSaturation must be lower than hsv-maxSaturation");
                return EXIT_FAILURE;
            }
            if(hsv.minValue > hsv.maxValue)
            {
                ALICEVISION_LOG_ERROR("hsv-minValue must be lower than hsv-maxValue");
                return EXIT_FAILURE;
            }

            process = [&](OutImage& result, const std::string& inputPath)
            {
                imageMasking::hsv(result, inputPath, hsv.hue, hsv.hueRange, hsv.minSaturation, hsv.maxSaturation, hsv.minValue, hsv.maxValue);
            };
            break;
        }
        case EAlgorithm::AutoGrayscaleThreshold:
        {
            process = imageMasking::autoGrayscaleThreshold;
            break;
        }
        case EAlgorithm::GrabCut:
            throw std::runtime_error("GrabCut not yet implemented.");
    }

    if(!process)
    {
        ALICEVISION_LOG_ERROR("Invalid algorithm");
        return EXIT_FAILURE;
    }

    system::Timer timer;
    const auto& views = sfmData.getViews();
    const auto viewPairItBegin = views.begin();
    const int size = std::min(int(views.size()) - rangeStart, (rangeSize < 0 ? std::numeric_limits<int>::max() : rangeSize));

    const std::string k_depthMapFolder = "{depthMapFolder}";
    const std::string k_inputFolder = "{inputFolder}";
    const std::string k_filename = "{filename}";
    const std::string k_stem = "{stem}";
    const std::string k_ext = "{ext}";

    bool useDepthMap = !depthMapExp.empty() || !depthMapFolder.empty();

    // #pragma omp parallel for
    for(int i = 0; i < size; ++i)
    {
        const auto& item = std::next(viewPairItBegin, rangeStart + i);
        const IndexT& index = item->first;
        const sfmData::View& view = *item->second;

        std::string imgPath = view.getImagePath();
        std::string depthMapPath;
        if(!depthMapExp.empty())
        {
            depthMapPath = depthMapExp;
            {
                const auto pos = depthMapPath.find(k_depthMapFolder);
                if(pos != std::string::npos)
                    depthMapPath.replace(pos, k_depthMapFolder.size(), depthMapFolder);
            }
            {
                const auto pos = depthMapPath.find(k_inputFolder);
                if(pos != std::string::npos)
                    depthMapPath.replace(pos, k_inputFolder.size(), fs::path(imgPath).parent_path().string());
            }
            {
                const auto pos = depthMapPath.find(k_filename);
                if(pos != std::string::npos)
                    depthMapPath.replace(pos, k_filename.size(), fs::path(imgPath).filename().string());
            }
            {
                const auto pos = depthMapPath.find(k_stem);
                if(pos != std::string::npos)
                    depthMapPath.replace(pos, k_stem.size(), fs::path(imgPath).stem().string());
            }
            {
                const auto pos = depthMapPath.find(k_ext);
                if(pos != std::string::npos)
                    depthMapPath.replace(pos, k_stem.size(), fs::path(imgPath).extension().string().substr(1));
            }
            if(!fs::exists(depthMapPath))
            {
                ALICEVISION_LOG_DEBUG("depthMapPath from expression: \"" << depthMapPath << "\" not found.");
                depthMapPath.clear();
            }
            else
            {
                ALICEVISION_LOG_DEBUG("depthMapPath from expression: \"" << depthMapPath << "\" found.");
            }
        }
        else if(!depthMapFolder.empty())
        {
            // Look for View UID
            fs::path p = fs::path(depthMapFolder) / (std::to_string(view.getViewId()) + fs::path(imgPath).extension().string());
            if(fs::exists(p))
            {
                depthMapPath = p.string();
                ALICEVISION_LOG_DEBUG("depthMapPath found from folder and View UID: \"" << depthMapPath << "\".");
            }
            else
            {
                // Look for an image with the same filename
                p = fs::path(depthMapFolder) / fs::path(imgPath).filename();
                if(fs::exists(p))
                {
                    depthMapPath = p.string();
                    ALICEVISION_LOG_DEBUG("depthMapPath found from folder and input filename: \"" << depthMapPath << "\".");
                }
            }
        }

        const std::string p = useDepthMap ? depthMapPath : imgPath;
        image::Image<unsigned char> result;
        process(result, p);

        if(invert)
        {
            imageMasking::postprocess_invert(result);
        }
        if(growRadius > 0)
        {
            imageMasking::postprocess_dilate(result, growRadius);
        }
        if(shrinkRadius > 0)
        {
            imageMasking::postprocess_erode(result, shrinkRadius);
        }

        if(useDepthMap)
        {
            bool viewHorizontal = view.getWidth() > view.getHeight();
            bool depthMapHorizontal = result.Width() > result.Height();
            if(viewHorizontal != depthMapHorizontal)
            {
                ALICEVISION_LOG_ERROR("Image " << imgPath << " : " << view.getWidth() << "x" << view.getHeight());
                ALICEVISION_LOG_ERROR("Depth Map " << depthMapPath << " : " << result.Width() << "x" << result.Height());
                throw std::runtime_error("Depth map orientation is not aligned with source image.");
            }
            if(view.getWidth() != result.Width())
            {
                ALICEVISION_LOG_DEBUG("Rescale depth map \"" << imgPath << "\" from: " << result.Width() << "x" << result.Height() << ", to: " << view.getWidth() << "x" << view.getHeight());

                image::Image<unsigned char> rescaled(view.getWidth(), view.getHeight());

                const oiio::ImageBuf inBuf(oiio::ImageSpec(result.Width(), result.Height(), 1, oiio::TypeDesc::UINT8), result.data());
                oiio::ImageBuf outBuf(oiio::ImageSpec(rescaled.Width(), rescaled.Height(), 1, oiio::TypeDesc::UINT8), rescaled.data());

                oiio::ImageBufAlgo::resize(outBuf, inBuf);

                result.swap(rescaled);
            }
        }
        const auto resultFilename = fs::path(std::to_string(index)).replace_extension("png");
        const std::string resultPath = (fs::path(outputFilePath) / resultFilename).string();
        image::writeImage(resultPath, result, image::EImageColorSpace::LINEAR);
    }

    ALICEVISION_LOG_INFO("Task done in (s): " + std::to_string(timer.elapsed()));
    return EXIT_SUCCESS;
}
