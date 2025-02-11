// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/utils/filesIO.hpp>
#include <boost/program_options.hpp>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = std::filesystem;

using NameFunction = std::function<std::string(const sfmData::View &)>;

template<typename T>
std::string to_string_with_zero_padding(const T& value, std::size_t total_length)
{
    auto str = std::to_string(value);
    if (str.length() < total_length)
    {
        str.insert(str.front() == '-' ? 1 : 0, total_length - str.length(), '0');
    }

    return str;
}

template<typename T>
void ImageIntrinsicsTransform(const image::Image<T>& imageIn,
                    const camera::IntrinsicBase & intrinsicSource,
                    const camera::IntrinsicBase & intrinsicOutput,
                    image::Image<T>& image_ud,
                    T fillcolor,
                    const oiio::ROI& roi = oiio::ROI())
{
    // There is distortion
    const Vec2 center(imageIn.width() * 0.5, imageIn.height() * 0.5);

    int widthRoi = intrinsicOutput.w();
    int heightRoi = intrinsicOutput.h();
    int xOffset = 0;
    int yOffset = 0;
    if (roi.defined())
    {
        widthRoi = roi.width();
        heightRoi = roi.height();
        xOffset = roi.xbegin;
        yOffset = roi.ybegin;
    }

    image_ud.resize(widthRoi, heightRoi, true, fillcolor);
    const image::Sampler2d<image::SamplerLinear> sampler;

#pragma omp parallel for
    for (int y = 0; y < heightRoi; ++y)
    {
        for (int x = 0; x < widthRoi; ++x)
        {
            const Vec2 undisto_pix(x + xOffset, y + yOffset);

            // compute coordinates with distortion
            const Vec3 intermediate = intrinsicOutput.backProjectUnit(undisto_pix);
            if (intermediate.z() < -0.2) continue;
            const Vec2 disto_pix = intrinsicSource.project(intermediate.homogeneous(), true);

            // pick pixel if it is in the image domain
            if (imageIn.contains(disto_pix(1), disto_pix(0)))
            {
                image_ud(y, x) = sampler(imageIn, disto_pix(1), disto_pix(0));
            }
        }
    }
}

void processImage(const std::string& dstFileName,
             const camera::IntrinsicBase & outputIntrinsic,
             const camera::IntrinsicBase & sourceIntrinsic,
             const oiio::ParamValueList& metadata,
             const std::string& srcFileName,
             bool evCorrection,
             float exposureCompensation)
{
    image::Image<image::RGBAfColor> image;
    image::Image<image::RGBAfColor> image_ud;

    readImage(srcFileName, image, image::EImageColorSpace::LINEAR);

    // exposure correction
    if (evCorrection)
    {
        for (int pix = 0; pix < image.width() * image.height(); ++pix)
        {
            image(pix)[0] *= exposureCompensation;
            image(pix)[1] *= exposureCompensation;
            image(pix)[2] *= exposureCompensation;
        }
    }
    
    // undistort the image and save it
    ImageIntrinsicsTransform(image, sourceIntrinsic, outputIntrinsic, image_ud, image::RGBAfColor(0.0));

    //Write the result
    writeImage(dstFileName, image_ud, image::ImageWriteOptions(), metadata);
}

bool process(const sfmData::SfMData & input, 
             const sfmData::SfMData & target, 
             const NameFunction & namingFunction,
             bool evCorrection,
             size_t rangeStart,
             size_t rangeEnd)
{
    rangeEnd = std::min(input.getViews().size(), rangeEnd);

    // for exposure correction
    const double medianCameraExposure = input.getMedianCameraExposureSetting().getExposure(); 

    for (int posImage = rangeStart; posImage < rangeEnd; posImage++)
    {
        auto viewsIt = input.getViews().begin();
        std::advance(viewsIt, posImage);

        //Retrieve view
        IndexT viewId = viewsIt->first;
        const sfmData::View & view = *viewsIt->second;

        //Retrieve intrinsic
        IndexT intrinsicId = view.getIntrinsicId();
        const auto & intrinsic = input.getIntrinsic(intrinsicId);

        //Make sure the target sfm contains the same thing
        const auto & targetIntrinsics = target.getIntrinsics();
        if (targetIntrinsics.find(intrinsicId) == targetIntrinsics.end())
        {
            continue;
        }

        const auto & targetIntrinsic = target.getIntrinsic(intrinsicId);

        //Retrieve image
        std::string srcFileName = view.getImage().getImagePath();
        oiio::ParamValueList metadata = image::readImageMetadata(srcFileName);
        

        // add exposure values to images metadata
        const double cameraExposure = view.getImage().getCameraExposureSetting().getExposure();
        const double ev = std::log2(1.0 / cameraExposure);
        const float exposureCompensation = float(medianCameraExposure / cameraExposure);
        metadata.push_back(oiio::ParamValue("AliceVision:EV", float(ev)));
        metadata.push_back(oiio::ParamValue("AliceVision:EVComp", exposureCompensation));

        //Process Image
        std::string outFileName = namingFunction(view);

        ALICEVISION_LOG_INFO("Process image " << srcFileName);
        processImage(outFileName,
                    targetIntrinsic,
                    intrinsic,
                    metadata,
                    srcFileName,
                    evCorrection,
                    exposureCompensation);
    }

    return true;
}

int aliceVision_main(int argc, char* argv[])
{
    // command-line parameters
    std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
    std::string inputSfmDataFilename;
    std::string targetSfmDataFilename;
    std::string outFolder;
    std::string outImageFileTypeName = image::EImageFileType_enumToString(image::EImageFileType::EXR);
    std::string namingMode = "frameid";
    int rangeStart = -1;
    int rangeSize = 1;
    bool evCorrection = false;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&inputSfmDataFilename)->required(),
         "Input SfMData file.")
        ("target,t", po::value<std::string>(&targetSfmDataFilename)->required(),
         "Target SfMData file.")
        ("output,o", po::value<std::string>(&outFolder)->required(),
         "Output folder.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("outputFileType", po::value<std::string>(&outImageFileTypeName)->default_value(outImageFileTypeName),
         image::EImageFileType_informations().c_str())
        ("rangeStart", po::value<int>(&rangeStart)->default_value(rangeStart),
         "Range image index start.")
        ("rangeSize", po::value<int>(&rangeSize)->default_value(rangeSize),
         "Range size.")
        ("evCorrection", po::value<bool>(&evCorrection)->default_value(evCorrection),
         "Correct exposure value.")
        ("namingMode", po::value<std::string>(&namingMode)->default_value(namingMode),
         "naming mode.");
    // clang-format on

    CmdLine cmdline("AliceVision prepareDenseScene");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // set output file type
    image::EImageFileType outputFileType = image::EImageFileType_stringToEnum(outImageFileTypeName);

    // Create output dir
    if (!utils::exists(outFolder))
    {
        fs::create_directory(outFolder);
    }

    sfmDataIO::ESfMData flagsPart = sfmDataIO::ESfMData(
                sfmDataIO::ESfMData::VIEWS | 
                sfmDataIO::ESfMData::INTRINSICS | 
                sfmDataIO::ESfMData::EXTRINSICS
            );

    // Read the input SfM scene
    sfmData::SfMData inputSfmData;
    if (!sfmDataIO::load(inputSfmData, inputSfmDataFilename, flagsPart))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << inputSfmDataFilename << "' cannot be read.");
        return EXIT_FAILURE;
    }

    // Read the target SfM scene
    sfmData::SfMData targetSfmData;
    if (!sfmDataIO::load(targetSfmData, targetSfmDataFilename, flagsPart))
    {
        ALICEVISION_LOG_ERROR("The target SfMData file '" << targetSfmDataFilename << "' cannot be read.");
        return EXIT_FAILURE;
    }

    int rangeEnd = inputSfmData.getViews().size();

    // set range
    if (rangeStart != -1)
    {
        if (rangeStart < 0 || rangeSize < 0)
        {
            ALICEVISION_LOG_ERROR("Range is incorrect");
            return EXIT_FAILURE;
        }

        if (rangeStart + rangeSize > inputSfmData.getViews().size())
        {
            rangeSize = inputSfmData.getViews().size() - rangeStart;
        }

        rangeEnd = rangeStart + rangeSize;

        if (rangeSize <= 0)
        {
            ALICEVISION_LOG_WARNING("Nothing to compute.");
            return EXIT_SUCCESS;
        }
    }
    else
    {
        rangeStart = 0;
    }

    NameFunction namingFunction;
    
    if (namingMode == "frameid")
    {
        namingFunction = [&outputFileType, outFolder](const sfmData::View & view) 
        {
            const std::string baseFilename = to_string_with_zero_padding(view.getFrameId(), 10);
            const std::string ext = image::EImageFileType_enumToString(outputFileType);
            return (fs::path(outFolder) / (baseFilename + "." + ext)).string();    
        };
    }
    else if (namingMode == "viewid")
    {
        namingFunction = [&outputFileType, outFolder](const sfmData::View & view) 
        {
            const std::string baseFilename = std::to_string(view.getViewId());
            const std::string ext = image::EImageFileType_enumToString(outputFileType);
            return (fs::path(outFolder) / (baseFilename + "." + ext)).string();    
        };
    }
    else 
    {
        namingFunction = [&outputFileType, outFolder](const sfmData::View & view) 
        {
            const fs::path imagePath = fs::path(view.getImage().getImagePath());
            const std::string baseFilename = imagePath.stem().string();
            const std::string ext = image::EImageFileType_enumToString(outputFileType);
            return (fs::path(outFolder) / (baseFilename + "." + ext)).string();    
        };
    }

    if (!process(inputSfmData, targetSfmData, namingFunction, evCorrection, rangeStart, rangeEnd))
    {
        ALICEVISION_LOG_ERROR("Process failed");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
