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
#include <aliceVision/image/Image.hpp>
#include <aliceVision/image/io.hpp>
#include <aliceVision/image/Sampler.hpp>
#include <boost/program_options.hpp>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 1

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = std::filesystem;

using NameFunction = std::function<std::string(const sfmData::View &)>;

oiio::ROI computeRod(const camera::IntrinsicBase & source, const camera::IntrinsicBase & dest)
{
    //Build the 4 corners of the source image
    std::vector<Vec2> pointToBeChecked;
    pointToBeChecked.push_back(Vec2(0, 0));
    pointToBeChecked.push_back(Vec2(source.w() - 1, 0));
    pointToBeChecked.push_back(Vec2(0, source.h() - 1));
    pointToBeChecked.push_back(Vec2(source.w() - 1, source.h() - 1));

    const Vec2 center(source.w() * 0.5, source.h() * 0.5);

    //Add the middle of the edges
    pointToBeChecked.push_back(Vec2(center.x(), 0));
    pointToBeChecked.push_back(Vec2(center.x(), source.h() - 1));
    pointToBeChecked.push_back(Vec2(0, center.y()));
    pointToBeChecked.push_back(Vec2(source.w() - 1, center.y()));

    //Undistort all points
    double minx = std::numeric_limits<double>::max();
    double miny = std::numeric_limits<double>::max();
    double maxx = std::numeric_limits<double>::lowest();
    double maxy = std::numeric_limits<double>::lowest();

    for (const Vec2& n : pointToBeChecked)
    {
        // compute every source pixel into the new destination camera
        const Vec3 intermediate = source.backProjectUnit(n);
        const Vec2 undisto_pix = dest.project(intermediate.homogeneous(), true);
 
        minx = std::min(minx, undisto_pix.x());
        miny = std::min(miny, undisto_pix.y());
        maxx = std::max(maxx, undisto_pix.x());
        maxy = std::max(maxy, undisto_pix.y());
    }

    oiio::ROI rod(minx, maxx + 1, miny, maxy + 1);
        
    return rod;
}

oiio::ROI convertRodToRoi(const camera::IntrinsicBase & intrinsic, const oiio::ROI& rod)
{
    const int xOffset = rod.xbegin;
    const int yOffset = rod.ybegin;

    const oiio::ROI roi(-xOffset, intrinsic.w() - xOffset, -yOffset, intrinsic.h() - yOffset);

    return roi;
}

template<typename T>
void ImageIntrinsicsTransform(const image::Image<T>& imageIn,
                    const camera::IntrinsicBase & intrinsicSource,
                    const camera::IntrinsicBase & intrinsicOutput,
                    image::Image<T>& image_ud,
                    T fillcolor,
                    const oiio::ROI& roi = oiio::ROI())
{
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
            const Vec2 disto_pix = intrinsicSource.project(intermediate.homogeneous(), true);

            // pick pixel if it is in the image domain
            if (imageIn.contains(disto_pix(1), disto_pix(0)))
            {
                image_ud(y, x) = sampler(imageIn, disto_pix(1), disto_pix(0));
            }
        }
    }
}

template<typename T>
void ImageRemoveDistortion(const image::Image<T>& imageIn,
                        const camera::IntrinsicBase & intrinsicSource,
                        const camera::IntrinsicBase & intrinsicOutput,
                        image::Image<T>& image_ud,
                        T fillcolor,
                        const oiio::ROI& roi = oiio::ROI())
{
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
            const Vec2 disto_pix = intrinsicSource.cam2ima(intrinsicSource.addDistortion(
              intrinsicOutput.ima2cam(undisto_pix)));

            // pick pixel if it is in the image domain
            if (imageIn.contains(disto_pix(1), disto_pix(0)))
            {
                image_ud(y, x) = sampler(imageIn, disto_pix(1), disto_pix(0));
            }
        }
    }
}

/**
 * @Brief process an image such that they appear captured by a new virtual intrinsic
 * @param dstFileName the image output file name
 * @param outputIntrinsic the virtual camera intrinsic
 * @param sourceIntrinsic read image real intrinsic
 * @param viewId the image view Id
 * @param srcFileName the initial image file path
 * @param evCorrection do we apply exposure compensation
 * @param exportFullRod do we export the full rod or not
 * @param cameraExposure current image camera exposure
 * @param medianCameraExposure median camera exposure for the sfmData
 * @param masksFolders the mask folders list
 * @param maskExtension the mask extension
 * @return false on error
*/
bool processImage(const std::string& dstFileName,
             const camera::IntrinsicBase & outputIntrinsic,
             const camera::IntrinsicBase & sourceIntrinsic,
             const IndexT & viewId,
             const std::string& srcFileName,
             bool evCorrection,
             bool exportFullRod,
             double cameraExposure,
             double medianCameraExposure,
             const std::vector<std::string> & masksFolders,
             const std::string & maskExtension)
{
    image::Image<image::RGBAfColor> image;
    image::Image<image::RGBAfColor> image_ud;

    oiio::ParamValueList metadata;
    try 
    {
        metadata = image::readImageMetadata(srcFileName);
    }
    catch (...)
    {
        ALICEVISION_LOG_ERROR("Impossible to read image metadata");
        return false;
    }
    
    //Compute exposure compensation
    const double ev = std::log2(1.0 / cameraExposure);
    const float exposureCompensation = float(medianCameraExposure / cameraExposure);

    // add exposure values to images metadata
    metadata.push_back(oiio::ParamValue("AliceVision:EV", float(ev)));
    metadata.push_back(oiio::ParamValue("AliceVision:EVComp", exposureCompensation));

    try
    {
        readImage(srcFileName, image, image::EImageColorSpace::LINEAR);
    }
    catch (...)
    {
        ALICEVISION_LOG_ERROR("Impossible to read image");
        return false;
    }


    //Applying optional external mask to image
    image::Image<unsigned char> mask;
    if (tryLoadMask(&mask, masksFolders, viewId, srcFileName, maskExtension))
    {
        if (image.width() * image.height() != mask.width() * mask.height())
        {
            ALICEVISION_LOG_DEBUG("Invalid image mask size: mask is ignored.");
        }
        else 
        {
            ALICEVISION_LOG_DEBUG("Applying mask.");
            for (int pix = 0; pix < image.width() * image.height(); ++pix)
            {
                const bool masked = (mask(pix) == 0);
                image(pix).a() = masked ? 0.f : 1.f;
            }
        }
    }
    

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

    oiio::ROI roi, rod;
    if (exportFullRod)
    {
        rod = computeRod(sourceIntrinsic, outputIntrinsic);
        roi = convertRodToRoi(outputIntrinsic, rod);
    }
    
    bool shortCut = (sourceIntrinsic.getType() == outputIntrinsic.getType()) &&
                    (outputIntrinsic.hasDistortion() == false);
    if (shortCut)
    {
        // undistort the image and save it
        ImageRemoveDistortion(image, sourceIntrinsic, outputIntrinsic, image_ud, image::RGBAfColor(0.0), rod);
    }
    else 
    {
        // Transform the image and save it
        ImageIntrinsicsTransform(image, sourceIntrinsic, outputIntrinsic, image_ud, image::RGBAfColor(0.0), rod);
    }

    //Write the result
    try
    {
        writeImage(dstFileName, image_ud, image::ImageWriteOptions(), metadata, roi);
    }
    catch (...)
    {
        ALICEVISION_LOG_ERROR("Impossible to write image");
        return false;
    }

    return true;
}

/**
 * @Brief process a set of images such that they appear captured by a new virtual intrinsic
 * @param input the original sfmData to parse
 * @param target the transformed sfmData which will be used for intrinsics properties
 * @param namingFunction function which defines how is the image named given the view object
 * @param evCorrection do we correct the exposure
 * @param exportFullRod do we export the full rod or not
 * @param masksFolders the mask folders list
 * @param maskExtension the mask extension
 * @param rangeStart the initial view index to process (range selection)
 * @param rangeEnd the last view index to process (range selection)
*/
bool process(const sfmData::SfMData & input, 
             sfmData::SfMData & target, 
             const NameFunction & namingFunction,
             bool evCorrection,
             bool exportFullRod,
             const std::vector<std::string> & masksFolders,
             const std::string & maskExtension,
             size_t rangeStart,
             size_t rangeEnd)
{
    size_t countElements = input.getViews().size();
    rangeEnd = std::min(countElements, rangeEnd);

    // for exposure correction
    const double medianCameraExposure = input.getMedianCameraExposureSetting().getExposure(); 

    for (int posImage = 0; posImage < countElements; posImage++)
    {
        auto viewsIt = input.getViews().begin();
        std::advance(viewsIt, posImage);

        //Retrieve view
        IndexT viewId = viewsIt->first;
        sfmData::View & view = *viewsIt->second;
        sfmData::View & outputView = target.getView(viewsIt->first);

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

        //Retrieve image name
        std::string srcFileName = view.getImage().getImagePath();

        //Update filename in sfmData
        std::string outFileName = namingFunction(view);
        outputView.getImage().setImagePath(outFileName);

        if (posImage < rangeStart && posImage >= rangeEnd)
        {
            continue;
        }

        //Process Image
        ALICEVISION_LOG_INFO("Process image " << srcFileName);
        processImage(outFileName,
                    targetIntrinsic,
                    intrinsic,
                    viewId,
                    srcFileName,
                    evCorrection,
                    exportFullRod,
                    view.getImage().getCameraExposureSetting().getExposure(),
                    medianCameraExposure,
                    masksFolders,
                    maskExtension);
    }

    return true;
}

int aliceVision_main(int argc, char* argv[])
{
    // command-line parameters
    std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
    std::string inputSfmDataFilename;
    std::string targetSfmDataFilename;
    std::string outputSfMDataFilename = "";
    std::string outFolder;
    std::string outImageFileTypeName = image::EImageFileType_enumToString(image::EImageFileType::EXR);
    std::string namingMode = "frameid";
    std::vector<std::string> masksFolders;
    std::string maskExtension = "png";
    int rangeStart = -1;
    int rangeSize = 1;
    bool evCorrection = false;
    bool exportFullROD = false;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&inputSfmDataFilename)->required(),
         "Input SfMData file.")
        ("target,t", po::value<std::string>(&targetSfmDataFilename)->required(),
         "Target SfMData file.")
        ("output,o", po::value<std::string>(&outFolder)->required(),
         "Output folder.")
        ("outputSfMData,o", po::value<std::string>(&outputSfMDataFilename)->default_value(outputSfMDataFilename), "Destination sfmData.");

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
        ("exportFullROD", po::value<bool>(&exportFullROD)->default_value(exportFullROD),
         "Export undistorted images with the full Region of Definition (RoD). Only supported by the EXR image file format.")
        ("namingMode", po::value<std::string>(&namingMode)->default_value(namingMode),
         "naming mode.")
        ("masksFolders", po::value<std::vector<std::string>>(&masksFolders)->multitoken(),
         "Use masks from specific folder(s).\n"
         "Filename should be the same or the image UID.")
        ("maskExtension", po::value<std::string>(&maskExtension)->default_value(maskExtension),
         "File extension of the masks to use.");
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

    // Check compatibility
    if (exportFullROD && outputFileType != image::EImageFileType::EXR)
    {
        ALICEVISION_LOG_ERROR("Export full RoD (Region Of Definition) is only possible in EXR file format and not in '" << outputFileType << "'.");
        return EXIT_FAILURE;
    }

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
            const std::string baseFilename = utils::to_string_with_zero_padding(view.getFrameId(), 10);
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

    if (!process(inputSfmData, 
                targetSfmData, 
                namingFunction, 
                evCorrection,
                exportFullROD,
                masksFolders, 
                maskExtension,
                rangeStart, 
                rangeEnd))
    {
        ALICEVISION_LOG_ERROR("Process failed");
        return EXIT_FAILURE;
    }

    if (rangeStart == 0)
    {
        if (!outputSfMDataFilename.empty())
        {
            sfmDataIO::save(targetSfmData, outputSfMDataFilename, sfmDataIO::ESfMData::ALL);
        }
    }

    return EXIT_SUCCESS;
}
