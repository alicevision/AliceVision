// This file is part of the AliceVision project.
// Copyright (c) 2018 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/sfmDataIO/AlembicExporter.hpp>
#include <aliceVision/sfmDataIO/viewIO.hpp>
#include <aliceVision/image/all.hpp>
#include <aliceVision/system/ProgressDisplay.hpp>
#include <aliceVision/utils/regexFilter.hpp>
#include <aliceVision/camera/camera.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <set>
#include <cstdlib>
#include <limits>
#include <string>
#include <regex>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 2
#define ALICEVISION_SOFTWARE_VERSION_MINOR 1

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

oiio::ROI computeRod(const std::shared_ptr<camera::IntrinsicsScaleOffset> intrinsicSource,
                     const std::shared_ptr<camera::IntrinsicsScaleOffset> intrinsicOutput,
                     const std::shared_ptr<camera::Undistortion> undistortion)
{
    std::vector<Vec2> pointToBeChecked;
    pointToBeChecked.push_back(Vec2(0, 0));
    pointToBeChecked.push_back(Vec2(intrinsicSource->w() - 1, 0));
    pointToBeChecked.push_back(Vec2(0, intrinsicSource->h() - 1));
    pointToBeChecked.push_back(Vec2(intrinsicSource->w() - 1, intrinsicSource->h() - 1));
    
    const Vec2 opticalCenter = intrinsicSource->getPrincipalPoint();
    pointToBeChecked.push_back(Vec2(opticalCenter[0], 0));
    pointToBeChecked.push_back(Vec2(opticalCenter[0], intrinsicSource->h() - 1));
    pointToBeChecked.push_back(Vec2(0, opticalCenter[1]));
    pointToBeChecked.push_back(Vec2(intrinsicSource->w() - 1, opticalCenter[1]));

    std::vector<Vec2> maxDistortionVector;
    for(const Vec2& n: pointToBeChecked)
    {
        // Undistort pixel without principal point correction
        Vec2 n_undist = intrinsicOutput->cam2ima(
            intrinsicSource->removeDistortion(intrinsicSource->ima2cam((undistortion) ? undistortion->undistort(n) : n))
        );
        maxDistortionVector.push_back(n_undist);
    }

    std::sort(std::begin(maxDistortionVector), std::end(maxDistortionVector), [](Vec2 a, Vec2 b) { return a[0] > b[0]; });
    const int xRoiMax = std::round(maxDistortionVector.front()[0]);
    const int xRoiMin = std::round(maxDistortionVector.back()[0]);
    std::sort(std::begin(maxDistortionVector), std::end(maxDistortionVector), [](Vec2 a, Vec2 b) { return a[1] > b[1]; });
    const int yRoiMax = std::round(maxDistortionVector.front()[1]);
    const int yRoiMin = std::round(maxDistortionVector.back()[1]);

    oiio::ROI rod(xRoiMin, xRoiMax + 1,yRoiMin, yRoiMax + 1);

    return rod;
}


oiio::ROI convertRodToRoi(const camera::IntrinsicBase* intrinsic, const oiio::ROI& rod)
{
    const int xOffset = rod.xbegin;
    const int yOffset = rod.ybegin; // (intrinsic->h() - rod.yend);
    const oiio::ROI roi(-xOffset, intrinsic->w() - xOffset, -yOffset, intrinsic->h() - yOffset);

    ALICEVISION_LOG_DEBUG("roi:" << roi.xbegin << ";" << roi.xend << ";" << roi.ybegin << ";" << roi.yend);
    return roi;
}

/// Undistortion 2D MAP according to a given camera and its distortion model
void UndistortMap(image::Image<image::RGBAfColor>& stmap,
                  const std::shared_ptr<camera::IntrinsicsScaleOffset> intrinsicSource,
                  const std::shared_ptr<camera::IntrinsicsScaleOffset> intrinsicOutput,
                  const std::shared_ptr<camera::Undistortion> undistortion,
                  const oiio::ROI& roi = oiio::ROI())
{
    int widthRoi = intrinsicOutput->w();
    int heightRoi = intrinsicOutput->h();
    int xOffset = 0;
    int yOffset = 0;
    if (roi.defined())
    {
        widthRoi = roi.width();
        heightRoi = roi.height();
        xOffset = roi.xbegin;
        yOffset = roi.ybegin;
    }

    stmap.resize(widthRoi, heightRoi, true, image::RGBAfColor(0.0f));
    const image::Sampler2d<image::SamplerLinear> sampler;

#pragma omp parallel for
    for (int i = 0; i < heightRoi; ++i)
    {
        for (int j = 0; j < widthRoi; ++j)
        {
            const Vec2 undisto_pix((j + xOffset), (i + yOffset));

            // compute coordinates with distortion
            const Vec2 disto_pix = intrinsicSource->cam2ima(
                intrinsicSource->addDistortion(intrinsicOutput->ima2cam((undistortion) ? undistortion->inverse(undisto_pix) : undisto_pix))
            );

            if (disto_pix.x() < 0 || disto_pix.x() >= intrinsicSource->w()) continue;
            if (disto_pix.y() < 0 || disto_pix.y() >= intrinsicSource->h()) continue;

            stmap(i, j).r() = float((disto_pix[0]) / (float(intrinsicSource->w()) - 1.0f));
            stmap(i, j).g() = float((float(intrinsicSource->h()) - 1.0f - disto_pix[1]) / (float(intrinsicSource->h()) - 1.0f));
        }
    }
}

/// Distortion 2D MAP according to a given camera and its distortion model
void distortMap(image::Image<image::RGBAfColor>& stmap,
                const std::shared_ptr<camera::IntrinsicsScaleOffset> intrinsicSource,
                const std::shared_ptr<camera::IntrinsicsScaleOffset> intrinsicOutput,
                const std::shared_ptr<camera::Undistortion> undistortion,
                const oiio::ROI& roi = oiio::ROI())
{
    int widthRoi = intrinsicSource->w();
    int heightRoi = intrinsicSource->h();
    int xOffset = 0;
    int yOffset = 0;
    if (roi.defined())
    {
        widthRoi = roi.width();
        heightRoi = roi.height();
        xOffset = roi.xbegin;
        yOffset = roi.ybegin;
    }

    stmap.resize(widthRoi, heightRoi, true, image::RGBAfColor(0.0f));
    const image::Sampler2d<image::SamplerLinear> sampler;
    
    #pragma omp parallel for
    for (int i = 0; i < heightRoi; ++i)
    {
        for (int j = 0; j < widthRoi; ++j)
        {
            const Vec2 disto_pix((j + xOffset), (i + yOffset));

            const Vec2 undisto_pix = intrinsicOutput->cam2ima(
                intrinsicSource->removeDistortion(intrinsicSource->ima2cam((undistortion) ? undistortion->undistort(disto_pix) : disto_pix))
            );

            stmap(i, j).b() = float((undisto_pix[0]) / (float(intrinsicOutput->w()) - 1.0f));
            stmap(i, j).a() = float((float(intrinsicOutput->h()) - 1.0f - undisto_pix[1]) / (float(intrinsicOutput->h()) - 1.0f));
            stmap(i, j).r() = stmap(i, j).b();
            stmap(i, j).g() = stmap(i, j).a();
        }
    }
}

/// Distortion 2D MAP according to a given camera and its distortion model
void distortMapChecker(image::Image<image::RGBAfColor>& checker,
                       const std::shared_ptr<camera::IntrinsicsScaleOffset> intrinsicSource,
                       const std::shared_ptr<camera::IntrinsicsScaleOffset> intrinsicOutput,
                       const std::shared_ptr<camera::Undistortion> undistortion,
                       const oiio::ROI& roi = oiio::ROI())
{
    int widthRoi = intrinsicSource->w();
    int heightRoi = intrinsicSource->h();
    int xOffset = 0;
    int yOffset = 0;
    if (roi.defined())
    {
        widthRoi = roi.width();
        heightRoi = roi.height();
        xOffset = roi.xbegin;
        yOffset = roi.ybegin;
    }

    checker.resize(widthRoi, heightRoi, true, image::RGBAfColor(1.0f));
    const image::Sampler2d<image::SamplerLinear> sampler;

 
#pragma omp parallel for
    for (int i = 0; i < heightRoi; ++i)
    {
        for (int j = 0; j < widthRoi; ++j)
        {
            const Vec2 undisto_pix((j + xOffset), (i + yOffset));

            // compute coordinates with distortion
            const Vec2 disto_pix = intrinsicSource->cam2ima(
                intrinsicSource->addDistortion(intrinsicOutput->ima2cam((undistortion) ? undistortion->inverse(undisto_pix) : undisto_pix))
            );

            if (disto_pix(0) < 0 || disto_pix(1) < 0) continue;
            if (disto_pix(0) >= widthRoi || disto_pix(1) >= heightRoi) continue;

            if (std::abs(i % 50) < 5 || std::abs(j % 50) < 5)
            {
                checker(disto_pix(1), disto_pix(0)) = image::RGBAfColor((double(i) / (50.0*255.0)) * 5.0, (double(i) / (50.0*255.0)) * 5.0, 0.0);
            }
        }
    }
}

int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string sfmDataFilename;
    std::string outFolder;

    // user optional parameters
    bool undistortedImages = false;
    bool exportUVMaps = false;
    bool exportFullROD = false;
    bool correctPrincipalPoint = true;
    bool correctPixelRatio = true;

    std::map<IndexT, oiio::ROI> roiForIntrinsic;
    std::string viewFilter;
    std::string sfmDataFilterFilepath;
    std::string outImageFileTypeName = image::EImageFileType_enumToString(image::EImageFileType::JPEG);
    std::string outMapFileTypeName = image::EImageFileType_enumToString(image::EImageFileType::EXR);

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
        "SfMData file containing a complete SfM.")
    ("output,o", po::value<std::string>(&outFolder)->required(),
        "Output folder.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
    ("exportUndistortedImages", po::value<bool>(&undistortedImages)->default_value(undistortedImages),
        "Export undistorted images for the animated camera(s).\n"
        "If false, animated camera(s) exported with original frame paths.")
    ("exportFullROD", po::value<bool>(&exportFullROD)->default_value(exportFullROD),
        "Export undistorted images and/or STMap with the full Region of Definition (RoD). Only supported by the EXR image file format.")
    ("exportUVMaps", po::value<bool>(&exportUVMaps)->default_value(exportUVMaps),
        "Export UV Maps in exr format to apply distort/undistort transformations in a compositing software.")
    ("correctPrincipalPoint", po::value<bool>(&correctPrincipalPoint)->default_value(correctPrincipalPoint),
        "apply an offset to correct the position of the principal point")
    ("correctPixelRatio", po::value<bool>(&correctPixelRatio)->default_value(correctPixelRatio),
        "apply a scale such that the output pixels are square")
    ("viewFilter", po::value<std::string>(&viewFilter)->default_value(viewFilter),
        "Select the cameras to export using an expression based on the image filepath. Export all cameras if empty.")
    ("sfmDataFilter", po::value<std::string>(&sfmDataFilterFilepath)->default_value(sfmDataFilterFilepath),
        "Filter out cameras from the export if they are part of this SfMData. Export all cameras if empty.")
    ("undistortedImageType", po::value<std::string>(&outImageFileTypeName)->default_value(outImageFileTypeName),
        image::EImageFileType_informations().c_str());

    CmdLine cmdline("AliceVision exportAnimatedCamera");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // set output file type
    const image::EImageFileType outputFileType = image::EImageFileType_stringToEnum(outImageFileTypeName);
    const image::EImageFileType outputMapFileType = image::EImageFileType_stringToEnum(outMapFileTypeName);

    if(exportFullROD && undistortedImages && outputFileType != image::EImageFileType::EXR)
    {
    ALICEVISION_LOG_ERROR("Export full RoD (Region Of Definition) is only possible in EXR file format and not in '" << outputFileType << "'.");
    return EXIT_FAILURE;
    }

    // load SfMData files
    sfmData::SfMData sfmData;
    if(!sfmDataIO::Load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
    {
    ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' cannot be read.");
    return EXIT_FAILURE;
    }

    if(sfmData.getViews().empty())
    {
    ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' is empty.");
    return EXIT_FAILURE;
    }

    sfmData::SfMData sfmDataFilter;
    if(!sfmDataFilterFilepath.empty())
    {
        if(!sfmDataIO::Load(sfmDataFilter, sfmDataFilterFilepath, sfmDataIO::ESfMData::VIEWS))
        {
        ALICEVISION_LOG_ERROR("The input filter SfMData file '" << sfmDataFilterFilepath << "' cannot be read.");
        return EXIT_FAILURE;
        }
    }
    system::Timer timer;

    // Decide the views and instrinsics to export
    sfmData::SfMData sfmDataExport;
    for(auto& viewPair : sfmData.getViews())
    {
    sfmData::View& view = *(viewPair.second);

    // regex filter
    if(!viewFilter.empty())
    {
        // Skip the view if it does not match the expression filter
        const std::regex regexFilter = utils::filterToRegex(viewFilter);
        if(!std::regex_match(view.getImagePath(), regexFilter))
            continue;
    }

    // sfmData filter
    if(!sfmDataFilterFilepath.empty())
    {
        // Skip the view if it exist in the sfmDataFilter
        if(sfmDataFilter.getViews().find(view.getViewId()) != sfmDataFilter.getViews().end())
            continue;
    }

    sfmDataExport.getViews().emplace(view.getViewId(), viewPair.second);

    // Export intrinsics if defined
    if(view.getIntrinsicId() != UndefinedIndexT)
    {
        // std::map::emplace does nothing if the key already exist
        sfmDataExport.getIntrinsics().emplace(view.getIntrinsicId(), sfmData.getIntrinsics().at(view.getIntrinsicId()));
    }

    // Export intrinsics if defined
    if (view.getUndistortionId() != UndefinedIndexT)
    {
        // std::map::emplace does nothing if the key already exist
        sfmDataExport.getUndistortions().emplace(view.getUndistortionId(), sfmData.getUndistortions().at(view.getUndistortionId()));
    }
    }

    const fs::path undistortedImagesFolderPath = fs::path(outFolder) / "undistort";
    const bool writeUndistordedResult = undistortedImages || exportUVMaps;

    if(writeUndistordedResult && !fs::exists(undistortedImagesFolderPath))
    fs::create_directory(undistortedImagesFolderPath);

    std::map<std::string, std::map<std::size_t, IndexT>> videoViewPerFrame;
    std::map<std::string, std::vector<std::pair<std::size_t, IndexT>> > dslrViewPerKey;

    std::set<std::pair<IndexT, IndexT>> unique_intrinsics;
    for (const auto& view : sfmDataExport.getViews())
    {
        unique_intrinsics.insert(std::make_pair(view.second->getIntrinsicId(), view.second->getUndistortionId()));
    }

    // export distortion map / one image per intrinsic/undistortion pair
    if(exportUVMaps)
    {
        oiio::ParamValueList targetMetadata;
        targetMetadata.push_back(oiio::ParamValue("AliceVision:storageDataType", "float"));
        for(const auto& intrinsicPair : unique_intrinsics)
        {
            const IndexT intrinsicId = intrinsicPair.first;
            const IndexT undistortionId = intrinsicPair.second;

            const auto intrinsic = sfmDataExport.getIntrinsicsharedPtr(intrinsicId);
            std::shared_ptr<camera::Undistortion> undistortion;
            if (undistortionId != UndefinedIndexT)
            {
                undistortion = sfmDataExport.getUndistortions()[undistortionId];
            } 
            
            image::Image<image::RGBAfColor> stmap;
            if (intrinsic->isValid())
            {
                std::shared_ptr<camera::IntrinsicsScaleOffset> iso_source = std::dynamic_pointer_cast<camera::IntrinsicsScaleOffset>(intrinsic);
                if (iso_source == nullptr)
                {
                continue;
                }

                std::shared_ptr<camera::IntrinsicBase> intrinsic_output(intrinsic->clone());
                std::shared_ptr<camera::IntrinsicsScaleOffset> iso_output =
                    std::dynamic_pointer_cast<camera::IntrinsicsScaleOffset>(intrinsic_output);
                if (correctPrincipalPoint)
                {
                    iso_output->setOffset({0, 0});
                }

                std::shared_ptr<camera::IntrinsicsScaleOffsetDisto> isod_output =
                    std::dynamic_pointer_cast<camera::IntrinsicsScaleOffsetDisto>(intrinsic_output);
                if (isod_output)
                {
                    isod_output->setDistortionObject(nullptr);
                }

                if (correctPixelRatio)
                {
                    Vec2 scale = iso_output->getScale();
                    scale(1) = scale(0);
                    iso_output->setScale(scale);
                }

                const std::string dstImage =
                    (undistortedImagesFolderPath / (std::to_string(intrinsicPair.first) + "_undistort_stmap.exr")).string();

                // undistort the image and save it
                if (exportFullROD)
                {
                    // build a ROI
                    const IndexT key = intrinsicPair.first;
                    oiio::ROI rod;
                    if (roiForIntrinsic.find(key) == roiForIntrinsic.end())
                    {
                        rod = computeRod(iso_source, iso_output, undistortion);
                        roiForIntrinsic[key] = rod;
                    }
                    else
                    {
                        rod = roiForIntrinsic[key];
                    }

                    UndistortMap(stmap, iso_source, iso_output, undistortion, rod);
                    const oiio::ROI roi = convertRodToRoi(intrinsic_output.get(), rod);
                    writeImage(dstImage, stmap, image::ImageWriteOptions(), targetMetadata, roi);
                }
                else
                {
                    UndistortMap(stmap, iso_source, iso_output, undistortion);
                    image::writeImage(dstImage, stmap, image::ImageWriteOptions(), targetMetadata);
                }

                //Distort st map
                {
                    const std::string dstImage =
                        (undistortedImagesFolderPath / (std::to_string(intrinsicPair.first) + "_distort_stmap.exr")).string();
                    distortMap(stmap, iso_source, iso_output, undistortion);
                    image::writeImage(dstImage, stmap, image::ImageWriteOptions(), targetMetadata);
                }

                //Distort st map checker
                {
                    const std::string dstImage =
                        (undistortedImagesFolderPath / (std::to_string(intrinsicPair.first) + "_distort_stmap_checker.exr")).string();
                    distortMapChecker(stmap, iso_source, iso_output, undistortion);
                    image::writeImage(dstImage, stmap, image::ImageWriteOptions(), targetMetadata);
                }
            }
        }
    }

    ALICEVISION_LOG_INFO("Build animated camera(s)...");

    image::Image<image::RGBfColor> image, image_ud;
    auto progressDisplay = system::createConsoleProgressDisplay(sfmDataExport.getViews().size(),
                                                                std::cout);

    for(const auto& viewPair : sfmDataExport.getViews())
    {
        const sfmData::View& view = *(viewPair.second);

        ++progressDisplay;

        const std::string imagePathStem = fs::path(viewPair.second->getImagePath()).stem().string();

        // undistort camera images
        if(undistortedImages)
        {
            sfmData::Intrinsics::const_iterator iterIntrinsic = sfmDataExport.getIntrinsics().find(view.getIntrinsicId());
            const std::string dstImage = (undistortedImagesFolderPath / (std::to_string(view.getIntrinsicId()) + "_" + imagePathStem + "." + image::EImageFileType_enumToString(outputFileType))).string();
            const std::shared_ptr<camera::IntrinsicBase> intrinsic = iterIntrinsic->second;

            image::readImage(view.getImagePath(), image, image::EImageColorSpace::LINEAR);
            oiio::ParamValueList metadata = image::readImageMetadata(view.getImagePath());

            if(intrinsic->isValid() && intrinsic->hasDistortion())
            {
                std::shared_ptr<camera::IntrinsicsScaleOffset> iso_source = std::dynamic_pointer_cast<camera::IntrinsicsScaleOffset>(intrinsic);
                if (iso_source == nullptr)
                {
                    continue;
                }

                std::shared_ptr<camera::IntrinsicBase> intrinsic_output(intrinsic->clone());
                std::shared_ptr<camera::IntrinsicsScaleOffset> iso_output = std::dynamic_pointer_cast<camera::IntrinsicsScaleOffset>(intrinsic_output);

                if (correctPrincipalPoint)
                {
                    iso_output->setOffset({0, 0});
                }

                if (correctPixelRatio)
                {
                    Vec2 scale = iso_output->getScale();
                    scale(1) = scale(0);
                    iso_output->setScale(scale);
                }

                std::shared_ptr<camera::IntrinsicsScaleOffsetDisto> isod_output =
                    std::dynamic_pointer_cast<camera::IntrinsicsScaleOffsetDisto>(intrinsic_output);
                if (isod_output)
                {
                    isod_output->setDistortionObject(nullptr);
                }

                IndexT undistortionId = view.getUndistortionId();
                std::shared_ptr<camera::Undistortion> undistortion = nullptr;
                if (undistortionId != UndefinedIndexT)
                {
                    undistortion = sfmData.getUndistortions()[undistortionId];
                }
                
                // undistort the image and save it
                if(exportFullROD)
                {
                    // build a ROI
                    const IndexT key = view.getIntrinsicId();
                    oiio::ROI rod;
                    
                    if(roiForIntrinsic.find(key) == roiForIntrinsic.end())
                    {
                        rod = computeRod(iso_source, iso_output, undistortion);
                        roiForIntrinsic[key] = rod;
                    }
                    else
                    {
                        rod = roiForIntrinsic[key];
                    }

                    ALICEVISION_LOG_DEBUG("rod:" + std::to_string(rod.xbegin) + ";" + std::to_string(rod.xend) + ";" +
                                            std::to_string(rod.ybegin) + ";" + std::to_string(rod.yend));

                    camera::UndistortImage(image,
                                        (camera::IntrinsicBase*)iso_source.get(),
                                        (camera::IntrinsicBase*)iso_output.get(),
                                        undistortion.get(),
                                        image_ud,
                                        image::FBLACK,
                                        rod);
                    const oiio::ROI roi = convertRodToRoi(intrinsic_output.get(), rod);
                    writeImage(dstImage, image_ud, image::ImageWriteOptions(), oiio::ParamValueList(), roi);
                }
                else
                {
                    camera::UndistortImage(image,
                                        (camera::IntrinsicBase*)iso_source.get(),
                                        (camera::IntrinsicBase*)iso_output.get(),
                                        undistortion.get(),
                                        image_ud,
                                        image::FBLACK);
                    image::writeImage(dstImage, image_ud, image::ImageWriteOptions(), metadata);
                }
            }
            else // (no distortion)
            {
                // copy the image since there is no distortion
                image::writeImage(dstImage, image, image::ImageWriteOptions(), metadata);
            }
        }

        // pose and intrinsic defined
        // Note: we use "sfmData" and not "sfmDataExport" to have access to poses
        if(!sfmData.isPoseAndIntrinsicDefined(&view))
            continue;

        std::string cameraName =  view.getMetadataMake() + "_" + view.getMetadataModel();
        IndexT frameN = 0;
        bool isSequence = false;

        if(view.isPartOfRig())
            cameraName += std::string("_") + std::to_string(view.getSubPoseId());
        {
            std::string prefix;
            std::string suffix;

            if(sfmDataIO::extractNumberFromFileStem(imagePathStem, frameN, prefix, suffix))
            {
                if(prefix.empty() && suffix.empty())
                    cameraName = std::string("Undefined") + "_" + cameraName;
                else
                    cameraName = prefix + "frame" + suffix + "_" + cameraName;

                isSequence = true;
            }
        }

        ALICEVISION_LOG_TRACE("imagePathStem: " << imagePathStem
                              << ", frameN: " << frameN
                              << ", isSequence: " << isSequence
                              << ", cameraName: " << cameraName);

        if(isSequence) // video
        {
            const std::size_t frame = frameN;
            videoViewPerFrame[cameraName][frame] = view.getViewId();
        }
        else if(view.hasMetadataDateTimeOriginal()) // picture
        {
            const std::size_t key = view.getMetadataDateTimestamp();

            dslrViewPerKey[cameraName].push_back({key, view.getViewId()});
        }
        else // no time or sequence information
        {
            dslrViewPerKey[cameraName].push_back({0, view.getViewId()});
        }
    }

    // print results
    {
        std::stringstream ss;

        ss << "Camera(s) found:" << std::endl << "\t- # video camera(s): " << videoViewPerFrame.size() << std::endl;

        for(const auto& camera : videoViewPerFrame)
            ss << "\t    - " << camera.first << " | " << camera.second.size() << " frame(s)" << std::endl;

        ss << "\t- # dslr camera(s): " << dslrViewPerKey.size() << std::endl;

        for(const auto& camera : dslrViewPerKey)
            ss << "\t    - " << camera.first << " | " << camera.second.size() << " image(s)" << std::endl;

        ss << "\t- # Used camera intrinsics: " << sfmDataExport.getIntrinsics().size() << std::endl;

        for(const auto& intrinsicIt : sfmDataExport.getIntrinsics())
        {
            const auto intrinsic = intrinsicIt.second;
            ss << "\t    - "
                << intrinsicIt.first << " | "
                << intrinsic->w() << "x" << intrinsic->h()
                << " " << intrinsic->serialNumber()
                << std::endl;
        }

        ALICEVISION_LOG_INFO(ss.str());
    }

    ALICEVISION_LOG_INFO("Export animated camera(s)...");

    sfmDataIO::AlembicExporter exporter((fs::path(outFolder) / "camera.abc").string());

    for(const auto& cameraViews : videoViewPerFrame)
    {
        const std::map<std::size_t, IndexT>& frameToView = cameraViews.second;
        const std::size_t firstFrame = cameraViews.second.begin()->first;

        exporter.initAnimatedCamera(cameraViews.first, firstFrame);

        for(std::size_t frame = firstFrame; frame <= frameToView.rbegin()->first; ++frame)
        {
            const auto findFrameIt = frameToView.find(frame);

            if(findFrameIt != frameToView.end())
            {
                const IndexT viewId = findFrameIt->second;

                // Note: we use "sfmData" and not "sfmDataExport" to have access to poses

                const auto findViewIt = sfmData.getViews().find(viewId);
                assert(findViewIt != sfmData.getViews().end());

                ALICEVISION_LOG_DEBUG("[" + cameraViews.first +"][video] Keyframe added");
                const IndexT intrinsicId = findViewIt->second->getIntrinsicId();
                const camera::Pinhole* cam = dynamic_cast<camera::Pinhole*>(sfmData.getIntrinsicPtr(intrinsicId));
                const sfmData::CameraPose pose = sfmData.getPose(*findViewIt->second);
                const std::string& imagePath = findViewIt->second->getImagePath();
                const std::string undistortedImagePath =
                    (undistortedImagesFolderPath / (std::to_string(intrinsicId)
                    + "_" + fs::path(imagePath).stem().string()
                    + "." + image::EImageFileType_enumToString(outputFileType))).string();

                exporter.addCameraKeyframe(pose.getTransform(), cam, (undistortedImages) ? undistortedImagePath : imagePath, viewId, intrinsicId);
            }
            else
            {
                exporter.jumpKeyframe(std::to_string(frame));
            }
        }
    }

    for(auto& cameraViews : dslrViewPerKey)
    {
        exporter.initAnimatedCamera(cameraViews.first);
        std::sort(cameraViews.second.begin(), cameraViews.second.end());

        for(const auto& cameraView : cameraViews.second)
        {
            ALICEVISION_LOG_DEBUG("[" + cameraViews.first +"][dslr] Keyframe added");
            const sfmData::View& view = *(sfmData.getViews().at(cameraView.second));
            const camera::Pinhole* cam = dynamic_cast<camera::Pinhole*>(sfmData.getIntrinsicPtr(view.getIntrinsicId()));
            const sfmData::CameraPose pose = sfmData.getPose(view);
            const std::string& imagePath = view.getImagePath();
            const std::string undistortedImagePath = (undistortedImagesFolderPath / (std::to_string(view.getIntrinsicId()) + "_" + fs::path(imagePath).stem().string() + "." + image::EImageFileType_enumToString(outputFileType))).string();

            exporter.addCameraKeyframe(pose.getTransform(),
                                       cam, (undistortedImages) ? undistortedImagePath : imagePath,
                                       view.getViewId(),
                                       view.getIntrinsicId());
        }
    }

    ALICEVISION_LOG_INFO("Task done in (s): " + std::to_string(timer.elapsed()));
    return EXIT_SUCCESS;
}
