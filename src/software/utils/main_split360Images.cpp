// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// Copyright (c) 2016 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/image/io.hpp>
#include <aliceVision/image/Sampler.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/main.hpp>

#include <dependencies/vectorGraphics/svgDrawer.hpp>
#include <aliceVision/panorama/sphericalMapping.hpp>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/math/constants/constants.hpp>

#include <OpenImageIO/imageio.h>
#include <OpenImageIO/imagebuf.h>
#include <OpenImageIO/imagebufalgo.h>

#include <string>
#include <iostream>
#include <iterator>
#include <fstream>
#include <vector>
#include <memory>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 2
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace fs = boost::filesystem;
namespace po = boost::program_options;
namespace oiio = OIIO;

/**
 * @brief A pinhole camera with its associated rotation
 * Used to sample the spherical image
 */
class PinholeCameraR
{
public:

    PinholeCameraR(int focal, int width, int height, const Mat3& R)
        : _R(R)
    {
    _K << focal,   0,  width/2.0,
            0, focal, height/2.0,
            0,     0,          1;
    }

    Vec3 getLocalRay(double x, double y) const
    {
        return (_K.inverse() * Vec3(x, y, 1.0)).normalized();
    }

    Vec3 getRay(double x, double y) const
    {
        return _R * getLocalRay(x, y);
    }

private:

    /// Rotation matrix
    Mat3 _R;
    /// Intrinsic matrix
    Mat3 _K;

};

/**
 * @brief Compute a rectilinear camera focal from an angular FoV
 * @param h
 * @param thetaMax camera FoV
 * @return
 */
double focalFromPinholeHeight(int height, double thetaMax = degreeToRadian(60.0))
{
    float f = 1.f;
    while (thetaMax < atan2(height / (2 * f) , 1))
    {
        ++f;
    }
    return f;
}

bool splitDualFisheye(sfmData::SfMData& outSfmData,
                      const std::string& imagePath, const std::string& outputFolder, const std::string& extension,
                      const std::string& splitPreset)
{
    // Load source image from disk
    image::Image<image::RGBfColor> imageSource;
    image::readImage(imagePath, imageSource, image::EImageColorSpace::LINEAR);

    // Retrieve its metadata
    auto metadataSource = image::readImageMetadata(imagePath);

    // Make sure image is horizontal
    if(imageSource.Height() > imageSource.Width())
    {
        ALICEVISION_THROW_ERROR("Cannot split dual fisheye from the vertical image " << imagePath);
    }

    // Retrieve useful dimensions for cropping
    const int outSide = std::min(imageSource.Height(), imageSource.Width() / 2);
    const int offset = std::abs((imageSource.Width() / 2) - imageSource.Height());
    const int halfOffset = offset / 2;

    // Make sure rig folder exists
    std::string rigFolder = outputFolder + "/rig";
    fs::create_directory(rigFolder);

    for (std::size_t i = 0; i < 2; ++i)
    {
        // Retrieve corner position of cropping area
        const int xbegin = i * outSide;
        int ybegin = 0;

        if (splitPreset == "bottom")
        {
            ybegin += offset;
        }
        else if (splitPreset == "center")
        {
            ybegin += halfOffset;
        }

        // Create new image containing the cropped area
        image::Image<image::RGBfColor> imageOut(imageSource.block(ybegin, xbegin, outSide, outSide));

        // Make sure sub-folder exists for complete rig structure
        std::string subFolder = rigFolder + std::string("/") + std::to_string(i);
        fs::create_directory(subFolder);

        // Save new image on disk
        fs::path path(imagePath);
        std::string filename = extension.empty() ?
            path.filename().string() : path.stem().string() + "." + extension;
        image::writeImage(subFolder + std::string("/") + filename,
                          imageOut, image::ImageWriteOptions(), metadataSource);
        
        // Initialize view and add it to SfMData
        #pragma omp critical (split360Images_addView)
        {
            auto& views = outSfmData.getViews();
            IndexT viewId = views.size();
            auto view = std::make_shared<sfmData::View>(
                /* image path */  subFolder + std::string("/") + filename,
                /* viewId */      viewId,
                /* intrinsicId */ UndefinedIndexT,
                /* poseId */      UndefinedIndexT,
                /* width */       outSide,
                /* height */      outSide,
                /* rigId */       0,
                /* subPoseId */   i,
                /* metadata */    image::getMapFromMetadata(metadataSource)
                );
            views[viewId] = view;
        }
    }

    // Success
    ALICEVISION_LOG_INFO(imagePath + " successfully split");
    return true;
}

bool splitEquirectangular(sfmData::SfMData& outSfmData,
                          const std::string& imagePath, const std::string& outputFolder, const std::string& extension,
                          std::size_t nbSplits, std::size_t splitResolution, double fovDegree)
{
    // Load source image from disk
    image::Image<image::RGBColor> imageSource;
    image::readImage(imagePath, imageSource, image::EImageColorSpace::LINEAR);

    const int inWidth = imageSource.Width();
    const int inHeight = imageSource.Height();

    std::vector<PinholeCameraR> cameras;

    const double twoPi = boost::math::constants::pi<double>() * 2.0;
    const double alpha = twoPi / static_cast<double>(nbSplits);

    const double fov = degreeToRadian(fovDegree);
    const double focal_px = (splitResolution / 2.0) / tan(fov / 2.0);

    double angle = 0.0;
    for(std::size_t i = 0; i < nbSplits; ++i)
    {
        cameras.emplace_back(focal_px, splitResolution, splitResolution, RotationAroundY(angle));
        angle += alpha;
    }

    const image::Sampler2d<image::SamplerLinear> sampler;
    image::Image<image::RGBColor> imaOut(splitResolution, splitResolution, image::BLACK);

    // Make sure rig folder exists
    std::string rigFolder = outputFolder + "/rig";
    fs::create_directory(rigFolder);

    size_t index = 0;
    for(const PinholeCameraR& camera : cameras)
    {
        imaOut.fill(image::BLACK);

        // Backward mapping:
        // - Find for each pixels of the pinhole image where it comes from the panoramic image
        for(int j = 0; j < splitResolution; ++j)
        {
            for(int i = 0; i < splitResolution; ++i)
            {
                const Vec3 ray = camera.getRay(i, j);
                const Vec2 x = SphericalMapping::toEquirectangular(ray, inWidth, inHeight);
                imaOut(j,i) = sampler(imageSource, x(1), x(0));
            }
        }

        // Retrieve image specs and metadata
        oiio::ImageBuf bufferOut;
        image::getBufferFromImage(imageSource, bufferOut);

        oiio::ImageSpec& outMetadataSpec = bufferOut.specmod();

        outMetadataSpec.extra_attribs = image::readImageMetadata(imagePath);

        // Override make and model in order to force camera model in SfM
        outMetadataSpec.attribute("Make",  "Custom");
        outMetadataSpec.attribute("Model", "Pinhole");
        const float focal_mm = focal_px / splitResolution; // muliplied by sensorWidth (which is 1 for "Custom")
        outMetadataSpec.attribute("Exif:FocalLength", focal_mm);

        // Make sure sub-folder exists for complete rig structure
        std::string subFolder = rigFolder + std::string("/") + std::to_string(index);
        fs::create_directory(subFolder);

        // Save new image on disk
        fs::path path(imagePath);
        std::string filename = extension.empty() ?
            path.filename().string() : path.stem().string() + "." + extension;
        image::writeImage(subFolder + std::string("/") + filename,
                          imaOut, image::ImageWriteOptions(), outMetadataSpec.extra_attribs);
        
        // Initialize view and add it to SfMData
        #pragma omp critical (split360Images_addView)
        {
            auto& views = outSfmData.getViews();
            IndexT viewId = views.size();
            auto view = std::make_shared<sfmData::View>(
                /* image path */  subFolder + std::string("/") + filename,
                /* viewId */      viewId,
                /* intrinsicId */ UndefinedIndexT,
                /* poseId */      UndefinedIndexT,
                /* width */       splitResolution,
                /* height */      splitResolution,
                /* rigId */       0,
                /* subPoseId */   index,
                /* metadata */    image::getMapFromMetadata(outMetadataSpec.extra_attribs)
                );
            views[viewId] = view;
        }
        
        // Increment index
        ++index;
    }
    ALICEVISION_LOG_INFO(imagePath + " successfully split");
    return true;
}


bool splitEquirectangularPreview(const std::string& imagePath, const std::string& outputFolder,
                                 std::size_t nbSplits, std::size_t splitResolution, double fovDegree)
{
    // Load source image from disk
    image::Image<image::RGBColor> imageSource;
    image::readImage(imagePath, imageSource, image::EImageColorSpace::LINEAR);

    const int inWidth = imageSource.Width();
    const int inHeight = imageSource.Height();

    std::vector<PinholeCameraR> cameras;

    const double twoPi = boost::math::constants::pi<double>() * 2.0;
    const double alpha = twoPi / static_cast<double>(nbSplits);

    const double fov = degreeToRadian(fovDegree);
    const double focal = (splitResolution / 2.0) / tan(fov / 2.0);

    double angle = 0.0;
    for(std::size_t i = 0; i < nbSplits; ++i)
    {
        cameras.emplace_back(focal, splitResolution, splitResolution, RotationAroundY(angle));
        angle += alpha;
    }

    svg::svgDrawer svgStream(inWidth, inHeight);
    svgStream.drawRectangle(0, 0, inWidth, inHeight, svg::svgStyle().fill("black"));
    svgStream.drawImage(imagePath, inWidth, inHeight, 0, 0, 0.7f);
    svgStream.drawLine(0,0,inWidth, inHeight, svg::svgStyle().stroke("white"));
    svgStream.drawLine(inWidth,0, 0, inHeight, svg::svgStyle().stroke("white"));

    // For each cam, reproject the image borders onto the panoramic image

    for (const PinholeCameraR& camera : cameras)
    {
        // Draw the shot border with the given step
        const int step = 10;
        Vec3 ray;

        // Vertical rectilinear image border
        for (double j = 0; j <= splitResolution; j += splitResolution/static_cast<double>(step))
        {
            Vec2 pt(0.,j);
            ray = camera.getRay(pt(0), pt(1));
            Vec2 x = SphericalMapping::toEquirectangular( ray, inWidth, inHeight);
            svgStream.drawCircle(x(0), x(1), 8, svg::svgStyle().fill("magenta").stroke("white", 4));

            pt[0] = splitResolution;
            ray = camera.getRay(pt(0), pt(1));
            x = SphericalMapping::toEquirectangular( ray, inWidth, inHeight);
            svgStream.drawCircle(x(0), x(1), 8, svg::svgStyle().fill("magenta").stroke("white", 4));
        }

        // Horizontal rectilinear image border
        for (double j = 0; j <= splitResolution; j += splitResolution/static_cast<double>(step))
        {
            Vec2 pt(j,0.);
            ray = camera.getRay(pt(0), pt(1));
            Vec2 x = SphericalMapping::toEquirectangular( ray, inWidth, inHeight);
            svgStream.drawCircle(x(0), x(1), 8, svg::svgStyle().fill("lime").stroke("white", 4));

            pt[1] = splitResolution;
            ray = camera.getRay(pt(0), pt(1));
            x = SphericalMapping::toEquirectangular( ray, inWidth, inHeight);
            svgStream.drawCircle(x(0), x(1), 8, svg::svgStyle().fill("lime").stroke("white", 4));
        }
    }

    fs::path path(imagePath);
    std::ofstream svgFile(outputFolder + std::string("/") + path.stem().string() + std::string(".svg"));
    svgFile << svgStream.closeSvgFile().str();
    return true;
}

int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string inputPath;                      // media file path list or SfMData file
    std::string outputFolder;                   // output folder for splited images
    std::string outSfmDataFilepath;             // output SfMData file
    std::string splitMode;                      // split mode (exif, dualfisheye, equirectangular)
    std::string dualFisheyeSplitPreset;         // dual-fisheye split type preset
    std::size_t equirectangularNbSplits;        // nb splits for equirectangular image
    std::size_t equirectangularSplitResolution; // split resolution for equirectangular image
    bool equirectangularPreviewMode = false;
    double fov = 110.0;                         // Field of View in degree
    int nbThreads = 3;
    std::string extension;                      // extension of output images

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&inputPath)->required(),
        "Input image file, image folder or SfMData.")
        ("output,o", po::value<std::string>(&outputFolder)->required(),
        "Output folder for extracted images.")
        ("outSfMData", po::value<std::string>(&outSfmDataFilepath)->required(),
        "Filepath for output SfMData.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("splitMode,m", po::value<std::string>(&splitMode)->default_value("equirectangular"),
        "Split mode (equirectangular, dualfisheye)")
        ("dualFisheyeSplitPreset", po::value<std::string>(&dualFisheyeSplitPreset)->default_value("center"),
        "Dual-Fisheye split type preset (center, top, bottom)")
        ("equirectangularNbSplits", po::value<std::size_t>(&equirectangularNbSplits)->default_value(2),
        "Equirectangular number of splits")
        ("equirectangularSplitResolution", po::value<std::size_t>(&equirectangularSplitResolution)->default_value(1200),
        "Equirectangular split resolution")
        ("equirectangularPreviewMode", po::value<bool>(&equirectangularPreviewMode)->default_value(equirectangularPreviewMode),
        "Export a SVG file that simulate the split")
        ("fov", po::value<double>(&fov)->default_value(fov),
        "Field of View to extract (in degree).")
        ("nbThreads", po::value<int>(&nbThreads)->default_value(nbThreads),
        "Number of threads.")
        ("extension", po::value<std::string>(&extension)->default_value(extension),
        "Output image extension (empty to keep the source file format).");

    CmdLine cmdline("This program is used to extract multiple images from equirectangular or dualfisheye images or image folder.\n"
                    "AliceVision split360Images");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // Check output folder and update to its absolute path
    {
        const fs::path outDir = fs::absolute(outputFolder);
        outputFolder = outDir.string();
        if (!fs::is_directory(outDir))
        {
            ALICEVISION_LOG_ERROR("Can't find folder " << outputFolder);
            return EXIT_FAILURE;
        }
    }

    // Check split mode
    {
        // splitMode to lower
        std::transform(splitMode.begin(), splitMode.end(), splitMode.begin(), ::tolower);

        if (splitMode != "exif" &&
            splitMode != "equirectangular" &&
            splitMode != "dualfisheye")
        {
            ALICEVISION_LOG_ERROR("Invalid split mode : " << splitMode);
            return EXIT_FAILURE;
        }
    }

    // Check dual-fisheye split preset
    {
        // dualFisheyeSplitPreset to lower
        std::transform(dualFisheyeSplitPreset.begin(), dualFisheyeSplitPreset.end(), dualFisheyeSplitPreset.begin(), ::tolower);

        if (dualFisheyeSplitPreset != "top" &&
            dualFisheyeSplitPreset != "bottom" &&
            dualFisheyeSplitPreset != "center")
        {
            ALICEVISION_LOG_ERROR("Invalid dual-fisheye split preset : " << dualFisheyeSplitPreset);
            return EXIT_FAILURE;
        }
    }

    std::vector<std::string> imagePaths;
    std::vector<std::string> badPaths;

    {
        const fs::path path = fs::absolute(inputPath);
        if (fs::exists(path))
        {
            // Input is either :
            // - an image folder
            // - a single image
            // - a SfMData file (in that case we split the views)
            if (fs::is_directory(path))
            {
                for (fs::directory_entry& entry : boost::make_iterator_range(fs::directory_iterator(path), {}))
                {
                    imagePaths.push_back(entry.path().string());
                }

                ALICEVISION_LOG_INFO("Find " << imagePaths.size() << " file paths.");
            }
            else
            {
                const std::string inputExt = boost::to_lower_copy(path.extension().string());
                if (inputExt == ".sfm" || inputExt == ".abc")
                {
                    sfmData::SfMData sfmData;
                    if (!sfmDataIO::Load(sfmData, path.string(), sfmDataIO::VIEWS))
                    {
                        ALICEVISION_LOG_ERROR("The input SfMData file '" << inputPath << "' cannot be read.");
                        return EXIT_FAILURE;
                    }

                    for (const auto& [_, view] : sfmData.getViews())
                    {
                        imagePaths.push_back(view->getImagePath());
                    }
                }
                else
                {
                    imagePaths.push_back(path.string());
                }
            }
        }
        else
        {
            ALICEVISION_LOG_ERROR("Can't find file or folder " << inputPath);
            return EXIT_FAILURE;
        }
    }

    // Output SfMData is constituted of:
    // - a rig
    // - a view for each extracted image
    // - all views are part of the rig
    // - their sub-pose ID corresponds to the extraction order
    sfmData::SfMData outSfmData;

    // Initialize rig
    const unsigned int nbSubPoses =
        (splitMode == "equirectangular") ? equirectangularNbSplits : 2;
    sfmData::Rig rig(nbSubPoses);

    // Add rig to SfMData
    // Note: rig ID is 0, this convention is used in several places in this file
    auto& rigs = outSfmData.getRigs();
    rigs[0] = rig;

    // Process images
    #pragma omp parallel for num_threads(nbThreads)
    for (int i = 0; i < imagePaths.size(); ++i)
    {
        const std::string& imagePath = imagePaths[i];
        bool hasCorrectPath = true;

        if (splitMode == "equirectangular")
        {
            if(equirectangularPreviewMode)
            {
                hasCorrectPath =
                    splitEquirectangularPreview(imagePath, outputFolder,
                                                equirectangularNbSplits, equirectangularSplitResolution, fov);
            }
            else
            {
                hasCorrectPath =
                    splitEquirectangular(outSfmData,
                                         imagePath, outputFolder, extension,
                                         equirectangularNbSplits, equirectangularSplitResolution, fov);
            }
        }
        else if(splitMode == "dualfisheye")
        {
            hasCorrectPath =
                splitDualFisheye(outSfmData,
                                 imagePath, outputFolder, extension,
                                 dualFisheyeSplitPreset);
        }
        else //exif
        {
            ALICEVISION_LOG_ERROR("Exif mode not implemented yet !");
        }

        if (!hasCorrectPath)
        {
            #pragma omp critical (split360Images_badPaths)
            badPaths.push_back(imagePath);
        }
    }

    // Log filepath for images that could not be split
    if (!badPaths.empty())
    {
        ALICEVISION_LOG_ERROR("Error: Can't open image file(s) below");
        for (const std::string& imagePath : imagePaths)
            ALICEVISION_LOG_ERROR("\t - " << imagePath);
    }

    // Save sfmData with modified path to images
    if(!sfmDataIO::Save(outSfmData, outSfmDataFilepath, sfmDataIO::ESfMData(sfmDataIO::ALL)))
    {
        ALICEVISION_LOG_ERROR("The output SfMData file '" << outSfmDataFilepath << "' cannot be written.");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
