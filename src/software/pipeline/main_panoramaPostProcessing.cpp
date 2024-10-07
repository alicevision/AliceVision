// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

// Image
#include <aliceVision/image/all.hpp>

// System
#include <aliceVision/system/Logger.hpp>

// Reading command line options
#include <boost/program_options.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/main.hpp>

#include <aliceVision/stl/mapUtils.hpp>

#include <filesystem>
#include <fstream>
#include <algorithm>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <OpenImageIO/imageio.h>
#include <OpenImageIO/imagebuf.h>
#include <OpenImageIO/imagebufalgo.h>
#include <OpenImageIO/color.h>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 2
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = std::filesystem;

bool downscaleTriangle(image::Image<image::RGBAfColor>& smaller, const image::Image<image::RGBAfColor>& source)
{
    int sw = source.width();
    int sh = source.height();
    int nw = sw / 2;
    int nh = sh / 2;

    smaller = image::Image<image::RGBAfColor>(nw, nh);

    for (int i = 0; i < nh; i++)
    {
        int di = i * 2;
        int pi = di - 1;
        int ni = di + 1;
        if (pi < 0)
            pi = ni;
        if (ni >= sh)
            ni = pi;

        for (int j = 0; j < nw; j++)
        {
            int dj = j * 2;
            int pj = dj - 1;
            int nj = dj + 1;
            if (pj < 0)
                pj = nj;
            if (nj >= sw)
                nj = pj;

            image::RGBAfColor c1 = (source(pi, pj) * 0.25f) + (source(di, pj) * 0.5f) + (source(ni, pj) * 0.25f);
            image::RGBAfColor c2 = (source(pi, dj) * 0.25f) + (source(di, dj) * 0.5f) + (source(ni, dj) * 0.25f);
            image::RGBAfColor c3 = (source(pi, nj) * 0.25f) + (source(di, nj) * 0.5f) + (source(ni, nj) * 0.25f);
            image::RGBAfColor dest = c1 * 0.25f + c2 * 0.5f + c3 * 0.25f;

            if (dest.a() > 0.0f)
            {
                dest.r() = dest.r() / dest.a();
                dest.g() = dest.g() / dest.a();
                dest.b() = dest.b() / dest.a();
                dest.a() = 1.0f;
            }

            smaller(i, j) = dest;
        }
    }

    return true;
}

bool readFullTile(image::Image<image::RGBAfColor>& output, std::unique_ptr<oiio::ImageInput>& input, int tx, int ty)
{
    const oiio::ImageSpec& inputSpec = input->spec();
    const int tileSize = inputSpec.tile_width;
    const int width = inputSpec.width;
    const int height = inputSpec.height;

    const int countWidth = std::ceil(double(width) / double(tileSize));
    const int countHeight = std::ceil(double(height) / double(tileSize));

    // Utility lambda to load only part of a tile (before of after a given split position)
    auto readTilePartial = [&](image::Image<image::RGBAfColor>& buf, int txLeft, int xOutput, int xBuf, int sliceWidth) -> bool {
        if (!input->read_tile(txLeft * tileSize, ty * tileSize, 0, oiio::TypeDesc::FLOAT, buf.data()))
        {
            return false;
        }

        output.block(0, xOutput, tileSize, sliceWidth) = buf.block(0, xBuf, tileSize, sliceWidth);

        return true;
    };

    // Default filling
    output.fill(image::RGBAfColor(0.0f, 0.0f, 0.0f, 0.0f));

    // Make sure tile y-coordinate is in the right range
    if (ty < 0 || ty >= countHeight)
    {
        return true;
    }

    if (tx == -1)
    {
        // Wrap tile x-coordinate
        const int offset = width - 1 - tileSize;
        const int txLeft = offset / tileSize;

        // Size of the left and right part of the tile
        const int leftside = (txLeft + 1) * tileSize - offset;
        const int rightside = tileSize - leftside;

        image::Image<image::RGBAfColor> buf(tileSize, tileSize);

        // Load left part tile
        if (!readTilePartial(buf, txLeft, 0, rightside, leftside))
            return false;

        // Load right part tile to complete filling output
        if (rightside > 0)
        {
            if (!readTilePartial(buf, txLeft + 1, leftside, 0, rightside))
                return false;
        }
    }
    else if (tx == countWidth - 1)
    {
        // Size of the left and right part of the tile
        const int leftside = width - tx * tileSize;
        const int rightside = tileSize - leftside;

        image::Image<image::RGBAfColor> buf(tileSize, tileSize);

        // Load last tile (which may be incomplete)
        if (!readTilePartial(buf, tx, 0, 0, leftside))
            return false;

        // Load first tile to complete filling output
        if (rightside > 0)
        {
            if (!readTilePartial(buf, 0, leftside, 0, rightside))
                return false;
        }
    }
    else if (tx == countWidth)
    {
        // Wrap tile x-coordinate
        const int offset = countWidth * tileSize - width;
        const int txLeft = offset / tileSize;

        // Size of the left and right part of the tile
        const int leftside = (txLeft + 1) * tileSize - offset;
        const int rightside = tileSize - leftside;

        image::Image<image::RGBAfColor> buf(tileSize, tileSize);

        // Load left part tile
        if (!readTilePartial(buf, txLeft, 0, rightside, leftside))
            return false;

        // Load right part tile to complete filling output
        if (rightside > 0)
        {
            if (!readTilePartial(buf, txLeft + 1, leftside, 0, rightside))
                return false;
        }
    }
    else if (tx >= 0 && tx < countWidth - 1)
    {
        // Load tile data directly into output
        if (!input->read_tile(tx * tileSize, ty * tileSize, 0, oiio::TypeDesc::FLOAT, output.data()))
        {
            return false;
        }
    }

    return true;
}

void colorSpaceTransform(image::Image<image::RGBAfColor>& inputImage,
                         image::EImageColorSpace fromColorSpace,
                         image::EImageColorSpace toColorSpace,
                         image::DCPProfile dcpProf,
                         image::DCPProfile::Triple neutral)
{
    const int width = inputImage.width();
    const int tileSize = inputImage.height();
    oiio::ImageBuf inBuf =
      oiio::ImageBuf(oiio::ImageSpec(width, tileSize, 4, oiio::TypeDesc::FLOAT), const_cast<image::RGBAfColor*>(inputImage.data()));
    oiio::ImageBuf* outBuf = &inBuf;

    if (fromColorSpace == image::EImageColorSpace::NO_CONVERSION)
    {
        double cct;
        dcpProf.applyLinear(inBuf, neutral, cct, true, true);
        fromColorSpace = image::EImageColorSpace::ACES2065_1;
    }

    oiio::ImageBuf colorspaceBuf = oiio::ImageBuf(oiio::ImageSpec(width, tileSize, 4, oiio::TypeDesc::FLOAT),
                                                  const_cast<image::RGBAfColor*>(inputImage.data()));  // buffer for image colorspace modification
    if ((fromColorSpace == toColorSpace) || (toColorSpace == image::EImageColorSpace::NO_CONVERSION))
    {
        // Do nothing. Note that calling imageAlgo::colorconvert() will copy the source buffer
        // even if no conversion is needed.
    }
    else if (EImageColorSpace_isSupportedOIIOEnum(toColorSpace) && EImageColorSpace_isSupportedOIIOEnum(fromColorSpace))
    {
        oiio::ColorConfig& colorConfig(image::getGlobalColorConfigOCIO());
        oiio::ImageBufAlgo::colorconvert(colorspaceBuf,
                                         *outBuf,
                                         EImageColorSpace_enumToOIIOString(fromColorSpace),
                                         EImageColorSpace_enumToOIIOString(toColorSpace),
                                         true,
                                         "",
                                         "",
                                         &colorConfig);
        outBuf = &colorspaceBuf;
    }
    else
    {
        oiio::ImageBufAlgo::colorconvert(
          colorspaceBuf, *outBuf, EImageColorSpace_enumToOIIOString(fromColorSpace), EImageColorSpace_enumToOIIOString(toColorSpace));
        outBuf = &colorspaceBuf;
    }

    oiio::ROI exportROI = outBuf->roi();
    exportROI.chbegin = 0;
    exportROI.chend = inputImage.channels();
    outBuf->get_pixels(exportROI, outBuf->pixeltype(), inputImage.data());
}

int aliceVision_main(int argc, char** argv)
{
    std::string inputPanoramaPath;
    std::string outputPanoramaPath;
    std::string outputPanoramaPreviewPath = "";
    image::EStorageDataType storageDataType = image::EStorageDataType::Float;
    image::EImageExrCompression compressionMethod = image::EImageExrCompression::Auto;
    int compressionLevel = 0;
    image::EImageColorSpace outputColorSpace = image::EImageColorSpace::LINEAR;
    size_t previewSize = 1000;
    bool fillHoles = false;
    bool exportLevels = false;
    int lastLevelMaxSize = 3840;

    // Description of mandatory parameters
    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("inputPanorama,i", po::value<std::string>(&inputPanoramaPath)->required(),
         "Input panorama.")
        ("outputPanorama,o", po::value<std::string>(&outputPanoramaPath)->required(),
         "Path of the output panorama.");

    // Description of optional parameters
    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("storageDataType", po::value<image::EStorageDataType>(&storageDataType)->default_value(storageDataType),
         ("Storage data type: " + image::EStorageDataType_informations()).c_str())
        ("compressionMethod", po::value<image::EImageExrCompression>(&compressionMethod)->default_value(compressionMethod),
         ("Compression method: " + image::EImageExrCompression_informations()).c_str())
        ("compressionLevel", po::value<int>(&compressionLevel)->default_value(compressionLevel),
         "Compression Level (must be strictly positive to be considered).\n"
         "Only dwaa, dwab, zip and zips compression methods are concerned.")
        ("fillHoles", po::value<bool>(&fillHoles)->default_value(fillHoles),
         "Execute fill holes algorithm.")
        ("exportLevels", po::value<bool>(&exportLevels)->default_value(exportLevels),
         "Export downscaled panorama levels.")
        ("lastLevelMaxSize", po::value<int>(&lastLevelMaxSize)->default_value(lastLevelMaxSize),
         "Maximum width of smallest downscaled panorama level.")
        ("previewSize", po::value<size_t>(&previewSize)->default_value(previewSize),
         "Preview image width.")
        ("outputColorSpace", po::value<image::EImageColorSpace>(&outputColorSpace)->default_value(outputColorSpace),
         "Color space for the output panorama.")
        ("outputPanoramaPreview,p", po::value<std::string>(&outputPanoramaPreviewPath)->default_value(outputPanoramaPreviewPath),
         "Path of the output panorama preview.");
    // clang-format on

    CmdLine cmdline("This program performs estimation of cameras orientation around a nodal point for 360° panorama.\n"
                    "AliceVision panoramaPostProcessing");

    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // Open input panorama
    std::unique_ptr<oiio::ImageInput> panoramaInput = oiio::ImageInput::open(inputPanoramaPath);
    if (!panoramaInput)
    {
        return EXIT_FAILURE;
    }

    // Get information about input panorama
    const oiio::ImageSpec& inputSpec = panoramaInput->spec();
    const int tileWidth = inputSpec.tile_width;
    const int tileHeight = inputSpec.tile_height;
    image::EImageColorSpace fromColorSpace = image::EImageColorSpace_stringToEnum(inputSpec.get_string_attribute("AliceVision:ColorSpace", "linear"));

    if (tileWidth != tileHeight)
    {
        ALICEVISION_LOG_ERROR("non square tiles !");
        return EXIT_FAILURE;
    }

    int tmpWidth, tmpHeight;
    std::map<std::string, std::string> imageMetadata = image::getMapFromMetadata(image::readImageMetadata(inputPanoramaPath, tmpWidth, tmpHeight));

    image::DCPProfile dcpProf;
    image::DCPProfile::Triple neutral = {1.0, 1.0, 1.0};
    if (fromColorSpace == image::EImageColorSpace::NO_CONVERSION)
    {
        // load DCP metadata
        dcpProf.Load(imageMetadata);

        std::string cam_mul =
          map_has_non_empty_value(imageMetadata, "raw:cam_mul") ? imageMetadata.at("raw:cam_mul") : imageMetadata.at("AliceVision:raw:cam_mul");
        std::vector<float> v_mult;
        size_t last = 0;
        size_t next = 1;
        while ((next = cam_mul.find(",", last)) != std::string::npos)
        {
            v_mult.push_back(std::stof(cam_mul.substr(last, next - last)));
            last = next + 1;
        }
        v_mult.push_back(std::stof(cam_mul.substr(last, cam_mul.find("}", last) - last)));

        for (int i = 0; i < 3; i++)
        {
            neutral[i] = v_mult[i] / v_mult[1];
        }
    }

    // Create output panorama
    std::unique_ptr<oiio::ImageOutput> panoramaOutput = oiio::ImageOutput::create(outputPanoramaPath);
    oiio::ImageSpec outputSpec(inputSpec);
    outputSpec.tile_width = 0;
    outputSpec.tile_height = 0;

    std::string compressionMethod_str = "none";

    if (compressionMethod == image::EImageExrCompression::Auto)
    {
        compressionMethod_str = "zip";
    }
    else if (compressionMethod != image::EImageExrCompression::None)
    {
        compressionMethod_str = EImageExrCompression_enumToString(compressionMethod);
        if (compressionLevel > 0)
        {
            if ((compressionMethod == image::EImageExrCompression::DWAA || compressionMethod == image::EImageExrCompression::DWAB))
            {
                compressionMethod_str += ":" + std::to_string(compressionLevel);
            }
            else if ((compressionMethod == image::EImageExrCompression::ZIP || compressionMethod == image::EImageExrCompression::ZIPS))
            {
                compressionMethod_str += ":" + std::to_string(std::min<int>(compressionLevel, 9));
            }
        }
    }
    outputSpec.attribute("compression", compressionMethod_str);

    outputSpec.extra_attribs.remove("openexr:lineOrder");
    outputSpec.attribute("AliceVision:ColorSpace", image::EImageColorSpace_enumToString(outputColorSpace));

    if (!panoramaOutput->open(outputPanoramaPath, outputSpec))
    {
        ALICEVISION_LOG_ERROR("Impossible to write to destination path");
        return EXIT_FAILURE;
    }

    // Compute various sizes
    const int tileSize = tileWidth;
    const int width = inputSpec.width;
    const int height = inputSpec.height;
    const int countWidth = std::ceil(double(width) / double(tileSize));
    const int countHeight = std::ceil(double(height) / double(tileSize));
    const int rowSize = countWidth + 2;

    if (previewSize > width || previewSize <= 0)
    {
        ALICEVISION_LOG_INFO("Preview size must be inferior to the original panorama size and superior to 0");
        ALICEVISION_LOG_INFO("Falling back to recommended size of 1000");
        previewSize = 1000;
    }

    const double ratioPreview = double(width) / double(previewSize);
    image::Image<image::RGBAfColor> previewImage(previewSize, previewSize / 2);
    int previewCurrentRow = 0;

    // Create image outputs for downscaled panorama levels
    const int nbLevels =
      exportLevels ? static_cast<int>(std::min(std::floor(std::log2(tileSize)), std::ceil(std::log2(width) - std::log2(lastLevelMaxSize)))) : 0;
    std::vector<std::unique_ptr<oiio::ImageOutput>> levelOutputs;

    ALICEVISION_LOG_INFO("Number of downscaled panorama levels to generate: " << nbLevels);

    fs::path outputFSPath(outputPanoramaPath);
    for (int levelIdx = 1; levelIdx <= nbLevels; ++levelIdx)
    {
        const int levelWidth = width / (1 << levelIdx);

        fs::path levelPath = outputFSPath.parent_path() / (outputFSPath.stem().string() + "_level_" + std::to_string(levelWidth) + ".exr");
        levelOutputs.push_back(std::move(oiio::ImageOutput::create(levelPath.string())));

        oiio::ImageSpec levelSpec(outputSpec);
        levelSpec.width /= (1 << levelIdx);
        levelSpec.height /= (1 << levelIdx);
        levelSpec.full_width /= (1 << levelIdx);
        levelSpec.full_height /= (1 << levelIdx);

        if (!levelOutputs[levelIdx - 1]->open(levelPath.string(), levelSpec))
        {
            ALICEVISION_LOG_ERROR("Impossible to write downscaled panorama at level " << levelIdx);
            return EXIT_FAILURE;
        }
    }

    if (fillHoles)
    {
        ALICEVISION_LOG_INFO("Reduce image (" << width << "x" << height << ")");

        // Downscale such that each tile == 1 pixel
        image::Image<image::RGBAfColor> smallImage(countWidth, countHeight);
        bool error = false;

#pragma omp parallel for
        for (int ty = 0; ty < countHeight; ty++)
        {
            // Build subimage
            image::Image<image::RGBAfColor> region(rowSize * tileSize, 3 * tileSize);
            image::Image<image::RGBAfColor> tile(tileWidth, tileWidth);

            for (int ry = 0; ry < 3; ry++)
            {
                int dy = ry - 1;

                for (int rx = 0; rx < rowSize; rx++)
                {
                    int dx = rx - 1;

                    if (!readFullTile(tile, panoramaInput, dx, ty + dy))
                    {
                        ALICEVISION_LOG_ERROR("Invalid tile");
                        error = true;
                        break;
                    }

                    region.block(ry * tileWidth, rx * tileWidth, tileWidth, tileWidth) = tile;
                }

                if (error)
                {
                    break;
                }
            }

            if (error)
            {
                continue;
            }

            int cs = tileSize * rowSize;
            while (cs != rowSize)
            {
                int ns = cs / 2;
                image::Image<image::RGBAfColor> smaller;
                downscaleTriangle(smaller, region);
                region = smaller;
                cs = ns;
            }

            for (int j = 0; j < countWidth; j++)
            {
                int rj = j + 1;
                int dj = j;

                smallImage(ty, dj) = region(1, rj);
            }
        }

        if (error)
        {
            return EXIT_FAILURE;
        }

        ALICEVISION_LOG_INFO("Process fill holes for reduced image (" << smallImage.width() << "x" << smallImage.height() << ")");
        image::Image<image::RGBAfColor> smallFiled(smallImage.width(), smallImage.height());
        oiio::ImageBuf inBuf(oiio::ImageSpec(smallImage.width(), smallImage.height(), 4, oiio::TypeDesc::FLOAT), smallImage.data());
        oiio::ImageBuf outBuf(oiio::ImageSpec(smallImage.width(), smallImage.height(), 4, oiio::TypeDesc::FLOAT), smallFiled.data());
        oiio::ImageBufAlgo::fillholes_pushpull(outBuf, inBuf);

        ALICEVISION_LOG_INFO("Upscaling and filling holes");
        // Process one full row of tiles each iteration
        for (int ty = 0; ty < countHeight; ty++)
        {
            int ybegin = ty * tileSize;
            int yend = (ty + 1) * tileSize - 1;

            // Build subimage
            image::Image<image::RGBAfColor> region(tileSize * rowSize, tileSize * 3);
            image::Image<image::RGBAfColor> subFiled(rowSize, 3, true, image::RGBAfColor(0.0f, 0.0f, 0.0f, 0.0f));
            image::Image<image::RGBAfColor> final(width, tileSize, true, image::RGBAfColor(0.0f, 0.0f, 0.0f, 0.0f));

            // Build a region
            for (int ry = 0; ry < 3; ry++)
            {
                int dy = ry - 1;
                int cy = ty + dy;

#pragma omp parallel for
                for (int rx = 0; rx < rowSize; rx++)
                {
                    int dx = rx - 1;
                    int cx = dx;

                    image::Image<image::RGBAfColor> tile(tileSize, tileSize);
                    if (!readFullTile(tile, panoramaInput, cx, cy))
                    {
                        ALICEVISION_LOG_ERROR("Invalid tile");
                        continue;
                    }

                    region.block(ry * tileSize, rx * tileSize, tileSize, tileSize) = tile;

                    if (cy < 0 || cy >= smallFiled.height())
                    {
                        continue;
                    }

                    if (cx < 0)
                    {
                        cx = smallFiled.width() + cx;
                    }

                    if (cx >= smallFiled.width())
                    {
                        cx = cx - smallFiled.width();
                    }

                    subFiled(ry, rx) = smallFiled(cy, cx);
                }
            }

            // First level is original image
            std::vector<image::Image<image::RGBAfColor>> pyramid;
            pyramid.push_back(region);

            // Build pyramid for tile
            int cs = tileSize * rowSize;
            while (cs != rowSize)
            {
                int ns = cs / 2;
                image::Image<image::RGBAfColor> smaller;
                downscaleTriangle(smaller, region);
                pyramid.push_back(smaller);
                region = smaller;
                cs = ns;
            }

            pyramid[pyramid.size() - 1] = subFiled;

            for (int level = pyramid.size() - 2; level >= 0; level--)
            {
                const image::Image<image::RGBAfColor>& source = pyramid[level + 1];

                image::Image<image::RGBAfColor>& dest = pyramid[level];

#pragma omp parallel for
                for (int i = 0; i < source.height(); i++)
                {
                    int di = i * 2;
                    int pi = i - 1;
                    int ni = i + 1;

                    if (pi < 0)
                        pi = -pi;
                    if (ni >= source.height())
                        ni = i;

                    for (int j = 0; j < source.width(); j++)
                    {
                        int dj = j * 2;
                        int pj = j - 1;
                        int nj = j + 1;

                        if (pj < 0)
                            pj = nj;
                        if (nj >= source.width())
                            nj = j;

                        image::RGBAfColor c11 = source(pi, pj);
                        image::RGBAfColor c12 = source(pi, j);
                        image::RGBAfColor c13 = source(pi, nj);
                        image::RGBAfColor c21 = source(i, pj);
                        image::RGBAfColor c22 = source(i, j);
                        image::RGBAfColor c23 = source(i, nj);
                        image::RGBAfColor c31 = source(ni, pj);
                        image::RGBAfColor c32 = source(ni, j);
                        image::RGBAfColor c33 = source(ni, nj);

                        image::RGBAfColor f1 = c12 * 0.75f + c13 * 0.25f;
                        image::RGBAfColor f2 = c22 * 0.75f + c23 * 0.25f;
                        image::RGBAfColor f3 = c32 * 0.75f + c33 * 0.25f;
                        image::RGBAfColor b1 = c12 * 0.75f + c11 * 0.25f;
                        image::RGBAfColor b2 = c22 * 0.75f + c21 * 0.25f;
                        image::RGBAfColor b3 = c32 * 0.75f + c31 * 0.25f;

                        image::RGBAfColor d11 = b1 * 0.25f + b2 * 0.75f;
                        image::RGBAfColor d21 = b3 * 0.25f + b2 * 0.75f;
                        image::RGBAfColor d12 = f1 * 0.25f + f2 * 0.75f;
                        image::RGBAfColor d22 = f3 * 0.25f + f2 * 0.75f;

                        // Here we normalize without checking for alpha validity,
                        // as we assume it is not possible to have zero alpha if the algorithm did its job.

                        if (dest(di, dj).a() != 1.0f)
                            dest(di, dj) = d11 / d11.a();
                        if (dest(di, dj + 1).a() != 1.0f)
                            dest(di, dj + 1) = d12 / d12.a();
                        if (dest(di + 1, dj).a() != 1.0f)
                            dest(di + 1, dj) = d21 / d21.a();
                        if (dest(di + 1, dj + 1).a() != 1.0f)
                            dest(di + 1, dj + 1) = d22 / d22.a();
                    }
                }
            }

            const image::Image<image::RGBAfColor>& finalTile = pyramid[0];
            final.block(0, 0, tileSize, width) = finalTile.block(tileSize, tileSize, tileSize, width);

            // Fill preview image
            while (previewCurrentRow < previewImage.rows())
            {
                double finalY = ratioPreview * double(previewCurrentRow) - ybegin;
                if (finalY < 0 || finalY >= tileSize)
                {
                    break;
                }

                int by = int(finalY);
                for (int px = 0; px < previewImage.cols(); px++)
                {
                    int bx = int(ratioPreview * double(px));
                    previewImage(previewCurrentRow, px) = final(by, bx);
                }

                previewCurrentRow++;
            }

            // Write panorama output
            colorSpaceTransform(final, fromColorSpace, outputColorSpace, dcpProf, neutral);
            panoramaOutput->write_scanlines(ty * tileSize, (ty + 1) * tileSize, 0, oiio::TypeDesc::FLOAT, final.data());

            // Write downscaled panorama levels
            for (int levelIdx = 1; levelIdx <= nbLevels; ++levelIdx)
            {
                image::Image<image::RGBAfColor>& levelTile = pyramid[levelIdx];
                const int levelTileSize = tileSize / (1 << levelIdx);
                const int levelWidth = width / (1 << levelIdx);
                image::Image<image::RGBAfColor> level(levelWidth, levelTileSize);
                level.block(0, 0, levelTileSize, levelWidth) = levelTile.block(levelTileSize, levelTileSize, levelTileSize, levelWidth);
                colorSpaceTransform(level, fromColorSpace, outputColorSpace, dcpProf, neutral);
                levelOutputs[levelIdx - 1]->write_scanlines(ty * levelTileSize, (ty + 1) * levelTileSize, 0, oiio::TypeDesc::FLOAT, level.data());
            }
        }
    }
    else
    {
        // Process one full row of tiles each iteration
        for (int ty = 0; ty < countHeight; ty++)
        {
            int ybegin = ty * tileSize;
            int yend = (ty + 1) * tileSize - 1;

            // Build subimage
            image::Image<image::RGBAfColor> region(tileSize * rowSize, tileSize * 3);
            image::Image<image::RGBAfColor> final(width, tileSize, true, image::RGBAfColor(0.0f, 0.0f, 0.0f, 0.0f));

            // Build a region
            for (int ry = 0; ry < 3; ry++)
            {
                int dy = ry - 1;
                int cy = ty + dy;

#pragma omp parallel for
                for (int rx = 0; rx < rowSize; rx++)
                {
                    int dx = rx - 1;
                    int cx = dx;

                    image::Image<image::RGBAfColor> tile(tileSize, tileSize);
                    if (!readFullTile(tile, panoramaInput, cx, cy))
                    {
                        ALICEVISION_LOG_ERROR("Invalid tile");
                        continue;
                    }

                    region.block(ry * tileSize, rx * tileSize, tileSize, tileSize) = tile;
                }
            }

            // First level is original image
            std::vector<image::Image<image::RGBAfColor>> pyramid;
            pyramid.push_back(region);

            // Build pyramid for tile
            int cs = tileSize * rowSize;
            while (cs != rowSize)
            {
                int ns = cs / 2;
                image::Image<image::RGBAfColor> smaller;
                downscaleTriangle(smaller, region);
                pyramid.push_back(smaller);
                region = smaller;
                cs = ns;
            }

            const image::Image<image::RGBAfColor>& finalTile = pyramid[0];
            final.block(0, 0, tileSize, width) = finalTile.block(tileSize, tileSize, tileSize, width);

            // Fill preview image
            while (previewCurrentRow < previewImage.rows())
            {
                double finalY = ratioPreview * double(previewCurrentRow) - ybegin;
                if (finalY < 0 || finalY >= tileSize)
                {
                    break;
                }

                int by = int(finalY);
                for (int px = 0; px < previewImage.cols(); px++)
                {
                    int bx = int(ratioPreview * double(px));
                    previewImage(previewCurrentRow, px) = final(by, bx);
                }

                previewCurrentRow++;
            }

            // Write panorama output
            colorSpaceTransform(final, fromColorSpace, outputColorSpace, dcpProf, neutral);
            panoramaOutput->write_scanlines(ty * tileSize, (ty + 1) * tileSize, 0, oiio::TypeDesc::FLOAT, final.data());

            // Write downscaled panorama levels
            for (int levelIdx = 1; levelIdx <= nbLevels; ++levelIdx)
            {
                image::Image<image::RGBAfColor>& levelTile = pyramid[levelIdx];
                const int levelTileSize = tileSize / (1 << levelIdx);
                const int levelWidth = width / (1 << levelIdx);
                image::Image<image::RGBAfColor> level(levelWidth, levelTileSize);
                level.block(0, 0, levelTileSize, levelWidth) = levelTile.block(levelTileSize, levelTileSize, levelTileSize, levelWidth);
                colorSpaceTransform(level, fromColorSpace, outputColorSpace, dcpProf, neutral);
                levelOutputs[levelIdx - 1]->write_scanlines(ty * levelTileSize, (ty + 1) * levelTileSize, 0, oiio::TypeDesc::FLOAT, level.data());
            }
        }
    }

    panoramaInput->close();
    panoramaOutput->close();
    for (int levelIdx = 1; levelIdx <= nbLevels; ++levelIdx)
    {
        levelOutputs[levelIdx - 1]->close();
    }

    if (outputPanoramaPreviewPath != "")
    {
        image::writeImage(outputPanoramaPreviewPath, previewImage, image::ImageWriteOptions());
    }

    return EXIT_SUCCESS;
}
