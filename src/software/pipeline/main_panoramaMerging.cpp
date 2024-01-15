// This file is part of the AliceVision project.
// Copyright (c) 2020 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

// Input and geometry
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

// Image
#include <aliceVision/image/all.hpp>
#include <aliceVision/image/imageAlgo.hpp>

// System
#include <aliceVision/system/Logger.hpp>

// Reading command line options
#include <boost/program_options.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/main.hpp>

// IO
#include <fstream>
#include <algorithm>
#include <filesystem>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace bpt = boost::property_tree;
namespace fs = std::filesystem;

int aliceVision_main(int argc, char** argv)
{
    std::string sfmDataFilepath;
    std::string compositingFolder;
    std::string outputPanoramaPath;
    image::EStorageDataType storageDataType = image::EStorageDataType::Float;
    const size_t tileSize = 256;
    bool useTiling = true;

    // Description of mandatory parameters
    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilepath)->required(),
         "Input SfMData.")
        ("compositingFolder,w", po::value<std::string>(&compositingFolder)->required(),
         "Folder with composited images.")
        ("outputPanorama,o", po::value<std::string>(&outputPanoramaPath)->required(),
         "Path of the output panorama.");

    // Description of optional parameters
    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("storageDataType", po::value<image::EStorageDataType>(&storageDataType)->default_value(storageDataType),
         ("Storage data type: " + image::EStorageDataType_informations()).c_str())
        ("useTiling,n", po::value<bool>(&useTiling)->default_value(useTiling),
         "Use tiling for compositing.");
    // clang-format on

    CmdLine cmdline("Merges all the image tiles created by the PanoramaCompositing.\n"
                    "AliceVision panoramaMerging");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    HardwareContext hwc = cmdline.getHardwareContext();
    oiio::attribute("threads", static_cast<int>(hwc.getMaxThreads()));
    oiio::attribute("exr_threads", static_cast<int>(hwc.getMaxThreads()));

    // load input scene
    sfmData::SfMData sfmData;
    if (!sfmDataIO::load(sfmData, sfmDataFilepath, sfmDataIO::ESfMData(sfmDataIO::VIEWS | sfmDataIO::EXTRINSICS | sfmDataIO::INTRINSICS)))
    {
        ALICEVISION_LOG_ERROR("The input file '" + sfmDataFilepath + "' cannot be read");
        return EXIT_FAILURE;
    }

    bool clampHalf = false;
    oiio::TypeDesc typeColor = oiio::TypeDesc::FLOAT;
    if (storageDataType == image::EStorageDataType::Half || storageDataType == image::EStorageDataType::HalfFinite)
    {
        typeColor = oiio::TypeDesc::HALF;
        if (storageDataType == image::EStorageDataType::HalfFinite)
        {
            clampHalf = true;
        }
    }

    std::vector<std::pair<IndexT, std::string>> sourcesList;
    if (useTiling)
    {
        for (auto viewItem : sfmData.getViews())
        {
            IndexT viewId = viewItem.first;
            if (!sfmData.isPoseAndIntrinsicDefined(viewId))
            {
                continue;
            }

            const std::string warpedPath = viewItem.second->getImage().getMetadata().at("AliceVision:warpedPath");

            // Get composited image path
            const std::string imagePath = (fs::path(compositingFolder) / (warpedPath + ".exr")).string();

            sourcesList.push_back(std::make_pair(viewId, imagePath));
        }
    }
    else
    {
        sourcesList.push_back(std::make_pair(0, (fs::path(compositingFolder) / "panorama.exr").string()));
    }

    if (sourcesList.size() == 0)
    {
        ALICEVISION_LOG_ERROR("Invalid number of sources");
        return EXIT_FAILURE;
    }

    int panoramaWidth = 0;
    int panoramaHeight = 0;
    oiio::ParamValueList metadata;

    auto sourceInput = sourcesList[0];
    metadata = image::readImageMetadata(sourceInput.second);
    panoramaWidth = metadata.find("AliceVision:panoramaWidth")->get_int();
    panoramaHeight = metadata.find("AliceVision:panoramaHeight")->get_int();

    int tileCountWidth = std::ceil(double(panoramaWidth) / double(tileSize));
    int tileCountHeight = std::ceil(double(panoramaHeight) / double(tileSize));

    std::map<std::pair<int, int>, IndexT> fullTiles;
    for (auto sourceItem : sourcesList)
    {
        std::string imagePath = sourceItem.second;

        // Get offset
        int width = 0;
        int height = 0;
        oiio::ParamValueList metadata = image::readImageMetadata(imagePath, width, height);
        const int offsetY = metadata.find("AliceVision:offsetY")->get_int();

        int offsetX = metadata.find("AliceVision:offsetX")->get_int();
        if (offsetX < 0)
        {
            offsetX += panoramaWidth;
        }

        int left = std::floor(double(offsetX) / double(tileSize));
        int top = std::floor(double(offsetY) / double(tileSize));
        int right = std::ceil(double(offsetX + width - 1) / double(tileSize));
        int bottom = std::ceil(double(offsetY + height - 1) / double(tileSize));

        // Loop over all tiles of this input
        for (int ty = top; ty <= bottom; ty++)
        {
            for (int tx = left; tx <= right; tx++)
            {
                int bleft = tx * tileSize;
                int bright = (tx + 1) * tileSize - 1;
                int btop = ty * tileSize;
                int bbottom = (ty + 1) * tileSize - 1;

                if (bleft < offsetX)
                    continue;
                if (bright >= offsetX + width)
                    continue;
                if (btop < offsetY)
                    continue;
                if (bbottom >= offsetY + height)
                    continue;

                std::pair<int, int> pos;
                pos.first = tx;
                pos.second = ty;

                if (fullTiles.find(pos) == fullTiles.end())
                {
                    fullTiles[pos] = sourceItem.first;
                }
            }
        }
    }

    std::unique_ptr<oiio::ImageOutput> panorama = oiio::ImageOutput::create(outputPanoramaPath);
    oiio::ImageSpec spec_panorama(panoramaWidth, panoramaHeight, 4, typeColor);
    spec_panorama.tile_width = tileSize;
    spec_panorama.tile_height = tileSize;
    spec_panorama.attribute("compression", "zips");
    spec_panorama.attribute("openexr:lineOrder", "randomY");

    metadata["openexr:lineOrder"] = "randomY";
    spec_panorama.extra_attribs = metadata;

    panorama->open(outputPanoramaPath, spec_panorama);

    struct TileInfo
    {
        bool filed = false;
        size_t used = 0;
        std::shared_ptr<image::Image<image::RGBAfColor>> tileContent = nullptr;
    };
    image::Image<TileInfo> tiles(tileCountWidth, tileCountHeight, true, {false, 0, nullptr});

    for (auto sourceItem : sourcesList)
    {
        std::string imagePath = sourceItem.second;

        // Get offset
        int width = 0;
        int height = 0;
        oiio::ParamValueList metadata = image::readImageMetadata(imagePath, width, height);
        const int offsetY = metadata.find("AliceVision:offsetY")->get_int();
        int offsetX = metadata.find("AliceVision:offsetX")->get_int();
        if (offsetX < 0)
        {
            offsetX += panoramaWidth;
        }

        image::Image<image::RGBAfColor> source;
        image::readImage(imagePath, source, image::EImageColorSpace::NO_CONVERSION);

        int left = std::floor(double(offsetX) / double(tileSize));
        int top = std::floor(double(offsetY) / double(tileSize));
        int right = std::ceil(double(offsetX + width - 1) / double(tileSize));
        int bottom = std::ceil(double(offsetY + height - 1) / double(tileSize));

        // Loop over all tiles of this input
        for (int ty = top; ty <= bottom; ty++)
        {
            if (ty < 0 || ty >= tileCountHeight)
            {
                continue;
            }

            int y = ty * tileSize;

            for (int iter_tx = left; iter_tx <= right; iter_tx++)
            {
                int tx = iter_tx;
                int offset_loop = 0;

                if (tx >= tileCountWidth)
                {
                    tx = tx - tileCountWidth;
                    offset_loop = -panoramaWidth;
                }

                if (tx < 0 || tx >= tileCountWidth)
                {
                    continue;
                }

                int x = tx * tileSize;

                // If this view is not registered as the main view, ignore
                std::pair<int, int> pos;
                pos.first = tx;
                pos.second = ty;
                if (fullTiles.find(pos) != fullTiles.end())
                {
                    if (fullTiles[pos] != sourceItem.first)
                    {
                        continue;
                    }
                }

                TileInfo& ti = tiles(ty, tx);
                if (ti.filed)
                {
                    continue;
                }

                if (ti.tileContent == nullptr)
                {
                    ti.tileContent =
                      std::make_shared<image::Image<image::RGBAfColor>>(tileSize, tileSize, true, image::RGBAfColor(0.0f, 0.0f, 0.0f, 0.0f));
                }

                for (int py = 0; py < tileSize; py++)
                {
                    int panorama_y = y + py;
                    int source_y = panorama_y - offsetY;

                    if (source_y < 0 || source_y >= height)
                    {
                        continue;
                    }

                    for (int px = 0; px < tileSize; px++)
                    {
                        int panorama_x = x + px;
                        int loffsetX = offsetX + offset_loop;
                        int source_x = panorama_x - loffsetX;

                        if (source_x < 0)
                        {
                            source_x = panoramaWidth + x + px - loffsetX;
                        }

                        if (source_x >= width)
                        {
                            source_x = x + px - panoramaWidth - loffsetX;
                        }

                        if (source_x < 0 || source_x >= width)
                        {
                            continue;
                        }

                        // Check if the pixel is already written
                        image::RGBAfColor& dpix = ti.tileContent->operator()(py, px);
                        image::RGBAfColor pix = source(source_y, source_x);
                        if (pix.a() > 0.9)
                        {
                            if (dpix.a() < 0.1)
                            {
                                ti.used++;
                            }

                            dpix = pix;
                            dpix.a() = 1.0;
                        }
                    }
                }

                if (ti.used >= tileSize * tileSize)
                {
                    panorama->write_tile(tx * tileSize, ty * tileSize, 0, oiio::TypeDesc::FLOAT, ti.tileContent->data());
                    ti.tileContent = nullptr;
                    ti.filed = true;
                    continue;
                }

                int bleft = tx * tileSize;
                int bright = (tx + 1) * tileSize - 1;
                int btop = ty * tileSize;
                int bbottom = (ty + 1) * tileSize - 1;

                if (bleft < offsetX)
                    continue;
                if (bright >= offsetX + width)
                    continue;
                if (btop < offsetY)
                    continue;
                if (bbottom >= offsetY + height)
                    continue;

                panorama->write_tile(tx * tileSize, ty * tileSize, 0, oiio::TypeDesc::FLOAT, ti.tileContent->data());
                ti.tileContent = nullptr;
                ti.filed = true;
            }
        }
    }

    image::Image<image::RGBAfColor> vide(tileSize, tileSize, true, image::RGBAfColor(0.0f, 0.0f, 0.0f, 0.0f));
    for (int ty = 0; ty < tileCountHeight; ty++)
    {
        for (int tx = 0; tx < tileCountWidth; tx++)
        {
            TileInfo& ti = tiles(ty, tx);

            if (ti.filed)
            {
                continue;
            }

            if (ti.tileContent)
            {
                panorama->write_tile(tx * tileSize, ty * tileSize, 0, oiio::TypeDesc::FLOAT, ti.tileContent->data());
            }
            else
            {
                panorama->write_tile(tx * tileSize, ty * tileSize, 0, oiio::TypeDesc::FLOAT, vide.data());
            }

            ti.filed = true;
        }
    }

    panorama->close();

    return EXIT_SUCCESS;
}
