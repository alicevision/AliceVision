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
#include <aliceVision/mvsData/imageAlgo.hpp>

// System
#include <aliceVision/system/MemoryInfo.hpp>
#include <aliceVision/system/Logger.hpp>

// Reading command line options
#include <boost/program_options.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/main.hpp>

// IO
#include <fstream>
#include <algorithm>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/filesystem.hpp>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace bpt = boost::property_tree;
namespace fs = boost::filesystem;

int aliceVision_main(int argc, char** argv)
{
    std::string sfmDataFilepath;
    std::string compositingFolder;
    std::string outputPanoramaPath;
    image::EStorageDataType storageDataType = image::EStorageDataType::Float;
    const size_t tileSize = 256;

    system::EVerboseLevel verboseLevel = system::Logger::getDefaultVerboseLevel();

    // Program description
    po::options_description allParams(
        "Perform panorama stiching of cameras around a nodal point for 360° panorama creation. \n"
        "AliceVision PanoramaCompositing");

    // Description of mandatory parameters
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilepath)->required(), "Input sfmData.")
        ("compositingFolder,w", po::value<std::string>(&compositingFolder)->required(), "Folder with composited images.")
        ("outputPanorama,o", po::value<std::string>(&outputPanoramaPath)->required(), "Path of the output panorama.");
    allParams.add(requiredParams);

    // Description of optional parameters
    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("storageDataType", po::value<image::EStorageDataType>(&storageDataType)->default_value(storageDataType), ("Storage data type: " + image::EStorageDataType_informations()).c_str());
    allParams.add(optionalParams);

    // Setup log level given command line
    po::options_description logParams("Log parameters");
    logParams.add_options()
        ("verboseLevel,v", po::value<system::EVerboseLevel>(&verboseLevel)->default_value(verboseLevel), "verbosity level (fatal, error, warning, info, debug, trace).");
    allParams.add(logParams);

    // Effectively parse command line given parse options
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

    // Set verbose level given command line
    system::Logger::get()->setLogLevel(verboseLevel);

    // load input scene
    sfmData::SfMData sfmData;
    if(!sfmDataIO::Load(sfmData, sfmDataFilepath, sfmDataIO::ESfMData(sfmDataIO::VIEWS | sfmDataIO::EXTRINSICS | sfmDataIO::INTRINSICS)))
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


    int panoramaWidth = 0;
    int panoramaHeight = 0;
    oiio::ParamValueList metadata;

    for (auto viewItem : sfmData.getViews())
    {
        IndexT viewId = viewItem.first;
        if(!sfmData.isPoseAndIntrinsicDefined(viewId))
            continue;

        const std::string warpedPath = viewItem.second->getMetadata().at("AliceVision:warpedPath");

        // Get composited image path
        const std::string imagePath = (fs::path(compositingFolder) / (warpedPath + ".exr")).string();

        // Get offset
        metadata = image::readImageMetadata(imagePath);
        panoramaWidth = metadata.find("AliceVision:panoramaWidth")->get_int();
        panoramaHeight = metadata.find("AliceVision:panoramaHeight")->get_int();
        break;
    }

    int tileCountWidth = std::ceil(double(panoramaWidth) / double(tileSize));
    int tileCountHeight = std::ceil(double(panoramaHeight) / double(tileSize));


    std::map<std::pair<int, int>, IndexT> fullTiles;
    for (auto viewItem : sfmData.getViews())
    {
        IndexT viewId = viewItem.first;
        if(!sfmData.isPoseAndIntrinsicDefined(viewId))
        {
            continue;
        }

        const std::string warpedPath = viewItem.second->getMetadata().at("AliceVision:warpedPath");

        // Get composited image path
        const std::string imagePath = (fs::path(compositingFolder) / (warpedPath + ".exr")).string();

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

        //Loop over all tiles of this input
        for (int ty = top; ty <= bottom; ty++)
        {
            for (int tx = left; tx <= right; tx++)
            {
                int bleft = tx * tileSize;
                int bright = (tx + 1) * tileSize - 1;
                int btop = ty * tileSize;
                int bbottom = (ty + 1) * tileSize - 1;
                
                if (bleft < offsetX) continue;
                if (bright >= offsetX + width) continue;
                if (btop < offsetY) continue;
                if (bbottom >= offsetY + height) continue;

                std::pair<int, int> pos;
                pos.first = tx;
                pos.second = ty;

                if (fullTiles.find(pos) == fullTiles.end())
                {
                    fullTiles[pos] = viewId;
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

    for (auto viewItem : sfmData.getViews())
    {
        IndexT viewId = viewItem.first;
        if(!sfmData.isPoseAndIntrinsicDefined(viewId))
        {
            continue;
        }

        const std::string warpedPath = viewItem.second->getMetadata().at("AliceVision:warpedPath");

        // Get composited image path
        const std::string imagePath = (fs::path(compositingFolder) / (warpedPath + ".exr")).string();

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

        //Loop over all tiles of this input
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
                    offset_loop = - panoramaWidth;
                }
                
                if (tx < 0 || tx >= tileCountWidth) 
                {
                    continue;
                }

                int x = tx * tileSize;

                //If this view is not registered as the main view, ignore
                std::pair<int, int> pos;
                pos.first = tx;
                pos.second = ty;
                if (fullTiles.find(pos) != fullTiles.end())
                {
                    if (fullTiles[pos] != viewId)
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
                    ti.tileContent = std::make_shared<image::Image<image::RGBAfColor>>(tileSize, tileSize, true, image::RGBAfColor(0.0f, 0.0f, 0.0f, 0.0f));
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

                        //Check if the pixel is already written
                        image::RGBAfColor & dpix = ti.tileContent->operator()(py, px);
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

                if (ti.used >= tileSize*tileSize)
                {
                    panorama->write_tile (tx * tileSize, ty * tileSize, 0, oiio::TypeDesc::FLOAT, ti.tileContent->data());
                    ti.tileContent = nullptr;
                    ti.filed = true;
                    continue;
                }

                int bleft = tx * tileSize;
                int bright = (tx + 1) * tileSize - 1;
                int btop = ty * tileSize;
                int bbottom = (ty + 1) * tileSize - 1;

                if (bleft < offsetX) continue;
                if (bright >= offsetX + width) continue;
                if (btop < offsetY) continue;
                if (bbottom >= offsetY + height) continue;

                panorama->write_tile (tx * tileSize, ty * tileSize, 0, oiio::TypeDesc::FLOAT, ti.tileContent->data());
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
                panorama->write_tile (tx * tileSize, ty * tileSize, 0, oiio::TypeDesc::FLOAT, ti.tileContent->data());
            }
            else
            { 
                panorama->write_tile (tx * tileSize, ty * tileSize, 0, oiio::TypeDesc::FLOAT, vide.data());
            }

            ti.filed = true;
        }
    }


    panorama->close();


    return EXIT_SUCCESS;
}
