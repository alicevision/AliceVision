// This file is part of the AliceVision project.
// Copyright (c) 2020 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.


// Image
#include <aliceVision/image/all.hpp>

// System
#include <aliceVision/system/MemoryInfo.hpp>
#include <aliceVision/system/Logger.hpp>

// Reading command line options
#include <boost/program_options.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/main.hpp>

#include <fstream>
#include <algorithm>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/filesystem.hpp>


#include <OpenImageIO/imageio.h>
#include <OpenImageIO/imagebuf.h>
#include <OpenImageIO/imagebufalgo.h>
#include <OpenImageIO/color.h>


// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

bool downscaleTriangle(image::Image<image::RGBAfColor> & smaller, const image::Image<image::RGBAfColor> & source)
{
    int sw = source.Width();
    int sh = source.Height();
    int nw = sw / 2;
    int nh = sh / 2;

    smaller = image::Image<image::RGBAfColor>(nw, nh);

    for (int i = 0; i < nh; i++)
    {
        int di = i * 2;
        int pi = di - 1;
        int ni = di + 1;
        if (pi < 0) pi = ni;
        if (ni >= sh) ni = pi;

        for (int j = 0; j < nw; j++)
        {
            int dj = j * 2;
            int pj = dj - 1;
            int nj = dj + 1;
            if (pj < 0) pj = nj;
            if (nj >= sw) nj = pj;

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

bool readFullTile(image::Image<image::RGBAfColor> & output, std::unique_ptr<oiio::ImageInput> & input, int tx, int ty)
{
    const oiio::ImageSpec &inputSpec = input->spec();
    int tileSize = inputSpec.tile_width; 
    int width = inputSpec.width;
    int height = inputSpec.height;

    int countWidth = std::ceil(double(width) / double(tileSize));
    int countHeight = std::ceil(double(height) / double(tileSize));

    if (tx < 0 || tx >= countWidth || ty < 0 || ty >= countHeight)
    {
        output.fill(image::RGBAfColor(0.0f, 0.0f, 0.0f, 0.0f));
        return true;
    }

    if (tx < 0)
    {
        tx = countWidth + tx;
    }

    if (tx >= countWidth)
    {
        tx = tx - countWidth;
    }

    /*int available = width - (tx * tileSize);

    if (available < tileSize)
    {
        image::Image<image::RGBAfColor> buf(tileSize, tileSize);
        if (!input->read_tile(tx * tileSize, ty * tileSize, 0, oiio::TypeDesc::FLOAT, buf.data()))
        {
            return false;
        }

        output.block(0, tileSize - available, tileSize, available) = buf.block(0, 0, tileSize, available);

        if (!input->read_tile((tx - 1) * tileSize, ty * tileSize, 0, oiio::TypeDesc::FLOAT, buf.data()))
        {
            return false;
        }

        output.block(0, 0, tileSize, tileSize - available) = buf.block(0, available, tileSize, tileSize - available);

        return true;
    }*/

    output.fill(image::RGBAfColor(0.0f, 0.0f, 0.0f, 0.0f));
    if (!input->read_tile(tx * tileSize, ty * tileSize, 0, oiio::TypeDesc::FLOAT, output.data()))
    {
        return false;
    }

    return true;
}

int aliceVision_main(int argc, char** argv)
{
    std::string inputPanoramaPath;
    std::string outputPanoramaPath;
    image::EStorageDataType storageDataType = image::EStorageDataType::Float;
    const size_t maxProcessingSize = 2000;    

    system::EVerboseLevel verboseLevel = system::Logger::getDefaultVerboseLevel();

    // Program description
    po::options_description allParams(
        "Perform panorama stiching of cameras around a nodal point for 360° panorama creation. \n"
        "AliceVision PanoramaPostProcessing");

    // Description of mandatory parameters
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("inputPanorama,i", po::value<std::string>(&inputPanoramaPath)->required(), "Input Panorama.")
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

    

    //Open input panorama
    std::unique_ptr<oiio::ImageInput> panoramaInput = oiio::ImageInput::open(inputPanoramaPath);
    if (!panoramaInput)
    {
        return EXIT_FAILURE;
    }

    //Get information about input panorama
    const oiio::ImageSpec &inputSpec = panoramaInput->spec();
    const int tileWidth = inputSpec.tile_width;    
    const int tileHeight = inputSpec.tile_height;
    if (tileWidth != tileHeight)
    {
        ALICEVISION_LOG_ERROR("non square tiles !");
        return EXIT_FAILURE;
    }

    //Create output panorama
    std::unique_ptr<oiio::ImageOutput> panoramaOutput = oiio::ImageOutput::create(outputPanoramaPath);
    oiio::ImageSpec outputSpec(inputSpec);
    outputSpec.tile_width = 0;
    outputSpec.tile_height = 0;
	outputSpec.attribute("compression", "zip");
    outputSpec.extra_attribs.remove("openexr:lineOrder");
    if (!panoramaOutput->open(outputPanoramaPath, outputSpec))
    {
        ALICEVISION_LOG_ERROR("Impossible to write to destination path");
        return EXIT_FAILURE;
    }

    //Compute various sizes
    const int tileSize = tileWidth;
    const int width = inputSpec.width;
    const int height = inputSpec.height;
    const int countWidth = std::ceil(double(width) / double(tileSize));
    const int countHeight = std::ceil(double(height) / double(tileSize));
    const int rowSize = countWidth + 2;

    ALICEVISION_LOG_INFO("Reduce image (" << width << "x" << height << ")");
    
    //Downscale such that each tile == 1 pixel
    image::Image<image::RGBAfColor> smallImage(countWidth, countHeight);
    bool error = false;

    #pragma omp parallel for
    for (int ty = 0; ty < countHeight; ty++)
    {
        //Build subimage
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
        
    ALICEVISION_LOG_INFO("Process fill holes for reduced image (" << smallImage.Width() << "x" << smallImage.Height() << ")");
    image::Image<image::RGBAfColor> smallFiled(smallImage.Width(), smallImage.Height());
    oiio::ImageBuf inBuf(oiio::ImageSpec(smallImage.Width(), smallImage.Height(), 4, oiio::TypeDesc::FLOAT), smallImage.data());
    oiio::ImageBuf outBuf(oiio::ImageSpec(smallImage.Width(), smallImage.Height(), 4, oiio::TypeDesc::FLOAT), smallFiled.data());
    oiio::ImageBufAlgo::fillholes_pushpull(outBuf, inBuf);

    //Process one full row of tiles each iteration
    for (int ty = 0; ty < countHeight; ty++)
    {
        //Build subimage
        image::Image<image::RGBAfColor> region(tileSize * rowSize, tileSize * 3);          
        image::Image<image::RGBAfColor> subFiled(rowSize, 3, true, image::RGBAfColor(0.0f, 0.0f, 0.0f, 0.0f));
        image::Image<image::RGBAfColor> final(width, tileSize, true, image::RGBAfColor(0.0f, 0.0f, 0.0f, 0.0f));
            
        //Build a region           
        for (int ry = 0; ry < 3; ry++)
        {
            int dy = ry - 1;
            int cy = ty + dy;
            
            for (int rx = 0; rx < rowSize; rx++)
            {
                int dx = rx - 1;
                int cx = dx;

                image::Image<image::RGBAfColor> tile(tileSize, tileSize);
                if (!readFullTile(tile, panoramaInput, cx, cy))
                {
                    ALICEVISION_LOG_ERROR("Invalid tile");
                    return EXIT_FAILURE;
                }

                region.block(ry * tileSize, rx * tileSize, tileSize, tileSize) = tile;

                if (cy < 0 || cy >= smallFiled.Height())
                {
                    continue;
                }

                if (cx < 0)
                {
                    cx = smallFiled.Width() + cx;
                }

                if (cx >= smallFiled.Width())
                {
                    cx = cx - smallFiled.Width();
                }

                subFiled(ry, rx) = smallFiled(cy, cx);
            }
        }

        //First level is original image
        std::vector<image::Image<image::RGBAfColor>> pyramid;
        pyramid.push_back(region);

        //Build pyramid for tile
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
            const image::Image<image::RGBAfColor> & source = pyramid[level + 1];
            
            image::Image<image::RGBAfColor> & dest = pyramid[level];

            for (int i = 0; i < source.Height(); i++)
            {
                int di = i * 2;
                int pi = i - 1;
                int ni = i + 1;

                if (pi < 0) pi = -pi;
                if (ni >= source.Height()) ni = i;
                
                for (int j = 0; j < source.Width(); j++)
                {
                    int dj = j * 2;
                    int pj = j - 1;
                    int nj = j + 1;

                    if (pj < 0) pj = nj;
                    if (nj >= source.Width()) nj = j;

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

                    if (dest(di, dj).a() != 1.0f) dest(di, dj) = d11;
                    if (dest(di, dj + 1).a() != 1.0f) dest(di, dj + 1) = d12;
                    if (dest(di + 1, dj).a() != 1.0f) dest(di + 1, dj) = d21;
                    if (dest(di + 1, dj + 1).a() != 1.0f) dest(di + 1, dj + 1) = d22;
                }
            }
        }

        const image::Image<image::RGBAfColor> & finalTile = pyramid[0];
        
        final.block(0, 0, tileSize, width) = finalTile.block(tileSize, tileSize, tileSize, width);

        panoramaOutput->write_scanlines(ty * tileSize, (ty + 1) * tileSize, 0, oiio::TypeDesc::FLOAT, final.data());
    }

    panoramaInput->close();
    panoramaOutput->close();

    return EXIT_SUCCESS;
}
