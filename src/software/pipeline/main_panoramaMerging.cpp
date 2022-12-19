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

    // Program description

    // Description of mandatory parameters
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilepath)->required(), "Input sfmData.")
        ("compositingFolder,w", po::value<std::string>(&compositingFolder)->required(), "Folder with composited images.")
        ("outputPanorama,o", po::value<std::string>(&outputPanoramaPath)->required(), "Path of the output panorama.");

    // Description of optional parameters
    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("storageDataType", po::value<image::EStorageDataType>(&storageDataType)->default_value(storageDataType), ("Storage data type: " + image::EStorageDataType_informations()).c_str());

    CmdLine cmdline("This program performs panorama stiching of cameras around a nodal point for 360° panorama creation. \n"
                    "AliceVision panoramaMerging");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // load input scene
    sfmData::SfMData sfmData;
    if(!sfmDataIO::Load(sfmData, sfmDataFilepath, sfmDataIO::ESfMData(sfmDataIO::VIEWS | sfmDataIO::EXTRINSICS | sfmDataIO::INTRINSICS)))
    {
        ALICEVISION_LOG_ERROR("The input file '" + sfmDataFilepath + "' cannot be read");
        return EXIT_FAILURE;
    }


    bool first = true;
    image::Image<image::RGBAfColor> panorama;
    std::string colorSpace;

    for (auto viewItem : sfmData.getViews())
    {
        IndexT viewId = viewItem.first;
        if(!sfmData.isPoseAndIntrinsicDefined(viewId))
            continue;

        // Get composited image path
        const std::string imagePath = (fs::path(compositingFolder) / (std::to_string(viewId) + ".exr")).string();
        
        // Get offset
        oiio::ParamValueList metadata = image::readImageMetadata(imagePath);
        const int offsetX = metadata.find("AliceVision:offsetX")->get_int();
        const int offsetY = metadata.find("AliceVision:offsetY")->get_int();
        const int panoramaWidth = metadata.find("AliceVision:panoramaWidth")->get_int();
        const int panoramaHeight = metadata.find("AliceVision:panoramaHeight")->get_int();

        if (first) 
        {
            panorama = image::Image<image::RGBAfColor>(panoramaWidth, panoramaHeight, true, image::RGBAfColor(0.0f, 0.0f, 0.f, 0.0f));
            colorSpace = metadata.find("AliceVision:ColorSpace")->get_string();
            first = false;
        }

        // Load image
        ALICEVISION_LOG_TRACE("Load image with path " << imagePath);
        image::Image<image::RGBAfColor> source;
        image::readImage(imagePath, source, image::EImageColorSpace::NO_CONVERSION);

        for (int i = 0; i < source.Height(); i++)
        {
            for (int j = 0; j < source.Width(); j++)
            {
                image::RGBAfColor pix = source(i, j);

                if (pix.a() > 0.9) {

                    int nx = offsetX + j;
                    if (nx < 0) nx += panoramaWidth;
                    if (nx >= panoramaWidth) nx -= panoramaWidth;

                    panorama(offsetY + i, nx) = pix;
                }
            }
        }
    }

    image::EImageColorSpace outputColorSpace = colorSpace.empty()
            ? image::EImageColorSpace::LINEAR : image::EImageColorSpace_stringToEnum(colorSpace);

    image::writeImage(outputPanoramaPath, panorama,
                      image::ImageWriteOptions().storageDataType(storageDataType)
                                                .toColorSpace(outputColorSpace));

    return EXIT_SUCCESS;
}
