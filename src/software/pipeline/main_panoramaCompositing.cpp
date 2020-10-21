// This file is part of the AliceVision project.
// Copyright (c) 2020 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

// Input and geometry
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

// Image stuff
#include <aliceVision/image/all.hpp>
#include <aliceVision/mvsData/imageAlgo.hpp>

// Logging stuff
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

#include <aliceVision/panorama/cachedImage.hpp>
#include <aliceVision/panorama/compositer.hpp>
#include <aliceVision/panorama/alphaCompositer.hpp>
#include <aliceVision/panorama/laplacianCompositer.hpp>
#include <aliceVision/panorama/seams.hpp>
#include <aliceVision/panorama/boundingBoxMap.hpp>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace bpt = boost::property_tree;
namespace fs = boost::filesystem;

bool buildMap(BoundingBoxMap & map, const sfmData::SfMData& sfmData, const std::string & inputPath)
{
    /*for (const auto& viewIt : sfmData.getViews())
    {
        if(!sfmData.isPoseAndIntrinsicDefined(viewIt.second.get()))
        {
            // skip unreconstructed views
            continue;
        }
        
        // Load mask
        const std::string maskPath = (fs::path(inputPath) / (std::to_string(viewIt.first) + "_mask.exr")).string();
        oiio::ParamValueList metadata = image::readImageMetadata(maskPath);

        const std::size_t offsetX = metadata.find("AliceVision:offsetX")->get_int();
        const std::size_t offsetY = metadata.find("AliceVision:offsetY")->get_int();

        BoundingBox bb;
        bb.left = offsetX + contentX;
        bb.top = offsetY + contentY;
        bb.width = contentW;
        bb.height = contentH;

        if (!map.append(viewIt.first, bb)) 
        {
            return false;
        }
    }*/

    return true;
}

bool computeWTALabels(CachedImage<IndexT> & labels, image::TileCacheManager::shared_ptr& cacheManager, const sfmData::SfMData& sfmData, const std::string & inputPath, std::pair<int, int> & panoramaSize)
{
    WTASeams seams(panoramaSize.first, panoramaSize.second);

    if (!seams.initialize(cacheManager))
    {
        return false;
    }

    for(const auto& viewIt : sfmData.getViews())
    {
        if(!sfmData.isPoseAndIntrinsicDefined(viewIt.second.get()))
        {
            // skip unreconstructed views
            continue;
        }

        // Load mask
        const std::string maskPath = (fs::path(inputPath) / (std::to_string(viewIt.first) + "_mask.exr")).string();
        ALICEVISION_LOG_INFO("Load mask with path " << maskPath);
        image::Image<unsigned char> mask;
        image::readImage(maskPath, mask, image::EImageColorSpace::NO_CONVERSION);

        // Get offset
        oiio::ParamValueList metadata = image::readImageMetadata(maskPath);
        const std::size_t offsetX = metadata.find("AliceVision:offsetX")->get_int();
        const std::size_t offsetY = metadata.find("AliceVision:offsetY")->get_int();

        // Load Weights
        const std::string weightsPath = (fs::path(inputPath) / (std::to_string(viewIt.first) + "_weight.exr")).string();
        ALICEVISION_LOG_INFO("Load weights with path " << weightsPath);
        image::Image<float> weights;
        image::readImage(weightsPath, weights, image::EImageColorSpace::NO_CONVERSION);

        if (!seams.append(mask, weights, viewIt.first, offsetX, offsetY)) 
        {
            return false;
        }
    }

    labels = seams.getLabels();

    

    return true;
}

bool computeGCLabels(CachedImage<IndexT> & labels, image::TileCacheManager::shared_ptr& cacheManager, const sfmData::SfMData& sfmData, const std::string & inputPath, std::pair<int, int> & panoramaSize) 
{   

    //Compute coarsest level possible for graph cut
    int initial_level = 0;
    int min_width_for_graphcut = 1000;
    double ratio = double(panoramaSize.first) / double(min_width_for_graphcut);
    if (ratio > 1.0) {
      initial_level = int(floor(log2(ratio)));
    }

    HierarchicalGraphcutSeams seams(cacheManager, panoramaSize.first, panoramaSize.second, initial_level + 1);

    if (!seams.initialize()) 
    {
        return false;
    }

    if (!seams.setOriginalLabels(labels)) 
    {
        return false;
    }

    for (const auto& viewIt : sfmData.getViews())
    {
        if(!sfmData.isPoseAndIntrinsicDefined(viewIt.second.get()))
        {
            // skip unreconstructed views
            continue;
        }

        // Load mask
        const std::string maskPath = (fs::path(inputPath) / (std::to_string(viewIt.first) + "_mask.exr")).string();
        ALICEVISION_LOG_INFO("Load mask with path " << maskPath);
        image::Image<unsigned char> mask;
        image::readImage(maskPath, mask, image::EImageColorSpace::NO_CONVERSION);

        // Load Color
        const std::string colorsPath = (fs::path(inputPath) / (std::to_string(viewIt.first) + ".exr")).string();
        ALICEVISION_LOG_INFO("Load colors with path " << colorsPath);
        image::Image<image::RGBfColor> colors;
        image::readImage(colorsPath, colors, image::EImageColorSpace::NO_CONVERSION);

        // Get offset
        oiio::ParamValueList metadata = image::readImageMetadata(maskPath);
        const std::size_t offsetX = metadata.find("AliceVision:offsetX")->get_int();
        const std::size_t offsetY = metadata.find("AliceVision:offsetY")->get_int();


        // Append to graph cut
        if (!seams.append(colors, mask, viewIt.first, offsetX, offsetY)) 
        {
            return false;
        }
    }

    if (!seams.process()) 
    {
        return false;
    }

    labels = seams.getLabels();

    return true;
}

int aliceVision_main(int argc, char** argv)
{
    std::string sfmDataFilepath;
    std::string warpingFolder;
    std::string outputPanorama;

    std::string compositerType = "multiband";
    std::string overlayType = "none";
    bool useGraphCut = true;
    bool showBorders = false;
    bool showSeams = false;

    image::EStorageDataType storageDataType = image::EStorageDataType::Float;

    system::EVerboseLevel verboseLevel = system::Logger::getDefaultVerboseLevel();

    // Program description
    po::options_description allParams(
        "Perform panorama stiching of cameras around a nodal point for 360° panorama creation. \n"
        "AliceVision PanoramaCompositing");

    // Description of mandatory parameters
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()("input,i", po::value<std::string>(&sfmDataFilepath)->required(), "Input sfmData.")(
        "warpingFolder,w", po::value<std::string>(&warpingFolder)->required(), "Folder with warped images.")(
        "output,o", po::value<std::string>(&outputPanorama)->required(), "Path of the output panorama.");
    allParams.add(requiredParams);

    // Description of optional parameters
    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()("compositerType,c", po::value<std::string>(&compositerType)->required(),
                                 "Compositer Type [replace, alpha, multiband].")(
        "overlayType,c", po::value<std::string>(&overlayType)->required(), "Overlay Type [none, borders, seams, all].")(
        "useGraphCut,c", po::value<bool>(&useGraphCut)->default_value(useGraphCut),
        "Do we use graphcut for ghost removal ?")(
        "storageDataType", po::value<image::EStorageDataType>(&storageDataType)->default_value(storageDataType),
        ("Storage data type: " + image::EStorageDataType_informations()).c_str());
    allParams.add(optionalParams);

    // Setup log level given command line
    po::options_description logParams("Log parameters");
    logParams.add_options()("verboseLevel,v",
                            po::value<system::EVerboseLevel>(&verboseLevel)->default_value(verboseLevel),
                            "verbosity level (fatal, error, warning, info, debug, trace).");
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
    if(!sfmDataIO::Load(sfmData, sfmDataFilepath,
                        sfmDataIO::ESfMData(sfmDataIO::VIEWS | sfmDataIO::EXTRINSICS | sfmDataIO::INTRINSICS)))
    {
        ALICEVISION_LOG_ERROR("The input file '" + sfmDataFilepath + "' cannot be read");
        return EXIT_FAILURE;
    }

    int tileSize;
    std::pair<int, int> panoramaSize;
    {
        const IndexT viewId = *sfmData.getValidViews().begin();
        const std::string viewFilepath = (fs::path(warpingFolder) / (std::to_string(viewId) + ".exr")).string();
        ALICEVISION_LOG_TRACE("Read panorama size from file: " << viewFilepath);

        oiio::ParamValueList metadata = image::readImageMetadata(viewFilepath);
        panoramaSize.first = metadata.find("AliceVision:panoramaWidth")->get_int();
        panoramaSize.second = metadata.find("AliceVision:panoramaHeight")->get_int();
        tileSize = metadata.find("AliceVision:tileSize")->get_int();

        if(panoramaSize.first == 0 || panoramaSize.second == 0)
        {
            ALICEVISION_LOG_ERROR("The output panorama size is empty.");
            return EXIT_FAILURE;
        }

        if(tileSize == 0)
        {
            ALICEVISION_LOG_ERROR("no information on tileSize");
            return EXIT_FAILURE;
        }

        ALICEVISION_LOG_INFO("Output panorama size set to " << panoramaSize.first << "x" << panoramaSize.second);
    }

    // Create a cache manager
    const std::string outputPath = boost::filesystem::path(outputPanorama).parent_path().string();
    image::TileCacheManager::shared_ptr cacheManager = image::TileCacheManager::create(outputPath, 256, 256, 65536);
    if(!cacheManager)
    {
        ALICEVISION_LOG_ERROR("Error creating the cache manager");
        return EXIT_FAILURE;
    }

    // Configure the cache manager memory
    cacheManager->setInCoreMaxObjectCount(1000);

    LaplacianCompositer compositer(cacheManager, panoramaSize.first, panoramaSize.second, 6);
    
    if (!compositer.initialize()) 
    {
        ALICEVISION_LOG_ERROR("Failed to initialize compositer");
        return EXIT_FAILURE;
    }

    BoundingBoxMap map;
    if (!buildMap(map, sfmData, warpingFolder))
    {
        ALICEVISION_LOG_ERROR("Error Building map");
        return EXIT_FAILURE;
    }

    CachedImage<IndexT> labels;
    if (!computeWTALabels(labels, cacheManager, sfmData, warpingFolder, panoramaSize)) 
    {
        ALICEVISION_LOG_ERROR("Error computing initial labels");
        return EXIT_FAILURE;
    }

    labels.writeImage("/home/mmoc/labels_wta.exr");

    /*if (!computeGCLabels(labels, cacheManager, sfmData, warpingFolder, panoramaSize)) 
    {
        ALICEVISION_LOG_ERROR("Error computing graph cut labels");
        return EXIT_FAILURE;
    }

    labels.writeImage("/home/mmoc/labels_gc.exr");*/

    //Get a list of views ordered by their image scale
    std::vector<std::shared_ptr<sfmData::View>> viewOrderedByScale;
    {
        std::map<size_t, std::vector<std::shared_ptr<sfmData::View>>> mapViewsScale;
        for(const auto & it : sfmData.getViews()) 
        {
            auto view = it.second;
            IndexT viewId = view->getViewId();

            if(!sfmData.isPoseAndIntrinsicDefined(view.get()))
            {
                // skip unreconstructed views
                continue;
            }

            // Load mask
            const std::string maskPath = (fs::path(warpingFolder) / (std::to_string(viewId) + "_mask.exr")).string();
            image::Image<unsigned char> mask;
            image::readImage(maskPath, mask, image::EImageColorSpace::NO_CONVERSION);

            //Estimate scale
            size_t scale = compositer.getOptimalScale(mask.Width(), mask.Height());
            mapViewsScale[scale].push_back(it.second);
        }
        for (auto scaledList : mapViewsScale)
        {
            std::cout << scaledList.first << std::endl;
            for (auto item : scaledList.second) 
            {   
                viewOrderedByScale.push_back(item);
            }
        }
    }

    size_t pos = 0;

    std::cout << "compositing" << std::endl;

    for(const auto & view : viewOrderedByScale)
    {
        IndexT viewId = view->getViewId();
        if(!sfmData.isPoseAndIntrinsicDefined(view.get()))
        {
            // skip unreconstructed views
            continue;
        }
        pos++;

    
        // Load image and convert it to linear colorspace
        const std::string imagePath = (fs::path(warpingFolder) / (std::to_string(viewId) + ".exr")).string();
        //ALICEVISION_LOG_INFO("Load image with path " << imagePath);
        image::Image<image::RGBfColor> source;
        image::readImage(imagePath, source, image::EImageColorSpace::NO_CONVERSION);

        oiio::ParamValueList metadata = image::readImageMetadata(imagePath);
        const std::size_t offsetX = metadata.find("AliceVision:offsetX")->get_int();
        const std::size_t offsetY = metadata.find("AliceVision:offsetY")->get_int();

        // Load mask
        const std::string maskPath = (fs::path(warpingFolder) / (std::to_string(viewId) + "_mask.exr")).string();
        
        //ALICEVISION_LOG_INFO("Load mask with path " << maskPath);
        image::Image<unsigned char> mask;
        image::readImage(maskPath, mask, image::EImageColorSpace::NO_CONVERSION);

        // Load Weights
        /*const std::string weightsPath = (fs::path(warpingFolder) / (std::to_string(viewId) + "_weight.exr")).string();
        ALICEVISION_LOG_INFO("Load weights with path " << weightsPath);
        image::Image<float> weights;
        image::readImage(weightsPath, weights, image::EImageColorSpace::NO_CONVERSION);*/

        
        image::Image<float> seams(mask.Width(), mask.Height());
        if (!getMaskFromLabels(seams, labels, viewId, offsetX, offsetY)) 
        {
            ALICEVISION_LOG_ERROR("Error estimating seams image");
            return EXIT_FAILURE;
        }

        std::cout << pos << std::endl;
        if (!compositer.append(source, mask, seams, offsetX, offsetY)) 
        {
            return EXIT_FAILURE;
        }
    }

    if (!compositer.terminate()) 
    {
        return EXIT_FAILURE;
    }

    compositer.save(outputPanorama);

    return EXIT_SUCCESS;
}
