// This file is part of the AliceVision project.
// Copyright (c) 2020 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/panorama/panoramaMap.hpp>
#include <aliceVision/panorama/compositer.hpp>
#include <aliceVision/panorama/alphaCompositer.hpp>
#include <aliceVision/panorama/laplacianCompositer.hpp>

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

size_t getCompositingOptimalScale(int width, int height)
{
    /*
    Look for the smallest scale such that the image is not smaller than the
    convolution window size.
    minsize / 2^x = 5
    minsize / 5 = 2^x
    x = log2(minsize/5)
    */

    size_t minsize = std::min(width, height);
    size_t gaussianFilterRadius = 2;

    int gaussianFilterSize = 1 + 2 * 2;
    
    size_t optimal_scale = size_t(floor(std::log2(double(minsize) / gaussianFilterSize)));
    
    return (optimal_scale - 1);
}

std::unique_ptr<PanoramaMap> buildMap(const sfmData::SfMData & sfmData, const std::string & inputPath, const size_t borderSize)
{   
    if (sfmData.getViews().size() == 0) 
    {
        return nullptr;
    }

    size_t min_scale = std::numeric_limits<size_t>::max();
    std::vector<std::pair<IndexT, BoundingBox>> listBoundingBox;
    std::pair<std::size_t, std::size_t> panoramaSize;

    for (const auto& viewIt : sfmData.getViews())
    {
        // Load mask
        const std::string maskPath = (fs::path(inputPath) / (std::to_string(viewIt.first) + "_mask.exr")).string();
        ALICEVISION_LOG_TRACE("Load metadata of mask with path " << maskPath);

        int width = 0;
        int height = 0;
        oiio::ParamValueList metadata = image::readImageMetadata(maskPath, width, height);
        const std::size_t offsetX = metadata.find("AliceVision:offsetX")->get_int();
        const std::size_t offsetY = metadata.find("AliceVision:offsetY")->get_int();
        panoramaSize.first = metadata.find("AliceVision:panoramaWidth")->get_int();
        panoramaSize.second = metadata.find("AliceVision:panoramaHeight")->get_int();

        BoundingBox bb;
        bb.left = offsetX;
        bb.top = offsetY;
        bb.width = width;
        bb.height = height;

        listBoundingBox.push_back(std::make_pair(viewIt.first, bb));
        size_t scale = getCompositingOptimalScale(width, height);
        if (scale < min_scale)
        {
            min_scale = scale;
        }
    }


    std::unique_ptr<PanoramaMap> ret(new PanoramaMap(panoramaSize.first, panoramaSize.second, min_scale, borderSize));
    for (const auto & bbitem : listBoundingBox)
    {
        ret->append(bbitem.first, bbitem.second);
    }

    return ret;
}

bool computeWTALabels(image::Image<IndexT> & labels, std::list<IndexT> & views, const std::string & inputPath, const BoundingBox & boundingBox, int panoramaWidth)
{
    ALICEVISION_LOG_INFO("Estimating initial labels for panorama");

    WTASeams seams(boundingBox.width, boundingBox.height);


    for (const IndexT & currentId : views)
    {
        // Load mask
        const std::string maskPath = (fs::path(inputPath) / (std::to_string(currentId) + "_mask.exr")).string();
        ALICEVISION_LOG_TRACE("Load mask with path " << maskPath);
        image::Image<unsigned char> mask;
        image::readImage(maskPath, mask, image::EImageColorSpace::NO_CONVERSION);

        // Get offset
        oiio::ParamValueList metadata = image::readImageMetadata(maskPath);
        const int offsetX = metadata.find("AliceVision:offsetX")->get_int();
        const int offsetY = metadata.find("AliceVision:offsetY")->get_int();

        // Load Weights
        const std::string weightsPath = (fs::path(inputPath) / (std::to_string(currentId) + "_weight.exr")).string();
        ALICEVISION_LOG_TRACE("Load weights with path " << weightsPath);
        image::Image<float> weights;
        image::readImage(weightsPath, weights, image::EImageColorSpace::NO_CONVERSION);

        BoundingBox currentBb;
        currentBb.top = offsetY;
        currentBb.width = mask.Width();
        currentBb.height = mask.Height();
        currentBb.left = offsetX;
        if (!boundingBox.intersectionWith(currentBb).isEmpty())
        {
            if (!seams.append(mask, weights, currentId, currentBb.left - boundingBox.left, offsetY - boundingBox.top)) 
            {
                return false;
            }
        }

    
        currentBb.left = offsetX - panoramaWidth;
        currentBb.top = offsetY;
        currentBb.width = mask.Width();
        currentBb.height = mask.Height();
        if (!boundingBox.intersectionWith(currentBb).isEmpty())
        {
            if (!seams.append(mask, weights, currentId, currentBb.left - boundingBox.left, offsetY - boundingBox.top)) 
            {
                return false;
            }
        }

        currentBb.left = offsetX + panoramaWidth;
        currentBb.top = offsetY;
        currentBb.width = mask.Width();
        currentBb.height = mask.Height();
        if (!boundingBox.intersectionWith(currentBb).isEmpty())
        {
            if (!seams.append(mask, weights, currentId, currentBb.left - boundingBox.left, offsetY - boundingBox.top)) 
            {
                return false;
            }
        }
    }

    labels = seams.getLabels();

    return true;
}

int aliceVision_main(int argc, char** argv)
{
    std::string sfmDataFilepath;
    std::string warpingFolder;
    std::string outputFolder;
    std::string temporaryCachePath;
    std::string compositerType = "multiband";
    int rangeStart = -1;
	int rangeSize = 1;

    image::EStorageDataType storageDataType = image::EStorageDataType::Float;

    system::EVerboseLevel verboseLevel = system::Logger::getDefaultVerboseLevel();

    // Program description
    po::options_description allParams(
        "Perform panorama stiching of cameras around a nodal point for 360° panorama creation. \n"
        "AliceVision PanoramaCompositing");

    // Description of mandatory parameters
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilepath)->required(), "Input sfmData.")
        ("warpingFolder,w", po::value<std::string>(&warpingFolder)->required(), "Folder with warped images.")
        ("output,o", po::value<std::string>(&outputFolder)->required(), "Path of the output panorama.")
        ("cacheFolder,f", po::value<std::string>(&temporaryCachePath)->required(), "Path of the temporary cache.");
    allParams.add(requiredParams);

    // Description of optional parameters
    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("compositerType,c", po::value<std::string>(&compositerType)->required(), "Compositer Type [replace, alpha, multiband].")
        ("storageDataType", po::value<image::EStorageDataType>(&storageDataType)->default_value(storageDataType), ("Storage data type: " + image::EStorageDataType_informations()).c_str())
        ("rangeStart", po::value<int>(&rangeStart)->default_value(rangeStart), "Range image index start.")
		("rangeSize", po::value<int>(&rangeSize)->default_value(rangeSize), "Range size.");
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

    // Define range to compute
    size_t viewsCount = sfmData.getViews().size();
    if(rangeStart != -1)
    {
        if(rangeStart < 0 || rangeSize < 0 ||
            std::size_t(rangeStart) > viewsCount)
        {
            ALICEVISION_LOG_ERROR("Range is incorrect");
            return EXIT_FAILURE;
        }

        if(std::size_t(rangeStart + rangeSize) > viewsCount)
        {
            rangeSize = int(viewsCount) - rangeStart;
        }
    }
    else
    {
        rangeStart = 0;
        rangeSize = int(viewsCount);
    }


    const size_t borderSize = 2;
    std::unique_ptr<PanoramaMap> panoramaMap = buildMap(sfmData, warpingFolder, borderSize);
    if (viewsCount == 0) 
    {
        ALICEVISION_LOG_ERROR("No valid views");
        return EXIT_FAILURE;
    }

    std::vector<std::shared_ptr<sfmData::View>> views;
    for (auto & vIterator : sfmData.getViews())
    {
        views.push_back(vIterator.second);
    }

    for (std::size_t posReference = std::size_t(rangeStart); posReference < std::size_t(rangeStart + rangeSize); ++posReference)
    {
        ALICEVISION_LOG_INFO("processing input region " << posReference + 1 << "/" << views.size());
        
        IndexT viewReference = views[posReference]->getViewId();
        /*if (viewReference != 347875201) 
        {
            continue;
        }*/

        std::list<IndexT> overlaps;
        if (!panoramaMap->getOverlaps(overlaps, viewReference)) 
        {
            ALICEVISION_LOG_ERROR("Problem analyzing neighboorhood");
            return EXIT_FAILURE;
        }
        overlaps.push_back(viewReference);

        BoundingBox referenceBoundingBox;
        if (!panoramaMap->getBoundingBox(referenceBoundingBox, viewReference)) 
        {
            ALICEVISION_LOG_ERROR("Problem getting reference bounding box");
            return EXIT_FAILURE;
        }

        bool shifted = false;
        if (referenceBoundingBox.getRight() >= panoramaMap->getWidth()) 
        {
            referenceBoundingBox.left -= panoramaMap->getWidth();
            shifted = true;
        }

        std::unique_ptr<Compositer> compositer;
        compositer = std::unique_ptr<Compositer>(new LaplacianCompositer(referenceBoundingBox.width, referenceBoundingBox.height, panoramaMap->getScale()));

        if (!compositer->initialize())
        {
            ALICEVISION_LOG_ERROR("Error initializing panorama");
            return false;
        }

        image::Image<IndexT> labels;
        if (!computeWTALabels(labels, overlaps, warpingFolder, referenceBoundingBox, panoramaMap->getWidth())) 
        {
            ALICEVISION_LOG_ERROR("Error estimating panorama labels for this input");
            return false;
        }
        
        {
        const std::string viewIdStr = std::to_string(viewReference);
        const std::string outputFilePath = (fs::path(outputFolder) / (viewIdStr + "_label.exr")).string();
        image::writeImage(outputFilePath, labels, image::EImageColorSpace::NO_CONVERSION);
        }
    
        int posCurrent = 0;
        for (IndexT viewCurrent : overlaps)
        {
            posCurrent++;
            ALICEVISION_LOG_INFO("Processing input " << posCurrent << "/" << overlaps.size());

        
            // Load image and convert it to linear colorspace
            const std::string imagePath = (fs::path(warpingFolder) / (std::to_string(viewCurrent) + ".exr")).string();
            ALICEVISION_LOG_TRACE("Load image with path " << imagePath);
            image::Image<image::RGBfColor> source;
            image::readImage(imagePath, source, image::EImageColorSpace::NO_CONVERSION);

            // Load mask
            const std::string maskPath = (fs::path(warpingFolder) / (std::to_string(viewCurrent) + "_mask.exr")).string();
            ALICEVISION_LOG_TRACE("Load mask with path " << maskPath);
            image::Image<unsigned char> mask;
            image::readImage(maskPath, mask, image::EImageColorSpace::NO_CONVERSION);

            // Retrieve position of image in panorama
            oiio::ParamValueList metadata = image::readImageMetadata(imagePath);
            int offsetX = metadata.find("AliceVision:offsetX")->get_int();
            int offsetY = metadata.find("AliceVision:offsetY")->get_int();

            if (offsetX > referenceBoundingBox.getRight())
            {
                offsetX -= panoramaMap->getWidth();
            }

            image::Image<float> weights;        
            /*const std::string weightsPath = (fs::path(warpingFolder) / (std::to_string(viewCurrent) + "_weight.exr")).string();
            ALICEVISION_LOG_TRACE("Load weights with path " << weightsPath);
            image::readImage(weightsPath, weights, image::EImageColorSpace::NO_CONVERSION);*/
            weights = image::Image<float>(mask.Width(), mask.Height());
            
            if (!getMaskFromLabels(weights, labels, viewCurrent, offsetX - referenceBoundingBox.left, offsetY - referenceBoundingBox.top)) 
            {
                ALICEVISION_LOG_ERROR("Error estimating seams image");
                return EXIT_FAILURE;
            }

            if (!compositer->append(source, mask, weights, offsetX - referenceBoundingBox.left, offsetY - referenceBoundingBox.top))
            {
                ALICEVISION_LOG_INFO("Error in compositer append");
                return EXIT_FAILURE;
            }  

            if (mask.Width() >= panoramaMap->getWidth())
            {
                if (!getMaskFromLabels(weights, labels, viewCurrent, offsetX - panoramaMap->getWidth() - referenceBoundingBox.left, offsetY - referenceBoundingBox.top)) 
                {
                    ALICEVISION_LOG_ERROR("Error estimating seams image");
                    return EXIT_FAILURE;
                }

                if (!compositer->append(source, mask, weights, offsetX - panoramaMap->getWidth() - referenceBoundingBox.left, offsetY - referenceBoundingBox.top))
                {
                    ALICEVISION_LOG_INFO("Error in compositer append");
                    return EXIT_FAILURE;
                } 
            }
        }


        std::cout << "terminate" << std::endl;
        if (!compositer->terminate())
        {
            ALICEVISION_LOG_ERROR("Error terminating panorama");
        }

        const std::string viewIdStr = std::to_string(viewReference);
        const std::string outputFilePath = (fs::path(outputFolder) / (viewIdStr + ".exr")).string();
        if (!compositer->save(outputFilePath, storageDataType))
        {
            ALICEVISION_LOG_ERROR("Error terminating panorama");
        }
    }

    return EXIT_SUCCESS;
}
