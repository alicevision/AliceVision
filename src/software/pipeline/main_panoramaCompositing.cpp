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

bool computeWTALabels(image::Image<IndexT> & labels, std::list<IndexT> & views, const std::string & inputPath, const PanoramaMap & map, const IndexT & referenceIndex)
{
    ALICEVISION_LOG_INFO("Estimating initial labels for panorama");

    BoundingBox referenceBoundingBox;
    
    if (!map.getBoundingBox(referenceBoundingBox, referenceIndex))
    {
        return false;
    }

    WTASeams seams(referenceBoundingBox.width, referenceBoundingBox.height);

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

        
        std::vector<BoundingBox> intersections;
        std::vector<BoundingBox> currentBoundingBoxes;
        if (!map.getIntersectionsList(intersections, currentBoundingBoxes, referenceIndex, currentId))
        {
            continue;
        }
        
        for (const BoundingBox & bbox : currentBoundingBoxes)
        {
            if (!seams.append(mask, weights, currentId, bbox.left - referenceBoundingBox.left, bbox.top - referenceBoundingBox.top)) 
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
    std::string compositerType = "multiband";
    int rangeIteration = -1;
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
        ("output,o", po::value<std::string>(&outputFolder)->required(), "Path of the output panorama.");
    allParams.add(requiredParams);

    // Description of optional parameters
    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("compositerType,c", po::value<std::string>(&compositerType)->required(), "Compositer Type [replace, alpha, multiband].")
        ("storageDataType", po::value<image::EStorageDataType>(&storageDataType)->default_value(storageDataType), ("Storage data type: " + image::EStorageDataType_informations()).c_str())
        ("rangeIteration", po::value<int>(&rangeIteration)->default_value(rangeIteration), "Range chunk id.")
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
    int viewsCount = sfmData.getViews().size();
    if(rangeIteration != -1)
    {
        if(rangeIteration < 0 || rangeSize < 0)
        {
            ALICEVISION_LOG_ERROR("Range is incorrect");
            return EXIT_FAILURE;
        }

        int countIterations = int(std::ceil(double(viewsCount) / double(rangeSize)));
       
        if(rangeIteration >= countIterations)
        {
            ALICEVISION_LOG_ERROR("Range is incorrect");
            return EXIT_FAILURE;
        }
    }
    else
    {
        rangeIteration = 0;
        rangeSize = int(viewsCount);
    }

    size_t borderSize;
    if (compositerType == "multiband")
    {
        borderSize = 2;
    }
    else if (compositerType == "alpha")
    {
        borderSize = 0;
    }
    else 
    {
        borderSize = 0;
    }

    // Build the map of inputs in the final panorama
    // This is mostly meant to compute overlaps between inputs
    std::unique_ptr<PanoramaMap> panoramaMap = buildMap(sfmData, warpingFolder, borderSize);
    if (viewsCount == 0) 
    {
        ALICEVISION_LOG_ERROR("No valid views");
        return EXIT_FAILURE;
    }

    // Distribute more smartly inputs among chunks
    std::vector<std::vector<IndexT>> chunks;
    if (!panoramaMap->optimizeChunks(chunks, rangeSize))
    {
        ALICEVISION_LOG_ERROR("Can't build chunks");
        return EXIT_FAILURE;
    }
    

    const std::vector<IndexT> & chunk = chunks[rangeIteration];
    for (std::size_t posReference = 0; posReference < chunk.size(); posReference++)
    {
        ALICEVISION_LOG_INFO("processing input region " << posReference + 1 << "/" << chunk.size());

        IndexT viewReference = chunk[posReference];
        
        // Get the list of input which should be processed for this reference view bounding box
        std::list<IndexT> overlaps;
        if (!panoramaMap->getOverlaps(overlaps, viewReference)) 
        {
            ALICEVISION_LOG_ERROR("Problem analyzing neighboorhood");
            return EXIT_FAILURE;
        }

        // Add the current input also for simpler processing
        overlaps.push_back(viewReference);

        // Get the input bounding box to define the ROI
        BoundingBox referenceBoundingBox;
        if (!panoramaMap->getBoundingBox(referenceBoundingBox, viewReference)) 
        {
            ALICEVISION_LOG_ERROR("Problem getting reference bounding box");
            return EXIT_FAILURE;
        }

        //Create a compositer depending on what we need
        bool needWeights;
        bool needSeams;
        std::unique_ptr<Compositer> compositer;
        if (compositerType == "multiband")
        {
            needWeights = false;
            needSeams = true;
            compositer = std::unique_ptr<Compositer>(new LaplacianCompositer(referenceBoundingBox.width, referenceBoundingBox.height, panoramaMap->getScale()));
        }
        else if (compositerType == "alpha")
        {
            needWeights = true;
            needSeams = false;
            compositer = std::unique_ptr<Compositer>(new AlphaCompositer(referenceBoundingBox.width, referenceBoundingBox.height));
        }
        else 
        {
            needWeights = false;
            needSeams = false;
            compositer = std::unique_ptr<Compositer>(new Compositer(referenceBoundingBox.width, referenceBoundingBox.height));
        }
        
        // Compositer initialization
        if (!compositer->initialize())
        {
            ALICEVISION_LOG_ERROR("Error initializing panorama");
            return false;
        }

        // Compute initial seams
        image::Image<IndexT> labels;
        if (needSeams)
        {
            if (!computeWTALabels(labels, overlaps, warpingFolder, *panoramaMap, viewReference)) 
            {
                ALICEVISION_LOG_ERROR("Error estimating panorama labels for this input");
                return false;
            }
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

            //Compute list of intersection between this view and the reference view
            std::vector<BoundingBox> intersections;
            std::vector<BoundingBox> currentBoundingBoxes;
            if (!panoramaMap->getIntersectionsList(intersections, currentBoundingBoxes, viewReference, viewCurrent))
            {
                continue;
            }
            
            for (int indexIntersection = 0; indexIntersection < intersections.size(); indexIntersection++)
            {
                const BoundingBox & bbox = currentBoundingBoxes[indexIntersection];
                const BoundingBox & bboxIntersect = intersections[indexIntersection];

                BoundingBox cutBoundingBox;
                cutBoundingBox.left = bboxIntersect.left - bbox.left;
                cutBoundingBox.top = bboxIntersect.top - bbox.top;
                cutBoundingBox.width = bboxIntersect.width;
                cutBoundingBox.height = bboxIntersect.height;
                if (cutBoundingBox.isEmpty())
                {
                    continue;
                }


                image::Image<float> weights; 
                if (needWeights)
                {
                    const std::string weightsPath = (fs::path(warpingFolder) / (std::to_string(viewCurrent) + "_weight.exr")).string();
                    ALICEVISION_LOG_TRACE("Load weights with path " << weightsPath);
                    image::readImage(weightsPath, weights, image::EImageColorSpace::NO_CONVERSION);
                }
                else if (needSeams)
                {
                    weights = image::Image<float>(mask.Width(), mask.Height());
                    if (!getMaskFromLabels(weights, labels, viewCurrent, bbox.left - referenceBoundingBox.left, bbox.top - referenceBoundingBox.top)) 
                    {
                        ALICEVISION_LOG_ERROR("Error estimating seams image");
                        return EXIT_FAILURE;
                    }
                }

                image::Image<image::RGBfColor> subsource(cutBoundingBox.width, cutBoundingBox.height); 
                image::Image<unsigned char> submask(cutBoundingBox.width, cutBoundingBox.height); 
                image::Image<float> subweights(cutBoundingBox.width, cutBoundingBox.height); 

                subsource = source.block(cutBoundingBox.top, cutBoundingBox.left, cutBoundingBox.height, cutBoundingBox.width);
                submask = mask.block(cutBoundingBox.top, cutBoundingBox.left, cutBoundingBox.height, cutBoundingBox.width);

                if (weights.Width() > 0)
                {
                    subweights = weights.block(cutBoundingBox.top, cutBoundingBox.left, cutBoundingBox.height, cutBoundingBox.width);
                }
        
                if (!compositer->append(subsource, submask, subweights, bboxIntersect.left - referenceBoundingBox.left, bboxIntersect.top - referenceBoundingBox.top))
                {
                    ALICEVISION_LOG_INFO("Error in compositer append");
                    return EXIT_FAILURE;
                }
            }
        }


        ALICEVISION_LOG_INFO("Terminate compositing for this view");
        if (!compositer->terminate())
        {
            ALICEVISION_LOG_ERROR("Error terminating panorama");
        }

        const std::string viewIdStr = std::to_string(viewReference);
        const std::string outputFilePath = (fs::path(outputFolder) / (viewIdStr + ".exr")).string();
        image::Image<image::RGBAfColor> output = compositer->getOutput();

         if (storageDataType == image::EStorageDataType::HalfFinite)
        {
            for (int i = 0; i < output.Height(); i++) 
            {
                for (int j = 0; j < output.Width(); j++)
                {
                    image::RGBAfColor ret;
                    image::RGBAfColor c = output(i, j);

                    const float limit = float(HALF_MAX);
                    
                    ret.r() = clamp(c.r(), -limit, limit);
                    ret.g() = clamp(c.g(), -limit, limit);
                    ret.b() = clamp(c.b(), -limit, limit);
                    ret.a() = c.a();

                    output(i, j) = ret;
                }
            }
        }

        oiio::ParamValueList metadata;
        metadata.push_back(oiio::ParamValue("AliceVision:storageDataType", EStorageDataType_enumToString(storageDataType)));
        metadata.push_back(oiio::ParamValue("AliceVision:offsetX", int(referenceBoundingBox.left)));
		metadata.push_back(oiio::ParamValue("AliceVision:offsetY", int(referenceBoundingBox.top)));
        metadata.push_back(oiio::ParamValue("AliceVision:panoramaWidth", int(panoramaMap->getWidth())));
		metadata.push_back(oiio::ParamValue("AliceVision:panoramaHeight", int(panoramaMap->getHeight())));

        image::writeImage(outputFilePath, output, image::EImageColorSpace::LINEAR, metadata);
    }

    return EXIT_SUCCESS;
}
