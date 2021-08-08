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
    
    return (optimal_scale);
}

std::unique_ptr<PanoramaMap> buildMap(const sfmData::SfMData & sfmData, const std::string & inputPath, const size_t borderSize)
{   
    if (sfmData.getViews().empty()) 
    {
        return nullptr;
    }

    size_t min_scale = std::numeric_limits<size_t>::max();
    std::vector<std::pair<IndexT, BoundingBox>> listBoundingBox;
    std::pair<std::size_t, std::size_t> panoramaSize;

    for (const auto& viewIt : sfmData.getViews())
    {
        if(!sfmData.isPoseAndIntrinsicDefined(viewIt.first))
            continue;

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

        if (viewIt.first == 0) continue;

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

bool processImage(const PanoramaMap & panoramaMap, const std::string & compositerType, const std::string & warpingFolder, const std::string & labelsFilePath, const std::string & outputFolder, const image::EStorageDataType & storageDataType, IndexT viewReference, const BoundingBox & referenceBoundingBox, bool showBorders, bool showSeams)
{
    // The laplacian pyramid must also contains some pixels outside of the bounding box to make sure 
    // there is a continuity between all the "views" of the panorama.
    BoundingBox panoramaBoundingBox = referenceBoundingBox;


    //Create a compositer depending on what was requested
    bool needWeights;
    bool needSeams;
    std::unique_ptr<Compositer> compositer;
    if (compositerType == "multiband")
    {
        needWeights = false;
        needSeams = true;

        //Enlarge the panorama boundingbox to allow consider neighboor pixels even at small scale
        panoramaBoundingBox = referenceBoundingBox.divide(panoramaMap.getScale()).dilate(panoramaMap.getBorderSize()).multiply(panoramaMap.getScale());
        compositer = std::unique_ptr<Compositer>(new LaplacianCompositer(panoramaBoundingBox.width, panoramaBoundingBox.height, panoramaMap.getScale()));
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


    // Get the list of input which should be processed for this reference view bounding box
    std::vector<IndexT> overlappingViews;
    if (!panoramaMap.getOverlaps(overlappingViews, referenceBoundingBox)) 
    {
        ALICEVISION_LOG_ERROR("Problem analyzing neighboorhood");
        return false;
    }
    
    // Compute the bounding box of the intersections with the reference bounding box 
    // (which may be larger than the reference Bounding box because of dilatation)
    BoundingBox globalUnionBoundingBox;
    for (IndexT viewCurrent : overlappingViews)
    {
        //Compute list of intersection between this view and the reference view
        std::vector<BoundingBox> intersections;
        std::vector<BoundingBox> currentBoundingBoxes;
        if (!panoramaMap.getIntersectionsList(intersections, currentBoundingBoxes, referenceBoundingBox, viewCurrent))
        {
            continue;
        }

        for (BoundingBox & bb : intersections) 
        {
            globalUnionBoundingBox = globalUnionBoundingBox.unionWith(bb);
        }
    }

    ALICEVISION_LOG_INFO("Building the visibility map");

    // Building a map of visible pixels 
    image::Image<std::vector<IndexT>> visiblePixels(globalUnionBoundingBox.width, globalUnionBoundingBox.height, true);
    for (IndexT viewCurrent : overlappingViews)
    {        
        // Load mask
        const std::string maskPath = (fs::path(warpingFolder) / (std::to_string(viewCurrent) + "_mask.exr")).string();
        ALICEVISION_LOG_TRACE("Load mask with path " << maskPath);
        image::Image<unsigned char> mask;
        image::readImageDirect(maskPath, mask);

        // Compute list of intersection between this view and the reference view
        std::vector<BoundingBox> intersections;
        std::vector<BoundingBox> currentBoundingBoxes;
        if (!panoramaMap.getIntersectionsList(intersections, currentBoundingBoxes, referenceBoundingBox, viewCurrent))
        {
            continue;
        }

        for (int indexIntersection = 0; indexIntersection < intersections.size(); indexIntersection++)
        {
            const BoundingBox & bbox = currentBoundingBoxes[indexIntersection];
            const BoundingBox & bboxIntersect = intersections[indexIntersection];

            for (int i = 0; i < mask.Height(); i++) 
            {
                int y = bbox.top + i - globalUnionBoundingBox.top;
                if (y < 0 || y >= globalUnionBoundingBox.height) 
                {
                    continue;
                }

                for (int j = 0; j < mask.Width(); j++) 
                {
                    if (!mask(i, j))
                    {
                        continue;
                    }

                    int x = bbox.left + j - globalUnionBoundingBox.left;
                    if (x < 0 || x >= globalUnionBoundingBox.width) 
                    {
                        continue;
                    }


                    visiblePixels(y, x).push_back(viewCurrent);
                }
            }
        }
    }
    

    ALICEVISION_LOG_INFO("Building the seams map");

    // Compute initial seams
    image::Image<IndexT> referenceLabels;
    if (needSeams)
    {
        image::Image<IndexT> panoramaLabels;
        image::readImageDirect(labelsFilePath, panoramaLabels);

        double scaleX = double(panoramaLabels.Width()) / double(panoramaMap.getWidth());
        double scaleY = double(panoramaLabels.Height()) / double(panoramaMap.getHeight());

        referenceLabels = image::Image<IndexT>(globalUnionBoundingBox.width, globalUnionBoundingBox.height, true, UndefinedIndexT);
        
        for (int i = 0; i < globalUnionBoundingBox.height; i++) 
        {
            int y = i + globalUnionBoundingBox.top;
            int scaledY = int(floor(scaleY * double(y)));

            for (int j = 0; j < globalUnionBoundingBox.width; j++)
            {
                int x = j + globalUnionBoundingBox.left;
                int scaledX = int(floor(scaleX * double(x)));

                if (scaledX < 0) 
                {
                    scaledX += panoramaLabels.Width();
                }    

                if (scaledX >= panoramaLabels.Width()) 
                {
                    scaledX -= panoramaLabels.Width();
                }

                if (scaledX < 0) continue;
                if (scaledX >= panoramaLabels.Width()) continue;

                IndexT label = panoramaLabels(scaledY, scaledX);

                bool found = false;
                auto & listValid = visiblePixels(i, j);
                for (auto item : listValid) 
                {
                    if (item == label) 
                    {
                        found = true;
                        break;
                    }
                }

                if (found)
                {
                    referenceLabels(i, j) = label;
                    continue;
                }

                found = false;
                for (int k = -1; k <= 1; k++)
                {
                    int nscaledY = scaledY + k;
                    if (nscaledY < 0) continue;
                    if (nscaledY >= panoramaLabels.Height()) continue;

                    for (int l = -1; l <= 1; l++)
                    {
                        if (k == 0 && l == 0) continue;
                        
                        int nscaledX = scaledX + l;
                        if (nscaledX < 0) continue;
                        if (nscaledX >= panoramaLabels.Width()) continue;

                        IndexT otherlabel = panoramaLabels(nscaledY, nscaledX);
                        for (auto item : listValid) 
                        {
                            if (item == otherlabel) 
                            {
                                label = otherlabel;
                                found = true;
                                break;
                            }
                        }

                        if (found) break;
                    }

                    if (found) break;
                }

                if (!found)
                {
                    referenceLabels(i, j) = UndefinedIndexT;
                    continue;
                }

                referenceLabels(i, j) = label;
            }
        }
    }    
    
    // Compute the roi of the output inside the compositer computed
    // image (which may be larger than required for algorithmic reasons)
    BoundingBox bbRoi;
    bbRoi.left = referenceBoundingBox.left - panoramaBoundingBox.left;
    bbRoi.top = referenceBoundingBox.top - panoramaBoundingBox.top;
    bbRoi.width = referenceBoundingBox.width;
    bbRoi.height = referenceBoundingBox.height;
    
    // Compositer initialization
    if (!compositer->initialize(bbRoi))
    {
        ALICEVISION_LOG_ERROR("Error initializing panorama");
        return false;
    }

    bool hasFailed = false;

    #pragma omp parallel for
    for (int posCurrent = 0; posCurrent < overlappingViews.size(); posCurrent++)
    {
        IndexT viewCurrent = overlappingViews[posCurrent];
        if (hasFailed)
        {
            continue;
        }

        ALICEVISION_LOG_INFO("Processing input " << posCurrent << "/" << overlappingViews.size());

        // Compute list of intersection between this view and the reference view
        std::vector<BoundingBox> intersections;
        std::vector<BoundingBox> currentBoundingBoxes;
        if (!panoramaMap.getIntersectionsList(intersections, currentBoundingBoxes, referenceBoundingBox, viewCurrent))
        {
            continue;
        }

        if (intersections.empty())
        {
            continue;
        }

        ALICEVISION_LOG_TRACE("Effective processing");
        for (int indexIntersection = 0; indexIntersection < intersections.size(); indexIntersection++)
        {
            if (hasFailed)
            {
                continue;
            }

            const BoundingBox & bbox = currentBoundingBoxes[indexIntersection];
            const BoundingBox & bboxIntersect = intersections[indexIntersection];

            // Load image
            const std::string imagePath = (fs::path(warpingFolder) / (std::to_string(viewCurrent) + ".exr")).string();
            ALICEVISION_LOG_TRACE("Load image with path " << imagePath);
            image::Image<image::RGBfColor> source;
            image::readImage(imagePath, source, image::EImageColorSpace::NO_CONVERSION);

            // Load mask
            const std::string maskPath = (fs::path(warpingFolder) / (std::to_string(viewCurrent) + "_mask.exr")).string();
            ALICEVISION_LOG_TRACE("Load mask with path " << maskPath);
            image::Image<unsigned char> mask;
            image::readImageDirect(maskPath, mask);

            // Load weights image if needed
            image::Image<float> weights; 
            if (needWeights)
            {
                const std::string weightsPath = (fs::path(warpingFolder) / (std::to_string(viewCurrent) + "_weight.exr")).string();
                ALICEVISION_LOG_TRACE("Load weights with path " << weightsPath);
                image::readImage(weightsPath, weights, image::EImageColorSpace::NO_CONVERSION);
            }
            
            if (needSeams)
            {
                int left = bboxIntersect.left - globalUnionBoundingBox.left;
                int top = bboxIntersect.top - globalUnionBoundingBox.top;

                weights = image::Image<float>(bboxIntersect.width, bboxIntersect.height);
                if (!getMaskFromLabels(weights, referenceLabels, viewCurrent, left, top)) 
                {
                    ALICEVISION_LOG_ERROR("Error estimating seams image");
                    hasFailed = true;
                }
            }

            BoundingBox cutBoundingBox;
            cutBoundingBox.left = bboxIntersect.left - bbox.left;
            cutBoundingBox.top = bboxIntersect.top - bbox.top;
            cutBoundingBox.width = bboxIntersect.width;
            cutBoundingBox.height = bboxIntersect.height;
            if (cutBoundingBox.isEmpty())
            {
                continue;
            }


            image::Image<image::RGBfColor> subsource(cutBoundingBox.width, cutBoundingBox.height); 
            image::Image<unsigned char> submask(cutBoundingBox.width, cutBoundingBox.height);  

            subsource = source.block(cutBoundingBox.top, cutBoundingBox.left, cutBoundingBox.height, cutBoundingBox.width);
            submask = mask.block(cutBoundingBox.top, cutBoundingBox.left, cutBoundingBox.height, cutBoundingBox.width);  

            source = image::Image<image::RGBfColor>(); 
            mask = image::Image<unsigned char>(); 

            if (!compositer->append(subsource, submask, weights, referenceBoundingBox.left - panoramaBoundingBox.left + bboxIntersect.left - referenceBoundingBox.left , referenceBoundingBox.top - panoramaBoundingBox.top + bboxIntersect.top  - referenceBoundingBox.top))
            {
                ALICEVISION_LOG_INFO("Error in compositer append");
                hasFailed = true;
                continue;
            }
        }
    }

    if (hasFailed) 
    {
        return false;
    }


    ALICEVISION_LOG_INFO("Terminate compositing for this view");
    if (!compositer->terminate())
    {
        ALICEVISION_LOG_ERROR("Error terminating panorama");
        return false;
    }

    const std::string viewIdStr = std::to_string(viewReference);
    const std::string outputFilePath = (fs::path(outputFolder) / (viewIdStr + ".exr")).string();
    image::Image<image::RGBAfColor> & output = compositer->getOutput();

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

    if (showBorders)
    {
        ALICEVISION_LOG_INFO("Draw borders");
        for (IndexT viewCurrent : overlappingViews)
        {
            // Compute list of intersection between this view and the reference view
            std::vector<BoundingBox> intersections;
            std::vector<BoundingBox> currentBoundingBoxes;
            if (!panoramaMap.getIntersectionsList(intersections, currentBoundingBoxes, referenceBoundingBox, viewCurrent))
            {
                continue;
            }

            // Load mask
            const std::string maskPath = (fs::path(warpingFolder) / (std::to_string(viewCurrent) + "_mask.exr")).string();
            ALICEVISION_LOG_TRACE("Load mask with path " << maskPath);
            image::Image<unsigned char> mask;
            image::readImageDirect(maskPath, mask);
            
            
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

                image::Image<unsigned char> submask(cutBoundingBox.width, cutBoundingBox.height);  
                submask = mask.block(cutBoundingBox.top, cutBoundingBox.left, cutBoundingBox.height, cutBoundingBox.width);    

                drawBorders(output, submask, bboxIntersect.left - referenceBoundingBox.left, bboxIntersect.top - referenceBoundingBox.top);
            }
        }
    }

    if (showSeams && needSeams)
    {
        drawSeams(output, referenceLabels, globalUnionBoundingBox.left - referenceBoundingBox.left, globalUnionBoundingBox.top- referenceBoundingBox.top);
    }

    oiio::ParamValueList metadata;
    metadata.push_back(oiio::ParamValue("AliceVision:storageDataType", EStorageDataType_enumToString(storageDataType)));
    metadata.push_back(oiio::ParamValue("AliceVision:offsetX", int(referenceBoundingBox.left)));
    metadata.push_back(oiio::ParamValue("AliceVision:offsetY", int(referenceBoundingBox.top)));
    metadata.push_back(oiio::ParamValue("AliceVision:panoramaWidth", int(panoramaMap.getWidth())));
    metadata.push_back(oiio::ParamValue("AliceVision:panoramaHeight", int(panoramaMap.getHeight())));

    image::writeImage(outputFilePath, output, image::EImageColorSpace::LINEAR, metadata);

    return true;
}

int aliceVision_main(int argc, char** argv)
{
    std::string sfmDataFilepath;
    std::string labelsFilepath;
    std::string warpingFolder;
    std::string outputFolder;
    std::string compositerType = "multiband";
    std::string overlayType = "none";
    int rangeIteration = -1;
	int rangeSize = 1;
    int maxThreads = 1;
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
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilepath)->required(), "Input sfmData.")
        ("warpingFolder,w", po::value<std::string>(&warpingFolder)->required(), "Folder with warped images.")
        ("output,o", po::value<std::string>(&outputFolder)->required(), "Path of the output panorama.");
    allParams.add(requiredParams);

    // Description of optional parameters
    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("compositerType,c", po::value<std::string>(&compositerType)->required(), "Compositer Type [replace, alpha, multiband].")
        ("overlayType,c", po::value<std::string>(&overlayType)->required(), "Overlay Type [none, borders, seams, all].")
        ("storageDataType", po::value<image::EStorageDataType>(&storageDataType)->default_value(storageDataType), ("Storage data type: " + image::EStorageDataType_informations()).c_str())
        ("rangeIteration", po::value<int>(&rangeIteration)->default_value(rangeIteration), "Range chunk id.")
		("rangeSize", po::value<int>(&rangeSize)->default_value(rangeSize), "Range size.")
        ("maxThreads", po::value<int>(&maxThreads)->default_value(maxThreads), "max number of threads to use.")
        ("labels,l", po::value<std::string>(&labelsFilepath)->required(), "Labels image from seams estimation.");
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

    if (overlayType == "borders" || overlayType == "all")
    {
        showBorders = true;
    }

    if (overlayType == "seams" || overlayType == "all") {
        showSeams = true;
    }

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
    
    if(rangeIteration >= chunks.size())
    {
        // nothing to compute for this chunk
        return EXIT_SUCCESS;
    }

    const std::vector<IndexT> & chunk = chunks[rangeIteration];
    
    bool succeeded = true;

    if(maxThreads > 0)
        omp_set_num_threads(std::min(omp_get_max_threads(), maxThreads));

    //#pragma omp parallel for
    for (std::size_t posReference = 0; posReference < chunk.size(); posReference++)
    {
        ALICEVISION_LOG_INFO("processing input region " << posReference + 1 << "/" << chunk.size());

        IndexT viewReference = chunk[posReference];
        if(!sfmData.isPoseAndIntrinsicDefined(viewReference))
            continue;

        BoundingBox referenceBoundingBox;
        if (!panoramaMap->getBoundingBox(referenceBoundingBox, viewReference))
        {
            ALICEVISION_LOG_ERROR("Invalid view ID as reference");
            return EXIT_FAILURE;
        }

        if (!processImage(*panoramaMap, compositerType, warpingFolder, labelsFilepath, outputFolder, storageDataType, viewReference, referenceBoundingBox, showBorders, showSeams)) 
        {
            succeeded = false;
            continue;
        }
    }

    if (!succeeded) 
    {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
