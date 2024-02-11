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
#include <aliceVision/image/imageAlgo.hpp>

// System
#include <aliceVision/system/Logger.hpp>

// Reading command line options
#include <boost/program_options.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/main.hpp>

// Numeric utils
#include <aliceVision/numeric/numeric.hpp>

// IO
#include <filesystem>
#include <fstream>
#include <algorithm>
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

size_t getCompositingOptimalScale(int width, int height)
{
    /*
    Look for the smallest scale such that the image is not smaller than the
    convolution window size.
    minsize / 2^x = 5
    minsize / 5 = 2^x
    x = log2(minsize/5)
    */

    const size_t minsize = std::min(width, height);

    /*
     * Ideally, should be gaussianFilterSize = 1 + 2 * gaussianFilterRadius with:
     * const size_t gaussianFilterRadius = 2;
     */
    const int gaussianFilterSize = 5;

    // Avoid negative values on scale
    if (minsize < gaussianFilterSize)
    {
        return 0;
    }

    const size_t optimal_scale = size_t(floor(std::log2(double(minsize) / gaussianFilterSize)));

    return optimal_scale;
}

std::unique_ptr<PanoramaMap> buildMap(const sfmData::SfMData& sfmData,
                                      const std::string& inputPath,
                                      const size_t borderSize,
                                      size_t forceMinPyramidLevels)
{
    if (sfmData.getViews().empty())
    {
        return nullptr;
    }

    size_t max_scale = 0;
    std::vector<std::pair<IndexT, BoundingBox>> listBoundingBox;
    std::pair<std::size_t, std::size_t> panoramaSize;

    for (const auto& viewIt : sfmData.getViews())
    {
        if (!sfmData.isPoseAndIntrinsicDefined(viewIt.first))
            continue;

        const std::string warpedPath = viewIt.second->getImage().getMetadata().at("AliceVision:warpedPath");

        // Load mask
        const std::string maskPath = (fs::path(inputPath) / (warpedPath + "_mask.exr")).string();
        ALICEVISION_LOG_TRACE("Load metadata of mask with path " << maskPath);

        int width = 0;
        int height = 0;
        oiio::ParamValueList metadata = image::readImageMetadata(maskPath, width, height);
        const int offsetX = metadata.find("AliceVision:offsetX")->get_int();
        const int offsetY = metadata.find("AliceVision:offsetY")->get_int();
        panoramaSize.first = metadata.find("AliceVision:panoramaWidth")->get_int();
        panoramaSize.second = metadata.find("AliceVision:panoramaHeight")->get_int();

        BoundingBox bb;
        bb.left = offsetX;
        bb.top = offsetY;
        bb.width = width;
        bb.height = height;

        if (viewIt.first == 0)
            continue;

        listBoundingBox.push_back(std::make_pair(viewIt.first, bb));
        size_t scale = getCompositingOptimalScale(width, height);
        if (scale > max_scale)
        {
            max_scale = scale;
        }
    }

    ALICEVISION_LOG_INFO("Estimated pyramid levels count: " << max_scale);

    if (forceMinPyramidLevels > max_scale)
    {
        max_scale = forceMinPyramidLevels;
        ALICEVISION_LOG_INFO("Forced pyramid levels count: " << max_scale);
    }

    std::unique_ptr<PanoramaMap> ret(new PanoramaMap(panoramaSize.first, panoramaSize.second, max_scale, borderSize));
    for (const auto& bbitem : listBoundingBox)
    {
        ret->append(bbitem.first, bbitem.second);
    }

    return ret;
}

bool processImage(const PanoramaMap& panoramaMap,
                  const sfmData::SfMData& sfmData,
                  const std::string& compositerType,
                  const std::string& warpingFolder,
                  const std::string& labelsFilePath,
                  const std::string& outputFolder,
                  const image::EStorageDataType& storageDataType,
                  IndexT viewReference,
                  const BoundingBox& referenceBoundingBox,
                  bool showBorders,
                  bool showSeams)
{
    // The laplacian pyramid must also contains some pixels outside of the bounding box to make sure
    // there is a continuity between all the "views" of the panorama.
    BoundingBox panoramaBoundingBox = referenceBoundingBox;

    // Create a compositer depending on what was requested
    bool needWeights;
    bool needSeams;
    std::unique_ptr<Compositer> compositer;
    if (compositerType == "multiband")
    {
        needWeights = false;
        needSeams = true;

        // Enlarge the panorama boundingbox to allow consider neighboor pixels even at small scale
        panoramaBoundingBox =
          referenceBoundingBox.divide(panoramaMap.getScale()).dilate(panoramaMap.getBorderSize()).multiply(panoramaMap.getScale());

        panoramaBoundingBox.clampTop();
        panoramaBoundingBox.clampBottom(panoramaMap.getHeight());

        compositer =
          std::unique_ptr<Compositer>(new LaplacianCompositer(panoramaBoundingBox.width, panoramaBoundingBox.height, panoramaMap.getScale()));
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
        // Compute list of intersection between this view and the reference view
        std::vector<BoundingBox> intersections;
        std::vector<BoundingBox> currentBoundingBoxes;
        if (!panoramaMap.getIntersectionsList(intersections, currentBoundingBoxes, referenceBoundingBox, viewCurrent))
        {
            continue;
        }

        for (BoundingBox& bb : intersections)
        {
            globalUnionBoundingBox = globalUnionBoundingBox.unionWith(bb);
        }
    }

    ALICEVISION_LOG_INFO("Building the visibility map");

    // Building a map of visible pixels
    image::Image<std::vector<IndexT>> visiblePixels(globalUnionBoundingBox.width, globalUnionBoundingBox.height, true);
    for (IndexT viewCurrent : overlappingViews)
    {
        const std::string warpedPath = sfmData.getViews().at(viewCurrent)->getImage().getMetadata().at("AliceVision:warpedPath");

        // Load mask
        const std::string maskPath = (fs::path(warpingFolder) / (warpedPath + "_mask.exr")).string();
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
            const BoundingBox& bbox = currentBoundingBoxes[indexIntersection];
            const BoundingBox& bboxIntersect = intersections[indexIntersection];

            for (int i = 0; i < mask.height(); i++)
            {
                int y = bbox.top + i - globalUnionBoundingBox.top;
                if (y < 0 || y >= globalUnionBoundingBox.height)
                {
                    continue;
                }

                for (int j = 0; j < mask.width(); j++)
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

        const double scaleX = double(panoramaLabels.width()) / double(panoramaMap.getWidth());
        const double scaleY = double(panoramaLabels.height()) / double(panoramaMap.getHeight());

        referenceLabels = image::Image<IndexT>(globalUnionBoundingBox.width, globalUnionBoundingBox.height, true, UndefinedIndexT);

        for (int i = 0; i < globalUnionBoundingBox.height; i++)
        {
            const int y = i + globalUnionBoundingBox.top;
            const int scaledY = int(floor(scaleY * double(y)));

            for (int j = 0; j < globalUnionBoundingBox.width; j++)
            {
                const int x = j + globalUnionBoundingBox.left;
                int scaledX = int(floor(scaleX * double(x)));

                if (scaledX < 0)
                {
                    scaledX += panoramaLabels.width();
                }

                if (scaledX >= panoramaLabels.width())
                {
                    scaledX -= panoramaLabels.width();
                }

                if (scaledX < 0)
                    continue;
                if (scaledX >= panoramaLabels.width())
                    continue;

                IndexT label = panoramaLabels(scaledY, scaledX);

                bool found = false;
                auto& listValid = visiblePixels(i, j);
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
                    if (nscaledY < 0)
                        continue;
                    if (nscaledY >= panoramaLabels.height())
                        continue;

                    for (int l = -1; l <= 1; l++)
                    {
                        if (k == 0 && l == 0)
                            continue;

                        int nscaledX = scaledX + l;
                        if (nscaledX < 0)
                            continue;
                        if (nscaledX >= panoramaLabels.width())
                            continue;

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

                        if (found)
                            break;
                    }

                    if (found)
                        break;
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

    // Load metadata to get image color space
    std::string colorSpace = "Linear";
    oiio::ParamValueList srcMetadata;
    if (!overlappingViews.empty())
    {
        const std::string warpedPath = sfmData.getViews().at(overlappingViews[0])->getImage().getMetadata().at("AliceVision:warpedPath");
        const std::string firstImagePath = (fs::path(warpingFolder) / (warpedPath + ".exr")).string();
        srcMetadata = image::readImageMetadata(firstImagePath);
        colorSpace = srcMetadata.get_string("AliceVision:ColorSpace", "Linear");
    }

#pragma omp parallel for
    for (int posCurrent = 0; posCurrent < overlappingViews.size(); posCurrent++)
    {
        IndexT viewCurrent = overlappingViews[posCurrent];
        if (hasFailed)
        {
            continue;
        }

        const std::string warpedPath = sfmData.getViews().at(viewCurrent)->getImage().getMetadata().at("AliceVision:warpedPath");

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

            const BoundingBox& bbox = currentBoundingBoxes[indexIntersection];
            const BoundingBox& bboxIntersect = intersections[indexIntersection];

            // Load image
            const std::string imagePath = (fs::path(warpingFolder) / (warpedPath + ".exr")).string();
            ALICEVISION_LOG_TRACE("Load image with path " << imagePath);
            image::Image<image::RGBfColor> source;
            image::readImage(imagePath, source, image::EImageColorSpace::NO_CONVERSION);

            // Load mask
            const std::string maskPath = (fs::path(warpingFolder) / (warpedPath + "_mask.exr")).string();
            ALICEVISION_LOG_TRACE("Load mask with path " << maskPath);
            image::Image<unsigned char> mask;
            image::readImageDirect(maskPath, mask);

            // Load weights image if needed
            image::Image<float> weights;
            if (needWeights)
            {
                const std::string weightsPath = (fs::path(warpingFolder) / (warpedPath + "_weight.exr")).string();
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

            if (!compositer->append(subsource,
                                    submask,
                                    weights,
                                    referenceBoundingBox.left - panoramaBoundingBox.left + bboxIntersect.left - referenceBoundingBox.left,
                                    referenceBoundingBox.top - panoramaBoundingBox.top + bboxIntersect.top - referenceBoundingBox.top))
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

    std::string warpedPath;

    if (viewReference == UndefinedIndexT)
    {
        warpedPath = "panorama";
    }
    else
    {
        warpedPath = sfmData.getViews().at(viewReference)->getImage().getMetadata().at("AliceVision:warpedPath");
    }

    const std::string outputFilePath = (fs::path(outputFolder) / (warpedPath + ".exr")).string();
    image::Image<image::RGBAfColor>& output = compositer->getOutput();

    if (storageDataType == image::EStorageDataType::HalfFinite)
    {
        for (int i = 0; i < output.height(); i++)
        {
            for (int j = 0; j < output.width(); j++)
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
            const std::string warpedPath = sfmData.getViews().at(viewCurrent)->getImage().getMetadata().at("AliceVision:warpedPath");
            const std::string maskPath = (fs::path(warpingFolder) / (warpedPath + "_mask.exr")).string();
            ALICEVISION_LOG_TRACE("Load mask with path " << maskPath);
            image::Image<unsigned char> mask;
            image::readImageDirect(maskPath, mask);

            for (int indexIntersection = 0; indexIntersection < intersections.size(); indexIntersection++)
            {
                const BoundingBox& bbox = currentBoundingBoxes[indexIntersection];
                const BoundingBox& bboxIntersect = intersections[indexIntersection];

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
        drawSeams(
          output, referenceLabels, globalUnionBoundingBox.left - referenceBoundingBox.left, globalUnionBoundingBox.top - referenceBoundingBox.top);
    }

    oiio::ParamValueList metadata = srcMetadata;
    metadata.remove("orientation", oiio::TypeDesc::UNKNOWN, false);
    metadata.remove("crop", oiio::TypeDesc::UNKNOWN, false);
    metadata.remove("width", oiio::TypeDesc::UNKNOWN, false);
    metadata.remove("height", oiio::TypeDesc::UNKNOWN, false);
    metadata.push_back(oiio::ParamValue("AliceVision:offsetX", int(referenceBoundingBox.left)));
    metadata.push_back(oiio::ParamValue("AliceVision:offsetY", int(referenceBoundingBox.top)));
    metadata.push_back(oiio::ParamValue("AliceVision:panoramaWidth", int(panoramaMap.getWidth())));
    metadata.push_back(oiio::ParamValue("AliceVision:panoramaHeight", int(panoramaMap.getHeight())));

    image::writeImage(outputFilePath,
                      output,
                      image::ImageWriteOptions()
                        .fromColorSpace(image::EImageColorSpace_stringToEnum(colorSpace))
                        .toColorSpace(image::EImageColorSpace_stringToEnum(colorSpace))
                        .storageDataType(storageDataType),
                      metadata);

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
    int forceMinPyramidLevels = 0;
    bool showBorders = false;
    bool showSeams = false;
    bool useTiling = true;

    image::EStorageDataType storageDataType = image::EStorageDataType::Float;

    // Description of mandatory parameters
    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilepath)->required(),
         "Input SfMData.")
        ("warpingFolder,w", po::value<std::string>(&warpingFolder)->required(),
         "Folder with warped images.")
        ("output,o", po::value<std::string>(&outputFolder)->required(),
         "Path of the output panorama.");

    // Description of optional parameters
    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("compositerType,c", po::value<std::string>(&compositerType)->required(),
         "Compositer type: [replace, alpha, multiband].")
        ("forceMinPyramidLevels,f", po::value<int>(&forceMinPyramidLevels)->default_value(forceMinPyramidLevels),
         "For multiband compositer, force a minimum number of levels in the image pyramid.")
        ("overlayType,c", po::value<std::string>(&overlayType)->required(),
         "Overlay type: [none, borders, seams, all].")
        ("storageDataType", po::value<image::EStorageDataType>(&storageDataType)->default_value(storageDataType),
         ("Storage data type: " + image::EStorageDataType_informations()).c_str())
        ("rangeIteration", po::value<int>(&rangeIteration)->default_value(rangeIteration),
         "Range chunk ID.")
        ("rangeSize", po::value<int>(&rangeSize)->default_value(rangeSize),
         "Range size.")
        ("maxThreads", po::value<int>(&maxThreads)->default_value(maxThreads),
         "Maximum number of threads to use.")
        ("labels,l", po::value<std::string>(&labelsFilepath)->required(),
         "Labels image from seams estimation.")
        ("useTiling,n", po::value<bool>(&useTiling)->default_value(useTiling),
         "Use tiling for compositing.");
    // clang-format on

    CmdLine cmdline("Performs the panorama stiching of warped images, with an option to use constraints from precomputed seams maps.\n"
                    "AliceVision panoramaCompositing");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // set maxThreads
    HardwareContext hwc = cmdline.getHardwareContext();
    hwc.setUserCoresLimit(maxThreads);

    omp_set_num_threads(hwc.getMaxThreads());
    oiio::attribute("threads", static_cast<int>(hwc.getMaxThreads()));
    oiio::attribute("exr_threads", static_cast<int>(hwc.getMaxThreads()));

    if (overlayType == "borders" || overlayType == "all")
    {
        showBorders = true;
    }

    if (overlayType == "seams" || overlayType == "all")
    {
        showSeams = true;
    }

    if (forceMinPyramidLevels > 16)
    {
        ALICEVISION_LOG_ERROR("forceMinPyramidLevels parameter has a value which is too large.");
        return EXIT_FAILURE;
    }

    // load input scene
    sfmData::SfMData sfmData;
    if (!sfmDataIO::load(sfmData, sfmDataFilepath, sfmDataIO::ESfMData(sfmDataIO::VIEWS | sfmDataIO::EXTRINSICS | sfmDataIO::INTRINSICS)))
    {
        ALICEVISION_LOG_ERROR("The input file '" + sfmDataFilepath + "' cannot be read");
        return EXIT_FAILURE;
    }

    ALICEVISION_LOG_TRACE("Sfm data loaded");

    std::set<std::string> uniquePreviousId;
    for (const auto pv : sfmData.getViews())
    {
        if (pv.second->getImage().getMetadata().find("AliceVision:previousViewId") == pv.second->getImage().getMetadata().end())
        {
            ALICEVISION_LOG_ERROR("You mixed different versions of alicevision.");
            ALICEVISION_LOG_ERROR("Warped images do not contain the required metadatas.");
            return EXIT_FAILURE;
        }
        const std::string pvid = pv.second->getImage().getMetadata().at("AliceVision:previousViewId");
        uniquePreviousId.insert(pvid);
    }
    const int oldViewsCount = uniquePreviousId.size();

    ALICEVISION_LOG_TRACE("Previous id loaded");

    // Define range to compute
    const int viewsCount = sfmData.getViews().size();
    if (!useTiling)
    {
        rangeIteration = 0;
        rangeSize = 1;
    }
    if (rangeIteration != -1)
    {
        if (rangeIteration < 0 || rangeSize < 0)
        {
            ALICEVISION_LOG_ERROR("Range is incorrect");
            return EXIT_FAILURE;
        }

        const int countIterations = divideRoundUp(oldViewsCount, rangeSize);

        if (rangeIteration >= countIterations)
        {
            // nothing to compute for this chunk
            return EXIT_SUCCESS;
        }

        rangeSize = rangeSize * divideRoundUp(viewsCount, oldViewsCount);
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
    std::unique_ptr<PanoramaMap> panoramaMap = buildMap(sfmData, warpingFolder, borderSize, forceMinPyramidLevels);
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

    if (rangeIteration >= chunks.size())
    {
        // nothing to compute for this chunk
        return EXIT_SUCCESS;
    }

    const std::vector<IndexT>& chunk = chunks[rangeIteration];

    bool succeeded = true;

    if (useTiling)
    {
        for (std::size_t posReference = 0; posReference < chunk.size(); posReference++)
        {
            ALICEVISION_LOG_INFO("processing input region " << posReference + 1 << "/" << chunk.size());

            const IndexT viewReference = chunk[posReference];
            if (!sfmData.isPoseAndIntrinsicDefined(viewReference))
                continue;

            BoundingBox referenceBoundingBox;
            if (!panoramaMap->getBoundingBox(referenceBoundingBox, viewReference))
            {
                ALICEVISION_LOG_ERROR("Invalid view ID as reference");
                return EXIT_FAILURE;
            }

            if (!processImage(*panoramaMap,
                              sfmData,
                              compositerType,
                              warpingFolder,
                              labelsFilepath,
                              outputFolder,
                              storageDataType,
                              viewReference,
                              referenceBoundingBox,
                              showBorders,
                              showSeams))
            {
                succeeded = false;
                continue;
            }
        }
    }
    else
    {
        BoundingBox referenceBoundingBox;
        referenceBoundingBox.left = 0;
        referenceBoundingBox.top = 0;
        referenceBoundingBox.width = panoramaMap->getWidth();
        referenceBoundingBox.height = panoramaMap->getHeight();

        if (!processImage(*panoramaMap,
                          sfmData,
                          compositerType,
                          warpingFolder,
                          labelsFilepath,
                          outputFolder,
                          storageDataType,
                          UndefinedIndexT,
                          referenceBoundingBox,
                          showBorders,
                          showSeams))
        {
            succeeded = false;
        }
    }

    if (!succeeded)
    {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
