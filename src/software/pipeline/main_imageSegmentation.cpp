// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
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
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/main.hpp>

#include <OpenImageIO/imagebufalgo.h>

// IO
#include <fstream>
#include <algorithm>
#include <memory>
#include <string>


#include <aliceVision/segmentation/segmentation.hpp>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;

void imageToPlanes(std::vector<float> & output, const image::Image<image::RGBfColor>::Base & source)
{
    size_t planeSize = source.rows() * source.cols();
    
    output.resize(planeSize * 3);

    float * planeR = output.data();
    float * planeG = planeR + planeSize;
    float * planeB = planeG + planeSize;

    size_t pos = 0;
    for (int i = 0; i < source.rows(); i++)
    {
        for (int j = 0; j < source.cols(); j++)
        {
            const image::RGBfColor & rgb = source(i, j);
            planeR[pos] = rgb.r();
            planeG[pos] = rgb.g();
            planeB[pos] = rgb.b();

            pos++;
        }
    }
}

void labelsToMask(image::Image<unsigned char> & mask, const image::Image<IndexT> & labels, const std::set<IndexT> & validClasses)
{
    for (int i = 0; i < mask.Height(); i++)
    {
        for (int j = 0; j < mask.Width(); j++)
        {
            IndexT label = labels(i, j);
            mask(i, j) = (validClasses.find(label) != validClasses.end()) ? 255 : 0;
        }
    }
}

int aliceVision_main(int argc, char** argv)
{
    std::string sfmDataFilepath;
    std::string outputPath;
    std::string modelWeightsPath;
    std::vector<std::string> validClasses;
    int rangeStart = -1;
    int rangeSize = 1;
    
    // Description of mandatory parameters
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilepath)->required(), "Input sfmData.")
        ("modelPath,m", po::value<std::string>(&modelWeightsPath)->required(), "Input Model weights file.")
        ("output,o", po::value<std::string>(&outputPath)->required(), "output folder.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("validClasses,c", po::value<std::vector<std::string>>(&validClasses)->multitoken(),
         "Names of classes which are to be considered")
        ("rangeStart", po::value<int>(&rangeStart)->default_value(rangeStart), 
        "Range start for processing views (ordered by image filepath). Set to -1 to process all images.")
        ("rangeSize", po::value<int>(&rangeSize)->default_value(rangeSize), 
        "Range size for processing views (ordered by image filepath).");

    CmdLine cmdline("AliceVision imageSegmentation");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // load input scene
    sfmData::SfMData sfmData;
    if(!sfmDataIO::Load(sfmData, sfmDataFilepath, sfmDataIO::ESfMData(sfmDataIO::VIEWS)))
    {
        ALICEVISION_LOG_ERROR("The input file '" + sfmDataFilepath + "' cannot be read");
        return EXIT_FAILURE;
    }

    // Order views by their image names
    std::vector<std::shared_ptr<sfmData::View>> viewsOrderedByName;
    for(auto& viewIt : sfmData.getViews())
    {
        viewsOrderedByName.push_back(viewIt.second);
    }
    std::sort(viewsOrderedByName.begin(), viewsOrderedByName.end(),
              [](const std::shared_ptr<sfmData::View>& a, const std::shared_ptr<sfmData::View>& b) -> bool
              {
                  if(a == nullptr || b == nullptr)
                      return true;
                  return (a->getImagePath() < b->getImagePath());
              });

    // Define range to compute
    if(rangeStart != -1)
    {
        if(rangeStart < 0 || rangeSize < 0 || static_cast<std::size_t>(rangeStart) > viewsOrderedByName.size())
        {
            ALICEVISION_LOG_ERROR("Range is incorrect");
            return EXIT_FAILURE;
        }

        if(static_cast<std::size_t>(rangeStart + rangeSize) > viewsOrderedByName.size())
        {
            rangeSize = static_cast<int>(viewsOrderedByName.size()) - rangeStart;
        }
    }
    else
    {
        rangeStart = 0;
        rangeSize = static_cast<int>(viewsOrderedByName.size());
    }

    aliceVision::segmentation::Segmentation::Parameters parameters;

    parameters.modelWeights = modelWeightsPath;
    parameters.classes = {
        "__background__",
        "aeroplane",
        "bicycle", "bird", "boat", "bottle", "bus",
        "car", "cat", "chair", "cow",
        "diningtable", "dog",
        "horse",
        "motorbike",
        "person", "pottedplant",
        "sheep", "sofa",
        "train", "tvmonitor"};
    parameters.center= {0.485, 0.456, 0.406};
    parameters.scale= {1.0 / 0.229, 1.0 / 0.224, 1.0 / 0.225};
    parameters.modelWidth = 1280;
    parameters.modelHeight = 720;
    parameters.overlapRatio = 0.3;

    aliceVision::segmentation::Segmentation seg(parameters);

    const auto & classes = seg.getClasses();

    
    //Compute the set of valid classes given parameters
    std::set<IndexT> validClassesIndices;
    for (const auto & s : validClasses)
    {
        std::string classInput = boost::to_lower_copy(s);
        boost::trim(classInput);

        for (int idc = 0; idc < classes.size(); idc++)
        {
            std::string classCompare = boost::to_lower_copy(classes[idc]);
            boost::trim(classCompare);

            if (classCompare.compare(classInput) == 0)
            {
                validClassesIndices.insert(idc);
                break;
            }
        }
    }

    for (int itemidx = 0; itemidx < rangeSize; itemidx++)
    {
        const auto& view = viewsOrderedByName[rangeStart + itemidx];

        std::string path = view->getImagePath();
        ALICEVISION_LOG_INFO("processing " << path);

        image::Image<image::RGBfColor> image;
        image::readImage(path, image, image::EImageColorSpace::NO_CONVERSION);

        double pixelRatio = 1.0;
        view->getDoubleMetadata({"PixelAspectRatio"}, pixelRatio);
        if (pixelRatio != 1.0)
        {
            // Resample input image in order to work with square pixels
            const int w = image.Width();
            const int h = image.Height();

            const int nw = static_cast<int>(static_cast<double>(w) * pixelRatio);
            const int nh = h;

            image::Image<image::RGBfColor> resizedInput(nw, nh);

            const oiio::ImageSpec imageSpecResized(nw, nh, 3, oiio::TypeDesc::FLOAT);
            const oiio::ImageSpec imageSpecOrigin(w, h, 3, oiio::TypeDesc::FLOAT);

            const oiio::ImageBuf inBuf(imageSpecOrigin, image.data());
            oiio::ImageBuf outBuf(imageSpecResized, resizedInput.data());

            oiio::ImageBufAlgo::resize(outBuf, inBuf);
            image.swap(resizedInput);
        }

        image::Image<IndexT> labels;
        if (!seg.processImage(labels, image))
        {
            ALICEVISION_LOG_INFO("Failed to segment image " << path);
        }


        image::Image<unsigned char> mask(labels.Width(), labels.Height());
        labelsToMask(mask, labels, validClassesIndices);

        if (pixelRatio != 1.0)
        {
            // Resample input image in order to work with square pixels
            const int w = mask.Width();
            const int h = mask.Height();

            const int nw = static_cast<int>(static_cast<double>(w) / pixelRatio);
            const int nh = h;

            image::Image<unsigned char> resizedMask(nw, nh);

            const oiio::ImageSpec imageSpecResized(nw, nh, 1, oiio::TypeDesc::UINT8);
            const oiio::ImageSpec imageSpecOrigin(w, h, 1, oiio::TypeDesc::UINT8);

            const oiio::ImageBuf inBuf(imageSpecOrigin, mask.data());
            oiio::ImageBuf outBuf(imageSpecResized, resizedMask.data());

            oiio::ImageBufAlgo::resize(outBuf, inBuf);
            mask.swap(resizedMask);
        }

        //Store image
        std::stringstream ss;
        ss << outputPath << "/" << view->getViewId() << ".exr";
        image::writeImage(ss.str(), mask, image::ImageWriteOptions());
    }

   
    return EXIT_SUCCESS;
}
