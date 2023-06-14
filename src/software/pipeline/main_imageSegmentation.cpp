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

// IO
#include <fstream>
#include <algorithm>
#include <boost/algorithm/string.hpp>    


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



int aliceVision_main(int argc, char** argv)
{
    std::string sfmDataFilepath;
    std::string outputPath;
    std::vector<std::string> validClasses = {"person"};
    
    // Description of mandatory parameters
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilepath)->required(), "Input sfmData.")
        ("output,o", po::value<std::string>(&outputPath)->required(), "output folder.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("validClasses,c", po::value<std::vector<std::string>>(&validClasses)->multitoken(),
         "Names of classes which are to be considered");

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

    aliceVision::segmentation::Segmentation seg;
    const auto & classes = seg.getClasses();

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

    for (const auto & pv : sfmData.getViews())
    {
        std::string path = pv.second->getImagePath();

        image::Image<image::RGBfColor> image;
        image::readImage(path, image, image::EImageColorSpace::NO_CONVERSION);

        image::Image<IndexT> labels;
        if (!seg.processImage(labels, image))
        {
            ALICEVISION_LOG_INFO("Failed to segment image " << path);
        }

        //Store image
        std::stringstream ss;
        ss << outputPath << "/" << pv.first << ".exr";
        image::writeImage(ss.str(), labels, image::ImageWriteOptions());
    }

   
    return EXIT_SUCCESS;
}
