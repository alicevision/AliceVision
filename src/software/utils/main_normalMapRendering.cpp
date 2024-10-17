// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/image/Image.hpp>
#include <aliceVision/mesh/MeshIntersection.hpp>

#include <filesystem>



// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;

int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string sfmDataFilename;
    std::string meshFilename;
    std::string outputDirectory;
    
    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
         "SfMData file.")
        ("mesh,i", po::value<std::string>(&meshFilename)->required(),
         "mesh file.")
        ("output,o", po::value<std::string>(&outputDirectory)->required(),
         "Output directory for depthmaps.");
    // clang-format on

    CmdLine cmdline("AliceVision normalmapRendering");
    cmdline.add(requiredParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

     // set maxThreads
    HardwareContext hwc = cmdline.getHardwareContext();
    omp_set_num_threads(hwc.getMaxThreads());

    std::filesystem::path pathOutputDirectory(outputDirectory);

    // Load input scene
    sfmData::SfMData sfmData;
    if (!sfmDataIO::load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' cannot be read");
        return EXIT_FAILURE;
    }

    //Load mesh in the mesh intersection object
    ALICEVISION_LOG_INFO("Loading mesh");
    mesh::MeshIntersection mi;
    if (!mi.initialize(meshFilename))
    {
        return EXIT_FAILURE;
    }

    for (const auto & [index, view] : sfmData.getViews())
    {
        if (!sfmData.isPoseAndIntrinsicDefined(index))
        {
            continue;
        }
        

        ALICEVISION_LOG_INFO("Generating depthmap for view " << index);

        //Retrieve metadatas for copying in the depthmap
        oiio::ParamValueList metadata = image::readImageMetadata(view->getImageInfo()->getImagePath());

        const auto & intrinsic = sfmData.getIntrinsicSharedPtr(*view);
        const auto pose = sfmData.getPose(*view);

        Vec3 center = pose.getTransform().center();

        mi.setPose(pose.getTransform());

        int w = view->getImageInfo()->getWidth();
        int h = view->getImageInfo()->getHeight();
        image::Image<image::RGBfColor> image(w, h, image::RGBfColor(0.0f,0.0f,0.0f));

        #pragma omp parallel for
        for (int i = 0; i < h; i++)
        {
            for (int j = 0; j < w; j++)
            {
                Vec2 pt;
                pt.x() = j;
                pt.y() = i;

                //Find the 3d point 
                //Which is the intersection of the ray and the mesh
                //And get its normal
                Vec3 normal;
                if (!mi.peekNormal(normal, *intrinsic, pt))
                {
                    continue;
                }

                Vec3 cnormal = pose.getTransform().rotation() * normal;

                auto & rgb = image(i, j);
                rgb.r() = cnormal.x();
                rgb.g() = cnormal.y();
                rgb.b() = cnormal.z();
            }
        }
        
        auto path = (pathOutputDirectory / (std::to_string(index) + "_normalMap.exr"));
        ALICEVISION_LOG_INFO("Ouput to " << path);
        image::writeImage(path.string(), image, image::ImageWriteOptions(), metadata);
    }

    return EXIT_SUCCESS;
}
