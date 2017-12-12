// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/delaunaycut/mv_delaunay_GC.hpp>
#include <aliceVision/largeScale/reconstructionPlan.hpp>
#include <aliceVision/planeSweeping/ps_refine_rc.hpp>
#include <aliceVision/CUDAInterfaces/refine.hpp>
#include <aliceVision/common/fileIO.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <iostream>
#include <bitset>
#include <string>

using namespace std;

namespace bfs = boost::filesystem;
namespace po = boost::program_options;

#define ALICEVISION_COUT(x) std::cout << x << std::endl
#define ALICEVISION_CERR(x) std::cerr << x << std::endl

int main(int argc, char* argv[])
{
    long startTime = clock();

    std::string imageFilepath;
    std::string depthMapFilepath;
    std::string cameraFilepath;
    int scale = 1;
    int step = 1;
    bool transpose = false;
    std::string outputWrlFilepath;

    po::options_description allParams("AliceVision exportDepthMap\n"
                                      "Export depth map into image and mesh");

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("image", po::value<std::string>(&imageFilepath)->required(),
            "image filepath.")
        ("depthMap", po::value<std::string>(&depthMapFilepath)->required(),
            "Depth map filepath.")
        ("cameraFilepath", po::value<std::string>(&cameraFilepath)->required(),
            "Camera filepath")
        ("output,o", po::value<std::string>(&outputWrlFilepath)->required(),
            "Output WRL filepath. It will also generate other wrl files with different scales in the same folder and one png file to visualize the depth map.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("scale", po::value<int>(&scale)->default_value(scale),
            "Depth Map Scale")
        ("step", po::value<int>(&step)->default_value(step),
            "Depth Map Step")
        ("transpose", po::value<bool>(&transpose)->default_value(transpose),
            "Transpose Depth Map buffer.");

    allParams.add(requiredParams).add(optionalParams);

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
      ALICEVISION_CERR("ERROR: " << e.what() << std::endl);
      ALICEVISION_COUT("Usage:\n\n" << allParams);
      return EXIT_FAILURE;
    }
    catch(boost::program_options::error& e)
    {
      ALICEVISION_CERR("ERROR: " << e.what() << std::endl);
      ALICEVISION_COUT("Usage:\n\n" << allParams);
      return EXIT_FAILURE;
    }

    ALICEVISION_COUT("imageFilepath: " << imageFilepath);
    ALICEVISION_COUT("depthMapFilepath: " << depthMapFilepath);
    ALICEVISION_COUT("cameraFilepath: " << cameraFilepath);
    ALICEVISION_COUT("outputWrlFilepath: " << outputWrlFilepath);

    multiviewInputParams mip;
    mip.addImageFile(imageFilepath);
    multiviewParams mp(0, &mip, 0.0f);

    std::string emptyString;
    mp.resizeCams(mip.getNbCameras());
    mp.loadCameraFile(0, cameraFilepath, emptyString);

    mv_fuse fuse(&mp, nullptr);

    int w = mip.getWidth(0);
    int h = mip.getHeight(0);
    std::cerr << "Image resolution: w=" << w << ", h=" << h << std::endl;

    std::unique_ptr<staticVector<float>> depthMap(loadArrayFromFile<float>(depthMapFilepath));

    std::cout << "Nb pixels in depth map: " << depthMap->size() << std::endl;
    std::cout << "Scale: " << scale << std::endl;
    std::cout << "Step: " << step << std::endl;
    if((w*h)/(scale*scale*step*step) != depthMap->size())
    {
        std::cerr << "Scale/step does not match between the image size and the number of pixels in the depth map." << std::endl;
        exit(-1);
    }

    if(transpose)
    {
        staticVector<float> depthMapT;
        depthMapT.swap(*depthMap);
        int sw = w / (scale*step);
        int sh = h / (scale*step);
        for(int y = 0; y < sh; ++y)
        {
            for(int x = 0; x < sw; ++x)
            {
                (*depthMap)[y*sw+x] = depthMapT[x*sh+y];
            }
        }
    }
    fuse.visualizeDepthMap(0, outputWrlFilepath, depthMap.get(), nullptr, std::max(1, scale), step);

    printfElapsedTime(startTime, "#");

    return EXIT_SUCCESS;
}
