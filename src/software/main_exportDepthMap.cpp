// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/delaunaycut/mv_delaunay_GC.hpp>
#include <aliceVision/largeScale/reconstructionPlan.hpp>
#include <aliceVision/planeSweeping/ps_refine_rc.hpp>
#include <aliceVision/CUDAInterfaces/refine.hpp>
#include <aliceVision/structures/mv_filesio.hpp>

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
    std::string outputWrlFilepath;

    po::options_description inputParams("Export depth map into image and mesh.");

    inputParams.add_options()
        ("image", po::value<std::string>(&imageFilepath)->required(),
            "image filepath.")
        ("depthMap", po::value<std::string>(&depthMapFilepath)->required(),
            "Depth map filepath.")
        ("cameraFilepath", po::value<std::string>(&cameraFilepath)->required(),
            "camera filepath")
        ("output", po::value<std::string>(&outputWrlFilepath)->required(),
            "Output WRL filepath. It will also generate other wrl files with different scales in the same folder and one png file to visualize the depth map.");
    po::variables_map vm;

    try
    {
      po::store(po::parse_command_line(argc, argv, inputParams), vm);

      if(vm.count("help") || (argc == 1))
      {
        ALICEVISION_COUT(inputParams);
        return EXIT_SUCCESS;
      }

      po::notify(vm);
    }
    catch(boost::program_options::required_option& e)
    {
      ALICEVISION_CERR("ERROR: " << e.what() << std::endl);
      ALICEVISION_COUT("Usage:\n\n" << inputParams);
      return EXIT_FAILURE;
    }
    catch(boost::program_options::error& e)
    {
      ALICEVISION_CERR("ERROR: " << e.what() << std::endl);
      ALICEVISION_COUT("Usage:\n\n" << inputParams);
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

    std::unique_ptr<staticVector<float>> depthMap(loadArrayFromFile<float>(depthMapFilepath));
    int scale = 1;
    int step = 1;
    int scales = 1;
    fuse.visualizeDepthMap(0, outputWrlFilepath, depthMap.get(), nullptr, std::max(1, scale), step, scales);

    printfElapsedTime(startTime, "#");

    return EXIT_SUCCESS;
}
