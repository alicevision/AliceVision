// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/camera/camera.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <string>
#include <sstream>
#include <fstream>
#include <memory>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

namespace po = boost::program_options;
namespace fs = boost::filesystem;

using namespace aliceVision;
using namespace aliceVision::camera;

std::string toNuke(std::shared_ptr<Undistortion> undistortion, EINTRINSIC intrinsicType)
{
    const std::vector<double>& params = undistortion->getParameters();

    std::stringstream ss;

    switch (intrinsicType)
    {
    case EINTRINSIC::PINHOLE_CAMERA_3DEANAMORPHIC4:
        ss << "LensDistortion2 {" << "\n"
           << " distortionModelPreset \"3DEqualizer/3DE4 Anamorphic - Standard, Degree 4\"" << "\n"
           << " lens Anamorphic" << "\n"
           << " distortionNumeratorX00 " << params[0] << "\n"
           << " distortionNumeratorX01 " << params[4] << "\n"
           << " distortionNumeratorX10 " << params[2] << "\n"
           << " distortionNumeratorX11 " << params[6] << "\n"
           << " distortionNumeratorX20 " << params[8] << "\n"
           << " distortionNumeratorY00 " << params[1] << "\n"
           << " distortionNumeratorY01 " << params[5] << "\n"
           << " distortionNumeratorY10 " << params[3] << "\n"
           << " distortionNumeratorY11 " << params[7] << "\n"
           << " distortionNumeratorY20 " << params[9] << "\n"
           << " output Undistort" << "\n"
           << " distortionModelType \"Radial Asymmetric\"" << "\n"
           << " distortionOrder {2 0}" << "\n"
           << " normalisationType Diagonal" << "\n"
           << " distortInFisheyeSpace false" << "\n"
           << "}" << "\n";
        break;
    default:
        ALICEVISION_THROW_ERROR("Unsupported intrinsic type for conversion to Nuke LensDistortion node: "
                                << EINTRINSIC_enumToString(intrinsicType));
    }

    return ss.str();
}

int aliceVision_main(int argc, char* argv[])
{
    std::string sfmInputDataFilepath;
    std::string outputFilePath;

    // Command line parameters
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmInputDataFilepath)->required(), 
        "SfMData file input.")
        ("output,o", po::value<std::string>(&outputFilePath)->required(), 
        "Output directory.");
    
    CmdLine cmdline("AliceVision export distortion");
    cmdline.add(requiredParams);

    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    sfmData::SfMData sfmData;
    if(!sfmDataIO::Load(sfmData, sfmInputDataFilepath, sfmDataIO::ESfMData(sfmDataIO::INTRINSICS)))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmInputDataFilepath << "' cannot be read.");
        return EXIT_FAILURE;
    }

    for (const auto& [intrinsicId, intrinsicPtr] : sfmData.getIntrinsics())
    {
        auto intrinsicDisto = std::dynamic_pointer_cast<IntrinsicsScaleOffsetDisto>(intrinsicPtr);
        
        if (!intrinsicDisto) continue;

        auto undistortion = intrinsicDisto->getUndistortion();

        if (!undistortion) continue;

        std::string nukeNodeStr = toNuke(undistortion, intrinsicDisto->getType());

        ALICEVISION_LOG_INFO("Writing Nuke LensDistortion node in " << intrinsicId << ".nk");
        std::stringstream ss;
        ss << outputFilePath << "/" << intrinsicId << ".nk";
        std::ofstream of(ss.str());
        of << nukeNodeStr;
        of.close();
    }

    return EXIT_SUCCESS;
}
