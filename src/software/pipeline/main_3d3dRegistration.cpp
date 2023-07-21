// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/registration/PointcloudRegistration.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/cmdline/cmdline.hpp>

using namespace aliceVision;
using namespace aliceVision::registration;

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

namespace po = boost::program_options;

int main(int argc, char** argv)
{
    std::string alignmentMethodName = EAlignmentMethod_enumToString(EAlignmentMethod::GICP);


    std::string sourceFile, targetFile;
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("sourceFile,s", po::value<std::string>(&sourceFile)->required(),
        "Path to file source (moving) 3D model.")
        ("targetFile,t", po::value<std::string>(&targetFile)->required(),
        "Path to the target (fixed) 3D model.");

    // to choose which transformed 3D model to export:
    std::string outputFile;

    float sourceMeasurement = 1.f;
    float targetMeasurement = 1.f;
    float scaleRatio = 1.f;
    float voxelSize = 0.1f;
    bool showTimeline = true;

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("outputFile,o", po::value<std::string>(&outputFile),
        "Path to save the transformed source 3D model.")
        ("method,m", po::value<std::string>(&alignmentMethodName)->default_value(alignmentMethodName),
            EAlignmentMethod_information().c_str())
        ("scaleRatio", po::value<float>(&scaleRatio)->default_value(scaleRatio),
        "Scale ratio between the two 3D models (= target size / source size)")
        ("sourceMeasurement", po::value<float>(&sourceMeasurement)->default_value(sourceMeasurement),
        "Measurement made ont the source 3D model (same unit as 'targetMeasurement'). It allows to compute the scale ratio between 3D models.")
        ("targetMeasurement", po::value<float>(&targetMeasurement)->default_value(targetMeasurement),
        "Measurement made ont the target 3D model (same unit as 'sourceMeasurement'). It allows to compute the scale ratio between 3D models.")
        ("voxelSize", po::value<float>(&voxelSize)->default_value(voxelSize),
        "Size of the voxel grid applied on each 3D model to downsample them. Downsampling reduces computing duration.")
        ("showTimeline", po::value<bool>(&showTimeline)->default_value(showTimeline),
        "To show the duration of each time of the alignment pipeline.");

    aliceVision::CmdLine cmdline("3D 3D registration.\n"
                                "Perform registration of 3D models (e.g. SfM & LiDAR model).\n"
                                "AliceVision 3d3dRegistration");

    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if(!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // ===========================================================
    // -- Run alignment
    // ===========================================================
    EAlignmentMethod method = EAlignmentMethod_stringToEnum(alignmentMethodName);
    ALICEVISION_COUT("Alignment Method: " << EAlignmentMethod_enumToString(method));

    ALICEVISION_COUT("Create PointcloudRegistration");
	PointcloudRegistration reg;

    if (reg.loadSourceCloud(sourceFile) == EXIT_FAILURE)
    {
        ALICEVISION_CERR("Failed to load source cloud: " << sourceFile);
        return EXIT_FAILURE;
    }

    if (reg.loadTargetCloud(targetFile) == EXIT_FAILURE)
    {
        ALICEVISION_CERR("Failed to load target cloud: " << targetFile);
        return EXIT_FAILURE;
    }

    ALICEVISION_LOG_INFO("Point clouds loaded.");

    reg.setScaleRatio(scaleRatio);
    reg.setTargetMeasurement(targetMeasurement);
    reg.setSourceMeasurement(sourceMeasurement);
    reg.setVoxelSize(voxelSize);

    ALICEVISION_LOG_INFO("Start alignment");

    const Eigen::Matrix4d T = reg.align(method);

    ALICEVISION_LOG_INFO("Alignment transform estimated:\n" << T);
    Eigen::Affine3d transformation(T);
    ALICEVISION_LOG_INFO("Alignment transform rotation:\n" << transformation.rotation());
    ALICEVISION_LOG_INFO("Alignment transform translation:\n" << transformation.translation());

    if (T.hasNaN())
    {
        ALICEVISION_CERR("3D3DRegistration failed. Final matrix contains NaN.");
        return EXIT_FAILURE;
    }
    if (showTimeline)
        reg.showTimeline();
    
	if (outputFile.empty())
    {
        ALICEVISION_CERR("Output file empty, nothing to export.");
        return EXIT_SUCCESS;
    }

    // export a transformed 3D model
    return reg.tranformAndSaveCloud(sourceFile, T, outputFile);
}
