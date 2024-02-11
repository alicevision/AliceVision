// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/sfmDataIO/AlembicExporter.hpp>
#include <aliceVision/rig/Rig.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/main.hpp>

#include <boost/program_options.hpp>

#include <iostream>
#include <string>
#include <chrono>
#include <memory>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;

static std::vector<double> ReadIntrinsicsFile(const std::string& fname)
{
    ALICEVISION_LOG_INFO("reading intrinsics: " << fname);

    std::vector<double> v(8);
    std::ifstream ifs(fname);
    if (!(ifs >> v[0] >> v[1] >> v[2] >> v[3] >> v[4] >> v[5] >> v[6] >> v[7]))
        throw std::runtime_error("failed to read intrinsics file");
    return v;
}

int aliceVision_main(int argc, char** argv)
{
    std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
    std::string exportFile;
    std::string importFile;
    std::string rigFile;
    std::string calibFile;
    std::vector<aliceVision::geometry::Pose3> extrinsics;  // the rig subposes

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&importFile)->required(),
         "The input file containing cameras.")
        ("output,o", po::value<std::string>(&exportFile)->required(),
         "Filename for the SfMData export file (where camera poses will be stored).\n"
         "Alembic file only.")
        ("calibrationFile,c", po::value<std::string>(&calibFile)->required(),
         "A calibration file for the target camera.")
        ("rigFile,e", po::value<std::string>(&rigFile)->required(),
         "Rig calibration file that will be applied to input.");
    // clang-format on

    CmdLine cmdline("This program is used to deduce the pose of the not localized cameras of the RIG.\n"
                    "Use if you have localized a single camera from an acquisition with a RIG of cameras.\n"
                    "AliceVision rigTransform");
    cmdline.add(requiredParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // load rig calibration file
    if (!rig::loadRigCalibration(rigFile, extrinsics))
    {
        ALICEVISION_LOG_ERROR("Unable to open " << rigFile);
        return EXIT_FAILURE;
    }
    assert(!extrinsics.empty());

    // import sfm data
    sfmData::SfMData sfmData;
    if (!sfmDataIO::load(sfmData, importFile, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << importFile << "' cannot be read");
        return EXIT_FAILURE;
    }

    // load intrinsics
    auto v = ReadIntrinsicsFile(calibFile);
    camera::Pinhole intrinsics = camera::Pinhole(v[0], v[1], v[2], v[2], v[3], v[4], std::make_shared<camera::DistortionRadialK3>(v[5], v[6], v[7]));

    // export to abc
    sfmDataIO::AlembicExporter exporter(exportFile);
    exporter.initAnimatedCamera("camera");

    std::size_t idx = 0;
    for (auto& p : sfmData.getPoses())
    {
        const geometry::Pose3 rigPose = extrinsics[0].inverse() * p.second.getTransform();
        exporter.addCameraKeyframe(rigPose, &intrinsics, "", idx, idx);
        ++idx;
    }
    exporter.addLandmarks(sfmData.getLandmarks());

    return EXIT_SUCCESS;
}
