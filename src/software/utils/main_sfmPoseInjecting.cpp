// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/types.hpp>
#include <aliceVision/config.hpp>

#include <aliceVision/system/Timer.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/cmdline/cmdline.hpp>

#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

#include <boost/program_options.hpp>
#include <boost/json.hpp>

#include <fstream>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;
namespace po = boost::program_options;

struct PoseInput
{
    IndexT frameId;
    Eigen::Matrix4d T;
};

/**
 * I/O for Rotation format choice
 */

enum class ERotationFormat
{
    EulerZXY
};

inline std::string ERotationFormat_enumToString(ERotationFormat format)
{
    switch (format)
    {
        case ERotationFormat::EulerZXY:
        {
            return "EulerZXY";
        }
    }
    throw std::out_of_range("Invalid RotationFormat type Enum: " + std::to_string(int(format)));
}

inline ERotationFormat ERotationFormat_stringToEnum(const std::string& format)
{
    if (format == "EulerZXY")
    {
        return ERotationFormat::EulerZXY;
    }

    throw std::out_of_range("Invalid RotationFormat type Enum: " + format);
}

inline std::ostream& operator<<(std::ostream& os, ERotationFormat s)
{
    return os << ERotationFormat_enumToString(s);
}

inline std::istream& operator>>(std::istream& in, ERotationFormat& s)
{
    std::string token(std::istreambuf_iterator<char>(in), {});
    s = ERotationFormat_stringToEnum(token);
    return in;
}

/**
 * @brief get a pose from a boost json object (assume the file format is ok)
 * @param obj the input json object
 * @param format the required rotation format to transform to rotation matrix
 * @param readPose the output pose information
 * @return false if the process failed
 */
bool getPoseFromJson(const boost::json::object& obj, ERotationFormat format, PoseInput& readPose)
{
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();

    if (format == ERotationFormat::EulerZXY)
    {
        // Reading information from lineup
        const double rx = degreeToRadian(boost::json::value_to<double>(obj.at("rx")));
        const double ry = degreeToRadian(boost::json::value_to<double>(obj.at("ry")));
        const double rz = degreeToRadian(boost::json::value_to<double>(obj.at("rz")));

        Eigen::AngleAxisd Rx(rx, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd Ry(ry, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd Rz(rz, Eigen::Vector3d::UnitZ());

        R = Ry.toRotationMatrix() * Rx.toRotationMatrix() * Rz.toRotationMatrix();
    }
    else
    {
        return false;
    }

    readPose.frameId = boost::json::value_to<IndexT>(obj.at("frame_no"));

    Eigen::Vector3d t;
    t.x() = boost::json::value_to<double>(obj.at("tx"));
    t.y() = boost::json::value_to<double>(obj.at("ty"));
    t.z() = boost::json::value_to<double>(obj.at("tz"));

    Eigen::Matrix4d world_T_camera = Eigen::Matrix4d::Identity();
    world_T_camera.block<3, 3>(0, 0) = R;
    world_T_camera.block<3, 1>(0, 3) = t;

    //Get transform in av coordinates
    Eigen::Matrix4d aliceTinput = Eigen::Matrix4d::Identity();
    aliceTinput(1, 1) = -1;
    aliceTinput(1, 2) = 0;
    aliceTinput(2, 1) = 0;
    aliceTinput(2, 2) = -1;


    world_T_camera = aliceTinput * world_T_camera * aliceTinput.inverse();
    readPose.T = world_T_camera.inverse();

    return true;
}

/**
 * @brief Get a set of poses from a JSON file (assumes the file format is ok).
 * The JSON file contains an array of objects. Each object describes a frameId, a rotation and a translation.
 * @param posesFilename the input JSON filename
 * @param format the required rotation format to transform to rotation matrix
 * @param readPose the output poses vector
 * @return false if the process failed, true otherwise
 */
bool getPosesFromJson(const std::string& posesFilename, ERotationFormat format, std::vector<PoseInput>& readPoses)
{
    std::ifstream inputfile(posesFilename);
    if (!inputfile.is_open())
    {
        return false;
    }

    std::stringstream buffer;
    buffer << inputfile.rdbuf();
    boost::json::value jv = boost::json::parse(buffer.str());

    if (!jv.is_array())
    {
        return false;
    }

    boost::json::array vobj = jv.as_array();

    for (auto item : vobj)
    {
        const boost::json::object& obj = item.as_object();

        PoseInput input;
        if (getPoseFromJson(obj, format, input))
        {
            readPoses.push_back(input);
        }
    }

    return true;
}

int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string sfmDataFilename;
    std::string sfmDataOutputFilename;
    std::string posesFilename;
    ERotationFormat format;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
         "Input SfMData file.")
        ("output,o", po::value<std::string>(&sfmDataOutputFilename)->required(),
         "SfMData output file with the injected poses.")
        ("posesFilename,p", po::value<std::string>(&posesFilename)->required(),
         "JSON file containing the poses to inject.")
        ("rotationFormat,r", po::value<ERotationFormat>(&format)->required(),
         "Rotation format for the input poses: EulerZXY.");
    // clang-format on

    CmdLine cmdline("AliceVision SfM Pose injecting");

    cmdline.add(requiredParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // Set maxThreads
    HardwareContext hwc = cmdline.getHardwareContext();
    omp_set_num_threads(hwc.getMaxThreads());

    // Load input SfMData scene
    sfmData::SfMData sfmData;
    if (!sfmDataIO::load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" + sfmDataFilename + "' cannot be read.");
        return EXIT_FAILURE;
    }

    std::vector<PoseInput> readPoses;
    if (!getPosesFromJson(posesFilename, format, readPoses))
    {
        ALICEVISION_LOG_ERROR("Cannot read the poses");
        return EXIT_FAILURE;
    }

    // Set the pose for all the views with frame IDs found in the JSON file
    for (const auto& [id, pview] : sfmData.getViews())
    {
        for (const auto& rpose : readPoses)
        {
            if (pview->getFrameId() == rpose.frameId)
            {
                geometry::Pose3 pose(rpose.T);
                sfmData::CameraPose cpose(pose, false);
                sfmData.setAbsolutePose(id, cpose);
            }
        }
    }

    sfmDataIO::save(sfmData, sfmDataOutputFilename, sfmDataIO::ESfMData::ALL);

    return EXIT_SUCCESS;
}
