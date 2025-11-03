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
#include <filesystem>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;
namespace po = boost::program_options;

struct PoseInput
{
    IndexT frameId;
    std::filesystem::path path;
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
 * I/O for Rotation format choice
 */

enum class EGeometricFrameFormat
{
    RHXrYbZf,
    RHXrYtZb
};

inline std::string EGeometricFrameFormat_enumToString(EGeometricFrameFormat format)
{
    switch (format)
    {
        case EGeometricFrameFormat::RHXrYbZf:
        {
            return "RHXrYbZf";
        }
        case EGeometricFrameFormat::RHXrYtZb:
        {
            return "RHXrYtZb";
        }
    }
    throw std::out_of_range("Invalid Geometric format type Enum: " + std::to_string(int(format)));
}

inline EGeometricFrameFormat EGeometricFrameFormat_stringToEnum(const std::string& format)
{
    if (format == "RHXrYbZf")
    {
        return EGeometricFrameFormat::RHXrYbZf;
    }

    if (format == "RHXrYtZb")
    {
        return EGeometricFrameFormat::RHXrYtZb;
    }

    throw std::out_of_range("Invalid Geometric frame format type Enum: " + format);
}

inline std::ostream& operator<<(std::ostream& os, EGeometricFrameFormat s)
{
    return os << EGeometricFrameFormat_enumToString(s);
}

inline std::istream& operator>>(std::istream& in, EGeometricFrameFormat& s)
{
    std::string token(std::istreambuf_iterator<char>(in), {});
    s = EGeometricFrameFormat_stringToEnum(token);
    return in;
}



/**
 * @brief get a pose from a boost json object (assume the file format is ok)
 * @param obj the input json object
 * @param rotationFormat the required rotation format to transform to rotation matrix
 * @param geometricFrameFormat the geometric frame format used to interpret the camera pose
 * @param readPose the output pose information
 * @return false if the process failed
 */
bool getPoseFromJson(const boost::json::object& obj, ERotationFormat rotationFormat, EGeometricFrameFormat & geometricFormat, PoseInput& readPose)
{
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();

    if (rotationFormat == ERotationFormat::EulerZXY)
    {
        try
        {
            const double rx = degreeToRadian(boost::json::value_to<double>(obj.at("rx")));
            const double ry = degreeToRadian(boost::json::value_to<double>(obj.at("ry")));
            const double rz = degreeToRadian(boost::json::value_to<double>(obj.at("rz")));

            Eigen::AngleAxisd Rx(rx, Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd Ry(ry, Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd Rz(rz, Eigen::Vector3d::UnitZ());

            R = Ry.toRotationMatrix() * Rx.toRotationMatrix() * Rz.toRotationMatrix();
        }
        catch (boost::system::system_error error)
        {
            ALICEVISION_LOG_ERROR("Incorrect rotation parameters");
            return false;
        }
    }
    else
    {
        ALICEVISION_LOG_ERROR("Unsupported rotation parameters");
        return false;
    }

    Eigen::Vector3d t;
    try
    {
        t.x() = boost::json::value_to<double>(obj.at("tx"));
        t.y() = boost::json::value_to<double>(obj.at("ty"));
        t.z() = boost::json::value_to<double>(obj.at("tz"));
    }
    catch (boost::system::system_error error)
    {
        //If no tx,ty,tz we assume 0 translation
    }

    readPose.frameId = UndefinedIndexT;
    if (obj.if_contains("frame_no"))
    {
        readPose.frameId = boost::json::value_to<IndexT>(obj.at("frame_no"));
    }
    else if (obj.if_contains("path"))
    {
        readPose.path = boost::json::value_to<std::string>(obj.at("filename"));
    }

    Eigen::Matrix4d world_T_camera = Eigen::Matrix4d::Identity();
    world_T_camera.block<3, 3>(0, 0) = R;
    world_T_camera.block<3, 1>(0, 3) = t;

    //Get transform in av coordinates
    Eigen::Matrix4d aliceTinput = Eigen::Matrix4d::Identity();

    switch (geometricFormat)
    {
        case EGeometricFrameFormat::RHXrYtZb:
        {
            aliceTinput(1, 1) = -1;
            aliceTinput(1, 2) = 0;
            aliceTinput(2, 1) = 0;
            aliceTinput(2, 2) = -1;
            break;
        }
        default:
            break;
    }


    world_T_camera = aliceTinput * world_T_camera * aliceTinput.inverse();
    readPose.T = world_T_camera.inverse();

    return true;
}

/**
 * @brief Get a set of poses from a JSON file (assumes the file format is ok).
 * The JSON file contains an array of objects. Each object describes a frameId, a rotation and a translation.
 * @param posesFilename the input JSON filename
 * @param format the required rotation format to transform to rotation matrix
 * @param geometricFrameFormat the geometric frame format used to interpret the camera pose
 * @param readPose the output poses vector
 * @return false if the process failed, true otherwise
 */
bool getPosesFromJson(const std::string& posesFilename, ERotationFormat format, EGeometricFrameFormat & geometricFormat, std::vector<PoseInput>& readPoses)
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
        if (getPoseFromJson(obj, format, geometricFormat, input))
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
    EGeometricFrameFormat gframe;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
         "Input SfMData file.")
        ("output,o", po::value<std::string>(&sfmDataOutputFilename)->required(),
         "SfMData output file with the injected poses.")
        ("rotationFormat,r", po::value<ERotationFormat>(&format)->required(),
         "Rotation format for the input poses: EulerZXY.")
        ("geometricFrame, g", po::value<EGeometricFrameFormat>(&gframe)->required(),
        "Defines the geometric frame for the input poses");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("posesFilename,p", po::value<std::string>(&posesFilename)->default_value(posesFilename),
        "JSON file containing the poses to inject.");
    // clang-format on

    CmdLine cmdline("AliceVision SfM Pose injecting");

    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }
    

    // Set maxThreads
    HardwareContext hwc = cmdline.getHardwareContext();
    omp_set_num_threads(hwc.getMaxThreads());

    if (posesFilename.empty())
    {
        ALICEVISION_LOG_INFO("Nothing to do.");
        return EXIT_SUCCESS;
    }

    // Load input SfMData scene
    sfmData::SfMData sfmData;
    if (!sfmDataIO::load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" + sfmDataFilename + "' cannot be read.");
        return EXIT_FAILURE;
    }

    std::vector<PoseInput> readPoses;
    if (!getPosesFromJson(posesFilename, format, gframe, readPoses))
    {
        ALICEVISION_LOG_ERROR("Cannot read the poses");
        return EXIT_FAILURE;
    }

    // Set the pose for all the views with frame IDs found in the JSON file
    for (const auto& [id, pview] : sfmData.getViews())
    {
        for (const auto& rpose : readPoses)
        {
            //Check if the frameid is the same than described
            bool found = false;
            if (rpose.frameId != UndefinedIndexT)
            {
                if (pview->getFrameId() == rpose.frameId)
                {
                    found = true;
                }
            }

            if (!found && !rpose.path.empty())
            {
                //Check if this view has the same filename
                std::filesystem::path viewPath(pview->getImage().getImagePath());
                if (rpose.path.has_relative_path())
                {
                    if (std::filesystem::canonical(viewPath) == std::filesystem::canonical(rpose.path))
                    {
                        found = true;
                    }
                }
                else 
                {
                    if (viewPath.filename() == rpose.path.filename())
                    {
                        found = true;
                    }
                }
            }

            if (!found)
            {
                continue;
            }

            ALICEVISION_LOG_INFO("Pose assigned to image with path " << pview->getImage().getImagePath());

            geometry::Pose3 pose(rpose.T);
            sfmData::CameraPose cpose(pose, false);
            cpose.setRemovable(false);
            sfmData.setAbsolutePose(id, cpose);
        }
    }

    sfmDataIO::save(sfmData, sfmDataOutputFilename, sfmDataIO::ESfMData::ALL);

    return EXIT_SUCCESS;
}
