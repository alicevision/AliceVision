// This file is part of the AliceVision project.
// Copyright (c) 2020 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/config.hpp>
#include <aliceVision/numeric/numeric.hpp>

#include <aliceVision/mesh/MeshEnergyOpt.hpp>
#include <aliceVision/mesh/Texturing.hpp>
#include <aliceVision/mvsUtils/common.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace bfs = boost::filesystem;
namespace po = boost::program_options;

/**
 * @brief Alignment method enum
 */
enum class EAlignmentMethod : unsigned char
{
    MANUAL = 0
};

/**
 * @brief Convert an EAlignmentMethod enum to its corresponding string
 * @param[in] alignmentMethod The given EAlignmentMethod enum
 * @return string
 */
std::string EAlignmentMethod_enumToString(EAlignmentMethod alignmentMethod)
{
    switch(alignmentMethod)
    {
        case EAlignmentMethod::MANUAL:
            return "manual";
    }
    throw std::out_of_range("Invalid EAlignmentMethod enum");
}

/**
 * @brief Convert a string to its corresponding EAlignmentMethod enum
 * @param[in] alignmentMethod The given string
 * @return EAlignmentMethod enum
 */
EAlignmentMethod EAlignmentMethod_stringToEnum(const std::string& alignmentMethod)
{
    std::string method = alignmentMethod;
    std::transform(method.begin(), method.end(), method.begin(), ::tolower); // tolower

    if(method == "manual")
        return EAlignmentMethod::MANUAL;
    throw std::out_of_range("Invalid SfM alignment method : " + alignmentMethod);
}

inline std::istream& operator>>(std::istream& in, EAlignmentMethod& alignment)
{
    std::string token;
    in >> token;
    alignment = EAlignmentMethod_stringToEnum(token);
    return in;
}

inline std::ostream& operator<<(std::ostream& os, EAlignmentMethod e)
{
    return os << EAlignmentMethod_enumToString(e);
}

static void parseManualTransform(const std::string& manualTransform, double& S, Mat3& R, Vec3& t)
{
    // Parse the string
    std::vector<std::string> dataStr;
    boost::split(dataStr, manualTransform, boost::is_any_of(","));
    if(dataStr.size() != 7)
    {
        throw std::runtime_error(
            "Invalid number of values for manual transformation with ZXY Euler: tx,ty,tz,rx,ry,rz,s.");
    }

    std::vector<double> data;
    data.reserve(7);
    for(const std::string& elt : dataStr)
    {
        data.push_back(boost::lexical_cast<double>(elt));
    }

    // Assignments
    t << data[0], data[1], data[2]; // Assign Translation
    S = data[6];                    // Assign Scale

    Vec3 eulerAngles(data[3], data[4], data[5]); // Temporary eulerAngles vector

    // Compute the rotation matrix from quaternion made with Euler angles in that order: ZXY (same as Qt algorithm)
    Mat3 rotateMat = Mat3::Identity();
    {
        double pitch = eulerAngles.x() * M_PI / 180;
        double yaw = eulerAngles.y() * M_PI / 180;
        double roll = eulerAngles.z() * M_PI / 180;

        pitch *= 0.5;
        yaw *= 0.5;
        roll *= 0.5;

        const double cy = std::cos(yaw);
        const double sy = std::sin(yaw);
        const double cr = std::cos(roll);
        const double sr = std::sin(roll);
        const double cp = std::cos(pitch);
        const double sp = std::sin(pitch);
        const double cycr = cy * cr;
        const double sysr = sy * sr;

        const double w = cycr * cp + sysr * sp;
        const double x = cycr * sp + sysr * cp;
        const double y = sy * cr * cp - cy * sr * sp;
        const double z = cy * sr * cp - sy * cr * sp;

        Eigen::Quaterniond quaternion(w, x, y, z);
        rotateMat = quaternion.matrix();
    }
    R = rotateMat; // Assign Rotation
}

int aliceVision_main(int argc, char** argv)
{
    system::Timer timer;

    // command-line parameters
    std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
    std::string inputMeshPath;
    std::string outputMeshPath;
    EAlignmentMethod alignmentMethod = EAlignmentMethod::MANUAL;

    // user optional parameters
    std::string transform;
    std::string landmarksDescriberTypesName;
    double userScale = 1;
    bool applyScale = true;
    bool applyRotation = true;
    bool applyTranslation = true;

    std::string manualTransform;

    po::options_description allParams("AliceVision meshTransform");

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&inputMeshPath)->required(), "SfMData file to align.")
        ("output,o", po::value<std::string>(&outputMeshPath)->required(), "Output SfMData scene.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("method", po::value<EAlignmentMethod>(&alignmentMethod)->default_value(alignmentMethod),
         "Transform Method:\n"
         "\t- manual: Apply the gizmo transformation\n")
        ("transformation", po::value<std::string>(&transform)->default_value(transform),
         "required only for 'transformation' and 'single camera' methods:\n"
         "Transformation: Align [X,Y,Z] to +Y-axis, rotate around Y by R deg, scale by S; syntax: X,Y,Z;R;S\n"
         "Single camera: camera UID or image filename")
        ("manualTransform", po::value<std::string>(&manualTransform),
         "Translation, rotation and scale defined with the manual mode.")
        ("scale", po::value<double>(&userScale)->default_value(userScale), "Additional scale to apply.")
        ("applyScale", po::value<bool>(&applyScale)->default_value(applyScale), "Apply scale transformation.")
        ("applyRotation", po::value<bool>(&applyRotation)->default_value(applyRotation), "Apply rotation transformation.")
        ("applyTranslation", po::value<bool>(&applyTranslation)->default_value(applyTranslation), "Apply translation transformation.");

    po::options_description logParams("Log parameters");
    logParams.add_options()
        ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
         "verbosity level (fatal,  error, warning, info, debug, trace).");

    allParams.add(requiredParams).add(optionalParams).add(logParams);

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
        ALICEVISION_CERR("ERROR: " << e.what());
        ALICEVISION_COUT("Usage:\n\n" << allParams);
        return EXIT_FAILURE;
    }
    catch(boost::program_options::error& e)
    {
        ALICEVISION_CERR("ERROR: " << e.what());
        ALICEVISION_COUT("Usage:\n\n" << allParams);
        return EXIT_FAILURE;
    }

    ALICEVISION_COUT("Program called with the following parameters:");
    ALICEVISION_COUT(vm);

    // set verbose level
    system::Logger::get()->setLogLevel(verboseLevel);

    double S = 1.0;
    Mat3 R = Mat3::Identity();
    Vec3 t = Vec3::Zero();

    switch(alignmentMethod)
    {
        case EAlignmentMethod::MANUAL:
        {
            if(manualTransform.empty())
                ALICEVISION_LOG_WARNING("No manualTransform option set, so the transform will be identity.");
            else
                parseManualTransform(manualTransform, S, R, t);

            break;
        }
    }

    // apply user scale
    S *= userScale;
    t *= userScale;

    if(!applyScale)
        S = 1;
    if(!applyRotation)
        R = Mat3::Identity();
    if(!applyTranslation)
        t = Vec3::Zero();

    {
        ALICEVISION_LOG_INFO("Transformation:\n"
            << "\t- Scale: " << S << "\n"
            << "\t- Rotation:\n" << R << "\n"
            << "\t- Translate: " << t.transpose());
    }

    mesh::Texturing texturing;
    texturing.loadOBJWithAtlas(inputMeshPath);
    mesh::Mesh* mesh = texturing.mesh;

    if(!mesh)
    {
        ALICEVISION_LOG_ERROR("Unable to read input mesh from the file: " << inputMeshPath);
        return EXIT_FAILURE;
    }

    if(mesh->pts.empty() || mesh->tris.empty())
    {
        ALICEVISION_LOG_ERROR("Error: empty mesh from the file " << inputMeshPath);
        ALICEVISION_LOG_ERROR("Input mesh: " << mesh->pts.size() << " vertices and " << mesh->tris.size()
                                             << " facets.");
        return EXIT_FAILURE;
    }

    ALICEVISION_LOG_INFO("Mesh file: \"" << inputMeshPath << "\" loaded.");
    ALICEVISION_LOG_INFO("Input mesh: " << mesh->pts.size() << " vertices and " << mesh->tris.size() << " facets.");

    // sfm::applyTransform(sfmData, S, R, t);

    // Save output mesh
    mesh->saveToObj(outputMeshPath);

    ALICEVISION_LOG_INFO("Mesh file: \"" << outputMeshPath << "\" saved.");

    ALICEVISION_LOG_INFO("Task done in (s): " + std::to_string(timer.elapsed()));

    return EXIT_SUCCESS;
}
