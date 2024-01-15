// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/sfm/utils/alignment.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/config.hpp>

#include <boost/program_options.hpp>

#include <iomanip>
#include <string>
#include <sstream>
#include <vector>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 1

using namespace aliceVision;

namespace po = boost::program_options;

namespace {

/**
 * @brief Alignment method enum
 */
enum class EAlignmentMethod : unsigned char
{
    TRANSFORMATION = 0,
    MANUAL,
    AUTO,
    AUTO_FROM_CAMERAS,
    AUTO_FROM_CAMERAS_X_AXIS,
    AUTO_FROM_LANDMARKS,
    FROM_SINGLE_CAMERA,
    FROM_CENTER_CAMERA,
    FROM_MARKERS,
    FROM_GPS,
    ALIGN_GROUND
};

/**
 * @brief Convert an EAlignmentMethod enum to its corresponding string
 * @param[in] alignmentMethod The given EAlignmentMethod enum
 * @return string
 */
std::string EAlignmentMethod_enumToString(EAlignmentMethod alignmentMethod)
{
    switch (alignmentMethod)
    {
        case EAlignmentMethod::TRANSFORMATION:
            return "transformation";
        case EAlignmentMethod::MANUAL:
            return "manual";
        case EAlignmentMethod::AUTO:
            return "auto";
        case EAlignmentMethod::AUTO_FROM_CAMERAS:
            return "auto_from_cameras";
        case EAlignmentMethod::AUTO_FROM_CAMERAS_X_AXIS:
            return "auto_from_cameras_x_axis";
        case EAlignmentMethod::AUTO_FROM_LANDMARKS:
            return "auto_from_landmarks";
        case EAlignmentMethod::FROM_SINGLE_CAMERA:
            return "from_single_camera";
        case EAlignmentMethod::FROM_CENTER_CAMERA:
            return "from_center_camera";
        case EAlignmentMethod::FROM_MARKERS:
            return "from_markers";
        case EAlignmentMethod::FROM_GPS:
            return "from_gps";
        case EAlignmentMethod::ALIGN_GROUND:
            return "align_ground";
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
    std::transform(method.begin(), method.end(), method.begin(), ::tolower);  // tolower

    if (method == "transformation")
        return EAlignmentMethod::TRANSFORMATION;
    if (method == "manual")
        return EAlignmentMethod::MANUAL;
    if (method == "auto")
        return EAlignmentMethod::AUTO;
    if (method == "auto_from_cameras")
        return EAlignmentMethod::AUTO_FROM_CAMERAS;
    if (method == "auto_from_cameras_x_axis")
        return EAlignmentMethod::AUTO_FROM_CAMERAS_X_AXIS;
    if (method == "auto_from_landmarks")
        return EAlignmentMethod::AUTO_FROM_LANDMARKS;
    if (method == "from_single_camera")
        return EAlignmentMethod::FROM_SINGLE_CAMERA;
    if (method == "from_center_camera")
        return EAlignmentMethod::FROM_CENTER_CAMERA;
    if (method == "from_markers")
        return EAlignmentMethod::FROM_MARKERS;
    if (method == "from_gps")
        return EAlignmentMethod::FROM_GPS;
    if (method == "align_ground")
        return EAlignmentMethod::ALIGN_GROUND;
    throw std::out_of_range("Invalid SfM alignment method : " + alignmentMethod);
}

inline std::istream& operator>>(std::istream& in, EAlignmentMethod& alignment)
{
    std::string token;
    in >> token;
    alignment = EAlignmentMethod_stringToEnum(token);
    return in;
}

inline std::ostream& operator<<(std::ostream& os, EAlignmentMethod e) { return os << EAlignmentMethod_enumToString(e); }

static bool parseAlignScale(const std::string& alignScale, double& S, Mat3& R, Vec3& t)
{
    double rx, ry, rz, rr;

    {
        char delim[4];
        std::istringstream iss(alignScale);
        if (!(iss >> rx >> delim[0] >> ry >> delim[1] >> rz >> delim[2] >> rr >> delim[3] >> S))
            return false;
        if (delim[0] != ',' || delim[1] != ',' || delim[2] != ';' || delim[3] != ';')
            return false;
    }

    auto q = Eigen::Quaterniond::FromTwoVectors(Vec3(rx, ry, rz), Vec3::UnitY());
    auto r = Eigen::AngleAxisd(rr * M_PI / 180, Vec3::UnitY());

    R = r * q.toRotationMatrix();
    t = Vec3::Zero();

    return true;
}

static void parseManualTransform(const std::string& manualTransform, double& S, Mat3& R, Vec3& t)
{
    // Parse the string
    std::vector<std::string> dataStr;
    boost::split(dataStr, manualTransform, boost::is_any_of(","));
    if (dataStr.size() != 7)
    {
        throw std::runtime_error("Invalid number of values for manual transformation with ZXY Euler: tx,ty,tz,rx,ry,rz,s.");
    }

    std::vector<double> data;
    data.reserve(7);
    for (const std::string& elt : dataStr)
    {
        data.push_back(boost::lexical_cast<double>(elt));
    }

    // Assignments
    t << data[0], data[1], data[2];  // Assign Translation
    S = data[6];                     // Assign Scale

    Vec3 eulerAngles(data[3], data[4], data[5]);  // Temporary eulerAngles vector

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
    R = rotateMat;  // Assign Rotation
}

}  // namespace

IndexT getReferenceViewId(const sfmData::SfMData& sfmData, const std::string& transform)
{
    IndexT refViewId;
    try
    {
        refViewId = sfm::getViewIdFromExpression(sfmData, transform);
        if (!sfmData.isPoseAndIntrinsicDefined(refViewId))
        {
            return UndefinedIndexT;
        }
    }
    catch (...)
    {
        refViewId = UndefinedIndexT;
    }

    // Default to select the view given timestamp
    if (refViewId == UndefinedIndexT)
    {
        // Sort views with poses per timestamps
        std::vector<std::pair<int64_t, IndexT>> sorted_views;
        for (auto v : sfmData.getViews())
        {
            if (!sfmData.isPoseAndIntrinsicDefined(v.first))
            {
                continue;
            }

            int64_t t = v.second->getImage().getMetadataDateTimestamp();
            sorted_views.push_back(std::make_pair(t, v.first));
        }
        std::sort(sorted_views.begin(), sorted_views.end());

        if (sorted_views.size() == 0)
        {
            return UndefinedIndexT;
        }

        // Get the view which was taken at the middle of the sequence
        int median = sorted_views.size() / 2;
        refViewId = sorted_views[sorted_views.size() - 1].second;
    }

    return refViewId;
}

int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string sfmDataFilename;
    std::string outSfMDataFilename;
    EAlignmentMethod alignmentMethod = EAlignmentMethod::AUTO_FROM_CAMERAS;

    // user optional parameters
    std::string transform;
    std::string landmarksDescriberTypesName;
    double userScale = 1;
    bool applyScale = true;
    bool applyRotation = true;
    bool applyTranslation = true;
    std::vector<sfm::MarkerWithCoord> markers;
    std::string outputViewsAndPosesFilepath;

    std::string manualTransform;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
         "SfMData file to align.")
        ("output,o", po::value<std::string>(&outSfMDataFilename)->required(),
         "Output SfMData scene.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("method", po::value<EAlignmentMethod>(&alignmentMethod)->default_value(alignmentMethod),
         "Transform Method:\n"
         "\t- transformation: Apply a given transformation.\n"
         "\t- manual: Apply the gizmo transformation.\n"
         "\t- auto: Determines scene orientation from the cameras' X axis, auto-scaling from GPS information if available, and defines ground level from the point cloud.\n"
         "\t- auto_from_cameras: Defines coordinate system from cameras.\n"
         "\t- auto_from_cameras_x_axis: Determines scene orientation from the cameras' X axis.\n"
         "\t- auto_from_landmarks: Defines coordinate system from landmarks.\n"
         "\t- from_single_camera: Refines the coordinate system from the camera specified by --tranformation.\n"
         "\t- from_markers: Refines the coordinate system from markers specified by --markers.\n"
         "\t- from_gps: Redefines coordinate system from GPS metadata.\n"
         "\t- align_ground: defines ground level from the point cloud density. It assumes that the scene is oriented.\n")
        ("transformation", po::value<std::string>(&transform)->default_value(transform),
         "Required only for 'transformation' and 'single camera' methods:\n"
         "Transformation: Align [X,Y,Z] to +Y-axis, rotate around Y by R deg, scale by S; syntax: X,Y,Z;R;S.\n"
         "Single camera: camera UID or image filename.")
        ("manualTransform", po::value<std::string>(&manualTransform),
         "Translation, rotation and scale defined with the manual mode.")
        ("landmarksDescriberTypes,d", po::value<std::string>(&landmarksDescriberTypesName)->default_value(landmarksDescriberTypesName),
         ("Optional for 'landmarks' method:\n"
         "Image describer types used to compute the mean of the point cloud.\n"
         "Use all of them if empty\n" + feature::EImageDescriberType_informations()).c_str())
        ("scale", po::value<double>(&userScale)->default_value(userScale),
         "Additional scale to apply.")
        ("applyScale", po::value<bool>(&applyScale)->default_value(applyScale),
         "Apply scale transformation.")
        ("applyRotation", po::value<bool>(&applyRotation)->default_value(applyRotation),
         "Apply rotation transformation.")
        ("applyTranslation", po::value<bool>(&applyTranslation)->default_value(applyTranslation),
         "Apply translation transformation.")
        ("markers", po::value<std::vector<sfm::MarkerWithCoord>>(&markers)->multitoken(),
         "Markers ID and target coordinates 'ID:x,y,z'.")
        ("outputViewsAndPoses", po::value<std::string>(&outputViewsAndPosesFilepath),
         "Path of the output SfMData file.");
    // clang-format on

    CmdLine cmdline("AliceVision sfmTransform");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    if (alignmentMethod == EAlignmentMethod::FROM_MARKERS && markers.empty())
    {
        ALICEVISION_LOG_ERROR("Missing --markers option");
        return EXIT_FAILURE;
    }

    // Load input scene
    sfmData::SfMData sfmData;
    if (!sfmDataIO::load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' cannot be read");
        return EXIT_FAILURE;
    }

    // Check that at least one view has a defined pose
    int count = 0;
    for (const auto p : sfmData.getViews())
    {
        if (sfmData.isPoseAndIntrinsicDefined(p.first))
        {
            count++;
        }
    }

    if (count == 0)
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' has no valid views with estimated poses");
        return EXIT_FAILURE;
    }

    double S = 1.0;
    Mat3 R = Mat3::Identity();
    Vec3 t = Vec3::Zero();

    switch (alignmentMethod)
    {
        case EAlignmentMethod::TRANSFORMATION:
        {
            if (transform.empty())
            {
                ALICEVISION_LOG_WARNING("No transformation option set, so the transform will be identity.");
            }
            else
            {
                if (!parseAlignScale(transform, S, R, t))
                {
                    ALICEVISION_LOG_ERROR("Failed to parse align/scale argument");
                    return EXIT_FAILURE;
                }
            }
        }
        break;

        case EAlignmentMethod::MANUAL:
        {
            if (manualTransform.empty())
                ALICEVISION_LOG_WARNING("No manualTransform option set, so the transform will be identity.");
            else
                parseManualTransform(manualTransform, S, R, t);
        }
        break;

        case EAlignmentMethod::AUTO_FROM_CAMERAS:
            sfm::computeNewCoordinateSystemFromCameras(sfmData, S, R, t);
            break;

        case EAlignmentMethod::AUTO:
        {
            sfm::computeNewCoordinateSystemAuto(sfmData, S, R, t);
        }
        break;

        case EAlignmentMethod::AUTO_FROM_CAMERAS_X_AXIS:
        {
            // Align with x axis
            sfm::computeNewCoordinateSystemFromCamerasXAxis(sfmData, S, R, t);

            const IndexT refViewId = getReferenceViewId(sfmData, transform);

            const Eigen::Matrix3d ref_R_world = sfmData.getPose(sfmData.getView(refViewId)).getTransform().rotation();

            // Apply x axis alignment before doing the y alignment
            const Eigen::Matrix3d refcam_R_updatedWorld = ref_R_world * R.transpose();

            Eigen::Matrix3d zeroX_R_world;
            sfm::getRotationNullifyX(zeroX_R_world, refcam_R_updatedWorld);
            R = zeroX_R_world * R;
        }
        break;

        case EAlignmentMethod::AUTO_FROM_LANDMARKS:
            sfm::computeNewCoordinateSystemFromLandmarks(sfmData, feature::EImageDescriberType_stringToEnums(landmarksDescriberTypesName), S, R, t);
            break;

        case EAlignmentMethod::FROM_SINGLE_CAMERA:
            if (transform.empty())
            {
                ALICEVISION_LOG_WARNING("No transformation option set, so the transform will be identity.");
            }
            else
            {
                const IndexT viewId = sfm::getViewIdFromExpression(sfmData, transform);
                sfm::computeNewCoordinateSystemFromSingleCamera(sfmData, viewId, S, R, t);
            }
            break;

        case EAlignmentMethod::FROM_CENTER_CAMERA:
        {
            const IndexT centerViewId = sfm::getCenterCameraView(sfmData);
            sfm::computeNewCoordinateSystemFromSingleCamera(sfmData, centerViewId, S, R, t);
            break;
        }

        case EAlignmentMethod::FROM_MARKERS:
        {
            std::vector<feature::EImageDescriberType> markersDescTypes = {
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CCTAG)
                feature::EImageDescriberType::CCTAG3,
                feature::EImageDescriberType::CCTAG4,
#endif
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_APRILTAG)
                feature::EImageDescriberType::APRILTAG16H5,
#endif
            };
            std::set<feature::EImageDescriberType> usedDescTypes = sfmData.getLandmarkDescTypes();

            std::vector<feature::EImageDescriberType> usedMarkersDescTypes;
            std::set_intersection(
              usedDescTypes.begin(), usedDescTypes.end(), markersDescTypes.begin(), markersDescTypes.end(), std::back_inserter(usedMarkersDescTypes));
            std::vector<feature::EImageDescriberType> inDescTypes = feature::EImageDescriberType_stringToEnums(landmarksDescriberTypesName);

            std::vector<feature::EImageDescriberType> vDescTypes;
            std::set_intersection(
              usedMarkersDescTypes.begin(), usedMarkersDescTypes.end(), inDescTypes.begin(), inDescTypes.end(), std::back_inserter(vDescTypes));
            if (vDescTypes.size() != 1)
            {
                ALICEVISION_LOG_ERROR("Alignment from markers: Invalid number of image describer types: " << vDescTypes.size());
                for (auto d : vDescTypes)
                {
                    ALICEVISION_LOG_ERROR(" - " << feature::EImageDescriberType_enumToString(d));
                }
                return EXIT_FAILURE;
            }
            const bool success = sfm::computeNewCoordinateSystemFromSpecificMarkers(sfmData, vDescTypes.front(), markers, applyScale, S, R, t);
            if (!success)
            {
                ALICEVISION_LOG_ERROR("Failed to find a valid transformation for these " << markers.size() << " markers.");
                return EXIT_FAILURE;
            }
            break;
        }
        case EAlignmentMethod::FROM_GPS:
        {
            std::mt19937 randomNumberGenerator;
            if (!sfm::computeNewCoordinateSystemFromGpsData(sfmData, randomNumberGenerator, S, R, t))
            {
                ALICEVISION_LOG_ERROR("Failed to find a valid transformation from the GPS metadata.");
                return EXIT_FAILURE;
            }
            break;
        }
        case EAlignmentMethod::ALIGN_GROUND:
        {
            sfm::computeNewCoordinateSystemGroundAuto(sfmData, t);
            break;
        }
    }

    if (!applyRotation)
    {
        // remove rotation from translation
        t = R.transpose() * t;
        // remove rotation
        R = Mat3::Identity();
    }
    if (applyScale)
    {
        // apply user scale
        S *= userScale;
        t *= userScale;
    }
    else
    {
        // remove scale from translation
        if (std::abs(S) > 0.00001)
        {
            t /= S;
        }
        // reset scale to 1
        S = 1.0;
    }
    if (!applyTranslation)
    {
        // remove translation
        t = Vec3::Zero();
    }

    {
        ALICEVISION_LOG_INFO(std::setprecision(17) << "Transformation:" << std::endl
                                                   << "\t- Scale: " << S << std::endl
                                                   << "\t- Rotation:\n"
                                                   << R << std::endl
                                                   << "\t- Translate: " << t.transpose());
    }

    sfm::applyTransform(sfmData, S, R, t);

    // In AUTO mode, ground detection and alignment is performed as a post-process
    if (alignmentMethod == EAlignmentMethod::AUTO && applyTranslation)
    {
        sfm::computeNewCoordinateSystemGroundAuto(sfmData, t);
        sfm::applyTransform(sfmData, 1.0, Eigen::Matrix3d::Identity(), t);
    }

    ALICEVISION_LOG_INFO("Save into '" << outSfMDataFilename << "'");

    // Export the SfMData scene in the expected format
    if (!sfmDataIO::save(sfmData, outSfMDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("An error occurred while trying to save '" << outSfMDataFilename << "'");
        return EXIT_FAILURE;
    }

    if (!outputViewsAndPosesFilepath.empty())
    {
        sfmDataIO::save(sfmData, outputViewsAndPosesFilepath, sfmDataIO::ESfMData(sfmDataIO::VIEWS | sfmDataIO::EXTRINSICS | sfmDataIO::INTRINSICS));
    }

    return EXIT_SUCCESS;
}
