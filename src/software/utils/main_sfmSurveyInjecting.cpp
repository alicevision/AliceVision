// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/types.hpp>
#include <aliceVision/alicevision_omp.hpp>
#include <aliceVision/config.hpp>

#include <aliceVision/system/Timer.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/cmdline/cmdline.hpp>

#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

#include <boost/program_options.hpp>
#include <boost/json.hpp>

#include <fstream>
#include <stdexcept>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 1

using namespace aliceVision;
namespace po = boost::program_options;

// Internal structures for json reading

struct Position3D
{
    double x;
    double y;
    double z;
};

struct ImagePoint
{
    int frame;
    double x;
    double y;
};

struct Point
{
    std::string point_id;
    std::string point_name;
    std::string calc_mode;
    std::string survey_mode;
    Position3D calc_3d;
    Position3D survey_3d;
    std::vector<ImagePoint> image_points;
};

struct Survey
{
    std::string pgroup_name;
    std::string camera_name;
    std::vector<Point> points;
};

Position3D tag_invoke(boost::json::value_to_tag<Position3D>, boost::json::value const& jv)
{
    if (!jv.is_object())
    {
        throw std::invalid_argument("Position3D: Expected JSON object");
    }

    boost::json::object const& obj = jv.as_object();

    if (!obj.contains("x") || !obj.contains("y") || !obj.contains("z"))
    {
        throw std::invalid_argument("Position3D: Missing required fields (x, y, z)");
    }

    if (!obj.at("x").is_number() || !obj.at("y").is_number() || !obj.at("z").is_number())
    {
        throw std::invalid_argument("Position3D: x, y, z must be numeric values");
    }

    return Position3D{
        boost::json::value_to<double>(obj.at("x")),
        boost::json::value_to<double>(obj.at("y")),
        boost::json::value_to<double>(obj.at("z"))
    };
}

ImagePoint tag_invoke(boost::json::value_to_tag<ImagePoint>, boost::json::value const& jv)
{
    if (!jv.is_object())
    {
        throw std::invalid_argument("ImagePoint: Expected JSON object");
    }

    boost::json::object const& obj = jv.as_object();

    if (!obj.contains("frame") || !obj.contains("x") || !obj.contains("y"))
    {
        throw std::invalid_argument("ImagePoint: Missing required fields (frame, x, y)");
    }

    if (!obj.at("frame").is_number())
    {
        throw std::invalid_argument("ImagePoint: frame must be a numeric value");
    }

    if (!obj.at("x").is_number() || !obj.at("y").is_number())
    {
        throw std::invalid_argument("ImagePoint: x and y must be numeric values");
    }

    int frame = boost::json::value_to<int>(obj.at("frame"));
    if (frame < 0)
    {
        throw std::invalid_argument("ImagePoint: frame must be positive");
    }

    return ImagePoint{
        frame,
        boost::json::value_to<double>(obj.at("x")),
        boost::json::value_to<double>(obj.at("y"))
    };
}

Point tag_invoke(boost::json::value_to_tag<Point>, boost::json::value const& jv)
{
    if (!jv.is_object())
    {
        throw std::invalid_argument("Point: Expected JSON object");
    }

    boost::json::object const& obj = jv.as_object();

    if (!obj.contains("point_id") || !obj.contains("point_name") ||
        !obj.contains("calc_mode") || !obj.contains("survey_mode") ||
        !obj.contains("calc_3d") || !obj.contains("survey_3d") ||
        !obj.contains("image_points"))
    {
        throw std::invalid_argument("Point: Missing required fields (point_id, point_name, calc_mode, survey_mode, calc_3d, survey_3d, image_points)");
    }

    if (!obj.at("point_id").is_string() || !obj.at("point_name").is_string() ||
        !obj.at("calc_mode").is_string() || !obj.at("survey_mode").is_string())
    {
        throw std::invalid_argument("Point: point_id, point_name, calc_mode, and survey_mode must be strings");
    }

    if (!obj.at("image_points").is_array())
    {
        throw std::invalid_argument("Point: image_points must be an array");
    }

    Point pt;
    pt.point_id = boost::json::value_to<std::string>(obj.at("point_id"));
    pt.point_name = boost::json::value_to<std::string>(obj.at("point_name"));
    pt.calc_mode = boost::json::value_to<std::string>(obj.at("calc_mode"));
    pt.survey_mode = boost::json::value_to<std::string>(obj.at("survey_mode"));
    pt.calc_3d = boost::json::value_to<Position3D>(obj.at("calc_3d"));
    pt.survey_3d = boost::json::value_to<Position3D>(obj.at("survey_3d"));

    boost::json::array const& img_points = obj.at("image_points").as_array();
    for (auto const& ip : img_points)
    {
        pt.image_points.push_back(boost::json::value_to<ImagePoint>(ip));
    }

    return pt;
}

Survey tag_invoke(boost::json::value_to_tag<Survey>, boost::json::value const& jv)
{
    if (!jv.is_object())
    {
        throw std::invalid_argument("Survey: Expected JSON object");
    }

    boost::json::object const& obj = jv.as_object();

    if (!obj.contains("pgroup_name") || !obj.contains("camera_name") || !obj.contains("points"))
    {
        throw std::invalid_argument("Survey: Missing required fields (pgroup_name, camera_name, points)");
    }

    if (!obj.at("pgroup_name").is_string() || !obj.at("camera_name").is_string())
    {
        throw std::invalid_argument("Survey: pgroup_name and camera_name must be strings");
    }

    if (!obj.at("points").is_array())
    {
        throw std::invalid_argument("Survey: points must be an array");
    }

    Survey survey;
    survey.pgroup_name = boost::json::value_to<std::string>(obj.at("pgroup_name"));
    survey.camera_name = boost::json::value_to<std::string>(obj.at("camera_name"));

    boost::json::array const& points = obj.at("points").as_array();
    if (points.empty())
    {
        throw std::invalid_argument("Survey: points array cannot be empty");
    }

    for (auto const& pt : points)
    {
        survey.points.push_back(boost::json::value_to<Point>(pt));
    }

    return survey;
}

/**
 * @brief Get a set of surveys from a JSON file (assumes the file format is ok).
 * The JSON file contains an array of objects. Each object describes a frameId, and a list of points.
 * @param surveyFilename the input JSON filename
 * @param output a Survey object filled
 * @return false if the process failed, true otherwise
 */
bool getSurveysFromJson(const std::string& surveyFilename, Survey & output)
{
    std::ifstream inputfile(surveyFilename);
    if (!inputfile.is_open())
    {
        return false;
    }

    std::stringstream buffer;
    buffer << inputfile.rdbuf();

    std::string jsonContent = buffer.str();
    if (jsonContent.empty())
    {
        throw std::runtime_error("JSON file is empty: " + surveyFilename);
    }

    boost::json::value jv;
    try
    {
        jv = boost::json::parse(jsonContent);
    }
    catch (const std::exception& e)
    {
        throw std::runtime_error("Failed to parse JSON file '" + surveyFilename + "': " + e.what());
    }

    try
    {
        output = boost::json::value_to<Survey>(jv);
    }
    catch (const std::exception& e)
    {
        throw std::runtime_error("Invalid JSON structure in '" + surveyFilename + "': " + e.what());
    }

    return true;
}

int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string sfmDataFilename;
    std::string sfmDataOutputFilename;
    std::string surveyFilename;
    int offset = -1;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
         "Input SfMData file.")
        ("output,o", po::value<std::string>(&sfmDataOutputFilename)->required(),
         "SfMData output file with the injected poses.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("surveyFilename,p", po::value<std::string>(&surveyFilename)->default_value(surveyFilename),
        "JSON file containing the survey to inject.")
        ("offset", po::value<int>(&offset)->default_value(offset),
        "Positive offset to use on the reference file frame number to match the input frame number (or -1 to try auto-detection).");
    // clang-format on

    CmdLine cmdline("AliceVision SfM Survey injecting");

    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }


    // Set maxThreads
    HardwareContext hwc = cmdline.getHardwareContext();
    omp_set_num_threads(hwc.getMaxThreads());

    if (surveyFilename.empty())
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

    // Get First Frame ID as 3DE gives a [0, nbFrames - 1] frame ID
    IndexT minFrameId = std::numeric_limits<IndexT>::max();
    for (const auto& [viewId, pView] : sfmData.getViews())
    {
        minFrameId = std::min(minFrameId, pView->getFrameId());
    }
    ALICEVISION_LOG_INFO("Minimal frame id in sfmData is " << minFrameId << ".");

    if (offset >= 0)
    {
        minFrameId = offset;
    }

    ALICEVISION_LOG_INFO("Using an offset between input frameId and real frameId of " << minFrameId << ".");

    // Read survey points in an intermediate structure
    Survey survey;
    try
    {
        if (!getSurveysFromJson(surveyFilename, survey))
        {
            ALICEVISION_LOG_ERROR("The survey file '" + surveyFilename + "' cannot be opened.");
            return EXIT_FAILURE;
        }
    }
    catch (const std::exception& e)
    {
        ALICEVISION_LOG_ERROR("Error processing survey file '" + surveyFilename + "': " + e.what());
        return EXIT_FAILURE;
    }

    sfmData::SurveyPoints & spoints = sfmData.getSurveyPoints();

    // Fill up sfmData with the intermediate structure
    for (const auto& [viewId, view] : sfmData.getViews().valueRange())
    {
        if (!sfmData.isPoseAndIntrinsicDefined(viewId))
        {
            continue;
        }

        const auto & intrinsic = sfmData.getIntrinsic(view.getIntrinsicId());

        std::vector<sfmData::SurveyPoint> spts;

        IndexT frameId = view.getFrameId();

        //We assume this pose is also coming from 3DE and has not been modified !
        geometry::Pose3 pose = sfmData.getPose(view).getTransform();

        const sfmData::ImageGroup & group = *sfmData.getImageGroups().at(view.getImageGroupId());
        if (group.isNodalCamera())
        {
            Vec3 center = group.getCenter();
            pose.setCenter(center);
        }

        for (const auto & point: survey.points)
        {
            sfmData::SurveyPoint p;

            p.point3d.x() = point.survey_3d.x;
            p.point3d.y() = -point.survey_3d.y;
            p.point3d.z() = -point.survey_3d.z;

            for (const auto & image_point: point.image_points)
            {
                // Json frame is the sequence offset starting by 1.
                if (frameId != (image_point.frame - 1 + minFrameId))
                {
                    continue;
                }

                // From 3DE representation (normalized [0,1] with origin at top-left)
                // to AliceVision (pixel coordinates with origin at bottom-left).
                // X is scaled by image width, Y is flipped and scaled by image height.
                p.survey.x() = image_point.x * intrinsic.w();
                p.survey.y() = (1.0 - image_point.y) * intrinsic.h();

                // Compute residual
                Vec2 obsEstimated = intrinsic.transformProject(pose, p.point3d.homogeneous(), true);
                p.residual.x() = p.survey.x() - obsEstimated.x();
                p.residual.y() = p.survey.y() - obsEstimated.y();

                spts.push_back(p);

                double rx = std::abs(p.residual.x());
                double ry = std::abs(p.residual.y());
                ALICEVISION_LOG_INFO("Found observation for frame " << frameId << " with residual [" << rx << ", " << ry << "].");
            }
        }

        if (spts.size() > 0)
        {
            spoints[viewId] = spts;
        }
    }

    sfmDataIO::save(sfmData, sfmDataOutputFilename, sfmDataIO::ESfMData::ALL);

    return EXIT_SUCCESS;
}
