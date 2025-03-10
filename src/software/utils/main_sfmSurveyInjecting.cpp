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

struct SurveyObservation
{
    Vec3 point3d;
    Vec2 point2d;
};

struct SurveyInput
{
    int frameId;
    std::vector<SurveyObservation> observations;
};

/**
 * @brief get a survey from a boost json object (assume the file format is ok)
 * @param obj the input json object
 * @param readSurvey the output survey information
 * @return false if the process failed
 */
bool getSurveyFromJson(const boost::json::object& obj, SurveyInput& readSurvey)
{
    readSurvey.observations.clear();
    readSurvey.frameId = boost::json::value_to<IndexT>(obj.at("frame_no"));

    boost::json::value jv = obj.at("points");
    if (!jv.is_array())
    {
        return false;
    }

    boost::json::array vobj = jv.as_array();
    for (auto item : vobj)
    {
        const boost::json::object& obj = item.as_object();

        SurveyObservation obs;
        obs.point3d.x() = boost::json::value_to<double>(obj.at("X"));
        obs.point3d.y() = boost::json::value_to<double>(obj.at("Y"));
        obs.point3d.z() = boost::json::value_to<double>(obj.at("Z"));

        obs.point2d.x() = boost::json::value_to<double>(obj.at("u"));
        obs.point2d.y() = boost::json::value_to<double>(obj.at("v"));

        readSurvey.observations.push_back(obs);
    }

    return true;
}

/**
 * @brief Get a set of surveys from a JSON file (assumes the file format is ok).
 * The JSON file contains an array of objects. Each object describes a frameId, and a list of points.
 * @param surveyFilename the input JSON filename
 * @param output the read vector of surveyinput
 * @return false if the process failed, true otherwise
 */
bool getSurveysFromJson(const std::string& surveyFilename, std::vector<SurveyInput> & output)
{
    std::ifstream inputfile(surveyFilename);
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

        SurveyInput input;
        if (getSurveyFromJson(obj, input))
        {
            output.push_back(input);
        }
    }

    return true;
}

int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string sfmDataFilename;
    std::string sfmDataOutputFilename;
    std::string surveyFilename;

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
        "JSON file containing the survey to inject.");
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

    auto & landmarks = sfmData.getLandmarks();

    std::vector<SurveyInput> surveys;
    if (!getSurveysFromJson(surveyFilename, surveys))
    {
        ALICEVISION_LOG_ERROR("The survey file '" + surveyFilename + "' cannot be read.");
        return EXIT_FAILURE;
    }

    sfmData::SurveyPoints & spoints = sfmData.getSurveyPoints();

    // Set the pose for all the views with frame IDs found in the JSON file
    size_t idFeature = 0;
    for (const auto& [viewId, pView] : sfmData.getViews())
    {
        IndexT frameId = pView->getFrameId();

        if (!sfmData.isIntrinsicDefined(viewId))
        {
            continue;
        }

        const auto & intrinsic = sfmData.getIntrinsic(pView->getIntrinsicId());

        for (const auto & survey: surveys)
        {
            if (frameId != survey.frameId)
            {
                continue;
            }

            for (const auto & obs : survey.observations)
            {
                sfmData::SurveyPoint p;

                p.survey.x() = obs.point2d.x() * intrinsic.w();
                p.survey.y() = intrinsic.h() - 1.0 - (obs.point2d.y() * intrinsic.h());

                p.point3d.x() = obs.point3d.x();
                p.point3d.y() = - obs.point3d.y();
                p.point3d.z() = - obs.point3d.z();

                spoints[viewId].push_back(p);

                idFeature++;
            }
        }
    }

    std::cout << sfmData.getSurveyPoints().size() << std::endl;

    sfmDataIO::save(sfmData, sfmDataOutputFilename, sfmDataIO::ESfMData::ALL);

    return EXIT_SUCCESS;
}
