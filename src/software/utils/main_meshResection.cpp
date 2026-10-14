// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/sfm/pipeline/localization/SfMLocalizer.hpp>
#include <aliceVision/sfmData/SurveyPoint.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

#include <boost/program_options.hpp>
#include <boost/json.hpp>

#include <array>
#include <fstream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <vector>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;

namespace {

using ObservationsPerView = std::unordered_map<IndexT, std::vector<sfmData::SurveyPoint>>;

bool parseObservation(const boost::json::value& value, sfmData::SurveyPoint& observation)
{
    if (!value.is_object())
    {
        throw std::invalid_argument("Observation entry must be a JSON object.");
    }

    const boost::json::object& obj = value.as_object();
    static constexpr std::array<const char*, 6> requiredKeys = {"X", "Y", "Z", "x", "y", "picked"};
    for (const char* key : requiredKeys)
    {
        if (!obj.contains(key))
        {
            throw std::invalid_argument(std::string("Observation entry is missing required field '") + key + "'.");
        }
    }

    if (!obj.at("X").is_number() || !obj.at("Y").is_number() || !obj.at("Z").is_number() ||
        !obj.at("x").is_number() || !obj.at("y").is_number() || !obj.at("picked").is_bool())
    {
        throw std::invalid_argument("Observation fields X, Y, Z, x, y must be numeric and picked must be boolean.");
    }

    observation.point3d = Vec3(boost::json::value_to<double>(obj.at("X")),
                               boost::json::value_to<double>(obj.at("Y")),
                               boost::json::value_to<double>(obj.at("Z")));
    observation.survey = Vec2(boost::json::value_to<double>(obj.at("x")),
                              boost::json::value_to<double>(obj.at("y")));
    observation.residual = Vec2::Zero();

    return obj.at("picked").as_bool();
}

bool loadObservationsFromJson(const std::string& jsonFilename, ObservationsPerView& observationsPerView)
{
    std::ifstream inputFile(jsonFilename);
    if (!inputFile.is_open())
    {
        return false;
    }

    std::stringstream buffer;
    buffer << inputFile.rdbuf();

    const std::string jsonContent = buffer.str();
    if (jsonContent.empty())
    {
        throw std::runtime_error("JSON file is empty: " + jsonFilename);
    }

    boost::json::value root;
    try
    {
        root = boost::json::parse(jsonContent);
    }
    catch (const std::exception& e)
    {
        throw std::runtime_error("Failed to parse JSON file '" + jsonFilename + "': " + e.what());
    }

    if (!root.is_array())
    {
        throw std::runtime_error("Expected the JSON root to be an array.");
    }

    for (const boost::json::value& entry : root.as_array())
    {
        if (!entry.is_object())
        {
            throw std::runtime_error("Each array entry must be a JSON object.");
        }

        const boost::json::object& pointEntry = entry.as_object();
        if (!pointEntry.contains("type") || !pointEntry.at("type").is_string() || pointEntry.at("type").as_string() != "SurveyPoint")
        {
            continue;
        }

        if (!pointEntry.contains("observations") || !pointEntry.at("observations").is_object())
        {
            throw std::runtime_error("Each array entry must contain an 'observations' object.");
        }

        const boost::json::object& viewEntries = pointEntry.at("observations").as_object();
        for (const auto& [key, value] : viewEntries)
        {
            const std::string keyString(key.data(), key.size());
            IndexT viewId = UndefinedIndexT;
            try
            {
                viewId = static_cast<IndexT>(std::stoull(keyString));
            }
            catch (const std::exception& e)
            {
                throw std::runtime_error("Invalid numeric key '" + keyString + "' in JSON file '" + jsonFilename + "': " + e.what());
            }

            sfmData::SurveyPoint observation;
            if (parseObservation(value, observation))
            {
                observationsPerView[viewId].push_back(std::move(observation));
            }
        }
    }

    return true;
}

bool updateResidualsInJson(const sfmData::SfMData& sfmData, const std::string& jsonFilename)
{
    std::ifstream inputFile(jsonFilename);
    if (!inputFile.is_open())
    {
        return false;
    }

    std::stringstream buffer;
    buffer << inputFile.rdbuf();

    const std::string jsonContent = buffer.str();
    if (jsonContent.empty())
    {
        throw std::runtime_error("JSON file is empty: " + jsonFilename);
    }

    boost::json::value root;
    try
    {
        root = boost::json::parse(jsonContent);
    }
    catch (const std::exception& e)
    {
        throw std::runtime_error("Failed to parse JSON file '" + jsonFilename + "': " + e.what());
    }

    if (!root.is_array())
    {
        throw std::runtime_error("Expected the JSON root to be an array.");
    }

    for (boost::json::value& entry : root.as_array())
    {
        if (!entry.is_object())
        {
            throw std::runtime_error("Each array entry must be a JSON object.");
        }

        boost::json::object& pointEntry = entry.as_object();
        if (!pointEntry.contains("type") || !pointEntry.at("type").is_string() || pointEntry.at("type").as_string() != "SurveyPoint")
        {
            continue;
        }

        if (!pointEntry.contains("observations") || !pointEntry.at("observations").is_object())
        {
            throw std::runtime_error("Each array entry must contain an 'observations' object.");
        }

        boost::json::object& viewEntries = pointEntry.at("observations").as_object();
        for (auto& [key, value] : viewEntries)
        {
            const std::string keyString(key.data(), key.size());
            IndexT viewId = UndefinedIndexT;
            try
            {
                viewId = static_cast<IndexT>(std::stoull(keyString));
            }
            catch (const std::exception& e)
            {
                throw std::runtime_error("Invalid numeric key '" + keyString + "' in JSON file '" + jsonFilename + "': " + e.what());
            }

            if (!value.is_object())
            {
                throw std::runtime_error("Observation entry must be a JSON object.");
            }

            boost::json::object& observationObj = value.as_object();
            static constexpr std::array<const char*, 3> requiredKeys = {"X", "Y", "Z"};
            for (const char* requiredKey : requiredKeys)
            {
                if (!observationObj.contains(requiredKey))
                {
                    throw std::runtime_error(std::string("Observation entry is missing required field '") + requiredKey + "'.");
                }
            }

            if (!observationObj.at("X").is_number() || !observationObj.at("Y").is_number() || !observationObj.at("Z").is_number())
            {
                throw std::runtime_error("Observation fields X, Y, Z must be numeric.");
            }

            if (!sfmData.isPoseAndIntrinsicDefined(viewId))
            {
                observationObj["ex"] = "nan";
                observationObj["ey"] = "nan";
                continue;
            }

            const sfmData::View& view = sfmData.getView(viewId);
            const auto intrinsicIt = sfmData.getIntrinsics().find(view.getIntrinsicId());
            if (intrinsicIt == sfmData.getIntrinsics().end())
            {
                observationObj["ex"] = "nan";
                observationObj["ey"] = "nan";
                continue;
            }

            const Vec3 point3d(boost::json::value_to<double>(observationObj.at("X")),
                               boost::json::value_to<double>(observationObj.at("Y")),
                               boost::json::value_to<double>(observationObj.at("Z")));

            const camera::IntrinsicBase& intrinsic = *intrinsicIt->second;
            const geometry::Pose3 pose = sfmData.getPose(view).getTransform();
            const Vec2 projected = intrinsic.transformProject(pose, point3d.homogeneous(), true);

            observationObj["ex"] = projected.x();
            observationObj["ey"] = projected.y();
        }
    }

    std::ofstream outputFile(jsonFilename);
    if (!outputFile.is_open())
    {
        return false;
    }

    outputFile << boost::json::serialize(root);
    return outputFile.good();
}

bool estimatePoseForView(sfmData::SfMData& sfmData,
                         IndexT viewId,
                         const std::vector<sfmData::SurveyPoint>& observations)
{
    auto viewIt = sfmData.getViews().find(viewId);
    if (viewIt == sfmData.getViews().end())
    {
        ALICEVISION_LOG_WARNING("Skipping JSON entry for unknown viewId " << viewId << ".");
        return false;
    }

    if (observations.empty())
    {
        ALICEVISION_LOG_WARNING("Skipping viewId " << viewId << " because it has no picked correspondences.");
        return false;
    }

    const sfmData::View& view = *viewIt->second;
    const auto intrinsicIt = sfmData.getIntrinsics().find(view.getIntrinsicId());
    if (intrinsicIt == sfmData.getIntrinsics().end())
    {
        return false;
    }

    const camera::IntrinsicBase & intrinsic = *intrinsicIt->second;

    const std::size_t minSamples = 4;
    if (observations.size() < minSamples)
    {
        ALICEVISION_LOG_WARNING("Skipping viewId " << viewId << " because it has only " << observations.size()
                                                    << " picked correspondences; at least " << minSamples << " are required.");
        return false;
    }

    sfm::ImageLocalizerMatchData matchingData;
    matchingData.pt2D = Mat2X(2, observations.size());
    matchingData.pt3D = Mat3X(3, observations.size());
    matchingData.vec_descType.assign(observations.size(), feature::EImageDescriberType::UNKNOWN);

    for (std::size_t index = 0; index < observations.size(); ++index)
    {
        matchingData.pt2D.col(index) = observations[index].survey;
        matchingData.pt3D.col(index) = observations[index].point3d;
    }

    geometry::Pose3 pose;
    std::mt19937 randomNumberGenerator(0);
    const Pair imageSize(view.getImage().getWidth(), view.getImage().getHeight());
    if (!sfm::SfMLocalizer::localize(imageSize, &intrinsic, randomNumberGenerator, matchingData, pose))
    {
        ALICEVISION_LOG_WARNING("Pose estimation failed for viewId " << viewId << ".");
        return false;
    }

    sfmData.setPose(view, sfmData::CameraPose(pose));

    std::vector<sfmData::SurveyPoint> surveyPoints = observations;
    for (sfmData::SurveyPoint& surveyPoint : surveyPoints)
    {
        const Vec2 projected = intrinsic.transformProject(pose, surveyPoint.point3d.homogeneous(), true);
        surveyPoint.residual = surveyPoint.survey - projected;
    }
    sfmData.getSurveyPoints()[viewId] = std::move(surveyPoints);

    ALICEVISION_LOG_INFO("Estimated pose for viewId " << viewId << " using " << matchingData.vec_inliers.size()
                                                       << " inliers out of " << observations.size() << " picked correspondences.");
    return true;
}

}  // namespace

int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string sfmDataFilename;
    std::string sfmDataOutputFilename;
    std::string jsonFilename;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilename)->required(), "SfMData file.")
        ("json,j", po::value<std::string>(&jsonFilename)->required(), "JSON shape file.")
        ("output,o", po::value<std::string>(&sfmDataOutputFilename)->required(), "SfMData output file.");

    // clang-format on

    CmdLine cmdline("Estimate camera poses from manual 2D/3D mesh correspondences stored in a JSON file.\n"
                    "AliceVision meshResection");
    cmdline.add(requiredParams);
    //cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // load input SfMData scene
    sfmData::SfMData sfmData;
    if(!sfmDataIO::load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" + sfmDataFilename + "' cannot be read.");
        return EXIT_FAILURE;
    }

    ObservationsPerView observationsPerView;
    try
    {
        if (!loadObservationsFromJson(jsonFilename, observationsPerView))
        {
            ALICEVISION_LOG_ERROR("The input JSON file '" + jsonFilename + "' cannot be read.");
            return EXIT_FAILURE;
        }
    }
    catch (const std::exception& e)
    {
        ALICEVISION_LOG_ERROR("Error while processing JSON file '" + jsonFilename + "': " + e.what());
        return EXIT_FAILURE;
    }

    std::size_t localizedViews = 0;
    for (const auto& [viewId, observations] : observationsPerView)
    {
        localizedViews += estimatePoseForView(sfmData, viewId, observations) ? 1 : 0;
    }

    if (localizedViews == 0)
    {
        ALICEVISION_LOG_ERROR("No pose could be estimated from the provided JSON correspondences.");
        return EXIT_FAILURE;
    }

    try
    {
        if (!updateResidualsInJson(sfmData, jsonFilename))
        {
            ALICEVISION_LOG_ERROR("Could not rewrite JSON file '" << jsonFilename << "' with ex/ey residual values.");
            return EXIT_FAILURE;
        }
    }
    catch (const std::exception& e)
    {
        ALICEVISION_LOG_ERROR("Error while updating ex/ey in JSON file '" << jsonFilename << "': " << e.what());
        return EXIT_FAILURE;
    }

    ALICEVISION_LOG_INFO("Export SfM: " << sfmDataOutputFilename << " (" << localizedViews << " localized views)");
    if (!sfmDataIO::save(sfmData, sfmDataOutputFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The output SfMData file '" << sfmDataOutputFilename << "' cannot be written.");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
