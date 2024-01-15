// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2015 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/stl/regex.hpp>
#include <aliceVision/config.hpp>
#include <aliceVision/utils/regexFilter.hpp>

#include <boost/program_options.hpp>
#include <boost/system/error_code.hpp>

#include <filesystem>
#include <algorithm>
#include <string>
#include <regex>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 2
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = std::filesystem;

// convert from a SfMData format to another
int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string sfmDataFilename;
    std::string outputSfMDataFilename;

    // user optional parameters

    std::string describerTypesName;
    std::vector<std::string> imageWhiteList;
    bool flagViews = true;
    bool flagIntrinsics = true;
    bool flagExtrinsics = true;
    bool flagStructure = true;
    bool flagObservations = true;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
         "SfMData file.")
        ("output,o", po::value<std::string>(&outputSfMDataFilename)->required(),
         "Path to the output Alembic file.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("describerTypes,d", po::value<std::string>(&describerTypesName)->default_value(describerTypesName),
         feature::EImageDescriberType_informations().c_str())
        ("imageWhiteList", po::value<std::vector<std::string>>(&imageWhiteList)->multitoken()->default_value(imageWhiteList),
         "Image white list containing uid(s), image filenames or regex(es) on the image file path (supported regex: '#'"
         " matches a single digit, '@' one or more digits, '?' one character and '*' zero or more).")
        ("views", po::value<bool>(&flagViews)->default_value(flagViews),
         "Export views.")
        ("intrinsics", po::value<bool>(&flagIntrinsics)->default_value(flagIntrinsics),
         "Export intrinsics.")
        ("extrinsics", po::value<bool>(&flagExtrinsics)->default_value(flagExtrinsics),
         "Export extrinsics.")
        ("structure", po::value<bool>(&flagStructure)->default_value(flagStructure),
         "Export structure.")
        ("observations", po::value<bool>(&flagObservations)->default_value(flagObservations),
         "Export observations.");
    // clang-format on

    CmdLine cmdline("AliceVision convertSfMFormat");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    if (sfmDataFilename.empty() || outputSfMDataFilename.empty())
    {
        ALICEVISION_LOG_ERROR("Invalid input or output filename");
        return EXIT_FAILURE;
    }

    int flags = (flagViews ? sfmDataIO::VIEWS : 0) | (flagIntrinsics ? sfmDataIO::INTRINSICS : 0) | (flagExtrinsics ? sfmDataIO::EXTRINSICS : 0) |
                (flagObservations ? sfmDataIO::OBSERVATIONS_WITH_FEATURES : 0) | (flagStructure ? sfmDataIO::STRUCTURE : 0);

    flags = (flags) ? flags : sfmDataIO::ALL;

    // load input SfMData scene
    sfmData::SfMData sfmData;
    if (!sfmDataIO::load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' cannot be read");
        return EXIT_FAILURE;
    }

    // image white list filter
    if (!imageWhiteList.empty())
    {
        std::vector<std::regex> imageWhiteRegexList;
        imageWhiteRegexList.reserve(imageWhiteList.size());
        for (const std::string& exp : imageWhiteList)
        {
            imageWhiteRegexList.emplace_back(simpleFilterToRegex_noThrow(exp));
        }

        std::vector<IndexT> viewsToRemove;
        std::vector<IndexT> posesToRemove;
        std::vector<IndexT> landmarksToRemove;

        for (const auto& viewPair : sfmData.getViews())
        {
            const sfmData::View& view = *(viewPair.second);
            bool toRemove = true;

            for (std::size_t i = 0; i < imageWhiteList.size(); ++i)
            {
                // Compare to filename, stem (filename without extension), view UID or regex on the full path
                if (imageWhiteList[i] == fs::path(view.getImage().getImagePath()).filename() ||
                    imageWhiteList[i] == fs::path(view.getImage().getImagePath()).stem() || imageWhiteList[i] == std::to_string(view.getViewId()) ||
                    std::regex_match(view.getImage().getImagePath(), imageWhiteRegexList[i]))
                {
                    toRemove = false;
                }
            }

            if (toRemove)
            {
                viewsToRemove.push_back(view.getViewId());
                // remove pose only if it exists and it is independent
                if (view.isPoseIndependant() && sfmData.existsPose(view))
                    posesToRemove.push_back(view.getPoseId());
            }
        }

        for (auto& landmarkPair : sfmData.getLandmarks())
        {
            sfmData::Landmark& landmark = landmarkPair.second;
            for (const IndexT viewId : viewsToRemove)
            {
                if (landmark.getObservations().find(viewId) != landmark.getObservations().end())
                    landmark.getObservations().erase(viewId);
            }
            if (landmark.getObservations().empty())
                landmarksToRemove.push_back(landmarkPair.first);
        }

        for (const IndexT viewId : viewsToRemove)
            sfmData.getViews().erase(viewId);

        for (const IndexT poseId : posesToRemove)
            sfmData.erasePose(poseId);

        for (const IndexT landmarkId : landmarksToRemove)
            sfmData.getLandmarks().erase(landmarkId);
    }

    // landmarks describer types filter
    if (describerTypesName.empty())
    {
        sfmData.getLandmarks().clear();
    }
    else
    {
        std::vector<feature::EImageDescriberType> imageDescriberTypes = feature::EImageDescriberType_stringToEnums(describerTypesName);

        std::vector<IndexT> toRemove;
        for (const auto& landmarkPair : sfmData.getLandmarks())
        {
            if (std::find(imageDescriberTypes.begin(), imageDescriberTypes.end(), landmarkPair.second.descType) == imageDescriberTypes.end())
                toRemove.push_back(landmarkPair.first);
        }
        for (const IndexT landmarkId : toRemove)
            sfmData.getLandmarks().erase(landmarkId);
    }
    // export the SfMData scene in the expected format
    if (!sfmDataIO::save(sfmData, outputSfMDataFilename, sfmDataIO::ESfMData(flags)))
    {
        ALICEVISION_LOG_ERROR("An error occured while trying to save '" << outputSfMDataFilename << "'");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
