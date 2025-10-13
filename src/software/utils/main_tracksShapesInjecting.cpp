// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/main.hpp>

#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

#include <aliceVision/track/Track.hpp>
#include <aliceVision/track/trackIO.hpp>
#include <aliceVision/track/TracksMerger.cpp>

#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/json.hpp>

#include <string>
#include <sstream>
#include <random>
#include <fstream>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;

bool readShapes(const std::string & filename, track::TracksMap & map)
{
    std::ifstream shapesFile(filename);
    if (shapesFile.is_open() == false)
    {
        return false;
    }

    // Parse json
    std::stringstream buffer;
    buffer << shapesFile.rdbuf();

    IndexT count = 0;
    
    try 
    {
        boost::json::value jv = boost::json::parse(buffer.str());

        // Iterate through the array
        for (const auto & item : jv.as_array()) 
        {
            boost::json::object obj = item.as_object();
            std::string type = boost::json::value_to<std::string>(obj.at("type"));
            
            boost::to_lower(type);

            if (type != "point2d")
            {
                ALICEVISION_LOG_TRACE("Unknown type " << type);
                continue;
            }   

            track::Track & track = map[count];
            track.descType = feature::EImageDescriberType::MANUAL;

            boost::json::object obs = obj.at("observations").as_object();
            for (const auto& [strViewId, value] : obs) 
            {
                IndexT viewId = std::stoul(std::string(strViewId));

                boost::json::object coords = value.as_object();
                double x = boost::json::value_to<double>(coords.at("x"));
                double y = boost::json::value_to<double>(coords.at("y"));

                track::TrackItem & item = track.featPerView[viewId];
                item.featureId = viewId;
                item.scale = 1.0;
                item.depth = -1.0;
                item.coords.x() = x;
                item.coords.y() = y;
            }

            count++;
        }
    }
    catch (const std::exception & e)
    {
        std::cout << e.what() << std::endl;
        return false;
    }
    
    return true;
}

int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string outputTracksFileName;
    std::string sfmDataFileName;
    std::string shapesFilename;
    bool markAsSpecial = false;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFileName)->required(),
         "SfmData used for sanity check.")
        ("shapesFile", po::value<std::string>(&shapesFilename)->required(),
         "Shapes to inject.")
        ("outputTracksFileName,o", po::value<std::string>(&outputTracksFileName)->required(),
         "Output Tracks file.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("markAsSpecial", po::value<bool>(&markAsSpecial)->default_value(markAsSpecial),
         "Consider these tracks as robust, precise measurements.");
    // clang-format on

    CmdLine cmdline("AliceVision tracksShapesInjecting");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // Load input scene
    sfmData::SfMData sfmData;
    if (!sfmDataIO::load(sfmData, sfmDataFileName, sfmDataIO::ESfMData::VIEWS))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFileName << "' cannot be read");
        return EXIT_FAILURE;
    }

    track::TracksMap mapTracks;
    if (!readShapes(shapesFilename, mapTracks))
    {
        ALICEVISION_LOG_ERROR("Can't read shapes to '" << shapesFilename << "'");
        return EXIT_FAILURE;
    }

    if (!saveTracks(mapTracks, outputTracksFileName))
    {
        ALICEVISION_LOG_ERROR("Can't save tracks to '" << outputTracksFileName << "'");
        return EXIT_FAILURE;
    }

    
    return EXIT_SUCCESS;
}
