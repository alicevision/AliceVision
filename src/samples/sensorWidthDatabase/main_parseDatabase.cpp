// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// Copyright (c) 2013 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/config.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/sensorDB/parseDatabase.hpp>

#include <boost/program_options.hpp>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

namespace po = boost::program_options;

int main(int argc, char ** argv)
{
  std::string sensorDatabasePath;
  std::string brandName;
  std::string modelName;

  po::options_description allParams("AliceVision Sample parseDatabase");
  allParams.add_options()
    ("sensorDatabase,s", po::value<std::string>(&sensorDatabasePath)->required(),
      "Camera sensor width database path.")
    ("brand,b", po::value<std::string>(&brandName)->required(),
      "Camera brand.")
    ("model,m", po::value<std::string>(&modelName)->required(),
      "Camera model.");

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

  std::vector<aliceVision::sensorDB::Datasheet> vec_database;
  aliceVision::sensorDB::Datasheet datasheet;

  if ( !aliceVision::sensorDB::parseDatabase( sensorDatabasePath, vec_database ) )
  {
    std::cout << "Database creation failure from the file : " << sensorDatabasePath  << std::endl;
    return EXIT_FAILURE;
  }

  if ( !aliceVision::sensorDB::getInfo( brandName, modelName, vec_database, datasheet ) )
  {
    ALICEVISION_LOG_ERROR("The camera " << modelName << " doesn't exist in the database.");
    return EXIT_FAILURE;
  }

  ALICEVISION_LOG_INFO("Result: " << std::endl
                       << datasheet._brand << "\t" << datasheet._model << "\t" << datasheet._sensorWidth);

  return EXIT_SUCCESS;
}

