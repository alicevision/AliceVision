// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/voctree/Database.hpp>
#include <aliceVision/voctree/databaseIO.hpp>
#include <aliceVision/voctree/VocabularyTree.hpp>
#include <aliceVision/voctree/descriptorLoader.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/main.hpp>

#include <boost/program_options.hpp> 
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/tail.hpp>

#include <Eigen/Core>

#include <iostream>
#include <fstream>
#include <ostream>
#include <string>
#include <chrono>
#include <iomanip>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

static const int DIMENSION = 128;

using namespace std;
using namespace boost::accumulators;
using namespace aliceVision;

namespace po = boost::program_options;

typedef aliceVision::feature::Descriptor<float, DIMENSION> DescriptorFloat;
typedef aliceVision::feature::Descriptor<unsigned char, DIMENSION> DescriptorUChar;

std::ostream& operator<<(std::ostream& os, const aliceVision::voctree::DocMatches &matches)
{
  os << "[ ";
  for(const auto &e : matches)
  {
    os << e.id << ", " << e.score << "; ";
  }
  os << "];\n";
  return os;
}

std::ostream& operator<<(std::ostream& os, const aliceVision::voctree::Document &doc)
{
  os << "[ ";
  for(const aliceVision::voctree::Word &w : doc)
  {
    os << w << ", ";
  }
  os << "];\n";
  return os;
}

std::string myToString(std::size_t i, std::size_t zeroPadding)
{
  stringstream ss;
  ss << std::setw(zeroPadding) << std::setfill('0') << i;
  return ss.str();
}

static const std::string programDescription =
        "This program is used to generate some statistics.\n ";

/*
 * This program is used to create a database with a provided dataset of image descriptors using a trained vocabulary tree
 * The database is then queried with the same images in order to retrieve for each image the set of most similar images in the dataset
 */
int aliceVision_main(int argc, char** argv)
{
  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string weightsName;                  // the filename for the voctree weights
  bool withWeights = false;            // flag for the optional weights file
  std::string treeName;                     // the filename of the voctree
  std::string sfmDataFilename;              // the file containing the list of features to use to build the database
  std::vector<std::string> featuresFolders;
  std::string querySfmDataFilename = "";    // the file containing the list of features to use as query
  std::string distance;

  po::options_description allParams("This program is used to create a database with a provided dataset of image descriptors using a trained vocabulary tree\n"
                                    "The database is then queried with the same images in order to retrieve for each image the set of most similar images in the dataset\n"
                                    "AliceVision voctreeStatistics");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(), "a SfMData file.")
    ("tree,t", po::value<std::string>(&treeName)->required(), "Input name for the tree file");

  po::options_description optionalParams("Optional parameters");
  optionalParams.add_options()
    ("weights,w", po::value<std::string>(&weightsName), "Input name for the weight file, if not provided the weights will be computed on the database built with the provided set")
    ("featuresFolders,f", po::value<std::vector<std::string>>(&featuresFolders)->multitoken(),
      "Path to folder(s) containing the extracted features.")
    ("querySfmDataFilename,q", po::value<std::string>(&querySfmDataFilename), "Path to the SfMData file to be used for querying the database")
    ("distance,d",po::value<std::string>(&distance)->default_value(""), "Method used to compute distance between histograms: \n "
                                                                          "-classic: eucledian distance \n"
                                                                          "-commonPoints: counts common points between histograms \n"
                                                                          "-strongCommonPoints: counts common 1 values \n"
                                                                          "-weightedStrongCommonPoints: strongCommonPoints with weights \n"
                                                                          "-inversedWeightedCommonPoints: strongCommonPoints with inverted weights");

  po::options_description logParams("Log parameters");
  logParams.add_options()
    ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
      "verbosity level (fatal, error, warning, info, debug, trace).");

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

  if(vm.count("weights"))
  {
    withWeights = true;
  }

  // load vocabulary tree
  ALICEVISION_LOG_INFO("Loading vocabulary tree\n");
  aliceVision::voctree::VocabularyTree<DescriptorFloat> tree(treeName);
  ALICEVISION_LOG_INFO("tree loaded with\n\t"
          << tree.levels() << " levels\n\t" 
          << tree.splits() << " branching factor");

  // load SfMData
  sfmData::SfMData sfmData;
  if(!sfmDataIO::Load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
  {
    ALICEVISION_LOG_ERROR("The input SfMData file '" + sfmDataFilename + "' cannot be read.");
    return EXIT_FAILURE;
  }

  // create the database
  ALICEVISION_LOG_INFO("Creating the database...");

  // add each object (document) to the database
  aliceVision::voctree::Database db(tree.words());

  if(withWeights)
  {
    ALICEVISION_LOG_INFO("Loading weights...");
    db.loadWeights(weightsName);
  }
  else
  {
    ALICEVISION_LOG_INFO("No weights specified, skipping...");
  }

  // read the descriptors and populate the database
  ALICEVISION_LOG_INFO("Reading descriptors from " << sfmDataFilename);
  auto detect_start = std::chrono::steady_clock::now();
  size_t numTotFeatures = aliceVision::voctree::populateDatabase<DescriptorUChar>(sfmData, featuresFolders, tree, db);
  auto detect_end = std::chrono::steady_clock::now();
  auto detect_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(detect_end - detect_start);

  if(numTotFeatures == 0)
  {
    ALICEVISION_LOG_INFO("No descriptors loaded!!");
    return EXIT_FAILURE;
  }

  ALICEVISION_LOG_INFO("Done! " << db.getSparseHistogramPerImage().size() << " sets of descriptors read for a total of " << numTotFeatures << " features");
  ALICEVISION_LOG_INFO("Reading took " << detect_elapsed.count() << " sec");

  if(!withWeights)
  {
    // compute and save the word weights
    ALICEVISION_LOG_INFO("Computing weights...");
    db.computeTfIdfWeights();
  }

  // query documents for Statistics
  std::map<int,int> globalHisto;

  ALICEVISION_LOG_INFO("Getting some stats for " << querySfmDataFilename);

  sfmData::SfMData querySfmData;
  if(!sfmDataIO::Load(querySfmData, querySfmDataFilename, sfmDataIO::ESfMData::ALL))
  {
    ALICEVISION_LOG_ERROR("The input SfMData file '" + querySfmDataFilename + "' cannot be read.");
    return EXIT_FAILURE;
  }
  
  aliceVision::voctree::voctreeStatistics<DescriptorUChar>(querySfmData, featuresFolders, tree, db, distance, globalHisto);
  
  std::cout << "-----------------" << std::endl;
  
  for(const auto &itHisto : globalHisto)
    std::cout << itHisto.first << ": " << itHisto.second  << ", ";

  std::cout << std::endl;
  
  return EXIT_SUCCESS;
}
