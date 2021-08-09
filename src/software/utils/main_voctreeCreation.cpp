// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/voctree/TreeBuilder.hpp>
#include <aliceVision/voctree/Database.hpp>
#include <aliceVision/voctree/VocabularyTree.hpp>
#include <aliceVision/voctree/descriptorLoader.hpp>
#include <aliceVision/feature/Descriptor.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/main.hpp>

#include <Eigen/Core>

#include <boost/program_options.hpp>

#include <iostream>
#include <fstream>
#include <string>
#include <chrono>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

static const int DIMENSION = 128;

using namespace std;
using namespace aliceVision;

//using namespace boost::accumulators;
namespace po = boost::program_options;

typedef aliceVision::feature::Descriptor<float, DIMENSION> DescriptorFloat;
typedef aliceVision::feature::Descriptor<unsigned char, DIMENSION> DescriptorUChar;

/*
 * This program is used to load the sift descriptors from a list of files and create a vocabulary tree
 */
int aliceVision_main(int argc, char** argv)
{
  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  int tbVerbosity = 2;
  std::string weightName;
  std::string treeName;
  std::string sfmDataFilename;
  std::vector<std::string> featuresFolders;
  std::uint32_t K = 10;
  std::uint32_t restart = 5;
  std::uint32_t LEVELS = 6;
  bool sanityCheck = true;

  po::options_description allParams("This program is used to load the sift descriptors from a SfMData file and create a vocabulary tree\n"
                                    "It takes as input either a list.txt file containing the a simple list of images (bundler format and older AliceVision version format)\n"
                                    "or a sfm_data file (JSON) containing the list of images. In both cases it is assumed that the .desc to load are in the same folder as the input file\n"
                                    "AliceVision voctreeCreation");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(), "a SfMData file.")
    ("weights,w", po::value<string>(&weightName)->required(), "Output name for the weight file")
    ("tree,t", po::value<string>(&treeName)->required(), "Output name for the tree file");

  po::options_description optionalParams("Optional parameters");
  optionalParams.add_options()
    ("featuresFolders,f", po::value<std::vector<std::string>>(&featuresFolders)->multitoken(),
      "Path to folder(s) containing the extracted features.")
    (",k", po::value<uint32_t>(&K)->default_value(10), "The branching factor of the tree")
    ("restart,r", po::value<uint32_t>(&restart)->default_value(5), "Number of times that the kmean is launched for each cluster, the best solution is kept")
    (",L", po::value<uint32_t>(&LEVELS)->default_value(6), "Number of levels of the tree")
    ("sanitycheck,s", po::value<bool>(&sanityCheck)->default_value(sanityCheck), "Perform a sanity check at the end of the creation of the vocabulary tree. The sanity check is a query to the database with the same documents/images useed to train the vocabulary tree");

  po::options_description logParams("Log parameters");
  logParams.add_options()
    ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
      "verbosity level (fatal, error, warning, info, debug, trace).")
    ("tbVerbose", po::value<int>(&tbVerbosity)->default_value(tbVerbosity), "Tree builder verbosity level, 3 should be just enough, 0 to mute");

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

  // load SfMData
  sfmData::SfMData sfmData;
  if(!sfmDataIO::Load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
  {
    ALICEVISION_LOG_ERROR("The input SfMData file '" + sfmDataFilename + "' cannot be read.");
    return EXIT_FAILURE;
  }

  std::vector<DescriptorFloat> descriptors;

  std::vector<size_t> descRead;
  ALICEVISION_COUT("Reading descriptors from " << sfmDataFilename);
  auto detect_start = std::chrono::steady_clock::now();
  size_t numTotDescriptors = aliceVision::voctree::readDescFromFiles<DescriptorFloat, DescriptorUChar>(sfmData, featuresFolders, descriptors, descRead);
  auto detect_end = std::chrono::steady_clock::now();
  auto detect_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(detect_end - detect_start);
  if(descriptors.empty())
  {
    ALICEVISION_CERR("No descriptors loaded!!");
    return EXIT_FAILURE;
  }

  ALICEVISION_COUT("Done! " << descRead.size() << " sets of descriptors read for a total of " << numTotDescriptors << " features");
  ALICEVISION_COUT("Reading took " << detect_elapsed.count() << " sec");

  // Create tree
  aliceVision::voctree::TreeBuilder<DescriptorFloat> builder(DescriptorFloat(0));
  builder.setVerbose(tbVerbosity);
  builder.kmeans().setRestarts(restart);
  ALICEVISION_COUT("Building a tree of L=" << LEVELS << " levels with a branching factor of k=" << K);
  detect_start = std::chrono::steady_clock::now();
  builder.build(descriptors, K, LEVELS);
  detect_end = std::chrono::steady_clock::now();
  detect_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(detect_end - detect_start);
  ALICEVISION_COUT("Tree created in " << ((float) detect_elapsed.count()) / 1000 << " sec");
  ALICEVISION_COUT(builder.tree().centers().size() << " centers");
  ALICEVISION_COUT("Saving vocabulary tree as " << treeName);
  builder.tree().save(treeName);

  aliceVision::voctree::SparseHistogramPerImage allSparseHistograms;
  // temporary vector used to save all the visual word for each image before adding them to documents
  std::vector<aliceVision::voctree::Word> imgVisualWords;
  ALICEVISION_COUT("Quantizing the features");
  size_t offset = 0; ///< this is used to align to the features of a given image in 'feature'
  detect_start = std::chrono::steady_clock::now();
  // pass each feature through the vocabulary tree to get the associated visual word
  // for each read images, recover the number of features in it from descRead and loop over the features
  for(size_t i = 0; i < descRead.size(); ++i)
  {
    // for each image:
    // clear the temporary vector used to save all the visual word and allocate the proper size
    imgVisualWords.clear();
    // allocate as many visual words as the number of the features in the image
    imgVisualWords.resize(descRead[i], 0);

    #pragma omp parallel for
    for(ptrdiff_t j = 0; j < static_cast<ptrdiff_t>(descRead[i]); ++j)
    {
      //	store the visual word associated to the feature in the temporary list
      imgVisualWords[j] = builder.tree().quantize(descriptors[ j + offset ]);
    }
    aliceVision::voctree::SparseHistogram histo;
    aliceVision::voctree::computeSparseHistogram(imgVisualWords, histo);
    // add the vector to the documents
    allSparseHistograms[i] = histo;

    // update the offset
    offset += descRead[i];
  }
  detect_end = std::chrono::steady_clock::now();
  detect_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(detect_end - detect_start);
  ALICEVISION_COUT("Feature quantization took " << detect_elapsed.count() << " sec");


  ALICEVISION_COUT("Creating the database...");
  // Add each object (document) to the database
  aliceVision::voctree::Database db(builder.tree().words());
  ALICEVISION_COUT("\tfound " << allSparseHistograms.size() << " documents");
  for(const auto &doc : allSparseHistograms)
  {
    db.insert(doc.first, doc.second);
  }
  ALICEVISION_COUT("Database created!");

  // Compute and save the word weights
  ALICEVISION_COUT("Computing weights...");
  detect_start = std::chrono::steady_clock::now();
  db.computeTfIdfWeights();
  detect_end = std::chrono::steady_clock::now();
  detect_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(detect_end - detect_start);
  ALICEVISION_COUT("Computing weights done in " << detect_elapsed.count() << " sec");
  ALICEVISION_COUT("Saving weights as " << weightName);
  db.saveWeights(weightName);


  if(sanityCheck)
  {
    // Now query each document (sanity check)
    std::vector<aliceVision::voctree::DocMatch> matches;
    size_t wrong = 0; // count the wrong matches
    double recval = 0.0;
    ALICEVISION_COUT("Sanity check: querying the database with the same documents");
    // for each document
    for(const auto &doc : allSparseHistograms)
    {
      detect_start = std::chrono::steady_clock::now();
      // retrieve the best 4 matches
      db.find(doc.second, 4, matches);
      detect_end = std::chrono::steady_clock::now();
      detect_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(detect_end - detect_start);
      ALICEVISION_COUT("query document " << doc.first 
			  << " took " << detect_elapsed.count()
			  << " ms and has " << matches.size() 
			  << " matches\tBest " << matches[0].id 
			  << " with score " << matches[0].score << endl);
      // for each found match print the score, ideally the first one should be the document itself
      for(size_t j = 0; j < matches.size(); ++j)
      {
        ALICEVISION_COUT("\t match " << matches[j].id << " with score " << matches[j].score);
        if(matches[j].id / 4 == (doc.first) / 4) recval += 1;
      }

      // if the first one is not the document itself notify and increment the counter
      if(doc.first != matches[0].id)
      {
        ++wrong;
        ALICEVISION_COUT("##### wrong match for document " << doc.first);
      }

    }

    if(wrong)
    {
      ALICEVISION_COUT("there are " << wrong << " wrong matches");
    }
    else
    {
      ALICEVISION_COUT("Yay! no wrong matches!");
    }
    ALICEVISION_COUT("recval: " << recval / (double) (allSparseHistograms.size()));
  }

  return EXIT_SUCCESS;
}
