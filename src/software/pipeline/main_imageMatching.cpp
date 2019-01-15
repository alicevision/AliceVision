// This file is part of the AliceVision project.
// Copyright (c) 2015 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/voctree/Database.hpp>
#include <aliceVision/voctree/VocabularyTree.hpp>
#include <aliceVision/voctree/databaseIO.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/config.hpp>

#include <Eigen/Core>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <iostream>
#include <fstream>
#include <ostream>
#include <string>
#include <set>
#include <chrono>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

static const int DIMENSION = 128;

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

typedef aliceVision::feature::Descriptor<float, DIMENSION> DescriptorFloat;
typedef aliceVision::feature::Descriptor<unsigned char, DIMENSION> DescriptorUChar;

typedef std::size_t ImageID;

using aliceVision::IndexT;

// just a list of doc id
typedef std::vector<ImageID> ListOfImageID;

// An ordered and unique list of doc id
typedef std::set<ImageID> OrderedListOfImageID;

// For each image ID it contains the  list of matching imagess
typedef std::map<ImageID, ListOfImageID> PairList;

// For each image ID it contains the ordered list of matching images
typedef std::map<ImageID, OrderedListOfImageID> OrderedPairList;

/**
 * @brief Function that prints a PairList
 * @param os The stream on which to print
 * @param pl The pair list
 * @return the stream
 */
std::ostream& operator<<(std::ostream& os, const PairList & pl)
{
  for(PairList::const_iterator plIter = pl.begin(); plIter != pl.end(); ++plIter)
  {
    os << plIter->first;
    for(ImageID id : plIter->second)
    {
      os << " " << id;
    }
    os << "\n";
  }
  return os;
}

/**
 * @brief Function that prints a OrderedPairList
 * @param os The stream on which to print
 * @param pl The pair list
 * @return the stream
 */
std::ostream& operator<<(std::ostream& os, const OrderedPairList & pl)
{
  for(OrderedPairList::const_iterator plIter = pl.begin(); plIter != pl.end(); ++plIter)
  {
    os << plIter->first;
    for(ImageID id : plIter->second)
    {
      os << " " << id;
    }
    os << "\n";
  }
  return os;
}

/**
 * @brief Mode to combine image matching between two SfMDatas
 */
enum class EImageMatchingMode
{
  A_A_AND_A_B,
  A_AB,
  A_B,
  A_A
};


/**
 * @brief get informations about each EImageMatchingMode
 * @return String
 */
std::string EImageMatchingMode_description()
{
  return "The mode to combine image matching between the input SfMData A and B: \n"
             "* a/a+a/b : A with A + A with B\n"
             "* a/ab    : A with A and B\n"
             "* a/b     : A with B\n"
             "* a/a     : A with A";
}

/**
 * @brief convert an enum EImageMatchingMode to its corresponding string
 * @param modeMultiSfM
 * @return String
 */
std::string EImageMatchingMode_enumToString(EImageMatchingMode modeMultiSfM)
{
  switch(modeMultiSfM)
  {
    case EImageMatchingMode::A_A_AND_A_B: return "a/a+a/b";
    case EImageMatchingMode::A_AB:        return "a/ab";
    case EImageMatchingMode::A_B:         return "a/b";
    case EImageMatchingMode::A_A:         return "a/a";
  }
  throw std::out_of_range("Invalid modeMultiSfM enum");
}

/**
 * @brief convert a string modeMultiSfM to its corresponding enum modeMultiSfM
 * @param String
 * @return EImageMatchingMode
 */
 EImageMatchingMode EImageMatchingMode_stringToEnum(const std::string& modeMultiSfM)
{
  std::string mode = modeMultiSfM;
  std::transform(mode.begin(), mode.end(), mode.begin(), ::tolower); //tolower

  if(mode == "a/a+a/b") return EImageMatchingMode::A_A_AND_A_B;
  if(mode == "a/ab")    return EImageMatchingMode::A_AB;
  if(mode == "a/b")     return EImageMatchingMode::A_B;
  if(mode == "a/a")     return EImageMatchingMode::A_A;

  throw std::out_of_range("Invalid modeMultiSfM : " + modeMultiSfM);
}

/**
 * It processes a pairlist containing all the matching images for each image ID and return
 * a similar list limited to a numMatches number of matching images and such that
 * there is no repetitions: eg if the image 1 matches with 2 in the list of image 2
 * there won't be the image 1
 *
 * @param[in] allMatches A pairlist containing all the matching images for each image of the dataset
 * @param[in] numMatches The maximum number of matching images to consider for each image (if 0, consider all matches)
 * @param[out] matches A processed version of allMatches that consider only the first numMatches without repetitions
 */
void convertAllMatchesToPairList(const PairList &allMatches, std::size_t numMatches, OrderedPairList &outPairList)
{
  outPairList.clear();

  if(numMatches == 0)
    numMatches = allMatches.size();  // disable image matching limit

  for(const auto& match : allMatches)
  {
    ImageID currImageId = match.first;
    OrderedListOfImageID bestMatches;

    for(const ImageID currMatchId : match.second)
    {
      // avoid self-matching
      if(currMatchId == currImageId)
        continue;

      // if the currMatchId ID is lower than the current image ID and
      // the current image ID is not already in the list of currMatchId
      //BOOST_ASSERT( ( currMatchId < currImageId ) && ( outPairList.find( currMatchId ) != outPairList.end() ) );
      if(currMatchId < currImageId)
      {
        OrderedPairList::const_iterator currMatches = outPairList.find(currMatchId);
        if(currMatches != outPairList.end() &&
                currMatches->second.find(currImageId) == currMatches->second.end())
        {
          // then add it to the list
          bestMatches.insert(currMatchId);
        }
      }
      else
      {
        bestMatches.insert(currMatchId);
      }

      // stop if numMatches is satisfied
      if(bestMatches.size() == numMatches)
        break;
    }

    // fill the output if we have matches
    if(!bestMatches.empty())
      outPairList[currImageId] = bestMatches;
  }
}

void generateAllMatchesInOneMap(const std::map<IndexT, std::string>& descriptorsFiles, OrderedPairList& outPairList)
{
  for(const auto& descItA: descriptorsFiles)
  {
    const IndexT imgA = descItA.first;
    OrderedListOfImageID outPerImg;

    for(const auto& descItB: descriptorsFiles)
    {
      const IndexT imgB = descItB.first;
      if(imgB > imgA)
        outPerImg.insert(imgB);
    }

    if(!outPerImg.empty())
    {
      OrderedPairList::iterator itFind = outPairList.find(imgA);

      if(itFind == outPairList.end())
        outPairList[imgA] = outPerImg;
      else
        itFind->second.insert(outPerImg.begin(), outPerImg.end());
    }
  }
}

void generateAllMatchesBetweenTwoMap(const std::map<IndexT, std::string>& descriptorsFilesA, const std::map<IndexT, std::string>& descriptorsFilesB, OrderedPairList& outPairList)
{
  for(const auto& descItA: descriptorsFilesA)
  {
    const IndexT imgA = descItA.first;
    OrderedListOfImageID outPerImg;

    for(const auto& descItB: descriptorsFilesB)
      outPerImg.insert(descItB.first);

    if(!outPerImg.empty())
    {
      OrderedPairList::iterator itFind = outPairList.find(imgA);

      if(itFind == outPairList.end())
        outPairList[imgA] = outPerImg;
      else
        itFind->second.insert(outPerImg.begin(), outPerImg.end());
    }
  }
}


void generateFromVoctree(PairList& allMatches,
                         const std::map<IndexT, std::string>& descriptorsFiles,
                         const aliceVision::voctree::Database& db,
                         const aliceVision::voctree::VocabularyTree<DescriptorFloat>& tree,
                         EImageMatchingMode modeMultiSfM,
                         std::size_t nbMaxDescriptors,
                         std::size_t numImageQuery)
{
  ALICEVISION_LOG_INFO("Generate matches in mode: " + EImageMatchingMode_enumToString(modeMultiSfM));

  if(numImageQuery == 0)
  {
    // if 0 retrieve the score for all the documents of the database
    numImageQuery = db.size();
  }

  //initialize allMatches

  for(const auto& descriptorPair : descriptorsFiles)
  {
    if(allMatches.find(descriptorPair.first) == allMatches.end())
      allMatches[descriptorPair.first] = {};
  }

  // query each document
  #pragma omp parallel for
  for(ptrdiff_t i = 0; i < static_cast<ptrdiff_t>(descriptorsFiles.size()); ++i)
  {
    auto itA = descriptorsFiles.cbegin();
    std::advance(itA, i);
    const IndexT viewIdA = itA->first;
    const std::string featuresPathA = itA->second;

    aliceVision::voctree::SparseHistogram imageSH;

    if(modeMultiSfM != EImageMatchingMode::A_B)
    {
      // sparse histogram of A is already computed in the DB
      imageSH = db.getSparseHistogramPerImage().at(viewIdA);
    }
    else // mode AB
    {
      // compute the sparse histogram of each image A
      std::vector<DescriptorUChar> descriptors;
      // read the descriptors
      loadDescsFromBinFile(featuresPathA, descriptors, false, nbMaxDescriptors);
      imageSH = tree.quantizeToSparse(descriptors);
    }

    std::vector<aliceVision::voctree::DocMatch> matches;

    db.find(imageSH, numImageQuery, matches);

    ListOfImageID& imgMatches = allMatches.at(viewIdA);
    imgMatches.reserve(imgMatches.size() + matches.size());

    for(const aliceVision::voctree::DocMatch& m : matches)
    {
      imgMatches.push_back(m.id);
    }
  }
}

int main(int argc, char** argv)
{
  // command-line parameters

  /// verbosity level
  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  /// the file containing a list of features
  std::string sfmDataFilenameA;
  /// the folder(s) containing the extracted features with their associated descriptors
  std::vector<std::string> featuresFolders;
  /// the filename of the voctree
  std::string treeName;
  /// the file in which to save the results
  std::string outputFile;

  // user optional parameters

  /// minimal number of images to use the vocabulary tree
  std::size_t minNbImages = 200;
  /// the file containing the list of features
  std::size_t nbMaxDescriptors = 500;
  /// the number of matches to retrieve for each image
  std::size_t numImageQuery = 50;
  /// the filename for the voctree weights
  std::string weightsName;
  /// flag for the optional weights file
  bool withWeights = false;

  // multiple SfM parameters

  /// a second file containing a list of features
  std::string sfmDataFilenameB;
  /// the multiple SfM mode
  std::string matchingModeName = EImageMatchingMode_enumToString(EImageMatchingMode::A_A);
  /// the combine SfM output
  std::string outputCombinedSfM;

  po::options_description allParams(
    "The objective of this software is to find images that are looking to the same areas of the scene. "
    "For that, we use the image retrieval techniques to find images that share content without "
    "the cost of resolving all feature matches in detail. The ambition is to simplify the image in "
    "a compact image descriptor which allows to compute the distance between all images descriptors efficiently.\n"
    "This program generates a pair list file to be passed to the aliceVision_featureMatching software. "
    "This file contains for each image the list of most similar images.\n"
    "AliceVision featureMatching");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilenameA)->required(),
      "SfMData file.")
    ("featuresFolders,f", po::value<std::vector<std::string>>(&featuresFolders)->multitoken()->required(),
      "Path to folder(s) containing the extracted features.")
    ("tree,t", po::value<std::string>(&treeName),
      "Input file path of the vocabulary tree. This file can be generated by createVoctree. "
      "This software is intended to be used with a generic, pre-trained vocabulary tree.")
    ("output,o", po::value<std::string>(&outputFile)->required(),
      "Filepath to the output file with the list of selected image pairs.");

  po::options_description optionalParams("Optional parameters");
  optionalParams.add_options()
    ("minNbImages", po::value<std::size_t>(&minNbImages)->default_value(minNbImages),
      "Minimal number of images to use the vocabulary tree. If we have less images than this threshold, we will compute all matching combinations.")
    ("maxDescriptors", po::value<std::size_t>(&nbMaxDescriptors)->default_value(nbMaxDescriptors),
      "Limit the number of descriptors you load per image. Zero means no limit.")
    ("nbMatches", po::value<std::size_t>(&numImageQuery)->default_value(numImageQuery),
      "The number of matches to retrieve for each image (If 0 it will "
      "retrieve all the matches).")
    ("weights,w", po::value<std::string>(&weightsName),
      "Input name for the vocabulary tree weight file, if not provided all voctree leaves will have the same weight.");

  po::options_description multiSfMParams("Multiple SfM");
  multiSfMParams.add_options()
      ("inputB", po::value<std::string>(&sfmDataFilenameB),
        "SfMData file.")
      ("matchingMode", po::value<std::string>(&matchingModeName)->default_value(matchingModeName),
        EImageMatchingMode_description().c_str())
      ("outputCombinedSfM", po::value<std::string>(&outputCombinedSfM)->default_value(outputCombinedSfM),
        "Output file path for the combined SfMData file (if empty, don't combine).");

  po::options_description logParams("Log parameters");
  logParams.add_options()
      ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
        "verbosity level (fatal, error, warning, info, debug, trace).");

  allParams.add(requiredParams).add(optionalParams).add(multiSfMParams).add(logParams);

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

  // multiple SfM
  const bool useMultiSfM = !sfmDataFilenameB.empty();
  const EImageMatchingMode matchingMode = EImageMatchingMode_stringToEnum(matchingModeName);

  if(useMultiSfM == (matchingMode == EImageMatchingMode::A_A))
  {
    ALICEVISION_LOG_ERROR("The number of SfMData inputs is not compatible with the selected mode.");
    return EXIT_FAILURE;
  }

  // load SfMData
  sfmData::SfMData sfmDataA, sfmDataB;

  if(!sfmDataIO::Load(sfmDataA, sfmDataFilenameA, sfmDataIO::ESfMData::ALL))
  {
    ALICEVISION_LOG_ERROR("The input SfMData file '" + sfmDataFilenameA + "' cannot be read.");
    return EXIT_FAILURE;
  }

  if(useMultiSfM)
  {
    if(!sfmDataIO::Load(sfmDataB, sfmDataFilenameB, sfmDataIO::ESfMData::ALL))
    {
      ALICEVISION_LOG_ERROR("The input SfMData file '" + sfmDataFilenameB + "' cannot be read.");
      return EXIT_FAILURE;
    }

    // remove duplicated view
    for(const auto& viewPair : sfmDataB.getViews())
    {
      sfmData::Views::iterator it = sfmDataA.getViews().find(viewPair.first);
      if(it != sfmDataA.getViews().end())
        sfmDataA.getViews().erase(it);
    }
  }

  OrderedPairList selectedPairs;

  std::map<IndexT, std::string> descriptorsFilesA, descriptorsFilesB;

  // load descriptor filenames
  aliceVision::voctree::getListOfDescriptorFiles(sfmDataA, featuresFolders, descriptorsFilesA);

  if(useMultiSfM)
    aliceVision::voctree::getListOfDescriptorFiles(sfmDataB, featuresFolders, descriptorsFilesB);

  if(treeName.empty() && (descriptorsFilesA.size() + descriptorsFilesB.size()) > 200)
    ALICEVISION_LOG_WARNING("No vocabulary tree argument, so it will use the brute force approach which can be compute intensive for aliceVision_featureMatching.");

  if(treeName.empty() || (descriptorsFilesA.size() + descriptorsFilesB.size()) < minNbImages)
  {
    ALICEVISION_LOG_INFO("Brute force generation");

    if((matchingMode == EImageMatchingMode::A_A_AND_A_B) ||
       (matchingMode == EImageMatchingMode::A_AB) ||
       (matchingMode == EImageMatchingMode::A_A))
      generateAllMatchesInOneMap(descriptorsFilesA, selectedPairs);

    if((matchingMode == EImageMatchingMode::A_A_AND_A_B) ||
       (matchingMode == EImageMatchingMode::A_AB) ||
       (matchingMode == EImageMatchingMode::A_B))
      generateAllMatchesBetweenTwoMap(descriptorsFilesA, descriptorsFilesB, selectedPairs);
  }

  // if selectedPairs is not already computed by a brute force approach,
  // we compute it with the vocabulary tree approach.
  if(selectedPairs.empty())
  {
    // load vocabulary tree
    ALICEVISION_LOG_INFO("Loading vocabulary tree");

    auto loadVoctree_start = std::chrono::steady_clock::now();
    aliceVision::voctree::VocabularyTree<DescriptorFloat> tree(treeName);
    auto loadVoctree_elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - loadVoctree_start);
    {
      std::stringstream ss;
      ss << "tree loaded with:" << std::endl << "\t- " << tree.levels() << " levels" << std::endl;
      ss << "\t- " << tree.splits() << " branching factor" << std::endl;
      ss << "\tin " << loadVoctree_elapsed.count() << " seconds" << std::endl;
      ALICEVISION_LOG_INFO(ss.str());
    }

    // create the databases
    ALICEVISION_LOG_INFO("Creating the databases...");

    // add each object (document) to the database
    aliceVision::voctree::Database db(tree.words());
    aliceVision::voctree::Database db2;

    if(withWeights)
    {
      ALICEVISION_LOG_INFO("Loading weights...");
      db.loadWeights(weightsName);
    }
    else
    {
      ALICEVISION_LOG_INFO("No weights specified, skipping...");
    }

    if(matchingMode == EImageMatchingMode::A_A_AND_A_B)
      db2 = db; // initialize database2 with database1 initialization

    // read the descriptors and populate the databases
    {
      std::stringstream ss;

      for(const std::string& featuresFolder : featuresFolders)
        ss << "\t- " << featuresFolder << std::endl;

      ALICEVISION_LOG_INFO("Reading descriptors from: " << std::endl << ss.str());

      std::size_t nbFeaturesLoadedInputA = 0;
      std::size_t nbFeaturesLoadedInputB = 0;
      std::size_t nbSetDescriptors = 0;

      auto detect_start = std::chrono::steady_clock::now();
      {

        if((matchingMode == EImageMatchingMode::A_A_AND_A_B) ||
           (matchingMode == EImageMatchingMode::A_AB) ||
           (matchingMode == EImageMatchingMode::A_A))
        {
          nbFeaturesLoadedInputA = voctree::populateDatabase<DescriptorUChar>(sfmDataA, featuresFolders, tree, db, nbMaxDescriptors);
          nbSetDescriptors = db.getSparseHistogramPerImage().size();

          if(nbFeaturesLoadedInputA == 0)
          {
            ALICEVISION_LOG_ERROR("No descriptors loaded in '" + sfmDataFilenameA + "'");
            return EXIT_FAILURE;
          }
        }

        if((matchingMode == EImageMatchingMode::A_AB) ||
           (matchingMode == EImageMatchingMode::A_B))
        {
          nbFeaturesLoadedInputB = voctree::populateDatabase<DescriptorUChar>(sfmDataB, featuresFolders, tree, db, nbMaxDescriptors);
          nbSetDescriptors = db.getSparseHistogramPerImage().size();
        }

        if(matchingMode == EImageMatchingMode::A_A_AND_A_B)
        {
          nbFeaturesLoadedInputB = voctree::populateDatabase<DescriptorUChar>(sfmDataB, featuresFolders, tree, db2, nbMaxDescriptors);
          nbSetDescriptors += db2.getSparseHistogramPerImage().size();
        }

        if(useMultiSfM && (nbFeaturesLoadedInputB == 0))
        {
          ALICEVISION_LOG_ERROR("No descriptors loaded in '" + sfmDataFilenameB + "'");
          return EXIT_FAILURE;
        }
      }

      auto detect_elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - detect_start);

      ALICEVISION_LOG_INFO("Read " << nbSetDescriptors << " sets of descriptors for a total of " << (nbFeaturesLoadedInputA + nbFeaturesLoadedInputB) << " features");
      ALICEVISION_LOG_INFO("Reading took " << detect_elapsed.count() << " sec.");
    }

    if(!withWeights)
    {
      // compute and save the word weights
      ALICEVISION_LOG_INFO("Computing weights...");

      db.computeTfIdfWeights();

      if(matchingMode == EImageMatchingMode::A_A_AND_A_B)
        db2.computeTfIdfWeights();
    }

    {
      PairList allMatches;

      ALICEVISION_LOG_INFO("Query all documents");

      auto detect_start = std::chrono::steady_clock::now();

      if(matchingMode == EImageMatchingMode::A_A_AND_A_B)
      {
        generateFromVoctree(allMatches, descriptorsFilesA, db,  tree, EImageMatchingMode::A_A, nbMaxDescriptors, numImageQuery);
        generateFromVoctree(allMatches, descriptorsFilesA, db2, tree, EImageMatchingMode::A_B, nbMaxDescriptors, numImageQuery);
      }
      else
      {
        generateFromVoctree(allMatches, descriptorsFilesA, db, tree, matchingMode,  nbMaxDescriptors, numImageQuery);
      }

      auto detect_elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - detect_start);
      ALICEVISION_LOG_INFO("Query all documents took " << detect_elapsed.count() << " sec.");

      // process pair list
      detect_start = std::chrono::steady_clock::now();

      ALICEVISION_LOG_INFO("Convert all matches to pairList");
      convertAllMatchesToPairList(allMatches, numImageQuery, selectedPairs);
      detect_elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - detect_start);
      ALICEVISION_LOG_INFO("Convert all matches to pairList took " << detect_elapsed.count() << " sec.");
    }
  }

  // check if the output folder exists
  const auto basePath = fs::path(outputFile).parent_path();
  if(!basePath.empty() && !fs::exists(basePath))
  {
    // then create the missing folder
    if(!fs::create_directories(basePath))
    {
      ALICEVISION_LOG_ERROR("Unable to create folders: " << basePath);
      return EXIT_FAILURE;
    }
  }

  // write it to file
  std::ofstream fileout;
  fileout.open(outputFile, std::ofstream::out);
  fileout << selectedPairs;
  fileout.close();

  ALICEVISION_LOG_INFO("pairList exported in: " << outputFile);

  if(useMultiSfM && !outputCombinedSfM.empty())
  {
    // combine A to B
    // should not loose B data
    sfmDataB.combine(sfmDataA);

    if(!sfmDataIO::Save(sfmDataB, outputCombinedSfM, sfmDataIO::ESfMData::ALL))
    {
      ALICEVISION_LOG_ERROR("Unable to save combined SfM: " << outputCombinedSfM);
      return EXIT_FAILURE;
    }
  }

  return EXIT_SUCCESS;
}
