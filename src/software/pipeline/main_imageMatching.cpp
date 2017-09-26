// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/voctree/Database.hpp>
#include <aliceVision/voctree/VocabularyTree.hpp>
#include <aliceVision/voctree/databaseIO.hpp>
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

static const int DIMENSION = 128;

using namespace std;
namespace po = boost::program_options;
namespace bfs = boost::filesystem;


typedef aliceVision::feature::Descriptor<float, DIMENSION> DescriptorFloat;
typedef aliceVision::feature::Descriptor<unsigned char, DIMENSION> DescriptorUChar;

typedef std::size_t ImageID;

// just a list of doc id
typedef std::vector< ImageID > ListOfImageID;

// An ordered and unique list of doc id
typedef std::set< ImageID > OrderedListOfImageID;

// For each image ID it contains the  list of matching images
typedef std::map< ImageID, ListOfImageID> PairList;

// For each image ID it contains the ordered list of matching images
typedef std::map< ImageID, OrderedListOfImageID> OrderedPairList;

/**
 * Function that prints a PairList
 *
 * @param os The stream on which to print
 * @param pl The pair list
 * @return the stream
 */
std::ostream& operator<<(std::ostream& os, const PairList & pl)
{
  for(PairList::const_iterator plIter = pl.begin(); plIter != pl.end(); ++plIter)
  {
    os << plIter->first << " ";
    for(ImageID id : plIter->second)
    {
      os << id << " ";
    }
    os << "\n";
  }
  return os;
}

/**
 * Function that prints a OrderedPairList
 *
 * @param os The stream on which to print
 * @param pl The pair list
 * @return the stream
 */
std::ostream& operator<<(std::ostream& os, const OrderedPairList & pl)
{
  for(OrderedPairList::const_iterator plIter = pl.begin(); plIter != pl.end(); ++plIter)
  {
    os << plIter->first << " ";
    for(ImageID id : plIter->second)
    {
      os << id << " ";
    }
    os << "\n";
  }
  return os;
}

/**
 * It processes a pairlist containing all the matching images for each image ID and return
 * a similar list limited to a numMatches number of matching images and such that
 * there is no repetitions: eg if the image 1 matches with 2 in the list of image 2
 * there won't be the image 1
 *
 * @param[in] allMatches A pairlist containing all the matching images for each image of the dataset
 * @param[in] numMatches The maximum number of matching images to consider for each image
 * @param[out] matches A processed version of allMatches that consider only the first numMatches without repetitions
 */
void convertAllMatchesToPairList(const PairList &allMatches, const std::size_t numMatches, OrderedPairList &outPairList)
{
  outPairList.clear();

  PairList::const_iterator allIter = allMatches.begin();
  if(allIter == allMatches.end())
    return;

  // For the first image, just copy the numMatches first numMatches
  {
    ImageID currImageId = allIter->first;
    OrderedListOfImageID& bestMatches = outPairList[ currImageId ] = OrderedListOfImageID();
    for(std::size_t i = 0; i < std::min(allIter->second.size(), numMatches); ++i)
    {
      // avoid self-matching
      if(allIter->second[i] == currImageId)
        continue;

      bestMatches.insert(allIter->second[i]);
    }
    ++allIter;
  }

  // All other images
  for(; allIter != allMatches.end(); ++allIter)
  {
    ImageID currImageId = allIter->first;

    OrderedListOfImageID bestMatches;

    std::size_t numFound = 0;

    // otherwise check each element
    for(std::size_t i = 0; i < allIter->second.size(); ++i)
    {
      ImageID currMatchId = allIter->second[i];

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
          ++numFound;
        }
      }
      else if(currMatchId > currImageId)
      {
        // then add it to the list
        bestMatches.insert(currMatchId);
        ++numFound;
      }

      // if we are done stop
      if(numFound == numMatches)
        break;
    }

    // fill the output if we have matches
    if(!bestMatches.empty())
      outPairList[ currImageId ] = bestMatches;
  }
}

int main(int argc, char** argv)
{
  using namespace aliceVision;
  namespace po = boost::program_options;

  // command-line parameters

  /// verbosity level
  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  /// the file containing the list of features
  std::string sfmDataFilename;
  /// the filename of the voctree
  std::string treeName;
  /// the file in which to save the results
  std::string outputFile;

  // user optional parameters

  /// the file containing the list of features
  std::size_t nbMaxDescriptors = 500;
  /// the number of matches to retrieve for each image
  std::size_t numImageQuery = 50;
  /// the filename for the voctree weights
  std::string weightsName;
  /// flag for the optional weights file
  bool withWeights = false;

  po::options_description allParams(
    "This program is used to generate the pair list file to be passed to AliceVision\n"
    "The pair list is a file containing, for each image of a dataset, the possible\n"
    "matching images in the dataset.\n"
    "The possible matching images are obtained in this case by querying a database\n"
    "and retrieving the list of the most similar images for each image in the dataset.\n"
    "Normally this works using a generic, pre-trained vocabulary tree, then a database\n"
    "is built with it using all the images of the dataset. Finally, each image of the \n"
    "dataset is queried in the database and the results are kept to form the pair list\n"
    "file.\n\n"
    "It takes as input either a list.txt file containing the a simple list of images (bundler\n"
    "format and older AliceVision version format) or a sfm_data file (JSON) containing the\n"
    "list of images. In both cases it is assumed that the .desc to load are in the same\n"
    "directory as the input file.\n"
    "Alternatively, it also can take as input the path to a directory containing the \n"
    "descriptor files.\n"
    "For the vocabulary tree, it takes as input the input.tree (and the input.weight)\n"
    "file generated by createVoctree\n"
    "AliceVision imageMatching");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
      "SfMData file or filepath to a simple text file "
      "with one image filepath per line, or path to the descriptors folder.")
    ("tree,t", po::value<std::string>(&treeName)->required(),
      "Input name for the vocabulary tree file.")
    ("output,o", po::value<std::string>(&outputFile)->required(),
      "Filepath to the output file with the list of selected image pairs.");


  po::options_description optionalParams("Optional parameters");
  optionalParams.add_options()
    ("maxDescriptors", po::value<std::size_t>(&nbMaxDescriptors)->default_value(nbMaxDescriptors),
      "Limit the number of descriptors you load per image. Zero means no limit.")
    ("nbMatches", po::value<std::size_t>(&numImageQuery)->default_value(numImageQuery),
      "The number of matches to retrieve for each image (If 0 it will "
      "retrieve all the matches).")
    ("weights,w", po::value<std::string>(&weightsName),
      "Input name for the weight file, if not provided the weights will be "
      "computed on the database built with the provided set.");

  po::options_description logParams("Log parameters");
  logParams.add_options()
      ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
        "verbosity level (fatal,  error, warning, info, debug, trace).");

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

  ALICEVISION_COUT("Program called with the following parameters: " <<std::endl
    << argv[0] << std::endl
    << "\t--input " << sfmDataFilename << std::endl
    << "\t--tree " << treeName << std::endl
    << "\t--output " << outputFile << std::endl
    << "\t--maxDescriptors " << nbMaxDescriptors << std::endl
    << "\t--nbMatches " << numImageQuery << std::endl
    << "\t--weights " << weightsName << std::endl
    << "\t--verboseLevel " << verboseLevel);

  // set verbose level
  system::Logger::get()->setLogLevel(verboseLevel);

  //**********************************************************
  // Load the voctree
  //**********************************************************

  // Load vocabulary tree

  printf("Loading vocabulary tree\n");
  auto loadVoctree_start = std::chrono::steady_clock::now();
  aliceVision::voctree::VocabularyTree<DescriptorFloat> tree(treeName);
  auto loadVoctree_elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - loadVoctree_start);
  std::cout << "tree loaded with" << endl << "\t" << tree.levels() << " levels" << std::endl
          << "\t" << tree.splits() << " branching factor" << std::endl
          << "\t in " << loadVoctree_elapsed.count() << " seconds" << std::endl;

  //**********************************************************
  // Create the database
  //**********************************************************

  ALICEVISION_COUT("Creating the database...");
  // Add each object (document) to the database
  aliceVision::voctree::Database db(tree.words());

  if(withWeights)
  {
    ALICEVISION_COUT("Loading weights...");
    db.loadWeights(weightsName);
  }
  else
  {
    ALICEVISION_COUT("No weights specified, skipping...");
  }


  //*********************************************************
  // Read the descriptors and populate the database
  //*********************************************************

  ALICEVISION_COUT("Reading descriptors from " << sfmDataFilename);
  auto detect_start = std::chrono::steady_clock::now();
  std::size_t numTotFeatures = aliceVision::voctree::populateDatabase<DescriptorUChar>(sfmDataFilename, tree, db, nbMaxDescriptors);
  auto detect_elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - detect_start);

  if(numTotFeatures == 0)
  {
    ALICEVISION_CERR("No descriptors loaded!!");
    return EXIT_FAILURE;
  }

  ALICEVISION_COUT("Done! " << db.getSparseHistogramPerImage().size() << " sets of descriptors read for a total of " << numTotFeatures << " features");
  ALICEVISION_COUT("Reading took " << detect_elapsed.count() << " sec");

  if(!withWeights)
  {
    // Compute and save the word weights
    ALICEVISION_COUT("Computing weights...");
    db.computeTfIdfWeights();
  }

  //**********************************************************
  // Query the database to get all the pair list
  //**********************************************************


  if(numImageQuery == 0)
  {
    // if 0 retrieve the score for all the documents of the database
    numImageQuery = db.size();
  }

  PairList allMatches;

  ALICEVISION_COUT("Query all documents");
  detect_start = std::chrono::steady_clock::now();
  // Now query each document
  #pragma omp parallel for
  for(ptrdiff_t i = 0; i < static_cast<ptrdiff_t>(db.getSparseHistogramPerImage().size()); ++i)
  {
    aliceVision::voctree::SparseHistogramPerImage::const_iterator docIt = db.getSparseHistogramPerImage().cbegin();
    std::advance(docIt, i);
    std::vector<aliceVision::voctree::DocMatch> matches;
    
    db.find(docIt->second, numImageQuery, matches);
    //    ALICEVISION_COUT("query document " << docIt->first
    //                  << " took " << detect_elapsed.count() 
    //                  << " ms and has " << matches.size() 
    //                  << " matches\tBest " << matches[0].id 
    //                  << " with score " << matches[0].score);

    ListOfImageID idMatches;
    idMatches.reserve(matches.size());
    for(const aliceVision::voctree::DocMatch& m : matches)
    {
      idMatches.push_back(m.id);
    }
      #pragma omp critical
    {
      allMatches[ docIt->first ] = idMatches;
    }
  }
  detect_elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - detect_start);
  ALICEVISION_COUT("Query of all documents took " << detect_elapsed.count() << " sec.");

  //**********************************************************
  // process pair list
  //**********************************************************

  detect_start = std::chrono::steady_clock::now();
  OrderedPairList selectedPairs;

  ALICEVISION_COUT("Convert all matches to pairList");
  convertAllMatchesToPairList(allMatches, numImageQuery, selectedPairs);
  detect_elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - detect_start);
  ALICEVISION_COUT("Convert all matches to pairList took " << detect_elapsed.count() << " sec.");

  // check if the output directory exists
  const auto basePath = bfs::path(outputFile).parent_path();
  if(!basePath.empty() && !bfs::exists(basePath))
  {
    // then create the missing directory
    if(!bfs::create_directories(basePath))
    {
      ALICEVISION_CERR("Unable to create directories: " << basePath);
      return EXIT_FAILURE;
    }
  }
  
  // write it to file
  std::ofstream fileout;
  fileout.open(outputFile, ofstream::out);
  fileout << selectedPairs;
  fileout.close();

  ALICEVISION_COUT("pairList exported in: " << outputFile);
  return EXIT_SUCCESS;
}
