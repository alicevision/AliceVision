#include <openMVG/sfm/sfm_data_io.hpp>
#include <openMVG/voctree/database.hpp>
#include <openMVG/voctree/databaseIO.hpp>
#include <openMVG/voctree/vocabulary_tree.hpp>
#include <openMVG/voctree/descriptor_loader.hpp>

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

#define POPART_COUT(x) std::cout << x << std::endl
#define POPART_CERR(x) std::cerr << x << std::endl

static const int DIMENSION = 128;

using namespace std;
using namespace boost::accumulators;
namespace bpo = boost::program_options;
namespace bfs = boost::filesystem;

typedef openMVG::features::Descriptor<float, DIMENSION> DescriptorFloat;
typedef openMVG::features::Descriptor<unsigned char, DIMENSION> DescriptorUChar;

std::ostream& operator<<(std::ostream& os, const openMVG::voctree::DocMatches &matches)
{
  os << "[ ";
  for(const auto &e : matches)
  {
    os << e.id << ", " << e.score << "; ";
  }
  os << "];\n";
  return os;
}

std::ostream& operator<<(std::ostream& os, const openMVG::voctree::Document &doc)
{
  os << "[ ";
  for(const openMVG::voctree::Word &w : doc)
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
 * This program is used to give some statistics about the number of features per leaf for a given the voctree:
 *     - a local histogram for each picture
 *     - a global histogram for the whole set
 */
int main(int argc, char** argv)
{
   
  int verbosity = 1; ///< verbosity level
  string treeName; ///< the filename of the voctree
  string queryList = ""; ///< the file containing the list of features to use as query
  size_t numImageQuery; ///< the number of matches to retrieve for each image

  openMVG::sfm::SfM_Data sfmdata;
  openMVG::sfm::SfM_Data *sfmdataQuery;

  bpo::options_description desc(programDescription);
  desc.add_options()
          ("help,h", "Print this message")
          ("verbose,v", bpo::value<int>(&verbosity)->default_value(1), "Verbosity level, 0 to mute")
          ("tree,t", bpo::value<string>(&treeName)->required(), "Input name for the tree file")
          ("querylist,q", bpo::value<string>(&queryList), "Path to the list file to be used for querying the database");
  


  bpo::variables_map vm;

  try
  {
    bpo::store(bpo::parse_command_line(argc, argv, desc), vm);

    if(vm.count("help") || (argc == 1))
    {
      std::cout << desc << std::endl;
      return EXIT_SUCCESS;
    }

    bpo::notify(vm);
  }
  catch(bpo::required_option& e)
  {
    std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
    std::cout << "Usage:\n\n" << desc << std::endl;
    return EXIT_FAILURE;
  }
  catch(bpo::error& e)
  {
    std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
    std::cout << "Usage:\n\n" << desc << std::endl;
    return EXIT_FAILURE;
  }


  //************************************************
  // Load vocabulary tree
  //************************************************

  POPART_COUT("Loading vocabulary tree\n");
  openMVG::voctree::VocabularyTree<DescriptorFloat> tree(treeName);
  POPART_COUT("tree loaded with\n\t" 
          << tree.levels() << " levels\n\t" 
          << tree.splits() << " branching factor");


  //************************************************
  // Create the database
  //************************************************

  POPART_COUT("Creating the database...");
  // Add each object (document) to the database
  openMVG::voctree::Database db(tree.words());

  /*if(withWeights)
  {
    POPART_COUT("Loading weights...");
    db.loadWeights(weightsName);
  }
  else
  {
    POPART_COUT("No weights specified, skipping...");
  }*/

  //************************************************
  // Query documents for Statistics
  //************************************************

  std::map<int,int> globalHisto;

  POPART_COUT("Getting some stats for " << queryList);
  
  openMVG::voctree::voctreeStatistics<DescriptorUChar>(queryList, tree, globalHisto);
  
  std::cout << "-----------------" << std::endl;
  
  for(auto itHisto = globalHisto.begin(); itHisto != globalHisto.end(); itHisto++)
    {
      std::cout << itHisto->first << ": " << itHisto->second  << ", ";
    }
    std::cout << std::endl;
  

  return EXIT_SUCCESS;
}
