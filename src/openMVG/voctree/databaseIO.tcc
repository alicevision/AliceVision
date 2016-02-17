#include "descriptor_loader.hpp"

#include <openMVG/sfm/sfm_data_io.hpp>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string/case_conv.hpp>
#include <boost/progress.hpp>

#include <exception>
#include <iostream>
#include <fstream>

namespace openMVG {
namespace voctree {

/**
 * @brief Given a vocabulary tree and a set of features it builds a database
 *
 * @param[in] fileFullPath A file containing the path the features to load, it could be a .txt or an OpenMVG .json
 * @param[in] tree The vocabulary tree to be used for feature quantization
 * @param[out] db The built database
 * @param[out] documents A map containing for each image the list of associated visual words
 * @param[in,out] numFeatures a vector collecting for each file read the number of features read
 * @return the number of overall features read
 */
template<class DescriptorT, class VocDescriptorT>
std::size_t populateDatabase(const std::string &fileFullPath,
                             const VocabularyTree<VocDescriptorT> &tree,
                             Database &db,
                             std::map<size_t, Document> &documents)
{ 
  std::map<IndexT, std::string> descriptorsFiles;
  getListOfDescriptorFiles(fileFullPath, descriptorsFiles);
  std::size_t numDescriptors = 0;
  
  // Read the descriptors
  std::cout << "Reading the descriptors from " << descriptorsFiles.size() <<" files..." << std::endl;
  boost::progress_display display(descriptorsFiles.size());

  // Run through the path vector and read the descriptors
  for(const auto &currentFile : descriptorsFiles)
  {
    std::vector<DescriptorT> descriptors;

    // Read the descriptors
    loadDescsFromBinFile(currentFile.second, descriptors, false);
    size_t result = descriptors.size();
    
    std::vector<openMVG::voctree::Word> imgVisualWords = tree.quantize(descriptors);

    // Add the vector to the documents
    documents[currentFile.first] = imgVisualWords;

    // Insert document in database
    db.insert(currentFile.first, imgVisualWords);

    // Update the overall counter
    numDescriptors += result;

    ++display;
  }

  // Return the result
  return numDescriptors;
}

template<class DescriptorT, class VocDescriptorT>
void queryDatabase(const std::string &fileFullPath,
                   const VocabularyTree<VocDescriptorT> &tree,
                   const Database &db,
                   size_t numResults,
                   std::map<size_t, DocMatches> &allMatches)
{
  std::map<size_t, Document> documents;
  queryDatabase<DescriptorT>(fileFullPath, tree, db, numResults, allMatches, documents);
}


template<class DescriptorT, class VocDescriptorT>
void queryDatabase(const std::string &fileFullPath,
                   const VocabularyTree<VocDescriptorT> &tree,
                   const Database &db,
                   size_t numResults,
                   std::map<size_t, DocMatches> &allMatches,
                   std::map<size_t, Document> &documents)
{  
  std::map<IndexT, std::string> descriptorsFiles;
  getListOfDescriptorFiles(fileFullPath, descriptorsFiles);
  
  // Read the descriptors
  std::cout << "Reading the descriptors from " << descriptorsFiles.size() <<" files..." << std::endl;
  boost::progress_display display(descriptorsFiles.size());

  // Run through the path vector and read the descriptors
  for(const auto &currentFile : descriptorsFiles)
  {
    std::vector<DescriptorT> descriptors;
    openMVG::voctree::DocMatches matches;

    // Read the descriptors
    loadDescsFromBinFile(currentFile.second, descriptors, false);
    
    // quantize the descriptors
    std::vector<openMVG::voctree::Word> imgVisualWords = tree.quantize(descriptors);

    // add the vector to the documents
    documents[currentFile.first] = imgVisualWords;

    // query the database
    db.find(imgVisualWords, numResults, matches);

    // add the matches to the result vector
    allMatches[currentFile.first] = matches;
    
    ++display;
  }
}


} //namespace voctree
} //namespace openMVG