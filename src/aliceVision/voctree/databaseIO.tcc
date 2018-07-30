// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "descriptorLoader.hpp"

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/config.hpp>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string/case_conv.hpp>
#include <boost/progress.hpp>

#include <exception>
#include <iostream>
#include <fstream>

namespace aliceVision {
namespace voctree {

template<class DescriptorT, class VocDescriptorT>
std::size_t populateDatabase(const sfmData::SfMData& sfmData,
                             const std::vector<std::string>& featuresFolders,
                             const VocabularyTree<VocDescriptorT>& tree,
                             Database& db,
                             const int Nmax)
{
  std::map<IndexT, std::string> descriptorsFiles;
  getListOfDescriptorFiles(sfmData, featuresFolders, descriptorsFiles);
  std::size_t numDescriptors = 0;
  
  // Read the descriptors
  ALICEVISION_LOG_DEBUG("Reading the descriptors from " << descriptorsFiles.size() <<" files...");
  boost::progress_display display(descriptorsFiles.size());

  // Run through the path vector and read the descriptors
  for(const auto &currentFile : descriptorsFiles)
  {
    
    std::vector<DescriptorT> descriptors;

    // Read the descriptors
    loadDescsFromBinFile(currentFile.second, descriptors, false, Nmax);
    size_t result = descriptors.size();
    
    SparseHistogram newDoc =  tree.quantizeToSparse(descriptors);

    // Insert document in database
    db.insert(currentFile.first, newDoc);

    // Update the overall counter
    numDescriptors += result;
    
    ++display;
  }

  // Return the result
  return numDescriptors;
}

template<class DescriptorT, class VocDescriptorT>
std::size_t populateDatabase(const sfmData::SfMData& sfmData,
                             const std::vector<std::string>& featuresFolders,
                             const VocabularyTree<VocDescriptorT>& tree,
                             Database& db,
                             std::map<size_t, std::vector<DescriptorT>>& allDescriptors,
                             const int Nmax)
{
  std::map<IndexT, std::string> descriptorsFiles;
  getListOfDescriptorFiles(sfmData, featuresFolders, descriptorsFiles);
  std::size_t numDescriptors = 0;

  // Read the descriptors
  ALICEVISION_LOG_DEBUG("Reading the descriptors from " << descriptorsFiles.size() <<" files...");
  boost::progress_display display(descriptorsFiles.size());

  // Run through the path vector and read the descriptors
  for(const auto &currentFile : descriptorsFiles)
  {
    std::vector<DescriptorT> descriptors;

    // Read the descriptors
    loadDescsFromBinFile(currentFile.second, descriptors, false, Nmax);
    size_t result = descriptors.size();
    
    allDescriptors[currentFile.first] = descriptors;
    
    SparseHistogram newDoc = tree.quantizeToSparse(descriptors);
    
    // Insert document in database
    db.insert(currentFile.first, newDoc);

    // Update the overall counter
    numDescriptors += result;

    ++display;
  }

  // Return the result
  return numDescriptors;
}

template<class DescriptorT, class VocDescriptorT>
void queryDatabase(const sfmData::SfMData& sfmData,
                   const VocabularyTree<VocDescriptorT>& tree,
                   const Database& db,
                   size_t numResults,
                   std::map<size_t, DocMatches>& allDocMatches,
                   const std::string& distanceMethod,
                   const int Nmax)
{
  std::map<size_t, Document> documents;
  queryDatabase<DescriptorT>(sfmData, tree, db, numResults, allDocMatches, documents, distanceMethod, Nmax);
}

/**
 * @brief Given an non empty database, it queries the database with a set of images
 * and their associated features and returns, for each image, the first \p numResults best
 * matching documents in the database
 * 
 * @param[in] filepath A file containing the path the features to load, it could be a .txt or an AliceVision .json
 * @param[in] featuresFolders The folder(s) containing the descriptor files (optional)
 * @param[in] tree The vocabulary tree to be used for feature quantization
 * @param[in] db The built database
 * @param[in] numResults The number of results to retrieve for each image
 * @param[out] allMatches The matches for all the images
 * @param[out] documents For each document, it contains the list of associated visual words 
 * @param[in] distanceMethod The method used to compute distance between histograms.
 * @param[in] Nmax The maximum number of features loaded in each desc file. For Nmax = 0 (default), all the descriptors are loaded.
 */
template<class DescriptorT, class VocDescriptorT>
void queryDatabase(const sfmData::SfMData& sfmData,
                   const std::vector<std::string>& featuresFolders,
                   const VocabularyTree<VocDescriptorT>& tree,
                   const Database& db,
                   size_t numResults,
                   std::map<size_t, DocMatches>& allDocMatches,
                   std::map<size_t, SparseHistogram>& documents,
                   const std::string& distanceMethod,
                   const int Nmax)
{
  std::map<IndexT, std::string> descriptorsFiles;
  getListOfDescriptorFiles(sfmData, featuresFolders, descriptorsFiles);
  
  // Read the descriptors
  ALICEVISION_LOG_DEBUG("queryDatabase: Reading the descriptors from " << descriptorsFiles.size() << " files...");
  boost::progress_display display(descriptorsFiles.size());

  #pragma omp parallel for
  // Run through the path vector and read the descriptors
  for(ptrdiff_t i = 0; i < static_cast<ptrdiff_t>(descriptorsFiles.size()); ++i)
  {
    std::map<IndexT, std::string>::const_iterator currentFileIt = descriptorsFiles.cbegin();
    std::advance(currentFileIt, i);
    std::vector<DescriptorT> descriptors;

    // Read the descriptors
    loadDescsFromBinFile(currentFileIt->second, descriptors, false, Nmax);

    // quantize the descriptors
    SparseHistogram query = tree.quantizeToSparse(descriptors);

    aliceVision::voctree::DocMatches docMatches;
    // query the database
    db.find(query, numResults, docMatches, distanceMethod);
    #pragma omp critical
    {
      // add the vector to the documents
      documents[currentFileIt->first] = query;

      // add the matches to the result vector
      allDocMatches[currentFileIt->first] = docMatches;

      ++display;
    }
  }
}

template<class DescriptorT, class VocDescriptorT>
void voctreeStatistics(const sfmData::SfMData& sfmData,
                       const std::vector<std::string>& featuresFolders,
                       const VocabularyTree<VocDescriptorT>& tree,
                       const Database& db,
                       const std::string& distanceMethod,
                       std::map<int, int>& globalHistogram)
{
  std::map<IndexT, std::string> descriptorsFiles;
  getListOfDescriptorFiles(sfmData, featuresFolders, descriptorsFiles);
  
  // Read the descriptors
  ALICEVISION_LOG_DEBUG("Reading the descriptors from " << descriptorsFiles.size() << " files.");

  // Run through the path vector and read the descriptors
  for(const auto &currentFile : descriptorsFiles)
  {
    std::vector<DescriptorT> descriptors;

    // Read the descriptors
    loadDescsFromBinFile(currentFile.second, descriptors, false);

    // query the database
    SparseHistogram query = tree.quantizeToSparse(descriptors);
    std::map<int,int> localHisto;
    
    for(auto q: query)
    {
      int nb = (int)q.second.size();
      if(globalHistogram.find(nb) == globalHistogram.end())
        globalHistogram[nb] = 1;
      else
        globalHistogram[nb] += 1;
      
      if(localHisto.find(nb) == localHisto.end())
        localHisto[nb] = 1;
      else
        localHisto[nb] += 1;   
    }
    
    ALICEVISION_LOG_DEBUG("Histogram of " << currentFile.first);
    
    for(auto itHisto = localHisto.begin(); itHisto != localHisto.end(); itHisto++)
    {
      ALICEVISION_LOG_DEBUG_OBJ << itHisto->first << ": " << itHisto->second  << ", ";
    }
    localHisto.clear();
    
    ALICEVISION_LOG_DEBUG_OBJ << std::endl;
  }
}

} //namespace voctree
} //namespace aliceVision
