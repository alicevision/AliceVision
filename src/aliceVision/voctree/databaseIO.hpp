// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/voctree/Database.hpp>
#include <aliceVision/voctree/VocabularyTree.hpp>
#include <aliceVision/vfs/filesystem.hpp>

#include <string>
#include <vector>

namespace aliceVision {

namespace sfm {
class SfMData;
}

namespace voctree {

/**
 * @brief Given a vocabulary tree and a set of features it builds a database
 *
 * @param[in] fs Virtual filesystem handle
 * @param[in] fileFullPath A file containing the path the features to load, it could be a .txt or an AliceVision .json
 * @param[in] featuresFolders The folder(s) containing the descriptor files (optional)
 * @param[in] tree The vocabulary tree to be used for feature quantization
 * @param[out] db The built database
 * @param[out] documents A map containing for each image the list of associated visual words
 * @param[in] Nmax The maximum number of features loaded in each desc file. For Nmax = 0 (default), all the descriptors are loaded.
 * @return the number of overall features read
 */
template<class DescriptorT, class VocDescriptorT>
std::size_t populateDatabase(vfs::filesystem& fs,
                             const sfmData::SfMData& sfmData,
                             const std::vector<std::string>& featuresFolders,
                             const VocabularyTree<VocDescriptorT>& tree,
                             Database& db,
                             const int Nmax = 0);

/**
 * @brief Given an non empty database, it queries the database with a set of images
 * and their associated features and returns, for each image, the first \p numResults best
 * matching documents in the database
 * 
 * @param[in] fs Virtual filesystem handle
 * @param[in] filepath A file containithe path the features to load, it could be a .txt or an AliceVision .json
 * @param[in] featuresFolders The folder(s) containing the descriptor files (optional)
 * @param[in] tree The vocabulary tree to be usedng  for feature quantization
 * @param[in] db The built database
 * @param[in] numResults The number of results to retrieve for each image
 * @param[out] allMatches The matches for all the images
 * @param[in] distanceMethod The distance method used to create the pair list
 * @param[in] Nmax The maximum number of features loaded in each desc file. For Nmax = 0 (default), all the descriptors are loaded. 
 * @see queryDatabase()
 */
template<class DescriptorT, class VocDescriptorT>
void queryDatabase(vfs::filesystem& fs,
                   const sfmData::SfMData& sfmData,
                   const std::vector<std::string>& featuresFolders,
                   const VocabularyTree<VocDescriptorT>& tree,
                   const Database& db,
                   std::size_t numResults,
                   std::map<std::size_t, DocMatches>& allMatches,
                   const std::string& distanceMethod,
                   const int Nmax = 0);

/**
 * @brief Given an non empty database, it queries the database with a set of images
 * and their associated features and returns, for each image, the first \p numResults best
 * matching documents in the database
 * 
 * @param[in] fs Virtual filesystem handle
 * @param[in] filepath A file containing the path the features to load, it could be a .txt or an AliceVision .json
 * @param[in] featuresFolders The folder(s) containing the descriptor files (optional)
 * @param[in] tree The vocabulary tree to be used for feature quantization
 * @param[in] db The built database
 * @param[in] numResults The number of results to retrieve for each image
 * @param[out] allMatches The matches for all the images
 * @param[out] documents For each document, it contains the list of associated visual words 
 * @param[in] distanceMethod The distance method used to create the pair list
 * @param[in] Nmax The maximum number of features loaded in each desc file. For Nmax = 0 (default), all the descriptors are loaded.
 */
template<class DescriptorT, class VocDescriptorT>
void queryDatabase(vfs::filesystem& fs,
                   const sfmData::SfMData& sfmData,
                   const std::vector<std::string>& featuresFolders,
                   const VocabularyTree<VocDescriptorT>& tree,
                   const Database& db,
                   std::size_t numResults,
                   std::map<std::size_t, DocMatches>& allMatches,
                   std::map<std::size_t, Document>& documents,
                   const std::string& distanceMethod,
                   const int Nmax = 0);

/**
 * @brief Returns some statistics (histogram) 
 * 
 * @param[in] fs Virtual filesystem handle
 * @param[in] fileFullPath A file containing the path the features to load, it could be a .txt or an AliceVision .json
 * @param[in] featuresFolders The folder(s) containing the descriptor files (optional)
 * @param[in] tree The vocabulary tree to be used for feature quantization
 * @param[in] db The built database
 * @param[in] distanceMethod The distance method used for create the pair list
 * @param[in/out] globalHistogram The histogram of the "population" of voctree leaves. 
 * @see queryDatabase()
 */
template<class DescriptorT, class VocDescriptorT>
void voctreeStatistics(vfs::filesystem& fs,
                       const sfmData::SfMData& sfmData,
                       const std::vector<std::string>& featuresFolders,
                       const VocabularyTree<VocDescriptorT>& tree,
                       const Database& db,
                       const std::string& distanceMethod,
                       std::map<int, int>& globalHistogram);

} //namespace voctree
} //namespace aliceVision

#include "databaseIO.tcc"
