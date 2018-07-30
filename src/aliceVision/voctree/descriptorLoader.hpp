// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/types.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/voctree/Database.hpp>
#include <aliceVision/voctree/VocabularyTree.hpp>

#include <string>

namespace aliceVision {
namespace voctree {

/**
 * @brief Get the number of descriptors contained inside a .desc file and the number of bytes
 * used to store each descriptor elements
 *
 * @param[in] path The .desc filename
 * @param[in] dim The number of elements per descriptor
 * @param[out] numDescriptors The number of descriptors stored in the file
 * @param[out] bytesPerElement The number of bytes used to store each element of the descriptor
 */
void getInfoBinFile(const std::string& path, int dim, std::size_t& numDescriptors, int& bytesPerElement);

/**
 * @brief Extract a list of decriptor files from a sfmData.
 * @param[in] sfmDataPath The input sfmData
 * @param[in] featuresFolders The folder(s) containing the descriptor files
 * @param[out] descriptorsFiles A list of descriptor files 
 */
void getListOfDescriptorFiles(const sfmData::SfMData& sfmData,
                              const std::vector<std::string>& featuresFolders,
                              std::map<IndexT, std::string>& descriptorsFiles);

/**
 * @brief Read a set of descriptors from a file containing the path to the descriptor files.
 * @param[in] sfmDataPath The input sfmData
 * @param[in] featuresFolders The folder(s) containing the descriptor files (optional)
 * @param[in,out] descriptors the vector to which append all the read descriptors
 * @param[in,out] numFeatures a vector collecting for each file read the number of features read
 * @return the total number of features read
 *
 */
template<class DescriptorT, class FileDescriptorT>
std::size_t readDescFromFiles(const sfmData::SfMData& sfmData,
                         const std::vector<std::string>& featuresFolders,
                         std::vector<DescriptorT>& descriptors,
                         std::vector<std::size_t>& numFeatures);

} // namespace voctree
} // namespace aliceVision

#include "descriptorLoader.tcc"
