// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/**
 * @file ImagePairListIO.hpp
 * @brief Input/Output functions for reading and writing image pair lists.
 * 
 * This file provides utilities for loading and saving sets of image pairs
 * to and from streams and files. Image pairs are used in the matching process
 * to define which images should be compared for feature matching.
 */

#include <aliceVision/types.hpp>

#include <iosfwd>
#include <string>

namespace aliceVision {
namespace matchingImageCollection {

/**
 * @brief Load image pairs from a file.
 * 
 * Reads image pair data from the specified file and populates the pairs set.
 * 
 * @param[in] sFileName Path to the file containing the pair data
 * @param[out] pairs Set of image pairs to be populated
 * @param[in] useSymmetry If true, add (min(i,j), max(i,j)) instead of (i,j) (default: true)
 * @return true if the file was opened and pairs were successfully loaded, false otherwise
 */
bool loadPairsFromFile(const std::string& sFileName,  // filename of the list file,
                       PairSet& pairs,
                       bool useSymmetry = true);

/**
 * @brief Save image pairs to a file.
 * 
 * Writes the image pair data to the specified file.
 * 
 * @param[in] sFileName Path to the output file
 * @param[in] pairs Set of image pairs to save
 * @return true if the file was created and pairs were successfully saved, false otherwise
 */
bool savePairsToFile(const std::string& sFileName, const PairSet& pairs);

}  // namespace matchingImageCollection
}  // namespace aliceVision
