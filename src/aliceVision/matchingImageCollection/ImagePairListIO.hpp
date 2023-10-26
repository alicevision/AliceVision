// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/types.hpp>
#include <iosfwd>

namespace aliceVision {
namespace matchingImageCollection {

/// Load a set of PairSet from a stream
/// I J K L (pair that link I)
bool loadPairs(std::istream& stream, PairSet& pairs, int rangeStart = -1, int rangeSize = 0);

/// Save a set of PairSet to a stream (one pair per line)
/// I J
/// I K
void savePairs(std::ostream& stream, const PairSet& pairs);

/// Same as loadPairs, but loads from a given file
bool loadPairsFromFile(const std::string& sFileName,  // filename of the list file,
                       PairSet& pairs,
                       int rangeStart = -1,
                       int rangeSize = 0);

/// Same as savePairs, but saves to a given file
bool savePairsToFile(const std::string& sFileName, const PairSet& pairs);

}  // namespace matchingImageCollection
}  // namespace aliceVision
