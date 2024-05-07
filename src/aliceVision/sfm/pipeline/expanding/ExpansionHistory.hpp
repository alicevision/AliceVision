// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/types.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/track/Track.hpp>

namespace aliceVision {
namespace sfm {

class ExpansionHistory
{
public:
    using sptr = std::shared_ptr<ExpansionHistory>;
    
public:  
    ExpansionHistory();
    
    /**
     * @brief initialize object
     * @param sfmData scene object
     * @return true if succeeded
    */
    bool initialize(const sfmData::SfMData & sfmData);

    /**
     * @brief Initialize an iteration
     * @param sfmData the scene object
     * @return true if succeeded
    */
    bool beginEpoch(const sfmData::SfMData & sfmData);


    /**
     * @brief Terminate an iteration
     * @param sfmData the scene object
     * @param selectedViews the list of selected views for the current chunk
    */
    void endEpoch(sfmData::SfMData & sfmData, const std::set<IndexT> & selectedViews);

    /**
     * @brief get the iteration id
     * @return iteration id
     */
    const size_t getEpoch() const 
    {
        return _epoch;
    }

    void saveState(const sfmData::SfMData & sfmData);

    /**
     * Get the history of focals over time for a particular intrinsic id
     * @param intrinsicId the id of the intrinsic
     * @return a vector defining over time pairs of <count usage, focal value> where count usage is the number of views using this intrinsic
    */
    const std::vector<std::pair<size_t, double>> & getFocalHistory(IndexT intrinsicId)
    {
        return _focalHistory[intrinsicId];
    }

private:
    // History of focals per intrinsics
    std::map<IndexT, std::vector<std::pair<size_t, double>>> _focalHistory;
    
    // epoch ID
    std::size_t _epoch = 0;
};

} // namespace sfm
} // namespace aliceVision

