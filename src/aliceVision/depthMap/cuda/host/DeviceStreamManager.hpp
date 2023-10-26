// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <cuda_runtime.h>

#include <vector>

namespace aliceVision {
namespace depthMap {

/**
 * @class Device stream manager
 * @brief Small class allowing a simple management of gpu streams
 */
class DeviceStreamManager
{
  public:
    /**
     * @brief DeviceStreamManager constructor.
     * @param[in] nbStreams the number of gpu streams managed
     */
    DeviceStreamManager(int nbStreams);

    // destructor
    ~DeviceStreamManager();

    // this class handles unique data, no copy constructor
    DeviceStreamManager(DeviceStreamManager const&) = delete;

    // this class handles unique data, no copy operator
    void operator=(DeviceStreamManager const&) = delete;

    /**
     * @brief Get the number of gpu streams managed.
     * @return number of gpu streams managed
     */
    inline int getNbStreams() const { return _nbStreams; }

    /**
     * @brief Get the stream object associated with the given index.
     * @param[in] streamIndex the stream index in the DeviceStreamManager
     * @note if streamIndex > nbStream, this function returns the stream object associated with streamIndex % nbStream
     * @return the associated stream object
     */
    cudaStream_t getStream(int streamIndex);

    /**
     * @brief Waits for stream tasks to complete.
     * @param[in] streamIndex the stream index in the DeviceStreamManager
     */
    void waitStream(int streamIndex);

  private:
    const int _nbStreams;
    std::vector<cudaStream_t> _streams;
};

}  // namespace depthMap
}  // namespace aliceVision
