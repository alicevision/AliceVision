// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "DeviceStreamManager.hpp"

#include<aliceVision/system/Logger.hpp>

namespace aliceVision {
namespace depthMap {

DeviceStreamManager::DeviceStreamManager(int nbStream) 
   : _nbStream(nbStream)
{
    assert(nbStream > 0);

    _streams.resize(nbStream);

    for(int i = 0; i < nbStream; ++i)
    {
        cudaError_t err = cudaStreamCreate(&_streams.at(i));
        if(err != cudaSuccess)
        {
            ALICEVISION_LOG_WARNING("DeviceStreamManager: Failed to create a CUDA stream object " << i << "/" << nbStream << ", " << cudaGetErrorString(err));
            _streams.at(i) = 0;
        }
    }
}

DeviceStreamManager::~DeviceStreamManager() 
{
    for(cudaStream_t& stream : _streams)
    {
        cudaStreamSynchronize(stream);

        if(stream != 0) 
        {
            cudaStreamDestroy(stream);
        }
    }
}

cudaStream_t DeviceStreamManager::getStream(int streamIndex)
{
    return _streams.at(streamIndex % _nbStream);
}

void DeviceStreamManager::waitStream(int streamIndex)
{
    cudaStreamSynchronize(getStream(streamIndex));
}

} // namespace depthMap
} // namespace aliceVision
