// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "patchPattern.hpp"

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/depthMap/cuda/host/memory.hpp>
#include <aliceVision/depthMap/cuda/device/DevicePatchPattern.hpp>

#include <map>

namespace aliceVision {
namespace depthMap {

void buildCustomPatchPattern(const CustomPatchPatternParams& patchParams)
{
    // check at least one patch subpart
    if (patchParams.subpartsParams.empty())
    {
        ALICEVISION_THROW_ERROR("Cannot build custom patch pattern in device constant memory: No patch pattern subpart given.");
    }

    // build nb coordinates per subpart map
    std::map<int, int> nbCoordsPerSubparts;  // <level or subpart index, nb coordinates>
    for (int i = 0; i < patchParams.subpartsParams.size(); ++i)
    {
        const auto& subpartParams = patchParams.subpartsParams.at(i);

        if (subpartParams.radius <= 0.f)
            ALICEVISION_THROW_ERROR("Cannot build custom patch pattern in device constant memory: A patch pattern subpart radius is incorrect.");

        if (subpartParams.isCircle && subpartParams.nbCoordinates <= 0)
            ALICEVISION_THROW_ERROR(
              "Cannot build custom patch pattern in device constant memory: A patch pattern subpart circle number of coordinates is incorrect.");

        if (patchParams.groupSubpartsPerLevel)
        {
            if (!subpartParams.isCircle && nbCoordsPerSubparts.find(subpartParams.level) != nbCoordsPerSubparts.end())
                ALICEVISION_THROW_ERROR(
                  "Cannot build custom patch pattern in device constant memory: Cannot group more than one full patch pattern subpart.");

            nbCoordsPerSubparts[subpartParams.level] += ((subpartParams.isCircle) ? subpartParams.nbCoordinates : 0);
        }
        else
        {
            nbCoordsPerSubparts[i] += ((subpartParams.isCircle) ? subpartParams.nbCoordinates : 0);
        }
    }

    // get the maximum number of coordinates
    int maxSubpartCoords = 0;
    for (const auto& nbCoordsPerSubpart : nbCoordsPerSubparts)
        maxSubpartCoords = std::max(maxSubpartCoords, nbCoordsPerSubpart.second);

    // get true number of subparts
    const int nbSubparts = nbCoordsPerSubparts.size();

    // check max number of subparts
    if (nbSubparts > ALICEVISION_DEVICE_PATCH_MAX_SUBPARTS)
    {
        ALICEVISION_THROW_ERROR("Cannot build custom patch pattern in device constant memory: Too many patch pattern subpart given."
                                << std::endl
                                << "\t- # given patch pattern subparts: " << nbSubparts << "(group by level is "
                                << ((patchParams.groupSubpartsPerLevel) ? "ON" : "OFF") << ")" << std::endl
                                << "\t- maximum number of patch pattern subparts: " << ALICEVISION_DEVICE_PATCH_MAX_SUBPARTS);
    }

    // check max number of subpart coordinates
    if (maxSubpartCoords > ALICEVISION_DEVICE_PATCH_MAX_COORDS_PER_SUBPARTS)
    {
        ALICEVISION_THROW_ERROR("Cannot build custom patch pattern in device constant memory: Too many patch pattern subpart coordinates given."
                                << std::endl
                                << "\t- # given patch pattern subpart coordinates: " << maxSubpartCoords << "(group by level is "
                                << ((patchParams.groupSubpartsPerLevel) ? "ON" : "OFF") << ")" << std::endl
                                << "\t- maximum number of patch pattern subpart coordinates: " << ALICEVISION_DEVICE_PATCH_MAX_COORDS_PER_SUBPARTS);
    }

    // log custom patch pattern parameters
    {
        std::stringstream ss;

        ss << "Build custom patch pattern with given parameter:" << std::endl
           << "\t- Group patch pattern subparts per level: " << ((patchParams.groupSubpartsPerLevel) ? "ON" : "OFF") << std::endl
           << "\t- Subparts:" << std::endl;

        for (const auto& subpartParams : patchParams.subpartsParams)
        {
            ss << "\t    - subpart:" << std::endl
               << "\t        - level: " << subpartParams.level << std::endl
               << "\t        - weight: " << subpartParams.weight << std::endl;

            if (subpartParams.isCircle)
            {
                ss << "\t        - type: circle" << std::endl
                   << "\t        - radius: " << subpartParams.radius << std::endl
                   << "\t        - # coordinates: " << subpartParams.nbCoordinates << std::endl;
            }
            else
            {
                ss << "\t        - type: full" << std::endl << "\t        - wsh: " << int(subpartParams.radius) << std::endl;
            }
        }
        ALICEVISION_LOG_DEBUG(ss.str());
    }

    // host-side patch pattern pointer
    DevicePatchPattern* patchPattern_h = nullptr;

    // allocate host-side patch pattern struct
    CHECK_CUDA_RETURN_ERROR(cudaMallocHost(&patchPattern_h, sizeof(DevicePatchPattern)));

    // set true number of subparts
    patchPattern_h->nbSubparts = nbSubparts;

    // fill the host-side patch pattern struct
    if (patchParams.groupSubpartsPerLevel)
    {
        // ensure that nbCoordinates and wsh are set to zero
        // CUDA doesn't support default initialization for constant memory struct
        for (int i = 0; i < patchPattern_h->nbSubparts; ++i)
        {
            DevicePatchPatternSubpart& subpart = patchPattern_h->subparts[i];
            subpart.nbCoordinates = 0;
            subpart.wsh = 0;
        }

        for (const auto& subpartParams : patchParams.subpartsParams)
        {
            const auto it = nbCoordsPerSubparts.find(subpartParams.level);
            DevicePatchPatternSubpart& subpart = patchPattern_h->subparts[std::distance(nbCoordsPerSubparts.begin(), it)];

            if (subpartParams.isCircle)
            {
                const float radiusValue = subpartParams.radius;
                const float angleDifference = (M_PI * 2.f) / subpartParams.nbCoordinates;

                // compute patch pattern relative coordinates
                for (int i = 0; i < subpartParams.nbCoordinates; ++i)
                {
                    float2& coords = subpart.coordinates[subpart.nbCoordinates + i];

                    const float radians = angleDifference * i;
                    coords.x = std::cos(radians) * radiusValue;
                    coords.y = std::sin(radians) * radiusValue;
                }

                subpart.wsh = std::max(subpart.wsh, int(subpartParams.radius + std::pow(2.f, subpartParams.level - 1.f)));
                subpart.nbCoordinates += subpartParams.nbCoordinates;
            }
            else
            {
                // full subpart
                subpart.wsh = std::max(subpart.wsh, int(subpartParams.radius));
            }

            subpart.level = subpartParams.level;
            subpart.downscale = std::pow(2.f, subpart.level);
            subpart.weight = subpartParams.weight;
            subpart.isCircle = subpartParams.isCircle;
        }
    }
    else
    {
        for (int i = 0; i < patchPattern_h->nbSubparts; ++i)
        {
            DevicePatchPatternSubpart& subpart = patchPattern_h->subparts[i];

            const auto& subpartParams = patchParams.subpartsParams.at(i);

            if (subpartParams.isCircle)
            {
                const float radiusValue = subpartParams.radius;
                const float angleDifference = (M_PI * 2.f) / subpart.nbCoordinates;

                // compute patch pattern relative coordinates
                for (int j = 0; j < subpart.nbCoordinates; ++j)
                {
                    float2& coords = subpart.coordinates[j];

                    const float radians = angleDifference * j;
                    coords.x = std::cos(radians) * radiusValue;
                    coords.y = std::sin(radians) * radiusValue;
                }

                subpart.wsh = int(subpartParams.radius + std::pow(2.f, subpartParams.level - 1.f));
                subpart.nbCoordinates = subpartParams.nbCoordinates;
            }
            else
            {
                // full subpart
                subpart.wsh = int(subpartParams.radius);
                subpart.nbCoordinates = 0;
            }

            subpart.level = subpartParams.level;
            subpart.downscale = std::pow(2.f, subpart.level);
            subpart.weight = subpartParams.weight;
            subpart.isCircle = subpartParams.isCircle;
        }
    }

    // log patch pattern
    {
        std::stringstream ss;

        ss << "Build custom patch pattern:" << std::endl << "\t- Subparts:" << std::endl;

        for (int i = 0; i < patchPattern_h->nbSubparts; ++i)
        {
            const DevicePatchPatternSubpart& subpart = patchPattern_h->subparts[i];

            ss << "\t    - subpart " << i << ":" << std::endl
               << "\t        - level: " << subpart.level << std::endl
               << "\t        - downscale: " << subpart.downscale << std::endl
               << "\t        - weight: " << subpart.weight << std::endl
               << "\t        - wsh: " << subpart.wsh << std::endl;

            if (subpart.isCircle)
            {
                ss << "\t        - type: circle(s)" << std::endl
                   << "\t        - # coordinates: " << subpart.nbCoordinates << std::endl
                   << "\t        - coordinates list:" << std::endl;

                for (int j = 0; j < subpart.nbCoordinates; ++j)
                {
                    const float2& coords = subpart.coordinates[j];
                    ss << "\t            - [x: " << coords.x << ", y: " << coords.y << "]" << std::endl;
                }
            }
            else
            {
                ss << "\t        - type: full" << std::endl;
            }
        }
        ALICEVISION_LOG_DEBUG(ss.str());
    }

    // fill the device-side (constant memory) patch pattern struct
    const cudaError_t err = cudaMemcpyToSymbol(&constantPatchPattern_d, patchPattern_h, sizeof(DevicePatchPattern), 0, cudaMemcpyHostToDevice);
    CHECK_CUDA_RETURN_ERROR(err);
    THROW_ON_CUDA_ERROR(err, "Failed to copy patch parameters from host to device.");

    // free host-side patch pattern struct
    CHECK_CUDA_RETURN_ERROR(cudaFreeHost(patchPattern_h));

    // check last error
    CHECK_CUDA_ERROR();
}

}  // namespace depthMap
}  // namespace aliceVision
