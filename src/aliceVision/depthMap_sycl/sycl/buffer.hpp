// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <sycl/sycl.hpp>

namespace aliceVision {
namespace depthMap_sycl {

/**
* @brief
* @param[sycl::<uint, Dim>] dims size of multidimensional array
* @param[sycl::<uint, Dim>] coords coordinates
* @return
*/
template<int Dim>
static inline constexpr uint getAddress(const sycl::vec<uint, Dim>& dims, const sycl::vec<uint, Dim>& coords) {
    uint sum = 0;
    uint product = 1;

#pragma unroll
    for(int i = 0; i < Dim; i++)
    {
        sum += coords[i]*product;
        product *= dims[i];
    }

    return sum;
};

template<>
inline constexpr uint getAddress<3>(const sycl::uint3& dims, const sycl::uint3& coords) {
    uint sum = 0;
    uint product = 1;

#pragma unroll
    for(int i : {2, 1, 0}) // put z coords at begining for cache efficiency
    {
        sum += coords[i]*product;
        product *= dims[i];
    }

    return sum;
};


} // namespace depthMap_sycl
} // namespace aliceVision

