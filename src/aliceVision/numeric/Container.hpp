// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>

namespace aliceVision {

template<typename TMat>
size_t CountElements(const TMat& A)
{
    static_assert(sizeof(TMat) != sizeof(TMat), "CountElements must be specialized");
    return 0;
}

template<>
size_t CountElements<Mat2X>(const Mat2X& A);
template<>
size_t CountElements<Mat3X>(const Mat3X& A);
template<>
size_t CountElements<Mat>(const Mat& A);
template<>
size_t CountElements<std::vector<Vec2>>(const std::vector<Vec2>& A);

template<typename TMat>
size_t ElementSize(const TMat& A)
{
    static_assert(sizeof(TMat) != sizeof(TMat), "CountElements must be specialized");
    return 0;
}

template<>
size_t ElementSize<Mat2X>(const Mat2X& A);
template<>
size_t ElementSize<Mat3X>(const Mat3X& A);
template<>
size_t ElementSize<Mat>(const Mat& A);
template<>
size_t ElementSize<std::vector<Vec2>>(const std::vector<Vec2>& A);

template<typename T>
struct Element;

template<>
struct Element<Mat2X>
{
    typedef Mat2X::ConstColXpr const_type;
    typedef Mat2X::ColXpr type;

    static Mat2X create(size_t elementSize, size_t count) { return Mat2X(elementSize, count); }
};

template<>
struct Element<Mat3X>
{
    typedef Mat3X::ConstColXpr const_type;
    typedef Mat3X::ColXpr type;

    static Mat3X create(size_t elementSize, size_t count) { return Mat3X(elementSize, count); }
};

template<>
struct Element<Mat>
{
    typedef Mat::ConstColXpr const_type;
    typedef Mat::ColXpr type;

    static Mat create(size_t elementSize, size_t count) { return Mat(elementSize, count); }
};

template<>
struct Element<std::vector<Vec2>>
{
    typedef const Vec2& const_type;
    typedef Vec2& type;

    static std::vector<Vec2> create(size_t elementSize, size_t count) { return std::vector<Vec2>(count); }
};

template<typename TMat>
typename Element<TMat>::const_type getElement(const TMat& A, size_t index)
{
    static_assert(sizeof(TMat) != sizeof(TMat), "CountElements must be specialized");
    return 0;
}

template<typename TMat>
typename Element<TMat>::type getElement(TMat& A, size_t index)
{
    static_assert(sizeof(TMat) != sizeof(TMat), "CountElements must be specialized");
    return 0;
}

template<>
typename Element<Mat2X>::const_type getElement<Mat2X>(const Mat2X& A, size_t index);
template<>
typename Element<Mat3X>::const_type getElement<Mat3X>(const Mat3X& A, size_t index);
template<>
typename Element<Mat>::const_type getElement<Mat>(const Mat& A, size_t index);
template<>
typename Element<std::vector<Vec2>>::const_type getElement<std::vector<Vec2>>(const std::vector<Vec2>& A, size_t index);

template<>
typename Element<Mat2X>::type getElement<Mat2X>(Mat2X& A, size_t index);
template<>
typename Element<Mat3X>::type getElement<Mat3X>(Mat3X& A, size_t index);
template<>
typename Element<Mat>::type getElement<Mat>(Mat& A, size_t index);
template<>
typename Element<std::vector<Vec2>>::type getElement<std::vector<Vec2>>(std::vector<Vec2>& A, size_t index);

/**
 * @brief It extracts the columns of given indices from the given matrix
 *
 * @param[in] A The NxM input matrix
 * @param[in] columns The list of K indices to extract
 * @return A NxK matrix
 */
template<typename TMat, typename TCols>
TMat buildSubsetMatrix(const TMat& A, const TCols& columns)
{
    using T = Element<TMat>;
    TMat compressed = T::create(ElementSize(A), columns.size());
    for (std::size_t i = 0; i < static_cast<std::size_t>(columns.size()); ++i)
    {
        // check for indices out of range
        assert(columns[i] < CountElements(A));
        getElement(compressed, i) = getElement(A, columns[i]);
    }

    return compressed;
}

}  // namespace aliceVision
