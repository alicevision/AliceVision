// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Container.hpp"

namespace aliceVision {

template<>
size_t CountElements<Mat2X>(const Mat2X& A)
{
    return A.cols();
}

template<>
size_t CountElements<Mat3X>(const Mat3X& A)
{
    return A.cols();
}

template<>
size_t CountElements<Mat>(const Mat& A)
{
    return A.cols();
}

template<>
size_t CountElements<std::vector<Vec2>>(const std::vector<Vec2>& A)
{
    return A.size();
}

template<>
size_t ElementSize<Mat2X>(const Mat2X& A)
{
    return 2;
}

template<>
size_t ElementSize<Mat3X>(const Mat3X& A)
{
    return 3;
}

template<>
size_t ElementSize<Mat>(const Mat& A)
{
    return A.rows();
}

template<>
size_t ElementSize<std::vector<Vec2>>(const std::vector<Vec2>& A)
{
    return 2;
}

template<>
Element<Mat2X>::const_type getElement<Mat2X>(const Mat2X& A, size_t index)
{
    return A.col(index);
}

template<>
Element<Mat2X>::type getElement<Mat2X>(Mat2X& A, size_t index)
{
    return A.col(index);
}

template<>
Element<Mat3X>::const_type getElement<Mat3X>(const Mat3X& A, size_t index)
{
    return A.col(index);
}

template<>
Element<Mat3X>::type getElement<Mat3X>(Mat3X& A, size_t index)
{
    return A.col(index);
}

template<>
Element<Mat>::const_type getElement<Mat>(const Mat& A, size_t index)
{
    return A.col(index);
}

template<>
Element<Mat>::type getElement<Mat>(Mat& A, size_t index)
{
    return A.col(index);
}

template<>
Element<std::vector<Vec2>>::const_type getElement<std::vector<Vec2>>(const std::vector<Vec2>& A, size_t index)
{
    return A[index];
}

template<>
Element<std::vector<Vec2>>::type getElement<std::vector<Vec2>>(std::vector<Vec2>& A, size_t index)
{
    return A[index];
}

}  // namespace aliceVision
