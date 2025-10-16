// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <string>
#include <vector>
#include <map>
#include <concepts>


namespace aliceVision {
namespace sfmDataIO {


/**
 * @brief Convert a flatbuffers string to a std::string
 * @param obj an input flatbuffers string
 * @return an std::string
*/
std::string Unpack(const flatbuffers::String & obj)
{
    return obj.str();
}

/**
 * @brief Convert a std::string  to a flatbuffers string
 * @param builder an instance of  FlatBufferBuilder used to do the computation
 * @param obj an input std::string
 * @return a flatbuffers string
*/
flatbuffers::Offset<flatbuffers::String> Pack(flatbuffers::FlatBufferBuilder & builder, const std::string & obj)
{
    return builder.CreateString(obj.c_str());
}

/**
 * @brief enable serialization of shared_ptr
 * @param builder an instance of  FlatBufferBuilder used to do the computation
 * @param obj an input shared_ptr containing a pointer to T
 * @param index T object index 
*/
template <typename PackedType, typename UnpackedType>
flatbuffers::Offset<PackedType> Pack(flatbuffers::FlatBufferBuilder & builder, 
                        const std::shared_ptr<UnpackedType> & obj, 
                        IndexT index)
{
    return Pack(builder, *obj, index);
}

/**
 * @brief enable serialization of containers
 * @param builder an instance of  FlatBufferBuilder used to do the computation
 * @param obj an input associative container 
 * @return a vector of flatbuffers objects
*/
template <typename PackedType, typename UnpackedType>
std::vector<flatbuffers::Offset<PackedType>> Pack(flatbuffers::FlatBufferBuilder & builder, const std::vector<UnpackedType> & obj)
{
    std::vector<flatbuffers::Offset<PackedType>> ret;

    for (const auto & item: obj)
    {
        ret.push_back(Pack(builder, item));
    }

    return ret;
}

/**
 * @brief enable serialization of containers
 * @param builder an instance of  FlatBufferBuilder used to do the computation
 * @param obj an input associative container 
 * @return a vector of flatbuffers objects
*/
template <typename PackedType, typename UnpackedType>
std::vector<flatbuffers::Offset<PackedType>> Pack(flatbuffers::FlatBufferBuilder & builder, const std::vector<std::shared_ptr<UnpackedType>> & obj)
{
    std::vector<flatbuffers::Offset<PackedType>> ret;

    for (const auto & item: obj)
    {
        ret.push_back(Pack<PackedType>(builder, item));
    }

    return ret;
}

/**
 * @brief enable serialization of containers
 * @param builder an instance of  FlatBufferBuilder used to do the computation
 * @param obj an input associative container 
 * @return a vector of flatbuffers objects
*/
template <typename PackedType, typename UnpackedType>
std::vector<flatbuffers::Offset<PackedType>> Pack(flatbuffers::FlatBufferBuilder & builder, 
                                    const std::map<IndexT, UnpackedType> & obj)
{
    std::vector<flatbuffers::Offset<PackedType>> ret;

    for (const auto & [index, item]: obj)
    {
        ret.push_back(Pack(builder, item, index));
    }

    return ret;
}

/**
 * @brief enable serialization of containers
 * @param builder an instance of  FlatBufferBuilder used to do the computation
 * @param obj an input associative container 
 * @return a vector of flatbuffers objects
*/
template <typename PackedType, typename UnpackedType>
std::vector<flatbuffers::Offset<PackedType>> Pack(flatbuffers::FlatBufferBuilder & builder, 
                                    const std::map<IndexT, std::vector<UnpackedType>> & obj)
{
    std::vector<flatbuffers::Offset<PackedType>> ret;

    for (const auto & [index, vec]: obj)
    {
        for (const auto & item : vec)
        {
            ret.push_back(Pack(builder, item, index));
        }
    }

    return ret;
}

/**
 * @brief enable serialization of containers
 * @param builder an instance of  FlatBufferBuilder used to do the computation
 * @param obj an input associative container 
 * @return a vector of flatbuffers objects
*/
template <typename PackedType, typename UnpackedType>
std::vector<flatbuffers::Offset<PackedType>> Pack(flatbuffers::FlatBufferBuilder & builder, 
                                    const std::map<IndexT, const std::shared_ptr<UnpackedType>> & obj)
{
    std::vector<flatbuffers::Offset<PackedType>> ret;

    for (const auto & [index, item]: obj)
    {
        ret.push_back(Pack<PackedType, UnpackedType>(builder, item, index));
    }

    return ret;
}

/**
 * @brief enable serialization of containers
 * @param builder an instance of  FlatBufferBuilder used to do the computation
 * @param obj an input associative container 
 * @return a vector of flatbuffers objects
*/
template <typename PackedType, typename UnpackedType>
std::vector<::flatbuffers::Offset<PackedType>> Pack(::flatbuffers::FlatBufferBuilder & builder, 
                                    const sfmData::SharedPtrMap<UnpackedType> & obj)
{
    std::vector<flatbuffers::Offset<PackedType>> ret;

    for (const auto & [index, item]: obj)
    {
        ret.push_back(Pack<PackedType>(builder, item, index));
    }

    return ret;
}

/**
 * @brief enable deserialization of containers
 * @param obj an input flatbuffers vector of string (pointer to the vector)
 * @return a vector of string
*/
std::vector<std::string> Unpack(const flatbuffers::Vector<flatbuffers::Offset<flatbuffers::String>> * obj)
{
    std::vector<std::string> ret;

    for (const auto item: *obj)
    {
        ret.push_back(item->str());
    }

    return ret;
}



}
}