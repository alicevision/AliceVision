// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/types.hpp>
#include <aliceVision/sfmData/Landmark.hpp>


namespace aliceVision {
namespace sfmDataIO {

AliceVisionIO::V1::IMAGEDESCRIBERTYPE Pack(const feature::EImageDescriberType val)
{
    return static_cast<AliceVisionIO::V1::IMAGEDESCRIBERTYPE>(val);
}  

feature::EImageDescriberType Unpack(const AliceVisionIO::V1::IMAGEDESCRIBERTYPE val)
{
    return static_cast<feature::EImageDescriberType>(val);
} 

/**
 * @brief Convert a vec3 to a flatbuffers vec3
 * @param obj an input vec3
 * @return a flatbuffer object containing the vec2
*/
AliceVisionIO::Rgb Pack(const image::RGBColor & obj)
{
    return AliceVisionIO::Rgb(obj.x(), obj.y(), obj.z());
}

image::RGBColor Unpack(const AliceVisionIO::Rgb & obj)
{
    return image::RGBColor(obj.r(), obj.g(), obj.b());
}

std::map<Pair, const sfmData::Observation*> getObservations(const sfmData::Landmarks & landmarks)
{
    std::map<Pair, const sfmData::Observation*> ret;

    for (const auto & [idLandmark, landmark] : landmarks)
    {
        for (const auto & [idView, observation] : landmark.getObservations())
        {
            Pair p = std::make_pair(idLandmark, idView);
            ret[p] = &observation;
        }
    }

    return ret;
}

AliceVisionIO::V1::Observation Pack(flatbuffers::FlatBufferBuilder & builder, 
                                                        const sfmData::Observation & obj, 
                                                        IndexT indexLandmark, 
                                                        IndexT indexView) 
{    
    AliceVisionIO::Vec2 vec = Pack(obj.getCoordinates());

    
    return AliceVisionIO::V1::Observation(indexLandmark, 
                                        indexView, 
                                        vec, 
                                        obj.getFeatureId(), 
                                        obj.getScale());
}

std::vector<AliceVisionIO::V1::Observation> Pack(flatbuffers::FlatBufferBuilder & builder, const std::map<Pair, const sfmData::Observation*> & obj)
{
    std::vector<AliceVisionIO::V1::Observation> ret;

    for (const auto & [pair, observationPtr]: obj)
    {
        ret.push_back(Pack(builder, *observationPtr, pair.first, pair.second));
    }

    return ret;
}

flatbuffers::Offset<AliceVisionIO::V1::Landmark> Pack(flatbuffers::FlatBufferBuilder & builder, 
                                                const sfmData::Landmark & obj, 
                                                IndexT index)
{   
    AliceVisionIO::Vec3 X = Pack(obj.X);
    AliceVisionIO::Rgb rgb = Pack(obj.rgb);

    return AliceVisionIO::V1::CreateLandmark(builder, 
                                            index, 
                                            &X, 
                                            Pack(obj.descType), 
                                            &rgb);
}

void Unpack(std::map<IndexT, sfmData::Landmark> & outLandmarks, 
            const flatbuffers::Vector<flatbuffers::Offset<AliceVisionIO::V1::Landmark>> * obj)
{
    outLandmarks.clear();

    for (const auto & landmark: *obj)
    {
        IndexT id = landmark->id();

        sfmData::Landmark l(Unpack(*landmark->coordinates()),
                            Unpack(landmark->describer_type()),
                            Unpack(*landmark->rgb()));

        outLandmarks.emplace(id, l);
    }
}

void Unpack(std::map<aliceVision::Pair, sfmData::Observation> & outObservations, 
            const flatbuffers::Vector<const AliceVisionIO::V1::Observation*> * obj)
{
    if (!obj)
    {
        return;
    }
    
    outObservations.clear();

    for (const auto & observation: *obj)
    {
        aliceVision::Pair id;
        id.first = observation->id_landmark();
        id.second = observation->id_view();
        

        sfmData::Observation obs(Unpack(observation->coordinates()),
                                observation->id_feature(),
                                observation->scale());

        outObservations.emplace(id, obs);
    }
}

}
}