// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/types.hpp>
#include <aliceVision/sfmData/SurveyPoint.hpp>

namespace aliceVision {
namespace sfmDataIO {


flatbuffers::Offset<AliceVisionIO::V1::SurveyPoint> Pack(flatbuffers::FlatBufferBuilder & builder, const sfmData::SurveyPoint & obj, IndexT index)
{
    AliceVisionIO::Vec3 point = Pack(obj.point3d);
    AliceVisionIO::Vec2 survey = Pack(obj.survey);

    return AliceVisionIO::V1::CreateSurveyPoint(builder,
                                            index,
                                            &point,
                                            &survey);
}

void Unpack(sfmData::SurveyPoints & outSurveys, 
            const flatbuffers::Vector<flatbuffers::Offset<AliceVisionIO::V1::SurveyPoint>> * obj)
{
    if (!obj)
    {
        return;
    }
    
    outSurveys.clear();

    for (const auto & surveyPoint : *obj)
    {
        IndexT id = surveyPoint->id_view();

        sfmData::SurveyPoint sp(Unpack(*surveyPoint->point3d()),
                            Unpack(*surveyPoint->survey()));

        outSurveys[id].push_back(sp);
    }
}

}
}