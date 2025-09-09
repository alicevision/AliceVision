// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/types.hpp>
#include <aliceVision/sfmData/CameraPose.hpp>
#include <aliceVision/sfmDataIO/FlatBuffersIO/Eigen.hpp>

namespace aliceVision {
namespace sfmDataIO {

/**
 * Pack a sfmData::CameraPose to flatbuffers
 * @param builder the flatbuffers builder to use
 * @param obj the CameraPose to serialize
 * @param index the CameraPose id
 * @return flatbuffers object
*/
flatbuffers::Offset<AliceVisionIO::V1::CameraPose> Pack(flatbuffers::FlatBufferBuilder & builder, 
                                        const sfmData::CameraPose & obj, 
                                        IndexT index)
{   
    const geometry::Pose3 & p = obj.getTransform();

    AliceVisionIO::Vec3 rotation = sfmDataIO::Pack(SO3::logm(p.rotation()));
    AliceVisionIO::Vec3 center = Pack(p.center());

    return AliceVisionIO::V1::CreateCameraPose(builder, 
                                            index, 
                                            &rotation, 
                                            &center, 
                                            obj.isLocked(), 
                                            obj.isRotationOnly(), 
                                            obj.isRemovable());
}

/**
 * Unpack a sfmData::CameraPose from flatbuffers
 * @param outPose the CameraPose to construct
 * @param obj the CameraPose flatbuffer object
*/
void Unpack(sfmData::CameraPose & outPose, 
            const AliceVisionIO::V1::CameraPose & obj)
{
    Eigen::Matrix3d R = SO3::expm(Unpack(*obj.rotation()));
    geometry::Pose3 p(R, Unpack(*obj.center()));

    sfmData::CameraPose cp(p);
    outPose.setRotationOnly(obj.rotation_only());
    
    if (obj.locked())
    {
        outPose.lock();
    }
    else
    {
        outPose.unlock();
    }

    outPose.setRemovable(obj.removable());
}

/**
 * Unpack a vector of flatbuffers camerapose to sfmData::Poses
 * @param outPoses the map of CameraPose to construct
 * @param obj the CameraPose flatbuffer vector
*/
void Unpack(sfmData::Poses & outPoses, const flatbuffers::Vector<flatbuffers::Offset<AliceVisionIO::V1::CameraPose>> * obj)
{
    if (!obj)
    {
        return;
    }
    
    outPoses.clear();

    for (const auto & pose: *obj)
    {
        sfmData::CameraPose cp;
        Unpack(cp, *pose);

        outPoses.assign(pose->id(), cp);
    }
    
}

}
}