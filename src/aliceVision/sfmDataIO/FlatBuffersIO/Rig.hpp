// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/types.hpp>
#include <aliceVision/sfmData/Rig.hpp>


namespace aliceVision {
namespace sfmDataIO {


AliceVisionIO::V1::RIGSUBPOSESTATUS Pack(const sfmData::ERigSubPoseStatus val)
{
    return static_cast<AliceVisionIO::V1::RIGSUBPOSESTATUS>(val);
}  

sfmData::ERigSubPoseStatus Unpack(const AliceVisionIO::V1::RIGSUBPOSESTATUS val)
{
    return static_cast<sfmData::ERigSubPoseStatus>(val);
}  

/**
 * Pack a sfmData::RigSubPose to flatbuffers
 * @param builder the flatbuffers builder to use
 * @param obj the subpose to serialize
 * @return flatbuffers object
*/
flatbuffers::Offset<AliceVisionIO::V1::RigSubPose> Pack(flatbuffers::FlatBufferBuilder & builder, 
                                        const sfmData::RigSubPose & obj)
{   
    AliceVisionIO::Vec3 packedRotation = Pack(SO3::logm(obj.pose.rotation()));
    AliceVisionIO::Vec3 packedCenter = Pack(obj.pose.center());

    return AliceVisionIO::V1::CreateRigSubPose(builder, 
                                            Pack(obj.status),
                                            &packedRotation,
                                            &packedCenter);
}


/**
 * Pack a sfmData::Rig to flatbuffers
 * @param builder the flatbuffers builder to use
 * @param obj the Rig to serialize
 * @param index the Rig id
 * @return flatbuffers object
*/
flatbuffers::Offset<AliceVisionIO::V1::Rig> Pack(flatbuffers::FlatBufferBuilder & builder, 
                                    const sfmData::Rig & obj, 
                                    IndexT index)
{   
    std::vector<flatbuffers::Offset<AliceVisionIO::V1::RigSubPose>> sub_poses;
    
    for (const auto & item : obj.getSubPoses())
    {
        sub_poses.push_back(Pack(builder, item));
    } 

    return AliceVisionIO::V1::CreateRigDirect(builder, 
                                            index,
                                            &sub_poses);
}

void Unpack(sfmData::Rigs & outRigs, const flatbuffers::Vector<flatbuffers::Offset<AliceVisionIO::V1::Rig>> * obj)
{
    if (!obj)
    {
        return;
    }
    
    outRigs.clear();

    for (const auto & rig: *obj)
    {
        sfmData::Rig outRig;

        for (const auto & subpose: *rig->sub_poses())
        {
            sfmData::RigSubPose rsp;
            rsp.status = Unpack(subpose->status());
            
            Eigen::Matrix3d R = SO3::expm(Unpack(*subpose->rotation()));
            rsp.pose = geometry::Pose3(R, Unpack(*subpose->center()));

            outRig.getSubPoses().push_back(rsp);
        }

        outRigs[rig->id()] = outRig;
    }
    
}

}
}