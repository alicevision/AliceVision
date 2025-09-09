// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/types.hpp>
#include <aliceVision/camera/IntrinsicBase.hpp>


namespace aliceVision {
namespace sfmDataIO {

AliceVisionIO::V1::INITMODE Pack(const camera::EInitMode val)
{
    return static_cast<AliceVisionIO::V1::INITMODE>(val);
}

camera::EInitMode Unpack(const AliceVisionIO::V1::INITMODE val)
{
    return static_cast<camera::EInitMode>(val);
}

AliceVisionIO::V1::TYPEINTRINSIC Pack(const camera::EINTRINSIC val)
{
    return static_cast<AliceVisionIO::V1::TYPEINTRINSIC>(val);
}

camera::EINTRINSIC Unpack(const AliceVisionIO::V1::TYPEINTRINSIC val)
{
    return static_cast<camera::EINTRINSIC>(val);
}

AliceVisionIO::V1::TYPEDISTORTION Pack(const camera::EDISTORTION val)
{
    return static_cast<AliceVisionIO::V1::TYPEDISTORTION>(val);
}

camera::EDISTORTION Unpack(const AliceVisionIO::V1::TYPEDISTORTION val)
{
    return static_cast<camera::EDISTORTION>(val);
}

AliceVisionIO::V1::TYPEUNDISTORTION Pack(const camera::EUNDISTORTION val)
{
    return static_cast<AliceVisionIO::V1::TYPEUNDISTORTION>(val);
} 

camera::EUNDISTORTION Unpack(const AliceVisionIO::V1::TYPEUNDISTORTION val)
{
    return static_cast<camera::EUNDISTORTION>(val);
} 

flatbuffers::Offset<AliceVisionIO::V1::Intrinsic> Pack(flatbuffers::FlatBufferBuilder & builder,  
                                        const camera::IntrinsicBase & obj, 
                                        IndexT index)
{
    const camera::IntrinsicScaleOffsetDisto & isod = dynamic_cast<const camera::IntrinsicScaleOffsetDisto &>(obj); 

    const AliceVisionIO::Vec2 packedOffset = Pack(isod.getOffset());

    double circleRadius = 0.0;
    Vec2 circleCenter(0.0, 0.0);

    if (obj.getType() == camera::EQUIDISTANT_CAMERA)
    {
        const camera::Equidistant & equidistant = dynamic_cast<const camera::Equidistant &>(obj); 
        circleRadius = equidistant.getCircleRadius();
        circleCenter = equidistant.getCircleCenter();
    }

    auto packedCircleCenter = Pack(circleCenter);


    return AliceVisionIO::V1::CreateIntrinsicDirect(
                                    builder, 
                                    index,     
                                    Pack(obj.getType()), 
                                    Pack(obj.getInitializationMode()),
                                    obj.isLocked(),
                                    obj.w(),
                                    obj.h(),
                                    obj.sensorWidth(),
                                    obj.sensorHeight(),
                                    obj.serialNumber().c_str(),
                                    isod.getFocalLength(),
                                    isod.getPixelAspectRatio(),
                                    isod.getInitialFocalLength(),
                                    &packedOffset,
                                    isod.isRatioLocked(),
                                    isod.isOffsetLocked(),
                                    isod.isScaleLocked(),
                                    Pack(isod.getDistortionInitializationMode()),
                                    circleRadius,
                                    &packedCircleCenter
                                    );
}

std::map<IndexT, const camera::Distortion::sptr> getDistortions(const sfmData::Intrinsics &intrinsics)
{
    std::map<IndexT, const camera::Distortion::sptr> ret;

    for (const auto & [idIntrinsic, intrinsic] : intrinsics.valueRange())
    {
        const camera::IntrinsicScaleOffsetDisto & isod = dynamic_cast<const camera::IntrinsicScaleOffsetDisto&>(intrinsic);
        
        const camera::Distortion::sptr d = isod.getDistortion();
        if (d)
        {
            ret.insert({idIntrinsic, d});
        }
    }

    return ret;
}

std::map<IndexT, const camera::Undistortion::sptr> getUndistortions(const sfmData::Intrinsics &intrinsics)
{
    std::map<IndexT, const camera::Undistortion::sptr> ret;

    for (const auto & [idIntrinsic, intrinsic] : intrinsics.valueRange())
    {
        const camera::IntrinsicScaleOffsetDisto & isod = dynamic_cast<const camera::IntrinsicScaleOffsetDisto&>(intrinsic);
        
        const camera::Undistortion::sptr d = isod.getUndistortion();
        if (d)
        {
            ret.insert({idIntrinsic, d});
        }
    }

    return ret;
}

flatbuffers::Offset<AliceVisionIO::V1::Distortion> Pack(flatbuffers::FlatBufferBuilder & builder, 
                                                    const camera::Distortion & obj, 
                                                    IndexT index)
{
    const std::vector<double> & params = obj.getParameters();
    return AliceVisionIO::V1::CreateDistortionDirect(
                                    builder, 
                                    index,     
                                    Pack(obj.getType()), 
                                    obj.isLocked(), 
                                    &params);
}

flatbuffers::Offset<AliceVisionIO::V1::Undistortion> Pack(flatbuffers::FlatBufferBuilder & builder, 
                                                        const camera::Undistortion & obj, 
                                                        IndexT index)
{
    const std::vector<double> & params = obj.getParameters();

    const AliceVisionIO::Vec2 packedSize = Pack(obj.getSize());
    const AliceVisionIO::Vec2 packedOffset = Pack(obj.getOffset());

    return AliceVisionIO::V1::CreateUndistortionDirect(
                                    builder, 
                                    index,     
                                    Pack(obj.getType()), 
                                    obj.isLocked(),
                                    &params,
                                    &packedSize,
                                    &packedOffset,
                                    obj.getPixelAspectRatio(),
                                    obj.isDesqueezed()
                                    );
}


void Unpack(sfmData::SharedPtrMap<camera::IntrinsicBase> & outIntrinsics, 
            const flatbuffers::Vector<flatbuffers::Offset<AliceVisionIO::V1::Intrinsic>> * obj)
{
    if (!obj)
    {
        return;
    }

    for (const auto & intrinsic: *obj)
    {
        IndexT id = intrinsic->id();

        
        camera::IntrinsicBase::sptr cam = createIntrinsic(Unpack(intrinsic->type()),
                                                        camera::EDISTORTION::DISTORTION_NONE,
                                                        camera::EUNDISTORTION::UNDISTORTION_NONE,
                                                        intrinsic->width(),
                                                        intrinsic->height());

        if (intrinsic->locked())
        {
            cam->lock();
        }
        else 
        {
            cam->unlock();
        }

        cam->setInitializationMode(Unpack(intrinsic->initialization_mode()));
        cam->setSensorWidth(intrinsic->sensor_width());
        cam->setSensorHeight(intrinsic->sensor_height());
        cam->setSerialNumber(intrinsic->serial_number()->str());
        cam->setDistortionInitializationMode(Unpack(intrinsic->distortion_initialization_mode()));
        
        auto isod = std::dynamic_pointer_cast<camera::IntrinsicScaleOffset>(cam);
        isod->setInitialFocalLength(intrinsic->initial_focal_length(), intrinsic->pixel_ratio());
        isod->setFocalLength(intrinsic->focal_length(), intrinsic->pixel_ratio());
        isod->setOffset(Unpack(*intrinsic->offset()));
        isod->setRatioLocked(intrinsic->ratio_locked());
        isod->setOffsetLocked(intrinsic->offset_locked());
        isod->setScaleLocked(intrinsic->scale_locked());

        auto equidistant = std::dynamic_pointer_cast<camera::Equidistant>(cam);
        if (equidistant)
        {
            equidistant->setCircleCenterX(intrinsic->circle_center()->x());
            equidistant->setCircleCenterY(intrinsic->circle_center()->y());
            equidistant->setCircleRadius(intrinsic->circle_radius());
        }

        outIntrinsics.emplace(id, cam);
    }
}

void Unpack(sfmData::SharedPtrMap<camera::Distortion> & outDistortions, 
            const flatbuffers::Vector<flatbuffers::Offset<AliceVisionIO::V1::Distortion>> * obj)
{
    if (!obj)
    {
        return;
    }

    outDistortions.clear();

    for (const auto & distortion: *obj)
    {
        IndexT id = distortion->id();

        std::vector<double> distortionParams;
        for (const double val: *distortion->parameters())
        {
            distortionParams.push_back(val);
        }
        
        auto outDistortion = createDistortion(Unpack(distortion->type()));
        outDistortion->setParameters(distortionParams);
        outDistortion->setLocked(distortion->locked());

        outDistortions.emplace(id, outDistortion);
    }
}

void Unpack(sfmData::SharedPtrMap<camera::Undistortion> & outUndistortions, 
            const flatbuffers::Vector<flatbuffers::Offset<AliceVisionIO::V1::Undistortion>> * obj)
{
    if (!obj)
    {
        return;
    }
    
    outUndistortions.clear();

    for (const auto & undistortion: *obj)
    {
        IndexT id = undistortion->id();

        std::vector<double> undistortionParams;
        for (const double val: *undistortion->parameters())
        {
            undistortionParams.push_back(val);
        }
        
        auto outUndistortion = createUndistortion(Unpack(undistortion->type()));

        outUndistortion->setLocked(undistortion->locked());
        outUndistortion->setParameters(undistortionParams);
        outUndistortion->setSize(Unpack(*undistortion->size()));
        outUndistortion->setOffset(Unpack(*undistortion->offset()));
        outUndistortion->setPixelAspectRatio(undistortion->pixel_aspect_ratio());
        outUndistortion->setDesqueezed(undistortion->is_desqueezed());

        outUndistortions.emplace(id, outUndistortion);
    }
}

}
}