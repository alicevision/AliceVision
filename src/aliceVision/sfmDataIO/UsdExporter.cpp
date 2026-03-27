// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmDataIO/UsdExporter.hpp>
#include <aliceVision/camera/IntrinsicScaleOffset.hpp>
#include <aliceVision/system/Logger.hpp>

#include <pxr/usd/usd/stage.h>

#include <pxr/usd/usdGeom/xform.h>
#include <pxr/usd/usdGeom/points.h>
#include <pxr/usd/usdGeom/camera.h>

#include <pxr/base/gf/vec3f.h>
#include <pxr/base/vt/array.h>


PXR_NAMESPACE_USING_DIRECTIVE

namespace aliceVision {
namespace sfmDataIO {


UsdExporter::UsdExporter(const std::string & filename, double frameRate)
{
    _stage = UsdStage::CreateNew(filename);
    if (!_stage) 
    {
        ALICEVISION_THROW_ERROR("UsdStage::CreateNew failed");
    }

    UsdGeomXform worldPrim = UsdGeomXform::Define(_stage, SdfPath("/World"));

    _stage->SetTimeCodesPerSecond(frameRate);
    _stage->SetFramesPerSecond(frameRate);
    _startTimeCode = std::numeric_limits<IndexT>::max();
    _endTimeCode = 0;
}

void UsdExporter::terminate()
{
    // If no frames have been exported, _startTimeCode will still be at its sentinel value.
    // In that case, avoid writing invalid start/end time codes to the USD stage.
    if (_startTimeCode == std::numeric_limits<IndexT>::max())
    {
        ALICEVISION_LOG_WARNING("UsdExporter::terminate called but no frames have been exported; "
                                "skipping start/end time code settings.");
    }
    else
    {
        _stage->SetStartTimeCode(_startTimeCode);
        _stage->SetEndTimeCode(_endTimeCode);
    }     

    _stage->Save();
}

void UsdExporter::createNewCamera(const std::string & cameraName)
{
    SdfPath cameraPath("/World/" + cameraName);
    UsdGeomCamera camera = UsdGeomCamera::Define(_stage, cameraPath);
    
    UsdAttribute projectionAttr = camera.GetProjectionAttr();
    projectionAttr.Set(UsdGeomTokens->perspective);
    
    UsdGeomXformable xformable(camera);
    UsdGeomXformOp motion = xformable.MakeMatrixXform();
    GfMatrix4d identity(1.0);
    motion.Set(identity);
}

void UsdExporter::addFrame(const std::string & cameraName, const sfmData::CameraPose & pose, const camera::Pinhole & intrinsic, IndexT frameId)
{
    SdfPath cameraPath("/World/" + cameraName);
    UsdGeomCamera camera = UsdGeomCamera::Get(_stage, cameraPath);

    _startTimeCode = std::min(_startTimeCode, frameId);
    _endTimeCode = std::max(_endTimeCode, frameId);

    UsdAttribute focalLengthAttr = camera.GetFocalLengthAttr();
    UsdAttribute horizontalApertureAttr = camera.GetHorizontalApertureAttr();
    UsdAttribute verticalApertureAttr = camera.GetVerticalApertureAttr();
    UsdAttribute horizontalApertureOffsetAttr = camera.GetHorizontalApertureOffsetAttr();
    UsdAttribute verticalApertureOffsetAttr = camera.GetVerticalApertureOffsetAttr();

    horizontalApertureAttr.Set(static_cast<float>(intrinsic.sensorWidth()));
    verticalApertureAttr.Set(static_cast<float>(intrinsic.sensorHeight()));
    
    UsdTimeCode t(frameId);
    double pixToMillimeters = intrinsic.sensorWidth() / intrinsic.w();

    horizontalApertureOffsetAttr.Set(static_cast<float>(intrinsic.getOffset().x() * pixToMillimeters), t);
    verticalApertureOffsetAttr.Set(static_cast<float>(intrinsic.getOffset().y() * pixToMillimeters), t);
    focalLengthAttr.Set(static_cast<float>(intrinsic.getFocalLength()), t);

    

    //Transform sfmData pose to usd pose
    Eigen::Matrix4d glTransform = Eigen::Matrix4d::Identity();
    glTransform(1, 1) = -1.0;
    glTransform(2, 2) = -1.0;

    // Inverse the pose and change the geometric frame
    Eigen::Matrix4d camera_T_world = pose.getTransform().getHomogeneous();
    Eigen::Matrix4d world_T_camera = camera_T_world.inverse();
    Eigen::Matrix4d world_gl_T_camera_gl = glTransform * world_T_camera * glTransform;

    //Copy element by element while transposing
    GfMatrix4d usdT;
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            usdT[j][i] = world_gl_T_camera_gl(i, j);
        }
    }
    
    //Assign pose to motion
    UsdGeomXformable xformable(camera);
    bool dummy = false;
    std::vector<UsdGeomXformOp> xformOps = xformable.GetOrderedXformOps(&dummy);

    
    if (!xformOps.empty()) {
        UsdGeomXformOp motion = xformOps[0];
        motion.Set(usdT, t);
    }
}

}  // namespace sfmDataIO
}  // namespace aliceVision
