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

void createCameras(const sfmData::SfMData & sfmData, UsdStageRefPtr & stage)
{
    SdfPath cameraPath("/World/Camera");

    UsdGeomCamera camera = UsdGeomCamera::Define(stage, cameraPath);
    UsdPrim cameraPrim = camera.GetPrim();

    UsdAttribute focalLengthAttr = camera.GetFocalLengthAttr();
    UsdAttribute horizontalApertureAttr = camera.GetHorizontalApertureAttr();
    UsdAttribute verticalApertureAttr = camera.GetVerticalApertureAttr();
    UsdAttribute horizontalApertureOffsetAttr = camera.GetHorizontalApertureOffsetAttr();
    UsdAttribute verticalApertureOffsetAttr = camera.GetVerticalApertureOffsetAttr();
    UsdAttribute projectionAttr = camera.GetProjectionAttr();
    projectionAttr.Set(UsdGeomTokens->perspective);

    UsdGeomXformable xformable = UsdGeomXformable(cameraPrim);

    Eigen::Matrix4d glTransform = Eigen::Matrix4d::Identity();
    glTransform(1, 1) = -1.0;
    glTransform(2, 2) = -1.0;

    UsdGeomXformOp motion = xformable.MakeMatrixXform();

    int pos = 0;
    bool first = true;
    for (const auto & [viewId, view] : sfmData.getViews().valueRange())
    {
        if (!sfmData.isPoseAndIntrinsicDefined(view))
        {
            continue;
        }

        sfmData::CameraPose cp = sfmData.getPose(view);
        const camera::IntrinsicBase * camPtr = sfmData.getIntrinsicPtr(view.getIntrinsicId());
        const camera::IntrinsicScaleOffset * iso = dynamic_cast<const camera::IntrinsicScaleOffset*>(camPtr);

        if (first)
        {
            horizontalApertureAttr.Set(static_cast<float>(camPtr->sensorWidth()));
            verticalApertureAttr.Set(static_cast<float>(camPtr->sensorHeight()));
            first = false;
        }

        Eigen::Matrix4d camera_T_world = cp.getTransform().getHomogeneous();
        Eigen::Matrix4d world_T_camera = camera_T_world.inverse();
        Eigen::Matrix4d world_gl_T_camera_gl = glTransform * world_T_camera * glTransform;

        GfMatrix4d usdT;
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                usdT[j][i] = world_gl_T_camera_gl(i, j);
            }
        }
        
        UsdTimeCode t(view.getFrameId());
        motion.Set(usdT, t);
        
        double pixToMillimeters = iso->sensorWidth() / iso->w();
        horizontalApertureOffsetAttr.Set(static_cast<float>(iso->getOffset().x() * pixToMillimeters), t);
        verticalApertureOffsetAttr.Set(static_cast<float>(iso->getOffset().y() * pixToMillimeters), t);
        focalLengthAttr.Set(static_cast<float>(iso->getFocalLength()), t);
    }
}

void createPointCloud(const sfmData::Landmarks & landmarks, UsdStageRefPtr & stage)
{
    SdfPath pointsPath("/World/PointCloud");
    UsdGeomPoints points = UsdGeomPoints::Define(stage, pointsPath);

    VtArray<GfVec3f> positions;
    VtArray<GfVec3f> colors;
    VtArray<long> ids;

    positions.reserve(landmarks.size());
    colors.reserve(colors.size());
    ids.reserve(ids.size());

    for (const auto & [lid, landmark] : landmarks)
    {
        ids.push_back(lid);
        positions.emplace_back(landmark.X.x(), -landmark.X.y(), -landmark.X.z());
        colors.emplace_back(landmark.rgb.r(), landmark.rgb.g(), landmark.rgb.b());
    }

    points.GetPointsAttr().Set(positions);
    points.GetIdsAttr().Set(ids);
    points.CreateDisplayColorPrimvar().Set(colors);
}

bool exportAnimatedUSD(const sfmData::SfMData & sfmData, const std::string & usdFilename)
{
    try 
    {
        UsdStageRefPtr stage = UsdStage::CreateNew(usdFilename);
        UsdGeomXform worldPrim = UsdGeomXform::Define(stage, SdfPath("/World"));

        //Find frames ranges
        IndexT frameMin = std::numeric_limits<IndexT>::max();
        IndexT frameMax = std::numeric_limits<IndexT>::min();
        for (const auto & [viewId, view] : sfmData.getViews().valueRange())
        {
            frameMin = std::min(frameMin, view.getFrameId());
            frameMax = std::max(frameMax, view.getFrameId());
        }

        if (frameMin >= frameMax)
        {
            ALICEVISION_LOG_ERROR("Only sequences are supported");
            return false;
        }

        IndexT length = frameMax - frameMin;
        if (length > sfmData.getViews().size())
        {
            ALICEVISION_LOG_ERROR("Only sequences are supported");
            return false;
        }

        stage->SetStartTimeCode(frameMin);
        stage->SetEndTimeCode(frameMax);       
        stage->SetTimeCodesPerSecond(24.0);
        stage->SetFramesPerSecond(24.0);

        createCameras(sfmData, stage);
        createPointCloud(sfmData.getLandmarks(), stage);

        
    } 
    catch (const std::exception& e) 
    {
        ALICEVISION_LOG_ERROR("Export failed");
        ALICEVISION_LOG_ERROR(e.what());
        return false;
    }

    return true;
}

UsdExporter::UsdExporter(const std::string & filename)
{
    _stage = UsdStage::CreateNew(usdFilename);
    UsdGeomXform worldPrim = UsdGeomXform::Define(_stage, SdfPath("/World"));
}

void UsdExporter::terminate()
{
    _stage->Save();
}

void UsdExporter::createNewCamera(long startTimeCode, long endTimeCode, double frameRate)
{
    _stage->SetStartTimeCode(startTimeCode);
    _stage->SetEndTimeCode(endTimeCode);       
    _stage->SetTimeCodesPerSecond(frameRate);
    _stage->SetFramesPerSecond(frameRate);

    SdfPath cameraPath("/World/Camera");
    
    UsdGeomCamera camera = UsdGeomCamera::Define(_stage, cameraPath);
    UsdPrim cameraPrim = camera.GetPrim();
}

}  // namespace sfmDataIO
}  // namespace aliceVision
