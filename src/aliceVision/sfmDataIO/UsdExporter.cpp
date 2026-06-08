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
#include <pxr/usd/usdGeom/sphere.h>
#include <pxr/usd/usdGeom/sphere.h>
#include <pxr/usd/usdGeom/xformCommonAPI.h>

#include <pxr/base/gf/vec3f.h>
#include <pxr/base/vt/array.h>

#include <Eigen/Eigenvalues>

#include <Eigen/Eigenvalues>


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

void UsdExporter::addFrameWithUncertainty(const std::string & cameraName,
                                          const sfmData::CameraPose & pose,
                                          const camera::Pinhole & intrinsic,
                                          const sfmData::PoseUncertainty & uncertainty,
                                          IndexT frameId)
{
    // Write the regular camera keyframe first
    addFrame(cameraName, pose, intrinsic, frameId);

    // --- Ellipsoid from position covariance (world frame) ---
    // DOF ordering: [angleAxis(0-2), center(3-5)]
    // Position covariance is the bottom-right 3x3 block.
    const Eigen::Matrix3d posCov = uncertainty.block<3, 3>(3, 3);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(posCov);
    const Eigen::Vector3d semiAxes = solver.eigenvalues().cwiseMax(0.0).cwiseSqrt().cwiseMax(1e-10);
    Eigen::Matrix3d eigVecs = solver.eigenvectors();
    // Ensure proper rotation (det = +1)
    if (eigVecs.determinant() < 0.0)
    {
        eigVecs.col(2) = -eigVecs.col(2);
    }

    // Camera center in world space (CV convention)
    const Vec3 center = pose.getTransform().center();

    // Build ellipsoid transform in CV world frame (no camera rotation)
    Eigen::Matrix4d ellipsoidCV = Eigen::Matrix4d::Identity();
    ellipsoidCV.block<3, 3>(0, 0) = eigVecs * semiAxes.asDiagonal();
    ellipsoidCV.block<3, 1>(0, 3) = center;

    // Convert CV -> CG convention (flip Y and Z): T_CG = M * T_CV * M
    Eigen::Matrix4d Mflip = Eigen::Matrix4d::Identity();
    Mflip(1, 1) = -1.0;
    Mflip(2, 2) = -1.0;
    const Eigen::Matrix4d ellipsoidCG = Mflip * ellipsoidCV * Mflip;

    // USD matrix is column-major: [col][row]
    GfMatrix4d usdEllipsoid;
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            usdEllipsoid[j][i] = ellipsoidCG(i, j);
        }
    }

    // Lazy-create the ellipsoid prim as a sibling of the camera under /World
    const std::string ellipsoidName = cameraName + "_uncertainty";
    SdfPath ellipsoidXformPath("/World/" + ellipsoidName);
    SdfPath spherePath("/World/" + ellipsoidName + "/sphere");

    if (!_stage->GetPrimAtPath(ellipsoidXformPath))
    {
        UsdGeomXform ellipsoidXform = UsdGeomXform::Define(_stage, ellipsoidXformPath);
        ellipsoidXform.MakeMatrixXform();
        // Define a unit sphere — the xform carries the scale+orientation+translation
        UsdGeomSphere sphere = UsdGeomSphere::Define(_stage, spherePath);
        sphere.GetRadiusAttr().Set(1.0);
    }

    UsdPrim ellipsoidXformPrim = _stage->GetPrimAtPath(ellipsoidXformPath);
    UsdGeomXformable ellipsoidXformable(ellipsoidXformPrim);
    bool dummy = false;
    std::vector<UsdGeomXformOp> ellipsoidOps = ellipsoidXformable.GetOrderedXformOps(&dummy);
    if (!ellipsoidOps.empty())
    {
        ellipsoidOps[0].Set(usdEllipsoid, UsdTimeCode(frameId));
    }
}

}  // namespace sfmDataIO
}  // namespace aliceVision
