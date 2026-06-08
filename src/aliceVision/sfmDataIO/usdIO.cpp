// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmDataIO/usdIO.hpp>

#include <aliceVision/camera/camera.hpp>
#include <aliceVision/camera/cameraCommon.hpp>
#include <aliceVision/camera/Distortion.hpp>
#include <aliceVision/camera/Equidistant.hpp>
#include <aliceVision/camera/Equirectangular.hpp>
#include <aliceVision/camera/IntrinsicScaleOffset.hpp>
#include <aliceVision/camera/IntrinsicScaleOffsetDisto.hpp>
#include <aliceVision/camera/Pinhole.hpp>
#include <aliceVision/camera/Undistortion.hpp>
#include <aliceVision/sfmData/ImageGroup.hpp>
#include <aliceVision/sfmData/LandmarkTable.hpp>
#include <aliceVision/sfmData/Observation.hpp>
#include <aliceVision/system/Logger.hpp>

#include <pxr/base/gf/half.h>
#include <pxr/base/gf/matrix4d.h>
#include <pxr/base/gf/matrix3d.h>
#include <pxr/base/gf/vec2d.h>
#include <pxr/base/gf/vec2f.h>
#include <pxr/base/gf/vec3f.h>
#include <pxr/base/gf/vec3i.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/sdf/payload.h>
#include <pxr/usd/sdf/types.h>
#include <pxr/usd/usd/attribute.h>
#include <pxr/usd/usd/payloads.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/relationship.h>
#include <pxr/usd/usdGeom/camera.h>
#include <pxr/usd/usdGeom/points.h>
#include <pxr/usd/usdGeom/primvarsAPI.h>
#include <pxr/usd/usdGeom/primvar.h>
#include <pxr/usd/usdGeom/tokens.h>
#include <pxr/usd/usdGeom/xformOp.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace aliceVision {
namespace sfmDataIO {

PXR_NAMESPACE_USING_DIRECTIVE

namespace fs = std::filesystem;

namespace {

/*
 * USD primer for reviewers unfamiliar with the format:
 * - A "stage" is the full scene/document being read or written.
 * - A "prim" is an object/node on the stage hierarchy (similar to a scene graph node).
 * - A "path" is the prim location (for example: /SfM/Views/view_000001).
 * - "Attributes" are typed values stored on a prim.
 * - "Relationships" are links to other prim paths.
 * - "Payload" is an external layer reference used to keep heavy data out of the main file.
 *
 * This file writes a custom SfM schema where most entities are prim-per-object, while the
 * landmark/observation data is stored as a compact table on a single Points prim.
 */

constexpr std::uint8_t kLandmarkParallaxRobustBit = 1u << 0;
constexpr std::uint8_t kLandmarkLockedBit = 1u << 1;

SdfPath childPath(const SdfPath& parent, const std::string& name)
{
    return parent.AppendChild(TfToken(name));
}

std::string idName(const std::string& prefix, std::uint32_t id)
{
    std::ostringstream oss;
    oss << prefix << "_" << std::setfill('0') << std::setw(6) << id;
    return oss.str();
}

// Thin wrapper used everywhere in this file to author a typed USD attribute.
template<typename T>
void setAttr(const UsdPrim& prim, const char* name, const SdfValueTypeName& typeName, const T& value)
{
    prim.CreateAttribute(TfToken(name), typeName).Set(value);
}

// Wrapper to author a relationship pointing to another prim path.
void setRelationship(const UsdPrim& prim, const char* name, const SdfPath& target)
{
    UsdRelationship rel = prim.CreateRelationship(TfToken(name));
    rel.SetTargets(SdfPathVector{target});
}

GfMatrix4d convertPoseToMayaWorldFromCamera(const geometry::Pose3& pose)
{
    Eigen::Matrix4d cvToMaya = Eigen::Matrix4d::Identity();
    cvToMaya(1, 1) = -1.0;
    cvToMaya(2, 2) = -1.0;

    Eigen::Matrix4d camera_T_world = pose.getHomogeneous();
    Eigen::Matrix4d world_T_camera = camera_T_world.inverse();
    Eigen::Matrix4d worldMaya_T_cameraMaya = cvToMaya * world_T_camera * cvToMaya;

    GfMatrix4d usd;
    for (int row = 0; row < 4; ++row)
    {
        for (int col = 0; col < 4; ++col)
        {
            usd[col][row] = worldMaya_T_cameraMaya(row, col);
        }
    }
    return usd;
}

GfMatrix3d convertRotationToMaya(const Eigen::Matrix3d& R)
{
    Eigen::Matrix3d cvToMaya = Eigen::Matrix3d::Identity();
    cvToMaya(1, 1) = -1.0;
    cvToMaya(2, 2) = -1.0;
    Eigen::Matrix3d mayaR = cvToMaya * R * cvToMaya;

    GfMatrix3d out(1.0);
    for (int r = 0; r < 3; ++r)
    {
        for (int c = 0; c < 3; ++c)
        {
            out[r][c] = mayaR(r, c);
        }
    }
    return out;
}

VtArray<std::string> metadataKeys(const sfmData::ImageInfo& image)
{
    VtArray<std::string> keys;
    for (const auto& [k, _] : image.getMetadata())
    {
        keys.push_back(k);
    }
    return keys;
}

VtArray<std::string> metadataValues(const sfmData::ImageInfo& image)
{
    VtArray<std::string> values;
    for (const auto& [_, v] : image.getMetadata())
    {
        values.push_back(v);
    }
    return values;
}

void writeDistortionObject(const UsdStageRefPtr& stage,
                           const UsdPrim& intrinsicPrim,
                           const std::shared_ptr<camera::Distortion>& distortion)
{
    if (!distortion)
    {
        return;
    }

    const SdfPath distortionPath = childPath(intrinsicPrim.GetPath(), "distortion");
    const UsdPrim distortionPrim = stage->DefinePrim(distortionPath, TfToken("AvDistortion"));
    setAttr(distortionPrim,
            "av:modelType",
            SdfValueTypeNames->Token,
            TfToken(camera::EDISTORTION_enumToString(distortion->getType())));
    setAttr(distortionPrim, "av:locked", SdfValueTypeNames->Bool, distortion->isLocked());

    VtArray<double> parameters;
    for (double p : distortion->getParameters())
    {
        parameters.push_back(p);
    }
    setAttr(distortionPrim, "av:parameters", SdfValueTypeNames->DoubleArray, parameters);

    setRelationship(intrinsicPrim, "av:distortion", distortionPath);
}

void writeUndistortionObject(const UsdStageRefPtr& stage,
                             const UsdPrim& intrinsicPrim,
                             const std::shared_ptr<camera::Undistortion>& undistortion)
{
    if (!undistortion)
    {
        return;
    }

    const SdfPath undistortionPath = childPath(intrinsicPrim.GetPath(), "undistortion");
    const UsdPrim undistortionPrim = stage->DefinePrim(undistortionPath, TfToken("AvUndistortion"));
    setAttr(undistortionPrim,
            "av:modelType",
            SdfValueTypeNames->Token,
            TfToken(camera::EUNDISTORTION_enumToString(undistortion->getType())));
    setAttr(undistortionPrim, "av:locked", SdfValueTypeNames->Bool, undistortion->isLocked());
    setAttr(undistortionPrim, "av:desqueezed", SdfValueTypeNames->Bool, undistortion->isDesqueezed());
    setAttr(undistortionPrim, "av:pixelAspectRatio", SdfValueTypeNames->Double, undistortion->getPixelAspectRatio());
    setAttr(undistortionPrim, "av:diagonal", SdfValueTypeNames->Double, undistortion->getDiagonal());

    const Vec2 size = undistortion->getSize();
    const Vec2 offset = undistortion->getOffset();
    setAttr(undistortionPrim, "av:size", SdfValueTypeNames->Double2, GfVec2d(size.x(), size.y()));
    setAttr(undistortionPrim, "av:offset", SdfValueTypeNames->Double2, GfVec2d(offset.x(), offset.y()));

    VtArray<double> parameters;
    for (double p : undistortion->getParameters())
    {
        parameters.push_back(p);
    }
    setAttr(undistortionPrim, "av:parameters", SdfValueTypeNames->DoubleArray, parameters);

    setRelationship(intrinsicPrim, "av:undistortion", undistortionPath);
}

struct ExportPaths
{
    // Root path for the exported SfM container.
    SdfPath sfmPath;

    // Child containers used to group each object family.
    SdfPath viewsPath;
    SdfPath viewImageInfosPath;
    SdfPath intrinsicsPath;
    SdfPath posesPath;
    SdfPath rigsPath;
    SdfPath ancestorsPath;
    SdfPath imageGroupsPath;
    SdfPath landmarksPath;
    SdfPath constraints2dPath;
    SdfPath constraintPointsPath;
    SdfPath rotationPriorsPath;
    SdfPath surveyPointsPath;
};

/**
 * @brief Create all canonical paths used by the USD writer and ensure they exist on stage.
 *
 * For non-USD readers:
 * - A USD "stage" is the output scene/document being written.
 * - A USD "path" is similar to a hierarchical folder/object path (for example: /SfM/Views).
 * - A USD "prim" is a node/object stored at a given path.
 *
 * This function centralizes all path names in one place so every writer uses the same hierarchy,
 * and it pre-creates the grouping nodes (typed as "Scope") so later writers can safely append
 * object prims under them.
 */
ExportPaths createExportPaths(const UsdStageRefPtr& stage, const SdfPath& sfmPath)
{
    ExportPaths paths;
    paths.sfmPath = sfmPath;
    paths.viewsPath = childPath(sfmPath, "Views");
    paths.viewImageInfosPath = childPath(sfmPath, "ViewImageInfos");
    paths.intrinsicsPath = childPath(sfmPath, "Intrinsics");
    paths.posesPath = childPath(sfmPath, "Poses");
    paths.rigsPath = childPath(sfmPath, "Rigs");
    paths.ancestorsPath = childPath(sfmPath, "Ancestors");
    paths.imageGroupsPath = childPath(sfmPath, "ImageGroups");
    paths.landmarksPath = childPath(sfmPath, "Landmarks");
    paths.constraints2dPath = childPath(sfmPath, "Constraints2D");
    paths.constraintPointsPath = childPath(sfmPath, "ConstraintPoints");
    paths.rotationPriorsPath = childPath(sfmPath, "RotationPriors");
    paths.surveyPointsPath = childPath(sfmPath, "SurveyPoints");

    // "Scope" is a lightweight USD container prim used only for grouping children.
    stage->DefinePrim(paths.viewsPath, TfToken("Scope"));
    stage->DefinePrim(paths.viewImageInfosPath, TfToken("Scope"));
    stage->DefinePrim(paths.intrinsicsPath, TfToken("Scope"));
    stage->DefinePrim(paths.posesPath, TfToken("Scope"));
    stage->DefinePrim(paths.rigsPath, TfToken("Scope"));
    stage->DefinePrim(paths.ancestorsPath, TfToken("Scope"));
    stage->DefinePrim(paths.imageGroupsPath, TfToken("Scope"));
    stage->DefinePrim(paths.constraints2dPath, TfToken("Scope"));
    stage->DefinePrim(paths.constraintPointsPath, TfToken("Scope"));
    stage->DefinePrim(paths.rotationPriorsPath, TfToken("Scope"));
    stage->DefinePrim(paths.surveyPointsPath, TfToken("Scope"));

    return paths;
}

void writeSfMMetadata(const sfmData::SfMData& sfmData, const UsdPrim& sfmPrim, ESfMData partFlag)
{
    setAttr(sfmPrim, "av:version", SdfValueTypeNames->Int3, GfVec3i(ALICEVISION_SFMDATAIO_VERSION_MAJOR, ALICEVISION_SFMDATAIO_VERSION_MINOR, ALICEVISION_SFMDATAIO_VERSION_REVISION));
    VtArray<std::string> features;
    for (const auto& f : sfmData.getRelativeFeaturesFolders())
    {
        features.push_back(f);
    }
    setAttr(sfmPrim, "av:featuresFolders", SdfValueTypeNames->StringArray, features);

    VtArray<std::string> matches;
    for (const auto& m : sfmData.getRelativeMatchesFolders())
    {
        matches.push_back(m);
    }
    setAttr(sfmPrim, "av:matchesFolders", SdfValueTypeNames->StringArray, matches);
}

void writeSfMRelationships(const UsdPrim& sfmPrim, const ExportPaths& paths)
{
    setRelationship(sfmPrim, "av:views", paths.viewsPath);
    setRelationship(sfmPrim, "av:viewImageInfos", paths.viewImageInfosPath);
    setRelationship(sfmPrim, "av:intrinsics", paths.intrinsicsPath);
    setRelationship(sfmPrim, "av:poses", paths.posesPath);
    setRelationship(sfmPrim, "av:rigs", paths.rigsPath);
    setRelationship(sfmPrim, "av:ancestors", paths.ancestorsPath);
    setRelationship(sfmPrim, "av:imageGroups", paths.imageGroupsPath);
    setRelationship(sfmPrim, "av:landmarks", paths.landmarksPath);
    setRelationship(sfmPrim, "av:constraints2d", paths.constraints2dPath);
    setRelationship(sfmPrim, "av:constraintPoints", paths.constraintPointsPath);
    setRelationship(sfmPrim, "av:rotationPriors", paths.rotationPriorsPath);
    setRelationship(sfmPrim, "av:surveyPoints", paths.surveyPointsPath);
}

void writeViewsAndViewImageInfos(const UsdStageRefPtr& stage, const sfmData::SfMData& sfmData, const ExportPaths& paths)
{
    std::uint32_t nextViewImageInfoId = 0;

    for (const auto& [viewId, viewPtr] : sfmData.getViews())
    {
        const sfmData::View& view = *viewPtr;
        const sfmData::ImageInfo& image = view.getImage();

        const std::uint32_t imageInfoId = nextViewImageInfoId++;
        (void)viewId;

        const UsdPrim imageInfoPrim = stage->DefinePrim(childPath(paths.viewImageInfosPath, idName("imageInfo", imageInfoId)), TfToken("AvImageInfo"));
        setAttr(imageInfoPrim, "av:id", SdfValueTypeNames->UInt, imageInfoId);
        setAttr(imageInfoPrim, "av:imagePath", SdfValueTypeNames->String, image.getImagePath());
        setAttr(imageInfoPrim, "av:imageWidth", SdfValueTypeNames->UInt, static_cast<std::uint32_t>(image.getWidth()));
        setAttr(imageInfoPrim, "av:imageHeight", SdfValueTypeNames->UInt, static_cast<std::uint32_t>(image.getHeight()));
        setAttr(imageInfoPrim, "av:metadataKeys", SdfValueTypeNames->StringArray, metadataKeys(image));
        setAttr(imageInfoPrim, "av:metadataValues", SdfValueTypeNames->StringArray, metadataValues(image));

        const UsdPrim viewPrim = stage->DefinePrim(childPath(paths.viewsPath, idName("view", view.getViewId())), TfToken("AvView"));
        setAttr(viewPrim, "av:viewId", SdfValueTypeNames->UInt, static_cast<std::uint32_t>(view.getViewId()));
        setAttr(viewPrim, "av:imageInfoId", SdfValueTypeNames->UInt, imageInfoId);
        setAttr(viewPrim, "av:intrinsicId", SdfValueTypeNames->UInt, static_cast<std::uint32_t>(view.getIntrinsicId()));
        setAttr(viewPrim, "av:poseId", SdfValueTypeNames->UInt, static_cast<std::uint32_t>(view.getPoseId()));
        setAttr(viewPrim, "av:rigId", SdfValueTypeNames->UInt, static_cast<std::uint32_t>(view.getRigId()));
        setAttr(viewPrim, "av:subPoseId", SdfValueTypeNames->UInt, static_cast<std::uint32_t>(view.getSubPoseId()));
        setAttr(viewPrim, "av:frameId", SdfValueTypeNames->UInt, static_cast<std::uint32_t>(view.getFrameId()));
        setAttr(viewPrim, "av:resectionId", SdfValueTypeNames->UInt, static_cast<std::uint32_t>(view.getResectionId()));
        setAttr(viewPrim, "av:imageGroupId", SdfValueTypeNames->UInt, static_cast<std::uint32_t>(view.getImageGroupId()));

        VtArray<std::uint32_t> ancestorIds;
        for (const IndexT a : view.getAncestors())
        {
            ancestorIds.push_back(static_cast<std::uint32_t>(a));
        }
        setAttr(viewPrim, "av:ancestorIds", SdfValueTypeNames->UIntArray, ancestorIds);
    }
}

void writeAncestors(const UsdStageRefPtr& stage, const sfmData::SfMData& sfmData, const ExportPaths& paths)
{
    for (const auto& [ancestorId, ancestorPtr] : sfmData.getAncestors())
    {
        const sfmData::ImageInfo& image = *ancestorPtr;
        const std::uint32_t imageInfoId = static_cast<std::uint32_t>(ancestorId);

        const UsdPrim ancestorPrim = stage->DefinePrim(childPath(paths.ancestorsPath, idName("ancestorImageInfo", imageInfoId)), TfToken("AvImageInfo"));
        setAttr(ancestorPrim, "av:id", SdfValueTypeNames->UInt, imageInfoId);
        setAttr(ancestorPrim, "av:imagePath", SdfValueTypeNames->String, image.getImagePath());
        setAttr(ancestorPrim, "av:imageWidth", SdfValueTypeNames->UInt, static_cast<std::uint32_t>(image.getWidth()));
        setAttr(ancestorPrim, "av:imageHeight", SdfValueTypeNames->UInt, static_cast<std::uint32_t>(image.getHeight()));
        setAttr(ancestorPrim, "av:metadataKeys", SdfValueTypeNames->StringArray, metadataKeys(image));
        setAttr(ancestorPrim, "av:metadataValues", SdfValueTypeNames->StringArray, metadataValues(image));
    }
}

void writePoses(const UsdStageRefPtr& stage, const sfmData::SfMData& sfmData, const ExportPaths& paths)
{
    for (const auto& [poseId, posePtr] : sfmData.getPoses())
    {
        const sfmData::CameraPose& pose = *posePtr;
        const UsdPrim posePrim = stage->DefinePrim(childPath(paths.posesPath, idName("pose", static_cast<std::uint32_t>(poseId))), TfToken("AvPose"));
        setAttr(posePrim, "av:id", SdfValueTypeNames->UInt, static_cast<std::uint32_t>(poseId));
        setAttr(posePrim, "av:worldFromCamera", SdfValueTypeNames->Matrix4d, convertPoseToMayaWorldFromCamera(pose.getTransform()));
        setAttr(posePrim, "av:state", SdfValueTypeNames->UChar, static_cast<std::uint8_t>(pose.getState()));
        setAttr(posePrim, "av:locked", SdfValueTypeNames->Bool, pose.isLocked());
        setAttr(posePrim, "av:rotationOnly", SdfValueTypeNames->Bool, pose.isRotationOnly());
        setAttr(posePrim, "av:removable", SdfValueTypeNames->Bool, pose.isRemovable());

        const auto it = sfmData.getPosesUncertainty().find(poseId);
        if (it != sfmData.getPosesUncertainty().end())
        {
            const auto& uncertainty = it->second;
            VtArray<double> uv(21);
            int idx = 0;
            for (int i = 0; i < 6; ++i)
            {
                for (int j = i; j < 6; ++j)
                {
                    uv[idx++] = uncertainty(i, j);
                }
            }
            setAttr(posePrim, "av:uncertainty", SdfValueTypeNames->DoubleArray, uv);
        }
    }
}

void writeIntrinsics(const UsdStageRefPtr& stage, const sfmData::SfMData& sfmData, const ExportPaths& paths)
{
    for (const auto& [intrinsicId, intrinsicPtr] : sfmData.getIntrinsics())
    {
        const auto& intrinsic = *intrinsicPtr;
        TfToken intrinsicTypeName("AvIntrinsic");
        if (std::dynamic_pointer_cast<camera::Pinhole>(intrinsicPtr))
        {
            intrinsicTypeName = TfToken("AvIntrinsicPinhole");
        }
        else if (std::dynamic_pointer_cast<camera::Equidistant>(intrinsicPtr))
        {
            intrinsicTypeName = TfToken("AvIntrinsicEquidistant");
        }
        else if (std::dynamic_pointer_cast<camera::Equirectangular>(intrinsicPtr))
        {
            intrinsicTypeName = TfToken("AvIntrinsicEquirectangular");
        }

        const UsdPrim intrinsicPrim = stage->DefinePrim(childPath(paths.intrinsicsPath, idName("intrinsic", static_cast<std::uint32_t>(intrinsicId))), intrinsicTypeName);
        setAttr(intrinsicPrim, "av:id", SdfValueTypeNames->UInt, static_cast<std::uint32_t>(intrinsicId));
        setAttr(intrinsicPrim, "av:modelType", SdfValueTypeNames->Token, TfToken(intrinsic.getTypeStr()));
        setAttr(intrinsicPrim, "av:locked", SdfValueTypeNames->Bool, intrinsic.isLocked());
        setAttr(intrinsicPrim, "av:imageWidth", SdfValueTypeNames->UInt, static_cast<std::uint32_t>(intrinsic.w()));
        setAttr(intrinsicPrim, "av:imageHeight", SdfValueTypeNames->UInt, static_cast<std::uint32_t>(intrinsic.h()));
        setAttr(intrinsicPrim, "av:sensorWidth", SdfValueTypeNames->Double, intrinsic.sensorWidth());
        setAttr(intrinsicPrim, "av:sensorHeight", SdfValueTypeNames->Double, intrinsic.sensorHeight());
        setAttr(intrinsicPrim, "av:serialNumber", SdfValueTypeNames->String, intrinsic.serialNumber());
        setAttr(intrinsicPrim,
                "av:initializationMode",
                SdfValueTypeNames->Int,
                static_cast<int>(intrinsic.getInitializationMode()));

        const auto scaleOffset = std::dynamic_pointer_cast<camera::IntrinsicScaleOffset>(intrinsicPtr);
        if (scaleOffset)
        {
            const Vec2 scale = scaleOffset->getScale();
            const Vec2 principalPoint = scaleOffset->getPrincipalPoint();
            setAttr(intrinsicPrim,
                "av:focalLengthPix",
                SdfValueTypeNames->Double2,
                GfVec2d(scale.x(), scale.y()));
            setAttr(intrinsicPrim,
                "av:principalPoint",
                SdfValueTypeNames->Double2,
                GfVec2d(principalPoint.x(), principalPoint.y()));
        }

        const auto equidistant = std::dynamic_pointer_cast<camera::Equidistant>(intrinsicPtr);
        if (equidistant)
        {
            const Vec2 center = equidistant->getCircleCenter();
            setAttr(intrinsicPrim, "av:circleRadius", SdfValueTypeNames->Double, equidistant->getCircleRadius());
            setAttr(intrinsicPrim,
                "av:circleCenter",
                SdfValueTypeNames->Double2,
                GfVec2d(center.x(), center.y()));
        }

        const auto withDisto = std::dynamic_pointer_cast<camera::IntrinsicScaleOffsetDisto>(intrinsicPtr);
        if (withDisto)
        {
            writeDistortionObject(stage, intrinsicPrim, withDisto->getDistortion());
            writeUndistortionObject(stage, intrinsicPrim, withDisto->getUndistortion());
        }
    }
}

/**
 * @brief Write one UsdGeomCamera prim per reconstructed view so that DCCs (Maya, Houdini, Blender, …)
 * can visualise the camera positions and FOV directly without parsing the AliceVision custom schema.
 *
 * Cameras are placed under a top-level "/Cameras" Scope (outside /SfM) so they are immediately
 * accessible in any DCC scene browser.
 *
 * For Pinhole-family intrinsics the film-gate attributes (focalLength, horizontalAperture, …) are
 * derived from the pixel-space intrinsic parameters and the physical sensor dimensions stored in the
 * intrinsic.  Non-Pinhole cameras (Equidistant, Equirectangular) receive only the world transform;
 * the projection attributes are left at their USD defaults.
 *
 * Coordinate convention: UsdGeomCamera lives in a Y-up, right-handed system with the camera looking
 * along −Z in camera space.  convertPoseToMayaWorldFromCamera() already produces a matrix in that
 * convention, so it is reused directly here.
 */
void writeDccCameras(const UsdStageRefPtr& stage, const sfmData::SfMData& sfmData, const ExportPaths& paths)
{
    const SdfPath camerasPath("/Cameras");
    stage->DefinePrim(camerasPath, TfToken("Scope"));

    for (const auto& [viewId, viewPtr] : sfmData.getViews())
    {
        const sfmData::View& view = *viewPtr;

        if (!sfmData.isPoseAndIntrinsicDefined(view))
        {
            continue;
        }

        const auto intrinsicIt = sfmData.getIntrinsics().find(view.getIntrinsicId());
        if (intrinsicIt == sfmData.getIntrinsics().end())
        {
            continue;
        }

        const sfmData::CameraPose cameraPose = sfmData.getPose(view);
        const auto& intrinsic = *intrinsicIt->second;

        const SdfPath cameraPath = childPath(camerasPath, idName("camera", static_cast<std::uint32_t>(viewId)));
        UsdGeomCamera usdCamera = UsdGeomCamera::Define(stage, cameraPath);
        if (!usdCamera)
        {
            continue;
        }

        // World transform (Y-up, camera looks along −Z — same convention used by UsdGeomCamera).
        const GfMatrix4d worldFromCamera = convertPoseToMayaWorldFromCamera(cameraPose.getTransform());
        usdCamera.MakeMatrixXform().Set(worldFromCamera);

        // Reference back to the AliceVision view prim so tools can cross-reference.
        setRelationship(usdCamera.GetPrim(), "av:view", childPath(paths.viewsPath, idName("view", static_cast<std::uint32_t>(viewId))));

        // Film-gate attributes – only meaningful for Pinhole-family cameras.
        const auto scaleOffset = std::dynamic_pointer_cast<camera::IntrinsicScaleOffset>(intrinsicIt->second);
        if (!scaleOffset)
        {
            // Non-Pinhole camera: write the transform only and mark it explicitly.
            usdCamera.GetProjectionAttr().Set(UsdGeomTokens->perspective);
            continue;
        }

        // Physical sensor dimensions in mm.  Fall back to a standard 35 mm full-frame sensor when
        // the intrinsic has not been calibrated against a known sensor size.
        double sensorW = intrinsic.sensorWidth();
        double sensorH = intrinsic.sensorHeight();
        if (sensorW <= 0.0) sensorW = 36.0;
        if (sensorH <= 0.0) sensorH = 24.0;

        const double imgW = static_cast<double>(intrinsic.w());
        const double imgH = static_cast<double>(intrinsic.h());
        if (imgW <= 0.0 || imgH <= 0.0)
        {
            continue;
        }

        const Vec2 scale = scaleOffset->getScale();  // (fx, fy) in pixels
        const Vec2 shift = scaleOffset->getOffset();  // (cx, cy) in pixels

        // Focal length in mm (use fx; fy is not represented in UsdGeomCamera).
        const float focalLength_mm = static_cast<float>(scale.x() * sensorW / imgW);

        // Aperture offsets: USD uses mm, Y-axis points up in camera space (opposite image row direction).
        const float horizOffset_mm = static_cast<float>(shift.x() * sensorW / imgW);
        const float vertOffset_mm  = static_cast<float>(- shift.y() * sensorH / imgH);

        usdCamera.GetProjectionAttr().Set(UsdGeomTokens->perspective);
        usdCamera.GetFocalLengthAttr().Set(focalLength_mm);
        usdCamera.GetHorizontalApertureAttr().Set(static_cast<float>(sensorW));
        usdCamera.GetVerticalApertureAttr().Set(static_cast<float>(sensorH));
        usdCamera.GetHorizontalApertureOffsetAttr().Set(horizOffset_mm);
        usdCamera.GetVerticalApertureOffsetAttr().Set(vertOffset_mm);
        usdCamera.GetClippingRangeAttr().Set(GfVec2f(0.1f, 1.0e6f));
    }
}

void writeRigs(const UsdStageRefPtr& stage, const sfmData::SfMData& sfmData, const ExportPaths& paths)
{
    for (const auto& [rigId, rig] : sfmData.getRigs())
    {
        const UsdPrim rigPrim = stage->DefinePrim(childPath(paths.rigsPath, idName("rig", static_cast<std::uint32_t>(rigId))), TfToken("AvRig"));
        setAttr(rigPrim, "av:id", SdfValueTypeNames->UInt, static_cast<std::uint32_t>(rigId));

        VtArray<std::uint32_t> subPoseIds;
        for (std::uint32_t subId = 0; subId < static_cast<std::uint32_t>(rig.getNbSubPoses()); ++subId)
        {
            subPoseIds.push_back(subId);
            const sfmData::RigSubPose& subPose = rig.getSubPose(subId);
            const UsdPrim subPosePrim = stage->DefinePrim(childPath(rigPrim.GetPath(), idName("subPose", subId)), TfToken("AvRigSubPose"));
            setAttr(subPosePrim, "av:id", SdfValueTypeNames->UInt, subId);
            setAttr(subPosePrim, "av:status", SdfValueTypeNames->UChar, static_cast<std::uint8_t>(subPose.status));
            setAttr(subPosePrim, "av:transform", SdfValueTypeNames->Matrix4d, convertPoseToMayaWorldFromCamera(subPose.pose));
        }
        setAttr(rigPrim, "av:subPoseIds", SdfValueTypeNames->UIntArray, subPoseIds);
    }
}

void writeImageGroups(const UsdStageRefPtr& stage, const sfmData::SfMData& sfmData, const ExportPaths& paths)
{
    for (const auto& [groupId, groupPtr] : sfmData.getImageGroups())
    {
        const UsdPrim groupPrim = stage->DefinePrim(childPath(paths.imageGroupsPath, idName("imageGroup", static_cast<std::uint32_t>(groupId))), TfToken("AvImageGroup"));
        setAttr(groupPrim, "av:id", SdfValueTypeNames->UInt, static_cast<std::uint32_t>(groupId));
        setAttr(groupPrim, "av:type", SdfValueTypeNames->Int, static_cast<int>(groupPtr->getType()));
    }
}

void writeConstraints2D(const UsdStageRefPtr& stage, const sfmData::SfMData& sfmData, const ExportPaths& paths)
{
    for (std::uint32_t cId = 0; cId < static_cast<std::uint32_t>(sfmData.getConstraints2D().size()); ++cId)
    {
        const sfmData::Constraint2D& c = sfmData.getConstraints2D().at(cId);
        const UsdPrim cPrim = stage->DefinePrim(childPath(paths.constraints2dPath, idName("constraint2D", cId)), TfToken("AvConstraint2D"));
        setAttr(cPrim, "av:descType", SdfValueTypeNames->UChar, static_cast<std::uint8_t>(c.descType));
        setAttr(cPrim, "av:viewFirst", SdfValueTypeNames->UInt, static_cast<std::uint32_t>(c.ViewFirst));
        setAttr(cPrim, "av:viewSecond", SdfValueTypeNames->UInt, static_cast<std::uint32_t>(c.ViewSecond));
        setAttr(cPrim, "av:observationFirstXY", SdfValueTypeNames->Float2, GfVec2f(static_cast<float>(c.ObservationFirst.getX()), static_cast<float>(c.ObservationFirst.getY())));
        setAttr(cPrim, "av:observationFirstFeatureId", SdfValueTypeNames->UInt, static_cast<std::uint32_t>(c.ObservationFirst.getFeatureId()));
        setAttr(cPrim, "av:observationFirstScale", SdfValueTypeNames->Half, GfHalf(static_cast<float>(c.ObservationFirst.getScale())));
        setAttr(cPrim, "av:observationFirstDepth", SdfValueTypeNames->Half, GfHalf(static_cast<float>(c.ObservationFirst.getDepth())));
        setAttr(cPrim, "av:observationSecondXY", SdfValueTypeNames->Float2, GfVec2f(static_cast<float>(c.ObservationSecond.getX()), static_cast<float>(c.ObservationSecond.getY())));
        setAttr(cPrim, "av:observationSecondFeatureId", SdfValueTypeNames->UInt, static_cast<std::uint32_t>(c.ObservationSecond.getFeatureId()));
        setAttr(cPrim, "av:observationSecondScale", SdfValueTypeNames->Half, GfHalf(static_cast<float>(c.ObservationSecond.getScale())));
        setAttr(cPrim, "av:observationSecondDepth", SdfValueTypeNames->Half, GfHalf(static_cast<float>(c.ObservationSecond.getDepth())));
    }
}

void writeConstraintPoints(const UsdStageRefPtr& stage, const sfmData::SfMData& sfmData, const ExportPaths& paths)
{
    for (const auto& [constraintId, c] : sfmData.getConstraintsPoint())
    {
        const UsdPrim cPrim = stage->DefinePrim(childPath(paths.constraintPointsPath, idName("constraintPoint", static_cast<std::uint32_t>(constraintId))), TfToken("AvConstraintPoint"));
        setAttr(cPrim, "av:id", SdfValueTypeNames->UInt, static_cast<std::uint32_t>(constraintId));
        setAttr(cPrim, "av:landmarkId", SdfValueTypeNames->UInt, static_cast<std::uint32_t>(c.landmarkId));
        setAttr(cPrim, "av:normal", SdfValueTypeNames->Float3, GfVec3f(static_cast<float>(c.normal.x()), static_cast<float>(c.normal.y()), static_cast<float>(c.normal.z())));
        setAttr(cPrim, "av:point", SdfValueTypeNames->Float3, GfVec3f(static_cast<float>(c.point.x()), static_cast<float>(c.point.y()), static_cast<float>(c.point.z())));
    }
}

void writeRotationPriors(const UsdStageRefPtr& stage, const sfmData::SfMData& sfmData, const ExportPaths& paths)
{
    for (std::uint32_t rId = 0; rId < static_cast<std::uint32_t>(sfmData.getRotationPriors().size()); ++rId)
    {
        const sfmData::RotationPrior& r = sfmData.getRotationPriors().at(rId);
        const UsdPrim rPrim = stage->DefinePrim(childPath(paths.rotationPriorsPath, idName("rotationPrior", rId)), TfToken("AvRotationPrior"));
        setAttr(rPrim, "av:id", SdfValueTypeNames->UInt, rId);
        setAttr(rPrim, "av:viewFirst", SdfValueTypeNames->UInt, static_cast<std::uint32_t>(r.ViewFirst));
        setAttr(rPrim, "av:viewSecond", SdfValueTypeNames->UInt, static_cast<std::uint32_t>(r.ViewSecond));
        setAttr(rPrim, "av:secondRFirst", SdfValueTypeNames->Matrix3d, convertRotationToMaya(r._second_R_first));
    }
}

void writeSurveyPoints(const UsdStageRefPtr& stage, const sfmData::SfMData& sfmData, const ExportPaths& paths)
{
    for (const auto& [groupId, points] : sfmData.getSurveyPoints())
    {
        const UsdPrim gPrim = stage->DefinePrim(childPath(paths.surveyPointsPath, idName("surveyGroup", static_cast<std::uint32_t>(groupId))), TfToken("AvSurveyPointGroup"));
        setAttr(gPrim, "av:id", SdfValueTypeNames->UInt, static_cast<std::uint32_t>(groupId));

        std::uint32_t localId = 0;
        for (const auto& sp : points)
        {
            const UsdPrim spPrim = stage->DefinePrim(childPath(gPrim.GetPath(), idName("surveyPoint", localId)), TfToken("AvSurveyPoint"));
            setAttr(spPrim, "av:id", SdfValueTypeNames->UInt, localId++);
            setAttr(spPrim, "av:point3d", SdfValueTypeNames->Float3, GfVec3f(static_cast<float>(sp.point3d.x()), static_cast<float>(sp.point3d.y()), static_cast<float>(sp.point3d.z())));
            setAttr(spPrim, "av:survey", SdfValueTypeNames->Float2, GfVec2f(static_cast<float>(sp.survey.x()), static_cast<float>(sp.survey.y())));
            setAttr(spPrim, "av:residual", SdfValueTypeNames->Float2, GfVec2f(static_cast<float>(sp.residual.x()), static_cast<float>(sp.residual.y())));
        }
    }
}

/*
 * Landmarks are exported as one UsdGeomPoints prim with side arrays.
 *
 * Why this layout:
 * - Point position/color are native USD geometry fields (fast to draw in DCCs).
 * - Landmark metadata and flattened observation arrays are stored as custom attributes.
 * - This avoids creating millions of tiny prims and keeps both disk size and load time low.
 */
void writeLandmarkTable(const UsdStageRefPtr& stage, const sfmData::SfMData& sfmData, const ExportPaths& paths)
{
    UsdGeomPoints usdPoints = UsdGeomPoints::Define(stage, paths.landmarksPath);
    const UsdPrim landmarksPrim = usdPoints.GetPrim();

    // Build a dense table representation from map-based SfM structures.
    sfmData::LandmarkTable table = sfmData::buildLandmarkTable(sfmData, true);

    VtArray<GfVec3f> points;
    points.reserve(table.points.size());
    for (const Vec3& p : table.points)
    {
        points.emplace_back(static_cast<float>(p.x()), static_cast<float>(-p.y()), static_cast<float>(-p.z()));
    }
    usdPoints.GetPointsAttr().Set(points);

    VtArray<GfVec3f> colors;
    colors.reserve(table.colors.size());
    for (const Vec3& c : table.colors)
    {
        colors.emplace_back(static_cast<float>(c.x()), static_cast<float>(c.y()), static_cast<float>(c.z()));
    }
    UsdGeomPrimvarsAPI(landmarksPrim).CreatePrimvar(TfToken("displayColor"), SdfValueTypeNames->Color3fArray, UsdGeomTokens->vertex).Set(colors);

    VtArray<std::uint32_t> ids;
    ids.reserve(table.ids.size());
    for (IndexT id : table.ids)
    {
        ids.push_back(static_cast<std::uint32_t>(id));
    }
    setAttr(landmarksPrim, "av:ids", SdfValueTypeNames->UIntArray, ids);

    VtArray<std::uint8_t> state = VtArray<std::uint8_t>(table.states.begin(), table.states.end());
    VtArray<std::uint8_t> desc = VtArray<std::uint8_t>(table.descTypes.begin(), table.descTypes.end());
    VtArray<std::uint8_t> flags = VtArray<std::uint8_t>(table.flags.begin(), table.flags.end());
    setAttr(landmarksPrim, "av:state", SdfValueTypeNames->UCharArray, state);
    setAttr(landmarksPrim, "av:descType", SdfValueTypeNames->UCharArray, desc);
    setAttr(landmarksPrim, "av:flags", SdfValueTypeNames->UCharArray, flags);

    VtArray<std::uint32_t> refView;
    refView.reserve(table.referenceViewIds.size());
    for (IndexT id : table.referenceViewIds)
    {
        refView.push_back(static_cast<std::uint32_t>(id));
    }
    setAttr(landmarksPrim, "av:referenceViewId", SdfValueTypeNames->UIntArray, refView);

    VtArray<std::uint32_t> obsOffsets;
    for (IndexT v : table.observationOffsets)
    {
        obsOffsets.push_back(static_cast<std::uint32_t>(v));
    }
    setAttr(landmarksPrim, "av:obsOffsets", SdfValueTypeNames->UIntArray, obsOffsets);

    VtArray<std::uint32_t> obsViewId;
    for (IndexT v : table.observationViewIds)
    {
        obsViewId.push_back(static_cast<std::uint32_t>(v));
    }
    setAttr(landmarksPrim, "av:obsViewId", SdfValueTypeNames->UIntArray, obsViewId);

    VtArray<std::uint32_t> obsFeatureId;
    for (IndexT v : table.observationFeatureIds)
    {
        obsFeatureId.push_back(static_cast<std::uint32_t>(v));
    }
    setAttr(landmarksPrim, "av:obsFeatureId", SdfValueTypeNames->UIntArray, obsFeatureId);

    VtArray<GfVec2f> obsXY;
    for (const Vec2& v : table.observationXY)
    {
        obsXY.emplace_back(static_cast<float>(v.x()), static_cast<float>(v.y()));
    }
    setAttr(landmarksPrim, "av:obsXY", SdfValueTypeNames->Float2Array, obsXY);

    VtArray<GfHalf> obsScale;
    for (float s : table.observationScales)
    {
        obsScale.push_back(GfHalf(s));
    }
    setAttr(landmarksPrim, "av:obsScale", SdfValueTypeNames->HalfArray, obsScale);

    VtArray<GfHalf> obsDepth;
    for (float d : table.observationDepths)
    {
        obsDepth.push_back(GfHalf(d));
    }
    setAttr(landmarksPrim, "av:obsDepth", SdfValueTypeNames->HalfArray, obsDepth);

    VtArray<std::uint32_t> viewIds;
    for (IndexT v : table.viewIds)
    {
        viewIds.push_back(static_cast<std::uint32_t>(v));
    }
    setAttr(landmarksPrim, "av:viewIds", SdfValueTypeNames->UIntArray, viewIds);

    VtArray<std::uint32_t> viewObsOffsets;
    for (IndexT v : table.viewObservationOffsets)
    {
        viewObsOffsets.push_back(static_cast<std::uint32_t>(v));
    }
    setAttr(landmarksPrim, "av:viewObsOffsets", SdfValueTypeNames->UIntArray, viewObsOffsets);

    VtArray<std::uint32_t> viewObsLandmarkIndex;
    for (IndexT v : table.viewObservationLandmarkIndices)
    {
        viewObsLandmarkIndex.push_back(static_cast<std::uint32_t>(v));
    }
    setAttr(landmarksPrim, "av:viewObsLandmarkIndex", SdfValueTypeNames->UIntArray, viewObsLandmarkIndex);

    VtArray<std::uint32_t> viewObsIndex;
    for (IndexT v : table.viewObservationIndices)
    {
        viewObsIndex.push_back(static_cast<std::uint32_t>(v));
    }
    setAttr(landmarksPrim, "av:viewObsIndex", SdfValueTypeNames->UIntArray, viewObsIndex);
}

template<typename T>
bool getAttr(const UsdPrim& prim, const char* name, T& value)
{
    const UsdAttribute attr = prim.GetAttribute(TfToken(name));
    if (!attr)
    {
        return false;
    }
    return attr.Get(&value);
}

ExportPaths computeExportPaths(const SdfPath& sfmPath)
{
    ExportPaths paths;
    paths.sfmPath = sfmPath;
    paths.viewsPath = childPath(sfmPath, "Views");
    paths.viewImageInfosPath = childPath(sfmPath, "ViewImageInfos");
    paths.intrinsicsPath = childPath(sfmPath, "Intrinsics");
    paths.posesPath = childPath(sfmPath, "Poses");
    paths.rigsPath = childPath(sfmPath, "Rigs");
    paths.ancestorsPath = childPath(sfmPath, "Ancestors");
    paths.imageGroupsPath = childPath(sfmPath, "ImageGroups");
    paths.landmarksPath = childPath(sfmPath, "Landmarks");
    paths.constraints2dPath = childPath(sfmPath, "Constraints2D");
    paths.constraintPointsPath = childPath(sfmPath, "ConstraintPoints");
    paths.rotationPriorsPath = childPath(sfmPath, "RotationPriors");
    paths.surveyPointsPath = childPath(sfmPath, "SurveyPoints");
    return paths;
}

geometry::Pose3 convertPoseFromMayaWorldFromCamera(const GfMatrix4d& usd)
{
    Eigen::Matrix4d worldMaya_T_cameraMaya = Eigen::Matrix4d::Identity();
    for (int row = 0; row < 4; ++row)
    {
        for (int col = 0; col < 4; ++col)
        {
            worldMaya_T_cameraMaya(row, col) = usd[col][row];
        }
    }

    Eigen::Matrix4d cvToMaya = Eigen::Matrix4d::Identity();
    cvToMaya(1, 1) = -1.0;
    cvToMaya(2, 2) = -1.0;

    const Eigen::Matrix4d world_T_camera = cvToMaya * worldMaya_T_cameraMaya * cvToMaya;
    const Eigen::Matrix4d camera_T_world = world_T_camera.inverse();
    return geometry::Pose3(camera_T_world);
}

Eigen::Matrix3d convertRotationFromMaya(const GfMatrix3d& usd)
{
    Eigen::Matrix3d mayaR = Eigen::Matrix3d::Identity();
    for (int r = 0; r < 3; ++r)
    {
        for (int c = 0; c < 3; ++c)
        {
            mayaR(r, c) = usd[r][c];
        }
    }

    Eigen::Matrix3d cvToMaya = Eigen::Matrix3d::Identity();
    cvToMaya(1, 1) = -1.0;
    cvToMaya(2, 2) = -1.0;
    return cvToMaya * mayaR * cvToMaya;
}

std::unordered_map<std::uint32_t, std::shared_ptr<sfmData::ImageInfo>> readImageInfoScope(const UsdStageRefPtr& stage, const SdfPath& scopePath)
{
    std::unordered_map<std::uint32_t, std::shared_ptr<sfmData::ImageInfo>> out;

    const UsdPrim scopePrim = stage->GetPrimAtPath(scopePath);
    if (!scopePrim)
    {
        return out;
    }

    for (const UsdPrim& imageInfoPrim : scopePrim.GetChildren())
    {
        std::uint32_t id = 0;
        if (!getAttr(imageInfoPrim, "av:id", id))
        {
            continue;
        }

        std::string imagePath;
        getAttr(imageInfoPrim, "av:imagePath", imagePath);

        std::uint32_t w = 0;
        std::uint32_t h = 0;
        getAttr(imageInfoPrim, "av:imageWidth", w);
        getAttr(imageInfoPrim, "av:imageHeight", h);

        VtArray<std::string> keys;
        VtArray<std::string> values;
        getAttr(imageInfoPrim, "av:metadataKeys", keys);
        getAttr(imageInfoPrim, "av:metadataValues", values);

        std::map<std::string, std::string> metadata;
        const std::size_t n = std::min(keys.size(), values.size());
        for (std::size_t i = 0; i < n; ++i)
        {
            metadata[keys[i]] = values[i];
        }

        out[id] = std::make_shared<sfmData::ImageInfo>(imagePath,
                                                       static_cast<std::size_t>(w),
                                                       static_cast<std::size_t>(h),
                                                       metadata);
    }

    return out;
}

std::shared_ptr<camera::Distortion> readDistortion(const UsdPrim& intrinsicPrim, const UsdStageRefPtr& stage)
{
    UsdRelationship rel = intrinsicPrim.GetRelationship(TfToken("av:distortion"));
    if (!rel)
    {
        return nullptr;
    }

    SdfPathVector targets;
    if (!rel.GetTargets(&targets) || targets.empty())
    {
        return nullptr;
    }

    const UsdPrim distortionPrim = stage->GetPrimAtPath(targets.front());
    if (!distortionPrim)
    {
        return nullptr;
    }

    TfToken modelTypeToken("none");
    getAttr(distortionPrim, "av:modelType", modelTypeToken);

    std::shared_ptr<camera::Distortion> distortion;
    try
    {
        distortion = camera::createDistortion(camera::EDISTORTION_stringToEnum(modelTypeToken.GetString()));
    }
    catch (const std::exception&)
    {
        return nullptr;
    }

    if (!distortion)
    {
        return nullptr;
    }

    VtArray<double> parameters;
    if (getAttr(distortionPrim, "av:parameters", parameters) && parameters.size() == distortion->getParameters().size())
    {
        distortion->setParameters(std::vector<double>(parameters.begin(), parameters.end()));
    }

    bool locked = false;
    if (getAttr(distortionPrim, "av:locked", locked))
    {
        distortion->setLocked(locked);
    }

    return distortion;
}

std::shared_ptr<camera::Undistortion> readUndistortion(const UsdPrim& intrinsicPrim, const UsdStageRefPtr& stage, unsigned int w, unsigned int h)
{
    UsdRelationship rel = intrinsicPrim.GetRelationship(TfToken("av:undistortion"));
    if (!rel)
    {
        return nullptr;
    }

    SdfPathVector targets;
    if (!rel.GetTargets(&targets) || targets.empty())
    {
        return nullptr;
    }

    const UsdPrim undistortionPrim = stage->GetPrimAtPath(targets.front());
    if (!undistortionPrim)
    {
        return nullptr;
    }

    TfToken modelTypeToken("none");
    getAttr(undistortionPrim, "av:modelType", modelTypeToken);

    std::shared_ptr<camera::Undistortion> undistortion;
    try
    {
        undistortion = camera::createUndistortion(camera::EUNDISTORTION_stringToEnum(modelTypeToken.GetString()), w, h);
    }
    catch (const std::exception&)
    {
        return nullptr;
    }

    if (!undistortion)
    {
        return nullptr;
    }

    VtArray<double> parameters;
    if (getAttr(undistortionPrim, "av:parameters", parameters) && parameters.size() == undistortion->getParameters().size())
    {
        undistortion->setParameters(std::vector<double>(parameters.begin(), parameters.end()));
    }

    bool locked = false;
    if (getAttr(undistortionPrim, "av:locked", locked))
    {
        undistortion->setLocked(locked);
    }

    bool desqueezed = false;
    if (getAttr(undistortionPrim, "av:desqueezed", desqueezed))
    {
        undistortion->setDesqueezed(desqueezed);
    }

    double pixelAspectRatio = 1.0;
    if (getAttr(undistortionPrim, "av:pixelAspectRatio", pixelAspectRatio))
    {
        undistortion->setPixelAspectRatio(pixelAspectRatio);
    }

    GfVec2d size(0.0);
    if (getAttr(undistortionPrim, "av:size", size))
    {
        undistortion->setSize(static_cast<int>(std::lround(size[0])), static_cast<int>(std::lround(size[1])));
    }

    GfVec2d offset(0.0);
    if (getAttr(undistortionPrim, "av:offset", offset))
    {
        undistortion->setOffset(Vec2(offset[0], offset[1]));
    }

    double diagonal = 0.0;
    if (getAttr(undistortionPrim, "av:diagonal", diagonal))
    {
        undistortion->setDiagonal(diagonal);
    }

    return undistortion;
}

void loadSfMMetadata(const UsdPrim& sfmPrim, sfmData::SfMData& sfmData)
{
    VtArray<std::string> featuresFolders;
    if (getAttr(sfmPrim, "av:featuresFolders", featuresFolders))
    {
        sfmData.setFeaturesFolders(std::vector<std::string>(featuresFolders.begin(), featuresFolders.end()));
    }

    VtArray<std::string> matchesFolders;
    if (getAttr(sfmPrim, "av:matchesFolders", matchesFolders))
    {
        sfmData.setMatchesFolders(std::vector<std::string>(matchesFolders.begin(), matchesFolders.end()));
    }
}

void loadViewsAndViewImageInfos(const UsdStageRefPtr& stage, const ExportPaths& paths, sfmData::SfMData& sfmData)
{
    const auto imageInfos = readImageInfoScope(stage, paths.viewImageInfosPath);

    const UsdPrim viewsPrim = stage->GetPrimAtPath(paths.viewsPath);
    if (!viewsPrim)
    {
        return;
    }

    for (const UsdPrim& viewPrim : viewsPrim.GetChildren())
    {
        std::uint32_t viewId = 0;
        if (!getAttr(viewPrim, "av:viewId", viewId))
        {
            continue;
        }

        std::uint32_t imageInfoId = 0;
        std::uint32_t intrinsicId = UndefinedIndexT;
        std::uint32_t poseId = UndefinedIndexT;
        std::uint32_t rigId = UndefinedIndexT;
        std::uint32_t subPoseId = UndefinedIndexT;
        std::uint32_t frameId = UndefinedIndexT;
        std::uint32_t resectionId = UndefinedIndexT;
        std::uint32_t imageGroupId = UndefinedIndexT;
        getAttr(viewPrim, "av:imageInfoId", imageInfoId);
        getAttr(viewPrim, "av:intrinsicId", intrinsicId);
        getAttr(viewPrim, "av:poseId", poseId);
        getAttr(viewPrim, "av:rigId", rigId);
        getAttr(viewPrim, "av:subPoseId", subPoseId);
        getAttr(viewPrim, "av:frameId", frameId);
        getAttr(viewPrim, "av:resectionId", resectionId);
        getAttr(viewPrim, "av:imageGroupId", imageGroupId);

        std::shared_ptr<sfmData::ImageInfo> imageInfo = std::make_shared<sfmData::ImageInfo>();
        const auto imageIt = imageInfos.find(imageInfoId);
        if (imageIt != imageInfos.end())
        {
            imageInfo = imageIt->second;
        }

        auto view = std::make_shared<sfmData::View>(imageInfo->getImagePath(),
                                                    static_cast<IndexT>(viewId),
                                                    static_cast<IndexT>(intrinsicId),
                                                    static_cast<IndexT>(poseId),
                                                    imageInfo->getWidth(),
                                                    imageInfo->getHeight(),
                                                    static_cast<IndexT>(rigId),
                                                    static_cast<IndexT>(subPoseId),
                                                    imageInfo->getMetadata());

        view->setFrameId(static_cast<IndexT>(frameId));
        view->setResectionId(static_cast<IndexT>(resectionId));
        view->setImageGroupId(static_cast<IndexT>(imageGroupId));

        VtArray<std::uint32_t> ancestorIds;
        if (getAttr(viewPrim, "av:ancestorIds", ancestorIds))
        {
            for (const std::uint32_t ancestorId : ancestorIds)
            {
                view->addAncestor(static_cast<IndexT>(ancestorId));
            }
        }

        sfmData.getViews().insert_or_assign(static_cast<IndexT>(viewId), view);
    }
}

void loadAncestors(const UsdStageRefPtr& stage, const ExportPaths& paths, sfmData::SfMData& sfmData)
{
    const auto imageInfos = readImageInfoScope(stage, paths.ancestorsPath);
    for (const auto& [id, imageInfo] : imageInfos)
    {
        sfmData.getAncestors().insert_or_assign(static_cast<IndexT>(id), imageInfo);
    }
}

void loadPoses(const UsdStageRefPtr& stage, const ExportPaths& paths, sfmData::SfMData& sfmData)
{
    const UsdPrim posesPrim = stage->GetPrimAtPath(paths.posesPath);
    if (!posesPrim)
    {
        return;
    }

    for (const UsdPrim& posePrim : posesPrim.GetChildren())
    {
        std::uint32_t poseId = 0;
        if (!getAttr(posePrim, "av:id", poseId))
        {
            continue;
        }

        GfMatrix4d worldFromCamera(1.0);
        getAttr(posePrim, "av:worldFromCamera", worldFromCamera);

        auto pose = std::make_shared<sfmData::CameraPose>(convertPoseFromMayaWorldFromCamera(worldFromCamera));

        bool locked = false;
        if (getAttr(posePrim, "av:locked", locked) && locked)
        {
            pose->lock();
        }

        std::uint8_t state = static_cast<std::uint8_t>(pose->getState());
        if (getAttr(posePrim, "av:state", state))
        {
            pose->setState(static_cast<EEstimatorParameterState>(state));
        }

        bool rotationOnly = false;
        if (getAttr(posePrim, "av:rotationOnly", rotationOnly))
        {
            pose->setRotationOnly(rotationOnly);
        }

        bool removable = true;
        if (getAttr(posePrim, "av:removable", removable))
        {
            pose->setRemovable(removable);
        }

        VtArray<double> uncertainty;
        if (getAttr(posePrim, "av:uncertainty", uncertainty))
        {
            sfmData::PoseUncertainty mat;

            int idx = 0;
            for (int i = 0; i < 6; ++i)
            {
                for (int j = i; j < 6; ++j)
                {
                    mat(i, j) = mat(j, i) = uncertainty[idx++];
                }
            }
            sfmData.setPoseUncertainty(static_cast<IndexT>(poseId), mat);
        }

        sfmData.getPoses().insert_or_assign(static_cast<IndexT>(poseId), pose);
    }
}

void loadIntrinsics(const UsdStageRefPtr& stage, const ExportPaths& paths, sfmData::SfMData& sfmData)
{
    const UsdPrim intrinsicsPrim = stage->GetPrimAtPath(paths.intrinsicsPath);
    if (!intrinsicsPrim)
    {
        return;
    }

    for (const UsdPrim& intrinsicPrim : intrinsicsPrim.GetChildren())
    {
        std::uint32_t intrinsicId = 0;
        if (!getAttr(intrinsicPrim, "av:id", intrinsicId))
        {
            continue;
        }

        std::uint32_t imageWidth = 0;
        std::uint32_t imageHeight = 0;
        getAttr(intrinsicPrim, "av:imageWidth", imageWidth);
        getAttr(intrinsicPrim, "av:imageHeight", imageHeight);

        TfToken modelToken("pinhole");
        getAttr(intrinsicPrim, "av:modelType", modelToken);

        camera::EINTRINSIC intrinsicType = camera::PINHOLE_CAMERA;
        try
        {
            intrinsicType = camera::EINTRINSIC_stringToEnum(modelToken.GetString());
        }
        catch (const std::exception&)
        {
            const TfToken typeName = intrinsicPrim.GetTypeName();
            if (typeName == TfToken("AvIntrinsicEquidistant"))
            {
                intrinsicType = camera::EQUIDISTANT_CAMERA;
            }
            else if (typeName == TfToken("AvIntrinsicEquirectangular"))
            {
                intrinsicType = camera::EQUIRECTANGULAR_CAMERA;
            }
        }

        GfVec2d focalLengthPix(1.0, 1.0);
        getAttr(intrinsicPrim, "av:focalLengthPix", focalLengthPix);

        GfVec2d principalPoint(static_cast<double>(imageWidth) * 0.5, static_cast<double>(imageHeight) * 0.5);
        getAttr(intrinsicPrim, "av:principalPoint", principalPoint);

        const double offsetX = principalPoint[0] - static_cast<double>(imageWidth) * 0.5;
        const double offsetY = principalPoint[1] - static_cast<double>(imageHeight) * 0.5;

        const auto distortion = readDistortion(intrinsicPrim, stage);
        const auto undistortion = readUndistortion(intrinsicPrim, stage, imageWidth, imageHeight);

        const camera::EDISTORTION distortionType = distortion ? distortion->getType() : camera::DISTORTION_NONE;
        const camera::EUNDISTORTION undistortionType = undistortion ? undistortion->getType() : camera::UNDISTORTION_NONE;

        std::shared_ptr<camera::IntrinsicBase> intrinsic =
            camera::createIntrinsic(intrinsicType,
                                    distortionType,
                                    undistortionType,
                                    imageWidth,
                                    imageHeight,
                                    focalLengthPix[0],
                                    focalLengthPix[1],
                                    offsetX,
                                    offsetY);

        if (!intrinsic)
        {
            intrinsic = std::make_shared<camera::Pinhole>(imageWidth,
                                                          imageHeight,
                                                          focalLengthPix[0],
                                                          focalLengthPix[1],
                                                          offsetX,
                                                          offsetY);
        }

        bool locked = false;
        if (getAttr(intrinsicPrim, "av:locked", locked) && locked)
        {
            intrinsic->lock();
        }

        double sensorWidth = 0.0;
        if (getAttr(intrinsicPrim, "av:sensorWidth", sensorWidth))
        {
            intrinsic->setSensorWidth(sensorWidth);
        }

        double sensorHeight = 0.0;
        if (getAttr(intrinsicPrim, "av:sensorHeight", sensorHeight))
        {
            intrinsic->setSensorHeight(sensorHeight);
        }

        std::string serialNumber;
        if (getAttr(intrinsicPrim, "av:serialNumber", serialNumber))
        {
            intrinsic->setSerialNumber(serialNumber);
        }

        int initializationMode = 0;
        if (getAttr(intrinsicPrim, "av:initializationMode", initializationMode))
        {
            intrinsic->setInitializationMode(static_cast<camera::EInitMode>(initializationMode));
        }

        const auto equidistant = std::dynamic_pointer_cast<camera::Equidistant>(intrinsic);
        if (equidistant)
        {
            double circleRadius = 0.0;
            if (getAttr(intrinsicPrim, "av:circleRadius", circleRadius))
            {
                equidistant->setCircleRadius(circleRadius);
            }

            GfVec2d circleCenter(0.0);
            if (getAttr(intrinsicPrim, "av:circleCenter", circleCenter))
            {
                equidistant->setCircleCenterX(circleCenter[0]);
                equidistant->setCircleCenterY(circleCenter[1]);
            }
        }

        const auto withDisto = std::dynamic_pointer_cast<camera::IntrinsicScaleOffsetDisto>(intrinsic);
        if (withDisto)
        {
            withDisto->setDistortionObject(distortion);
            withDisto->setUndistortionObject(undistortion);
        }

        sfmData.getIntrinsics().insert_or_assign(static_cast<IndexT>(intrinsicId), intrinsic);
    }
}

void loadRigs(const UsdStageRefPtr& stage, const ExportPaths& paths, sfmData::SfMData& sfmData)
{
    const UsdPrim rigsPrim = stage->GetPrimAtPath(paths.rigsPath);
    if (!rigsPrim)
    {
        return;
    }

    for (const UsdPrim& rigPrim : rigsPrim.GetChildren())
    {
        std::uint32_t rigId = 0;
        if (!getAttr(rigPrim, "av:id", rigId))
        {
            continue;
        }

        VtArray<std::uint32_t> subPoseIds;
        getAttr(rigPrim, "av:subPoseIds", subPoseIds);

        std::size_t subPoseCount = subPoseIds.size();
        for (const std::uint32_t id : subPoseIds)
        {
            subPoseCount = std::max(subPoseCount, static_cast<std::size_t>(id + 1));
        }

        sfmData::Rig rig(static_cast<unsigned int>(subPoseCount));
        for (const UsdPrim& subPosePrim : rigPrim.GetChildren())
        {
            std::uint32_t subPoseId = 0;
            if (!getAttr(subPosePrim, "av:id", subPoseId))
            {
                continue;
            }
            if (subPoseId >= rig.getNbSubPoses())
            {
                continue;
            }

            std::uint8_t status = 0;
            getAttr(subPosePrim, "av:status", status);

            GfMatrix4d transform(1.0);
            getAttr(subPosePrim, "av:transform", transform);

            sfmData::RigSubPose rigSubPose;
            rigSubPose.status = static_cast<sfmData::ERigSubPoseStatus>(status);
            rigSubPose.pose = convertPoseFromMayaWorldFromCamera(transform);
            rig.setSubPose(subPoseId, rigSubPose);
        }

        sfmData.getRigs()[static_cast<IndexT>(rigId)] = rig;
    }
}

void loadImageGroups(const UsdStageRefPtr& stage, const ExportPaths& paths, sfmData::SfMData& sfmData)
{
    const UsdPrim groupsPrim = stage->GetPrimAtPath(paths.imageGroupsPath);
    if (!groupsPrim)
    {
        return;
    }

    for (const UsdPrim& groupPrim : groupsPrim.GetChildren())
    {
        std::uint32_t groupId = 0;
        int groupType = static_cast<int>(sfmData::ImageGroup::Type::ImageSet);
        if (!getAttr(groupPrim, "av:id", groupId))
        {
            continue;
        }
        getAttr(groupPrim, "av:type", groupType);

        sfmData::ImageGroup::sptr group;
        try
        {
            group = sfmData::ImageGroup::create(static_cast<sfmData::ImageGroup::Type>(groupType));
        }
        catch (const std::exception&)
        {
            continue;
        }

        if (group)
        {
            sfmData.getImageGroups().insert_or_assign(static_cast<IndexT>(groupId), group);
        }
    }
}

void loadConstraints2D(const UsdStageRefPtr& stage, const ExportPaths& paths, sfmData::SfMData& sfmData)
{
    const UsdPrim constraintsPrim = stage->GetPrimAtPath(paths.constraints2dPath);
    if (!constraintsPrim)
    {
        return;
    }

    for (const UsdPrim& cPrim : constraintsPrim.GetChildren())
    {
        sfmData::Constraint2D c;

        std::uint8_t descType = static_cast<std::uint8_t>(feature::EImageDescriberType::UNINITIALIZED);
        std::uint32_t viewFirst = UndefinedIndexT;
        std::uint32_t viewSecond = UndefinedIndexT;
        getAttr(cPrim, "av:descType", descType);
        getAttr(cPrim, "av:viewFirst", viewFirst);
        getAttr(cPrim, "av:viewSecond", viewSecond);

        c.descType = static_cast<feature::EImageDescriberType>(descType);
        c.ViewFirst = static_cast<IndexT>(viewFirst);
        c.ViewSecond = static_cast<IndexT>(viewSecond);

        GfVec2f obsFirstXY(0.0f);
        std::uint32_t obsFirstFeatureId = UndefinedIndexT;
        GfHalf obsFirstScale(0.0f);
        GfHalf obsFirstDepth(-1.0f);
        getAttr(cPrim, "av:observationFirstXY", obsFirstXY);
        getAttr(cPrim, "av:observationFirstFeatureId", obsFirstFeatureId);
        getAttr(cPrim, "av:observationFirstScale", obsFirstScale);
        getAttr(cPrim, "av:observationFirstDepth", obsFirstDepth);

        c.ObservationFirst.setCoordinates(obsFirstXY[0], obsFirstXY[1]);
        c.ObservationFirst.setFeatureId(static_cast<IndexT>(obsFirstFeatureId));
        c.ObservationFirst.setScale(static_cast<float>(obsFirstScale));
        c.ObservationFirst.setDepth(static_cast<float>(obsFirstDepth));

        GfVec2f obsSecondXY(0.0f);
        std::uint32_t obsSecondFeatureId = UndefinedIndexT;
        GfHalf obsSecondScale(0.0f);
        GfHalf obsSecondDepth(-1.0f);
        getAttr(cPrim, "av:observationSecondXY", obsSecondXY);
        getAttr(cPrim, "av:observationSecondFeatureId", obsSecondFeatureId);
        getAttr(cPrim, "av:observationSecondScale", obsSecondScale);
        getAttr(cPrim, "av:observationSecondDepth", obsSecondDepth);

        c.ObservationSecond.setCoordinates(obsSecondXY[0], obsSecondXY[1]);
        c.ObservationSecond.setFeatureId(static_cast<IndexT>(obsSecondFeatureId));
        c.ObservationSecond.setScale(static_cast<float>(obsSecondScale));
        c.ObservationSecond.setDepth(static_cast<float>(obsSecondDepth));

        sfmData.getConstraints2D().push_back(c);
    }
}

void loadConstraintPoints(const UsdStageRefPtr& stage, const ExportPaths& paths, sfmData::SfMData& sfmData)
{
    const UsdPrim constraintsPrim = stage->GetPrimAtPath(paths.constraintPointsPath);
    if (!constraintsPrim)
    {
        return;
    }

    for (const UsdPrim& cPrim : constraintsPrim.GetChildren())
    {
        std::uint32_t id = 0;
        if (!getAttr(cPrim, "av:id", id))
        {
            continue;
        }

        sfmData::ConstraintPoint c;
        std::uint32_t landmarkId = UndefinedIndexT;
        GfVec3f normal(0.0f);
        GfVec3f point(0.0f);
        getAttr(cPrim, "av:landmarkId", landmarkId);
        getAttr(cPrim, "av:normal", normal);
        getAttr(cPrim, "av:point", point);

        c.landmarkId = static_cast<IndexT>(landmarkId);
        c.normal = Vec3(normal[0], normal[1], normal[2]);
        c.point = Vec3(point[0], point[1], point[2]);
        sfmData.getConstraintsPoint()[static_cast<IndexT>(id)] = c;
    }
}

void loadRotationPriors(const UsdStageRefPtr& stage, const ExportPaths& paths, sfmData::SfMData& sfmData)
{
    const UsdPrim priorsPrim = stage->GetPrimAtPath(paths.rotationPriorsPath);
    if (!priorsPrim)
    {
        return;
    }

    for (const UsdPrim& rPrim : priorsPrim.GetChildren())
    {
        std::uint32_t id = 0;
        std::uint32_t viewFirst = UndefinedIndexT;
        std::uint32_t viewSecond = UndefinedIndexT;
        if (!getAttr(rPrim, "av:id", id))
        {
            continue;
        }
        getAttr(rPrim, "av:viewFirst", viewFirst);
        getAttr(rPrim, "av:viewSecond", viewSecond);

        GfMatrix3d secondRFirst(1.0);
        getAttr(rPrim, "av:secondRFirst", secondRFirst);

        sfmData::RotationPrior prior;
        prior.ViewFirst = static_cast<IndexT>(viewFirst);
        prior.ViewSecond = static_cast<IndexT>(viewSecond);
        prior._second_R_first = convertRotationFromMaya(secondRFirst);

        if (id >= sfmData.getRotationPriors().size())
        {
            sfmData.getRotationPriors().resize(id + 1);
        }
        sfmData.getRotationPriors()[id] = prior;
    }
}

void loadSurveyPoints(const UsdStageRefPtr& stage, const ExportPaths& paths, sfmData::SfMData& sfmData)
{
    const UsdPrim groupsPrim = stage->GetPrimAtPath(paths.surveyPointsPath);
    if (!groupsPrim)
    {
        return;
    }

    for (const UsdPrim& gPrim : groupsPrim.GetChildren())
    {
        std::uint32_t groupId = 0;
        if (!getAttr(gPrim, "av:id", groupId))
        {
            continue;
        }

        auto& points = sfmData.getSurveyPoints()[static_cast<IndexT>(groupId)];
        for (const UsdPrim& spPrim : gPrim.GetChildren())
        {
            sfmData::SurveyPoint sp;

            GfVec3f point3d(0.0f);
            GfVec2f survey(0.0f);
            GfVec2f residual(0.0f);
            getAttr(spPrim, "av:point3d", point3d);
            getAttr(spPrim, "av:survey", survey);
            getAttr(spPrim, "av:residual", residual);

            sp.point3d = Vec3(point3d[0], point3d[1], point3d[2]);
            sp.survey = Vec2(survey[0], survey[1]);
            sp.residual = Vec2(residual[0], residual[1]);
            points.push_back(sp);
        }
    }
}

void loadLandmarkTable(const UsdStageRefPtr& stage, const ExportPaths& paths, sfmData::SfMData& sfmData, bool viewsLoaded)
{
    const UsdPrim landmarksPrim = stage->GetPrimAtPath(paths.landmarksPath);
    if (!landmarksPrim)
    {
        return;
    }

    const UsdGeomPoints usdPoints(landmarksPrim);
    if (!usdPoints)
    {
        return;
    }

    VtArray<GfVec3f> points;
    usdPoints.GetPointsAttr().Get(&points);
    VtArray<GfVec3f> colors;
    const UsdGeomPrimvar displayColorPrimvar = UsdGeomPrimvarsAPI(landmarksPrim).GetPrimvar(TfToken("displayColor"));
    if (displayColorPrimvar)
    {
        displayColorPrimvar.Get(&colors);
    }

    VtArray<std::uint32_t> ids;
    VtArray<std::uint8_t> states;
    VtArray<std::uint8_t> descTypes;
    VtArray<std::uint8_t> flags;
    VtArray<std::uint32_t> referenceViewIds;
    VtArray<std::uint32_t> obsOffsets;
    VtArray<std::uint32_t> obsViewIds;
    VtArray<std::uint32_t> obsFeatureIds;
    VtArray<GfVec2f> obsXY;
    VtArray<GfHalf> obsScale;
    VtArray<GfHalf> obsDepth;

    getAttr(landmarksPrim, "av:ids", ids);
    getAttr(landmarksPrim, "av:state", states);
    getAttr(landmarksPrim, "av:descType", descTypes);
    getAttr(landmarksPrim, "av:flags", flags);
    getAttr(landmarksPrim, "av:referenceViewId", referenceViewIds);
    getAttr(landmarksPrim, "av:obsOffsets", obsOffsets);
    getAttr(landmarksPrim, "av:obsViewId", obsViewIds);
    getAttr(landmarksPrim, "av:obsFeatureId", obsFeatureIds);
    getAttr(landmarksPrim, "av:obsXY", obsXY);
    getAttr(landmarksPrim, "av:obsScale", obsScale);
    getAttr(landmarksPrim, "av:obsDepth", obsDepth);

    const std::size_t lmCount = std::min(ids.size(), points.size());
    for (std::size_t i = 0; i < lmCount; ++i)
    {
        sfmData::Landmark lm;

        const GfVec3f p = points[i];
        lm.setX(Vec3(p[0], -p[1], -p[2]));
        if (i < colors.size())
        {
            const GfVec3f c = colors[i];
            const auto toByte = [](float v) -> std::uint8_t
            {
                const float clamped = std::clamp(v, 0.0f, 1.0f);
                return static_cast<std::uint8_t>(std::lround(clamped * 255.0f));
            };
            lm.setRgb(image::RGBColor(toByte(c[0]), toByte(c[1]), toByte(c[2])));
        }

        if (i < descTypes.size())
        {
            lm.setDescType(static_cast<feature::EImageDescriberType>(descTypes[i]));
        }
        if (i < states.size())
        {
            lm.setState(static_cast<EEstimatorParameterState>(states[i]));
        }
        if (i < flags.size())
        {
            const std::uint8_t packedFlags = flags[i];
            lm.setParallaxRobust((packedFlags & kLandmarkParallaxRobustBit) != 0);
            lm.setLocked((packedFlags & kLandmarkLockedBit) != 0);
        }
        if (viewsLoaded && i < referenceViewIds.size())
        {
            const IndexT refId = static_cast<IndexT>(referenceViewIds[i]);
            if (sfmData.getViews().count(refId) != 0)
            {
                lm.setReferenceView(sfmData.getViewSharedPtr(refId));
            }
        }

        const std::size_t begin = (i < obsOffsets.size()) ? obsOffsets[i] : 0;
        const std::size_t end = (i + 1 < obsOffsets.size()) ? obsOffsets[i + 1] : obsViewIds.size();
        const std::size_t safeEnd = std::min({end, obsViewIds.size(), obsFeatureIds.size(), obsXY.size(), obsScale.size(), obsDepth.size()});
        for (std::size_t j = begin; j < safeEnd; ++j)
        {
            sfmData::Observation obs;
            obs.setCoordinates(obsXY[j][0], obsXY[j][1]);
            obs.setFeatureId(static_cast<IndexT>(obsFeatureIds[j]));
            obs.setScale(static_cast<float>(obsScale[j]));
            obs.setDepth(static_cast<float>(obsDepth[j]));
            lm.getObservations()[static_cast<IndexT>(obsViewIds[j])] = obs;
        }

        const IndexT landmarkId = static_cast<IndexT>(ids[i]);
        sfmData.getLandmarks()[landmarkId] = lm;
    }
}

}  // namespace

/*
 * Save strategy:
 * - `filename` is the actual write target (can be a temporary path during atomic save).
 * - `finalFilename` is the eventual user-facing name, used to author stable payload links.
 * - Non-structure data is authored directly in the main stage.
 * - Structure (landmarks/observations) can be authored in a separate binary payload layer.
 */
bool saveUSD(const sfmData::SfMData& sfmData,
             const std::string& filename,
             const std::string& finalFilename,
             ESfMData partFlag)
{
    UsdStageRefPtr stage = UsdStage::CreateNew(filename);
    if (!stage)
    {
        ALICEVISION_LOG_ERROR("UsdStage::CreateNew failed for file: " << filename);
        return false;
    }

    const SdfPath sfmPath("/SfM");
    const UsdPrim sfmPrim = stage->DefinePrim(sfmPath, TfToken("AvSfMData"));
    stage->SetDefaultPrim(sfmPrim);

    const ExportPaths paths = createExportPaths(stage, sfmPath);

    const bool saveViews = (partFlag & VIEWS) == VIEWS;
    const bool saveAncestors = (partFlag & ANCESTORS) == ANCESTORS;
    const bool saveIntrinsics = (partFlag & INTRINSICS) == INTRINSICS;
    const bool saveExtrinsics = (partFlag & EXTRINSICS) == EXTRINSICS;
    const bool saveStructure = (partFlag & STRUCTURE) == STRUCTURE;
    const bool saveSurveys = (partFlag & SURVEYS) == SURVEYS;
    const bool saveConstraints2d = (partFlag & CONSTRAINTS2D) == CONSTRAINTS2D;

    writeSfMMetadata(sfmData, sfmPrim, partFlag);
    writeSfMRelationships(sfmPrim, paths);
    if (saveViews)
    {
        writeViewsAndViewImageInfos(stage, sfmData, paths);
        writeImageGroups(stage, sfmData, paths);
    }
    if (saveAncestors)
    {
        writeAncestors(stage, sfmData, paths);
    }
    if (saveExtrinsics)
    {
        writePoses(stage, sfmData, paths);
        writeRigs(stage, sfmData, paths);
        writeRotationPriors(stage, sfmData, paths);
    }
    if (saveIntrinsics)
    {
        writeIntrinsics(stage, sfmData, paths);
    }
    if (saveViews && saveExtrinsics && saveIntrinsics)
    {
        writeDccCameras(stage, sfmData, paths);
    }
    if (saveConstraints2d)
    {
        writeConstraints2D(stage, sfmData, paths);
        writeConstraintPoints(stage, sfmData, paths);
    }
    if (saveSurveys)
    {
        writeSurveyPoints(stage, sfmData, paths);
    }
    if (saveStructure)
    {
        // Heavy landmark arrays are stored in a sidecar binary layer to keep the main USD light.
        const std::string payloadFile = landmarksPayloadPath(filename);
        const std::string finalPayloadFile = landmarksPayloadPath(finalFilename);


        UsdStageRefPtr payloadStage = UsdStage::CreateNew(payloadFile);
        if (!payloadStage)
        {
            ALICEVISION_LOG_ERROR("USD export failed to create landmarks payload: " << payloadFile);
            return false;
        }

        // The landmark prim path must be rooted under /SfM, so create the parent.
        payloadStage->DefinePrim(sfmPath);
        writeLandmarkTable(payloadStage, sfmData, paths);
        if (!payloadStage->GetRootLayer()->Save())
        {
            ALICEVISION_LOG_ERROR("USD export failed to save landmarks payload: " << payloadFile);
            return false;
        }


        // Now rename the temporary payload to the final filename
        if (payloadFile != finalPayloadFile)
        {
            if (fs::exists(finalPayloadFile))
            {
                fs::remove(finalPayloadFile);
            }

            fs::rename(payloadFile, finalPayloadFile);
        }

        // Add a payload arc on a typed Landmarks prim in the main stage.
        UsdGeomPoints landmarksPoints = UsdGeomPoints::Define(stage, paths.landmarksPath);
        UsdPrim landmarksPrim = landmarksPoints.GetPrim();

        const std::string relPayloadName = fs::path(finalPayloadFile).filename().string();
        landmarksPrim.GetPayloads().AddPayload(SdfPayload(relPayloadName, paths.landmarksPath));
    }
    else 
    {
        //Create a dummy point cloud
        UsdGeomPoints::Define(stage, paths.landmarksPath);
    }

    if (!stage->GetRootLayer()->Save())
    {
        ALICEVISION_LOG_ERROR("USD export failed while saving layer: " << filename);
        return false;
    }

    return true;
}

/*
 * Load strategy:
 * - Open one stage from `filename`.
 * - USD composition resolves relationships and payload arcs automatically.
 * - Readers below only access composed prims/attributes and do not need payload-specific logic.
 */
bool loadUSD(sfmData::SfMData& sfmData, const std::string& filename, ESfMData partFlag)
{
    UsdStageRefPtr stage = UsdStage::Open(filename);
    if (!stage)
    {
        ALICEVISION_LOG_ERROR("UsdStage::Open failed for file: " << filename);
        return false;
    }

    sfmData.clear();

    UsdPrim sfmPrim = stage->GetDefaultPrim();
    if (!sfmPrim)
    {
        sfmPrim = stage->GetPrimAtPath(SdfPath("/SfM"));
    }
    if (!sfmPrim)
    {
        ALICEVISION_LOG_ERROR("Cannot find AvSfMData root prim in file: " << filename);
        return false;
    }

    const ExportPaths paths = computeExportPaths(sfmPrim.GetPath());

    const bool loadViews = (partFlag & VIEWS) == VIEWS;
    const bool loadAncestorsFlag = (partFlag & ANCESTORS) == ANCESTORS;
    const bool loadIntrinsicsFlag = (partFlag & INTRINSICS) == INTRINSICS;
    const bool loadExtrinsicsFlag = (partFlag & EXTRINSICS) == EXTRINSICS;
    const bool loadStructureFlag = (partFlag & STRUCTURE) == STRUCTURE;
    const bool loadSurveysFlag = (partFlag & SURVEYS) == SURVEYS;
    const bool loadConstraints2dFlag = (partFlag & CONSTRAINTS2D) == CONSTRAINTS2D;

    loadSfMMetadata(sfmPrim, sfmData);

    if (loadViews)
    {
        loadViewsAndViewImageInfos(stage, paths, sfmData);
        loadImageGroups(stage, paths, sfmData);
    }
    if (loadAncestorsFlag)
    {
        loadAncestors(stage, paths, sfmData);
    }
    if (loadExtrinsicsFlag)
    {
        loadPoses(stage, paths, sfmData);
        loadRigs(stage, paths, sfmData);
        loadRotationPriors(stage, paths, sfmData);
    }
    if (loadIntrinsicsFlag)
    {
        loadIntrinsics(stage, paths, sfmData);
    }
    if (loadConstraints2dFlag)
    {
        loadConstraints2D(stage, paths, sfmData);
        loadConstraintPoints(stage, paths, sfmData);
    }
    if (loadSurveysFlag)
    {
        loadSurveyPoints(stage, paths, sfmData);
    }
    if (loadStructureFlag)
    {
        loadLandmarkTable(stage, paths, sfmData, loadViews);
    }

    return true;
}

// Convention helper for the sidecar layer that stores landmark payload data.
std::string landmarksPayloadPath(const std::string& mainUsdFilename)
{
    const fs::path p(mainUsdFilename);
    return (p.parent_path() / (p.stem().string() + "_landmarks.usdc")).string();
}

}  // namespace sfmDataIO
}  // namespace aliceVision
