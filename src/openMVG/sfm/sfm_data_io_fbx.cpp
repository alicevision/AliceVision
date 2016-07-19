// Copyright (c) 2015 Stian Z. Vrba.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.
//
// FBX has no native support for point clouds.  Therefore two different conventions
// are supported for exporting a PC:
//
// 1. As a mesh containing only control points with corresponding colors.  No
//    polygons are present in the mesh.  Blender, f.ex., can import such meshes
//    (though Blender doesn't truly support per-vertex colors.).
//
// 2. As a collection of "null objects", where each object is an empty FbxNode
//    with LclTranslation component defining the position.  This is the convention
//    used by, e.g., Nuke. It is slow to read and write, the file size is orders
//    of magnitude larger, and per-vertex colors are not supported.
//
// Convention 1 is the default as it produces compact files with per-vertex colors.
// Convention 2 is selected if the filename ends with '_null.fbx'.

#if HAVE_FBX
#include <stdio.h>
#include <iostream>

// OpenMVG
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
using namespace openMVG::sfm;

// FBX
#include <fbxsdk.h>

namespace openMVG {
namespace sfm {

// MESHES ///////////////////////////////////////////////////////////////////

static bool useNullObjects(const std::string& filename)
{
  static const std::string enableNullExtension = "_null.fbx";
  if (filename.size() < enableNullExtension.size())
    return false;
  return filename.substr(filename.size()-enableNullExtension.size()) == enableNullExtension;
}

static void exportPointsAsMesh(const SfM_Data& sfm_data, FbxScene* fbxscene)
{
  FbxNode* fbxnode = FbxNode::Create(fbxscene, "sfm_node");
  fbxscene->GetRootNode()->AddChild(fbxnode);

  FbxMesh* fbxmesh = FbxMesh::Create(fbxscene, "sfm_mesh");
  fbxnode->SetNodeAttribute(fbxmesh);

  FbxGeometryElementVertexColor* vcelement = fbxmesh->CreateElementVertexColor();
  vcelement->SetMappingMode(FbxGeometryElement::eByControlPoint);
  vcelement->SetReferenceMode(FbxGeometryElement::eDirect);
  
  {
    const int point_count = sfm_data.GetLandmarks().size();
    fbxmesh->InitControlPoints(point_count);
    FbxVector4* out_v = fbxmesh->GetControlPoints();
    auto& vcarray = vcelement->GetDirectArray();
    
    for (const auto& lm: sfm_data.GetLandmarks()) {
      const auto& pt = lm.second.X;
      const auto& rgb = lm.second.rgb;
      *out_v++ = FbxVector4(pt[0], pt[1], pt[2]);
      vcarray.Add(FbxColor(rgb.r()/255.f, rgb.g()/255.f, rgb.b()/255.f));
    }
  }
}

static void exportPointsAsNullObjects(const SfM_Data& sfm_data, FbxScene* fbxscene)
{
  char nodeName[64];
  int currentPoint = 0;
  
  for (const auto& lm: sfm_data.GetLandmarks()) {
    sprintf(nodeName, "P%08u", currentPoint++); // Buffer cannot overflow even with maximum 64-bit int
    const auto& pt = lm.second.X;
    
    FbxNode* ptNode = FbxNode::Create(fbxscene, nodeName);
    ptNode->LclTranslation.Set(FbxDouble3(pt[0], pt[1], pt[2]));
    fbxscene->GetRootNode()->AddChild(ptNode);
  }
}

// CAMERAS //////////////////////////////////////////////////////////////////

static void setCameraXform(FbxNode* fbxnode, const geometry::Pose3& pose)
{
  // TODO: UNIMPLEMENTED.
}

static void setCameraAperture(FbxCamera* fbxcamera, const cameras::Pinhole_Intrinsic* intrinsics, const float sensorWidth_mm)
{
  const float imgWidth = intrinsics->w();
  const float imgHeight = intrinsics->h();
  const float sensorWidth_pix = std::max(imgWidth, imgHeight);
  const float sensorHeight_pix = std::min(imgWidth, imgHeight);
  const float imgRatio = sensorHeight_pix / sensorWidth_pix;
  const float focalLength_pix = intrinsics->focal();

  const float sensorHeight_mm = sensorWidth_mm * imgRatio;
  const float focalLength_mm = sensorWidth_mm * focalLength_pix / sensorWidth_pix;
  const float pix2mm = sensorWidth_mm / sensorWidth_pix;

#if 0
  // openMVG: origin is (top,left) corner and orientation is (bottom,right)
  // ABC: origin is centered and orientation is (up,right)
  // Following values are in cm, hence the 0.1 multiplier
  const float haperture_cm = 0.1 * imgWidth * pix2mm;
  const float vaperture_cm = 0.1 * imgHeight * pix2mm;

  camSample.setFocalLength(focalLength_mm);
  camSample.setHorizontalAperture(haperture_cm);
  camSample.setVerticalAperture(vaperture_cm);
#endif
}

static void addCamera(FbxScene* fbxscene, const std::string& cameraName, const geometry::Pose3& pose,
    const cameras::Pinhole_Intrinsic* intrinsics, const std::string& imagePath, const IndexT id_view,
    const IndexT id_intrinsic, const float sensorWidth_mm, const IndexT id_pose)
{
  FbxNode* fbxnode = FbxNode::Create(fbxscene, (std::string("camnode_") + cameraName).c_str());
  fbxscene->GetRootNode()->AddChild(fbxnode);
  
  FbxCamera* fbxcamera = FbxCamera::Create(fbxscene, (std::string("cam_") + cameraName).c_str());
  fbxnode->SetNodeAttribute(fbxcamera);
  
  setCameraXform(fbxnode, pose);
  setCameraAperture(fbxcamera, intrinsics, sensorWidth_mm);

  // User data
  {
    
  }  
  
}


// MAIN ENTRY POINT /////////////////////////////////////////////////////////

bool Save_FBX(const SfM_Data& sfm_data, const std::string& filename, ESfM_Data /* unused flags_part*/)
{
  // Dispatched by caller on ".fbx" extension.
  if (filename.substr(filename.size()-4) != ".fbx")
    throw std::invalid_argument("Save_FBX: invalid filename");

  //=================================================================================
  // Create scene and export the point cloud
  //=================================================================================

  FbxAutoDestroyPtr<FbxManager> fbxmanager(FbxManager::Create());
  FbxScene* fbxscene = FbxScene::Create(fbxmanager, "openMVG");
  
  if (useNullObjects(filename))
    exportPointsAsNullObjects(sfm_data, fbxscene);
  else
    exportPointsAsMesh(sfm_data, fbxscene);

  //=================================================================================
  // Add cameras to the scene; 1 camera per shot
  //=================================================================================
  
  for (const auto& it: sfm_data.GetViews()) {
    const sfm::View* view = it.second.get();
    if (!sfm_data.IsPoseAndIntrinsicDefined(view))
      continue;
    
    const geometry::Pose3 pose = sfm_data.GetPoseOrDie(view);
    const cameras::IntrinsicBase* camera = sfm_data.GetIntrinsics().at(view->id_intrinsic).get();
    const std::string cameraName = stlplus::basename_part(view->s_Img_path);
    const std::string fileName = stlplus::create_filespec(sfm_data.s_root_path, view->s_Img_path);
    
    // Extract sensor width if present, otherwise default to 36mm
    float sensorWidth_mm = 36;
    if (const auto viewMetadata = dynamic_cast<const sfm::View_Metadata*>(view)) {
      auto it = viewMetadata->metadata.find("sensor_width");
      if (it != viewMetadata->metadata.end())
        sensorWidth_mm = std::stof(it->second);
    }
    
    addCamera(fbxscene, cameraName, pose, dynamic_cast<const cameras::Pinhole_Intrinsic*>(camera),
        fileName, view->id_view, view->id_intrinsic, sensorWidth_mm, view->id_pose);
  }

  //=================================================================================
  // Save FBX; choose older format for compatibility
  //=================================================================================

  {
    FbxIOSettings* ios = FbxIOSettings::Create(fbxmanager, IOSROOT);
    FbxExporter* exporter = FbxExporter::Create(fbxmanager, "");

    if (!exporter->Initialize(filename.c_str(), -1, ios)) {
      std::cerr << "SfM FBX exporter error: Initialize: " << exporter->GetStatus().GetErrorString();
      return false;
    }
    if (!exporter->SetFileExportVersion("FBX201400", FbxSceneRenamer::eNone)) {
      std::cerr << "SfM FBX exporter error: set version to FBX201400: " << exporter->GetStatus().GetErrorString();
      return false;
    }
    if (!exporter->Export(fbxscene)) {
      std::cerr << "FBX exporter error: export: " << exporter->GetStatus().GetErrorString();
      return false;
    }
  }
  
  return true;
}

} // sfm
} // openMVG

#endif  // HAVE_FBX

