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
  const int point_count = sfm_data.GetLandmarks().size();
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

