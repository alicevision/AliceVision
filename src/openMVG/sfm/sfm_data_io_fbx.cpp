// Copyright (c) 2015 Stian Z. Vrba.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#if HAVE_FBX
#include <iostream>

// OpenMVG
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
using namespace openMVG::sfm;

// FBX
#include <fbxsdk.h>

namespace openMVG {
namespace sfm {

bool Save_FBX(const SfM_Data & sfm_data, const std::string & filename, ESfM_Data /* unused flags_part*/)
{
  //=================================================================================
  // Create scene and mesh
  //=================================================================================

  FbxAutoDestroyPtr<FbxManager> fbxmanager(FbxManager::Create());
  FbxScene* fbxscene = FbxScene::Create(fbxmanager, "openMVG");

  FbxNode* fbxnode = FbxNode::Create(fbxscene, "sfm_node");
  fbxscene->GetRootNode()->AddChild(fbxnode);

  FbxMesh* fbxmesh = FbxMesh::Create(fbxscene, "sfm_mesh");
  fbxnode->SetNodeAttribute(fbxmesh);

  FbxGeometryElementVertexColor* vcelement = fbxmesh->CreateElementVertexColor();
  vcelement->SetMappingMode(FbxGeometryElement::eByControlPoint);
  vcelement->SetReferenceMode(FbxGeometryElement::eDirect);
  
  //=================================================================================
  // Add points and colors.  FBX has no native support for point clouds, so we create
  // a mesh containing only vertices, w/o any other geometry.
  //=================================================================================
  
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
  
  //=================================================================================
  // TODO: Add cameras
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

