// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/mesh/Mesh.hpp>
#include <aliceVision/mesh/Texturing.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/system/Timer.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>

#include <pxr/base/tf/token.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usd/zipFile.h>
#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/usd/usdGeom/metrics.h>
#include <pxr/usd/usdGeom/primvar.h>
#include <pxr/usd/usdGeom/primvarsAPI.h>
#include <pxr/usd/usdGeom/xform.h>
#include <pxr/usdImaging/usdImaging/tokens.h>
#include <pxr/usd/usdShade/material.h>
#include <pxr/usd/usdShade/materialBindingAPI.h>
#include <pxr/usd/usdShade/shader.h>

#include <filesystem>

#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = std::filesystem;

PXR_NAMESPACE_USING_DIRECTIVE

TF_DEFINE_PRIVATE_TOKENS(AvUsdTokens,
                         (bias)(colorSpace)(diffuseColor)(fallback)(file)(normal)(raw)(Raw)(result)(rgb)(scale)(sourceColorSpace)(st)(varname));

enum class EUSDFileType
{
    USDA,
    USDC,
    USDZ
};

std::string EUSDFileType_informations() noexcept
{
    return "USD file type :\n"
           "*.usda\n"
           "*.usdc\n"
           "*.usdz";
}

EUSDFileType EUSDFileType_stringToEnum(const std::string& usdFileType) noexcept
{
    const std::string type = boost::to_lower_copy(usdFileType);

    if (type == "usda")
        return EUSDFileType::USDA;
    if (type == "usdc")
        return EUSDFileType::USDC;
    if (type == "usdz")
        return EUSDFileType::USDZ;

    return EUSDFileType::USDA;
}

std::string EUSDFileType_enumToString(const EUSDFileType usdFileType) noexcept
{
    switch (usdFileType)
    {
        case EUSDFileType::USDZ:
            return "usdz";
        case EUSDFileType::USDC:
            return "usdc";
        case EUSDFileType::USDA:
        default:
            return "usda";
    }
}

std::ostream& operator<<(std::ostream& os, EUSDFileType usdFileType) { return os << EUSDFileType_enumToString(usdFileType); }

std::istream& operator>>(std::istream& in, EUSDFileType& usdFileType)
{
    std::string token;
    in >> token;
    usdFileType = EUSDFileType_stringToEnum(token);
    return in;
}

int aliceVision_main(int argc, char** argv)
{
    system::Timer timer;

    // command line parameters
    std::string inputMeshPath;
    std::string outputFolderPath;
    EUSDFileType fileType = EUSDFileType::USDA;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input", po::value<std::string>(&inputMeshPath),
         "Input textured mesh to export.")
        ("output", po::value<std::string>(&outputFolderPath),
         "Output folder for USD file and textures.")
        ("fileType", po::value<EUSDFileType>(&fileType)->default_value(fileType),
         EUSDFileType_informations().c_str());
    // clang-format on

    CmdLine cmdline("The program converts a textured mesh to USD.\nAliceVision exportUSD");
    cmdline.add(requiredParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // load input mesh and textures
    mesh::Texturing texturing;
    texturing.loadWithMaterial(inputMeshPath);
    const mesh::Mesh* inputMesh = texturing.mesh;

    if (inputMesh == nullptr)
    {
        ALICEVISION_LOG_ERROR("Unable to read input mesh from the file: " << inputMeshPath);
        return EXIT_FAILURE;
    }

    const std::string extension = fileType == EUSDFileType::USDC || fileType == EUSDFileType::USDZ ? "usdc" : "usda";
    const fs::path stagePath = fs::canonical(outputFolderPath) / ("texturedMesh." + extension);
    UsdStageRefPtr stage = UsdStage::CreateNew(stagePath.string());
    if (!stage)
    {
        ALICEVISION_LOG_ERROR("Cannot create USD stage");
        return EXIT_FAILURE;
    }
    UsdGeomSetStageUpAxis(stage, UsdGeomTokens->y);
    UsdGeomSetStageMetersPerUnit(stage, 0.01);

    // create mesh
    UsdGeomXform xform = UsdGeomXform::Define(stage, SdfPath("/root"));
    UsdGeomMesh mesh = UsdGeomMesh::Define(stage, SdfPath("/root/mesh"));
    stage->SetDefaultPrim(xform.GetPrim());

    // write mesh properties
    UsdAttribute doubleSided = mesh.CreateDoubleSidedAttr();
    doubleSided.Set(false);

    UsdAttribute subdSchema = mesh.CreateSubdivisionSchemeAttr();
    subdSchema.Set(UsdGeomTokens->none);

    // write points
    UsdAttribute points = mesh.CreatePointsAttr();

    VtArray<GfVec3f> pointsData;
    pointsData.resize(inputMesh->pts.size());

    for (int i = 0; i < inputMesh->pts.size(); ++i)
    {
        const Point3d& point = inputMesh->pts[i];
        pointsData[i] = {static_cast<float>(point.x), static_cast<float>(-point.y), static_cast<float>(-point.z)};
    }

    points.Set(pointsData);

    // write bounding box
    const GfBBox3d bounds = mesh.ComputeLocalBound(UsdTimeCode::Default(), UsdGeomTokens->default_);
    UsdAttribute extent = mesh.CreateExtentAttr();

    const GfVec3d& bboxMin = bounds.GetRange().GetMin();
    const GfVec3d& bboxMax = bounds.GetRange().GetMax();
    VtArray<GfVec3f> extentData{{static_cast<float>(bboxMin[0]), static_cast<float>(bboxMin[1]), static_cast<float>(bboxMin[2])},
                                {static_cast<float>(bboxMax[0]), static_cast<float>(bboxMax[1]), static_cast<float>(bboxMax[2])}};
    extent.Set(extentData);

    // write topology
    UsdAttribute faceVertexCounts = mesh.CreateFaceVertexCountsAttr();
    VtArray<int> faceVertexCountsData(inputMesh->tris.size(), 3);
    faceVertexCounts.Set(faceVertexCountsData);

    UsdAttribute faceVertexIndices = mesh.CreateFaceVertexIndicesAttr();
    VtArray<int> faceVertexIndicesData;
    faceVertexIndicesData.resize(inputMesh->tris.size() * 3);

    for (int i = 0; i < inputMesh->tris.size(); ++i)
    {
        faceVertexIndicesData[i * 3] = inputMesh->tris[i].v[0];
        faceVertexIndicesData[i * 3 + 1] = inputMesh->tris[i].v[1];
        faceVertexIndicesData[i * 3 + 2] = inputMesh->tris[i].v[2];
    }
    faceVertexIndices.Set(faceVertexIndicesData);

    // write face varying normals as primvar
    if (!inputMesh->normals.empty() && !inputMesh->trisNormalsIds.empty())
    {
        VtArray<GfVec3f> normalsData;
        normalsData.resize(inputMesh->normals.size());

        for (int i = 0; i < inputMesh->normals.size(); ++i)
        {
            const Point3d& normal = inputMesh->normals[i];
            normalsData[i] = {static_cast<float>(normal.x), static_cast<float>(-normal.y), static_cast<float>(-normal.z)};
        }

        VtIntArray normalIndices;
        normalIndices.resize(inputMesh->trisNormalsIds.size() * 3);

        for (int i = 0; i < inputMesh->trisNormalsIds.size(); ++i)
        {
            const Voxel& indices = inputMesh->trisNormalsIds[i];
            normalIndices[i * 3] = indices.x;
            normalIndices[i * 3 + 1] = indices.y;
            normalIndices[i * 3 + 2] = indices.z;
        }

        UsdGeomPrimvarsAPI primvarsApi = UsdGeomPrimvarsAPI(mesh);
        UsdGeomPrimvar uvs = primvarsApi.CreateIndexedPrimvar(
          TfToken("normals"), SdfValueTypeNames->Normal3fArray, normalsData, normalIndices, UsdGeomTokens->faceVarying);
    }
    else  // compute smooth vertex normals
    {
        StaticVector<Point3d> normals;
        inputMesh->computeNormalsForPts(normals);

        VtArray<GfVec3f> normalsData;
        normalsData.resize(normals.size());

        for (int i = 0; i < normals.size(); ++i)
        {
            const Point3d& normal = normals[i];
            normalsData[i] = {static_cast<float>(normal.x), static_cast<float>(-normal.y), static_cast<float>(-normal.z)};
        }

        UsdAttribute normalsAttr = mesh.CreateNormalsAttr();
        normalsAttr.Set(normalsData);
    }

    // write UVs
    if (!inputMesh->uvCoords.empty())
    {
        VtArray<GfVec2f> uvsData;
        uvsData.resize(inputMesh->uvCoords.size());

        for (int i = 0; i < inputMesh->uvCoords.size(); ++i)
        {
            const Point2d& coord = inputMesh->uvCoords[i];
            uvsData[i] = {static_cast<float>(coord.x), static_cast<float>(coord.y)};
        }

        VtIntArray uvsIndices;
        uvsIndices.resize(inputMesh->trisUvIds.size() * 3);

        for (int i = 0; i < inputMesh->trisUvIds.size(); ++i)
        {
            const Voxel& indices = inputMesh->trisUvIds[i];
            uvsIndices[i * 3] = indices.x;
            uvsIndices[i * 3 + 1] = indices.y;
            uvsIndices[i * 3 + 2] = indices.z;
        }

        UsdGeomPrimvarsAPI primvarsApi = UsdGeomPrimvarsAPI(mesh);
        UsdGeomPrimvar uvs =
          primvarsApi.CreateIndexedPrimvar(TfToken("st"), SdfValueTypeNames->TexCoord2fArray, uvsData, uvsIndices, UsdGeomTokens->faceVarying);
    }

    // create material and shaders
    UsdShadeMaterial material = UsdShadeMaterial::Define(stage, SdfPath("/root/mesh/mat"));

    UsdShadeShader preview = UsdShadeShader::Define(stage, SdfPath("/root/mesh/mat/preview"));
    preview.CreateIdAttr(VtValue(UsdImagingTokens->UsdPreviewSurface));
    material.CreateSurfaceOutput().ConnectToSource(preview.ConnectableAPI(), UsdShadeTokens->surface);

    UsdShadeShader uvReader = UsdShadeShader::Define(stage, SdfPath("/root/mesh/mat/uvReader"));
    uvReader.CreateIdAttr(VtValue(UsdImagingTokens->UsdPrimvarReader_float2));
    uvReader.CreateInput(AvUsdTokens->varname, SdfValueTypeNames->Token).Set(AvUsdTokens->st);

    // add textures (only supporting diffuse and normal maps)
    if (texturing.material.hasTextures(mesh::Material::TextureType::DIFFUSE))
    {
        SdfAssetPath diffuseTexturePath{texturing.material.textureName(mesh::Material::TextureType::DIFFUSE, -1)};
        UsdShadeShader diffuseTexture = UsdShadeShader::Define(stage, SdfPath("/root/mesh/mat/diffuseTexture"));
        diffuseTexture.CreateIdAttr(VtValue(UsdImagingTokens->UsdUVTexture));
        diffuseTexture.CreateInput(AvUsdTokens->st, SdfValueTypeNames->Float2).ConnectToSource(uvReader.ConnectableAPI(), AvUsdTokens->result);
        diffuseTexture.CreateInput(AvUsdTokens->file, SdfValueTypeNames->Asset).Set(diffuseTexturePath);
        preview.CreateInput(AvUsdTokens->diffuseColor, SdfValueTypeNames->Color3f).ConnectToSource(diffuseTexture.ConnectableAPI(), AvUsdTokens->rgb);
    }

    if (texturing.material.hasTextures(mesh::Material::TextureType::NORMAL))
    {
        SdfAssetPath normalTexturePath{texturing.material.textureName(mesh::Material::TextureType::NORMAL, -1)};
        UsdShadeShader normalTexture = UsdShadeShader::Define(stage, SdfPath("/root/mesh/mat/normalTexture"));
        normalTexture.CreateIdAttr(VtValue(UsdImagingTokens->UsdUVTexture));
        normalTexture.CreateInput(AvUsdTokens->st, SdfValueTypeNames->Float2).ConnectToSource(uvReader.ConnectableAPI(), AvUsdTokens->result);
        UsdShadeInput file = normalTexture.CreateInput(AvUsdTokens->file, SdfValueTypeNames->Asset);
        file.Set(normalTexturePath);
        file.GetAttr().SetMetadata(AvUsdTokens->colorSpace, AvUsdTokens->Raw);
        preview.CreateInput(AvUsdTokens->normal, SdfValueTypeNames->Normal3f).ConnectToSource(normalTexture.ConnectableAPI(), AvUsdTokens->rgb);

        normalTexture.CreateInput(AvUsdTokens->fallback, SdfValueTypeNames->Float4).Set(GfVec4f{0.5, 0.5, 0.5, 1.0});
        normalTexture.CreateInput(AvUsdTokens->scale, SdfValueTypeNames->Float4).Set(GfVec4f{2.0, 2.0, 2.0, 0.0});
        normalTexture.CreateInput(AvUsdTokens->bias, SdfValueTypeNames->Float4).Set(GfVec4f{-1.0, -1.0, -1.0, 1.0});
        normalTexture.CreateInput(AvUsdTokens->sourceColorSpace, SdfValueTypeNames->Token).Set(AvUsdTokens->raw);
    }

    mesh.GetPrim().ApplyAPI(UsdShadeTokens->MaterialBindingAPI);
    UsdShadeMaterialBindingAPI(mesh).Bind(material);

    stage->GetRootLayer()->Save();

    // Copy textures to output folder
    const fs::path sourceFolder = fs::path(inputMeshPath).parent_path();
    const fs::path destinationFolder = fs::canonical(outputFolderPath);

    for (int i = 0; i < texturing.material.numAtlases(); ++i)
    {
        for (const auto& texture : texturing.material.getAllTextures())
        {
            if (fs::exists(sourceFolder / texture))
            {
                fs::copy_file(sourceFolder / texture, destinationFolder / texture, fs::copy_options::update_existing);
            }
        }
    }

    // write out usdz if requested
    if (fileType == EUSDFileType::USDZ)
    {
        const fs::path usdzPath = fs::canonical(outputFolderPath) / "texturedMesh.usdz";
        UsdZipFileWriter writer = UsdZipFileWriter::CreateNew(usdzPath.string());

        if (!writer)
        {
            ALICEVISION_LOG_ERROR("Cannot create USDZ archive");
            return EXIT_FAILURE;
        }

        writer.AddFile(stagePath.string(), "texturedMesh." + extension);
        for (int i = 0; i < texturing.material.numAtlases(); ++i)
        {
            for (const auto& texture : texturing.material.getAllTextures())
            {
                if (fs::exists(destinationFolder / texture))
                {
                    writer.AddFile((destinationFolder / texture).string(), texture);
                }
            }
        }
        writer.Save();
    }

    ALICEVISION_LOG_INFO("Task done in (s): " + std::to_string(timer.elapsed()));
    return EXIT_SUCCESS;
}
