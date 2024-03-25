// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "plyIO.hpp"

#include <assimp/Importer.hpp>
#include <assimp/scene.h>

#include <fstream>
#include <iostream>

namespace aliceVision {
namespace sfmDataIO {

bool savePLY(const sfmData::SfMData& sfmData, const std::string& filename, ESfMData partFlag)
{
    const bool b_structure = (partFlag & STRUCTURE) == STRUCTURE;
    const bool b_extrinsics = (partFlag & EXTRINSICS) == EXTRINSICS;

    if (!(b_structure || b_extrinsics))
        return false;

    bool b_binary = filename.find(".bin.") != std::string::npos;

    // Create the stream and check it is ok
    std::ofstream stream(filename, b_binary ? std::ios::out | std::ios::binary : std::ios::out);
    if (!stream.is_open())
        return false;

    bool bOk = false;
    {
        // Count how many views having valid poses:
        IndexT view_with_pose_count = 0;
        if (b_extrinsics)
        {
            for (const auto& view : sfmData.getViews())
            {
                view_with_pose_count += sfmData.isPoseAndIntrinsicDefined(view.second.get());
            }
        }

        stream << "ply" << '\n'
               << "format " << (b_binary ? "binary_little_endian" : "ascii") << " 1.0 " << '\n'
               << "element vertex "
               // Vertex count: (#landmark + #view_with_valid_pose)
               << ((b_structure ? sfmData.getLandmarks().size() : 0) + view_with_pose_count) << '\n'
               << "property float x" << '\n'
               << "property float y" << '\n'
               << "property float z" << '\n'
               << "property uchar red" << '\n'
               << "property uchar green" << '\n'
               << "property uchar blue" << '\n'
               << "end_header" << std::endl;

        if (b_extrinsics)
        {
            for (const auto& view : sfmData.getViews())
            {
                if (sfmData.isPoseAndIntrinsicDefined(view.second.get()))
                {
                    const geometry::Pose3 pose = sfmData.getPose(*(view.second.get())).getTransform();

                    if (b_binary)
                    {
                        Vec3f point = pose.center().cast<float>();
                        stream.write(reinterpret_cast<const char*>(&point), sizeof(float) * 3);
                        stream.write(reinterpret_cast<const char*>(&image::GREEN), sizeof(uint8_t) * 3);
                    }
                    else
                    {
                        stream << pose.center().transpose() << " 0 255 0"
                               << "\n";
                    }
                }
            }
        }

        if (b_structure)
        {
            const sfmData::Landmarks& landmarks = sfmData.getLandmarks();

            for (sfmData::Landmarks::const_iterator iterLandmarks = landmarks.begin(); iterLandmarks != landmarks.end(); ++iterLandmarks)

            {
                const auto& landmark = iterLandmarks->second;
                if (b_binary)
                {
                    Vec3f point = landmark.X.cast<float>();
                    stream.write(reinterpret_cast<const char*>(&point), sizeof(float) * 3);
                    stream.write(reinterpret_cast<const char*>(&landmark.rgb), sizeof(uint8_t) * 3);
                }
                else
                {
                    stream << landmark.X.transpose() << " " << (int)landmark.rgb.r() << " " << (int)landmark.rgb.g() << " " << (int)landmark.rgb.b()
                           << "\n";
                }
            }
        }
        stream.flush();
        bOk = stream.good();
        stream.close();
    }
    return bOk;
}

bool loadPLY(sfmData::SfMData& sfmData, const std::string& filename)
{
    Assimp::Importer importer;

    sfmData.clear();

    const aiScene* scene = importer.ReadFile(filename, 0);
    if (!scene)
    {
        return false;
    }

    sfmData::Landmarks & landmarks = sfmData.getLandmarks();

    std::list<aiNode*> nodes;
    nodes.push_back(scene->mRootNode);

    while (!nodes.empty())
    {
        aiNode* node = nodes.back();
        nodes.pop_back();

        for (int idMesh = 0; idMesh < node->mNumMeshes; ++idMesh)
        {
            const unsigned int meshId = node->mMeshes[idMesh];
            const aiMesh* mesh = scene->mMeshes[meshId];

            for (int idPoint = 0; idPoint < mesh->mNumVertices; ++idPoint)
            {
                const aiVector3D v = mesh->mVertices[idPoint];

                sfmData::Landmark l;

                l.X.x() = v.x;
                l.X.y() = -v.y;
                l.X.z() = -v.z;

                if (mesh->HasVertexColors(0))
                {
                    const aiColor4D c = mesh->mColors[0][idPoint];
                    l.rgb.r() = c.r * 255.0;
                    l.rgb.g() = c.g * 255.0;
                    l.rgb.b() = c.b * 255.0;
                }

                landmarks[landmarks.size()] = l;
            }
        }

        for (int idChild = 0; idChild < node->mNumChildren; ++idChild)
        {
            nodes.push_back(node->mChildren[idChild]);
        }
    }

    return true;
}

}  // namespace sfmDataIO
}  // namespace aliceVision
