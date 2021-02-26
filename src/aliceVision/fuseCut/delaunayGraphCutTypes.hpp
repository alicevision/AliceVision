// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/Point3d.hpp>
#include <aliceVision/mvsData/StaticVector.hpp>

#include <array>

namespace aliceVision {
namespace fuseCut {

struct GC_cellInfo
{
    /// initialized to a large value if the tetrahedron is directly in front of one camera ELSE set to 0
    float cellSWeight = 0.0f;
    /// strong fullness: sum of weights for being the tetrahedron 2*sigma behind the point p
    float cellTWeight = 0.0f;
    // float gEdgePhotoWeight[4];
    /// score for emptiness along each egde/facet
    std::array<float, 4> gEdgeVisWeight{{0.0f, 0.0f, 0.0f, 0.0f}};
    /// fullness score: sum of all weights for fullness (just after the point p)
    float fullnessScore = 0.0f;
    /// emptiness score: sum of all weights for emptiness (before the point p)
    float emptinessScore = 0.0f;
    /// first full tetrahedron score: sum of weights for T1 (tetrahedron just after the point p)
    float on = 0.0f;

    void fwriteinfo(FILE* f) const
    {
        fwrite(&cellSWeight, sizeof(float), 1, f);
        fwrite(&cellTWeight, sizeof(float), 1, f);
        fwrite(&fullnessScore, sizeof(float), 1, f);
        fwrite(&emptinessScore, sizeof(float), 1, f);
        fwrite(&on, sizeof(float), 1, f);

        // fwrite(gEdgePhotoWeight,sizeof(float),4,f);
        fwrite(&gEdgeVisWeight.front(), sizeof(float), 4, f);
    }

    void freadinfo(FILE* f)
    {
        fread(&cellSWeight, sizeof(float), 1, f);
        fread(&cellTWeight, sizeof(float), 1, f);
        fread(&fullnessScore, sizeof(float), 1, f);
        fread(&emptinessScore, sizeof(float), 1, f);
        fread(&on, sizeof(float), 1, f);

        // fread(gEdgePhotoWeight,sizeof(float),4,f);
        fread(&gEdgeVisWeight.front(), sizeof(float), 4, f);
    }
};

struct GC_Seg
{
    int segSize = 0;
    int segId = -1;
};

struct GC_vertexInfo
{
    float pixSize = 0.0f;
    /// Number of cameras which have contributed to the refinement of the vertex position, so nrc >= cams.size().
    int nrc = 0;
    /// All cameras having a visibility of this vertex. Some of them may not have contributed to the vertex position
    StaticVector<int> cams;

    /**
     * @brief Is the vertex a virtual point without associated camera? Like helper points or camera points.
     */
    inline bool isVirtual() const { return cams.empty(); }
    inline bool isReal() const { return !cams.empty(); }

    inline std::size_t getNbCameras() const
    {
        return cams.size();
    }

    inline int getCamera(std::size_t index) const
    {
        return cams[index];
    }

    void fwriteinfo(FILE* f) const
    {
        fwrite(&pixSize, sizeof(float), 1, f);
        fwrite(&nrc, sizeof(int), 1, f);
        int n = cams.size();
        fwrite(&n, sizeof(int), 1, f);
        if(n > 0)
        {
            fwrite(&cams[0], sizeof(int), n, f);
        }
    }

    void freadinfo(FILE* f)
    {
        fread(&pixSize, sizeof(float), 1, f);
        fread(&nrc, sizeof(int), 1, f);
        int n;
        fread(&n, sizeof(int), 1, f);
        if(n > 0)
        {
            cams.resize(n);
            fread(&cams[0], sizeof(int), n, f);
        }
    }
};

struct GC_camVertexInfo
{
    float sim; // TODO FACA: default value?
    int nrc = 0;
    int ncams = 0;
    Point3d point;

    void fwriteinfo(FILE* f)
    {
        fwrite(&sim, sizeof(float), 1, f);
        fwrite(&nrc, sizeof(int), 1, f);
        fwrite(&ncams, sizeof(int), 1, f);
        fwrite(&point, sizeof(Point3d), 1, f);
    }

    void freadinfo(FILE* f)
    {
        fread(&sim, sizeof(float), 1, f);
        fread(&nrc, sizeof(int), 1, f);
        fread(&ncams, sizeof(int), 1, f);
        fread(&point, sizeof(Point3d), 1, f);
    }
};

inline std::ostream& operator<<(std::ostream& stream, const GC_cellInfo& cellInfo)
{
    stream << "cellSWeight:" << cellInfo.cellSWeight
           << ",cellTWeight:" << cellInfo.cellTWeight
           << ",gEdgeVisWeight[0]:" << cellInfo.gEdgeVisWeight[0]
           << ",gEdgeVisWeight[1]:" << cellInfo.gEdgeVisWeight[1]
           << ",gEdgeVisWeight[2]:" << cellInfo.gEdgeVisWeight[2]
           << ",gEdgeVisWeight[3]:" << cellInfo.gEdgeVisWeight[3]
           << ",fullnessScore:" << cellInfo.fullnessScore
           << ",emptinessScore:" << cellInfo.emptinessScore
           << ",on:" << cellInfo.on
        ;
    return stream;
}

} // namespace fuseCut
} // namespace aliceVision
