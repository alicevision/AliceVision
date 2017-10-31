#pragma once

#include "mv_mesh.hpp"
#include <vector>

class mv_mesh_uvatlas
{
public:
    struct Edge
    {
        std::pair<int, int> pointIDs;
        std::vector<int> triangleIDs;
        bool operator<(const Edge& other) const { return pointIDs < other.pointIDs; }
        bool operator==(const Edge& other) const { return pointIDs == other.pointIDs; }
    };

    struct Chart
    {
        int refCameraID = -1;                                   // refCamera, used to project all contained triangles
        std::vector<int> commonCameraIDs;                       // list of common cameras
        std::vector<int> triangleIDs;                           // list of all contained triangles
        pixel sourceLU;                                         // left-up pixel coordinates (in refCamera space)
        pixel sourceRD;                                         // right-down pixel coordinates (in refCamera space)
        pixel targetLU;                                         // left-up pixel coordinates (in uvatlas texture)
        int mergedWith = -1;                                    // ID of target chart, or -1 (not merged)
        int width() const { return sourceRD.x - sourceLU.x; }
        int height() const { return sourceRD.y - sourceLU.y; }
    };

    struct ChartRect
    {
        Chart* c = nullptr;
        ChartRect* child[2] {nullptr, nullptr};
        pixel LU;
        pixel RD;
        void clear();
        ChartRect* insert(Chart& chart, size_t gutter);
    };

public:
    mv_mesh_uvatlas(const mv_mesh& mesh, multiviewParams& mp, staticVector<staticVector<int>*>* ptsCams,
                    unsigned int textureSide, unsigned int gutterSize);

public:
    const std::vector<std::vector<Chart>>& atlases() const { return _atlases; }
    const std::vector<int>& visibleCameras(int triangleID) const { return _triangleCameraIDs[triangleID]; }
    int textureSide() const { return _textureSide; }
    int gutterSize() const { return _gutterSize; }
    const mv_mesh& mesh() const { return _mesh; }

private:
    void createCharts(std::vector<Chart>& charts, multiviewParams& mp, staticVector<staticVector<int>*>* ptsCams);
    void packCharts(std::vector<Chart>& charts, multiviewParams& mp);
    void finalizeCharts(std::vector<Chart>& charts, multiviewParams& mp);
    void createTextureAtlases(std::vector<Chart>& charts, multiviewParams& mp);

private:
    std::vector<std::vector<Chart>> _atlases;
    std::vector<std::vector<int>> _triangleCameraIDs;
    int _textureSide;
    int _gutterSize;
    const mv_mesh& _mesh;
};
