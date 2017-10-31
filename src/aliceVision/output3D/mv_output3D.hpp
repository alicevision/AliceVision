#pragma once

#include <aliceVision/mesh/mv_mesh_uvatlas.hpp>
#include <aliceVision/structures/mv_multiview_params.hpp>
#include <aliceVision/structures/quaternion.hpp>
#include <aliceVision/structures/mv_images_cache.hpp>
#include <aliceVision/structures/mv_color.hpp>
#include <boost/filesystem.hpp>


class mv_output3D
{
    const multiviewParams* mp;
    int g_border;
    float pixelSizeErrThr;

public:
    mv_output3D(const multiviewParams* _mp);

public:
    ~mv_output3D(void);

public:
    // mv_mesh *depthMap2Mesh(staticVector<float> *depthMap, int rc);
    void depthMap2Ply(const std::string& plyFileName, staticVector<float>* depthMap, int rc);
    void savePrematchToWrl(const std::string& wrname);
    void savePrematchToWrl(const std::string& wrname, int shift, int step);
    void printfGroupCameras(mv_mesh* meCams, staticVector<int>* tcams, FILE* f, const multiviewParams* mp, float ps,
                            float upStep);
    void printfGroupCameras(mv_mesh* meCams, staticVector<int>* tcams, FILE* f, const multiviewParams* mp, float ps,
                            int shift, int step, float upStep);
    void printfGroupCamerasOmniSequnece(staticVector<int>* tcams, FILE* f, const multiviewParams* mp, float ps,
                                        int camerasPerOneOmni, float upStep);
    void create_wrl(const multiviewParams* mp, int filerPerc, int winSizeHalf, int colorType, float simThr,
                    const std::string& wrlname, int maxPointsNum, bool savepset);
    void create_wrl(const multiviewParams* mp, int filerPerc, int winSizeHalf, int colorType, float simThr,
                    const std::string& wrlname, int maxPointsNum, bool savepset, int camshift, int camstep);
    void printfGroupCamera_rc(FILE* f, const multiviewParams* mp, float ps, int rc);

    void initNtrisNptsAndMap2Pts(int* _ntris, int* _npts, staticVector<float>* depthMap, int rc,
                                 staticVector<int>** _map2pts);
    int write2PlyPts(FILE* plyf, bool dobinary, int ntris, int npts, staticVector<float>* depthMap, int rc,
                     staticVector<int>* map2pts, int ptIndexPrefix, bool textureOrRc, voxel* col, bool write_color);
    int write2PlyTris(FILE* plyf, bool dobinary, int ntris, int npts, staticVector<float>* depthMap, int rc,
                      staticVector<int>* map2pts, int ptIndexPrefix, int gnpts, bool islast);

    staticVector<pixel>* findNeighbours(const pixel& seed, int r, staticVector<float>* depthMap);
    void create_wrl_pts(const multiviewParams* mp, const std::string& wrlname, int shift, int step);
    void printf_wrl_pts(FILE* f, staticVector<point3d>* pts, const multiviewParams* mp);
    void create_wrl_pts(staticVector<point3d>* pts, const multiviewParams* mp, const std::string& wrlname);
    void create_wrl_pts_nms(staticVector<point3d>* pts, staticVector<point3d>* nms, const multiviewParams* mp,
                            const std::string& wrlname, float r = 0.0f, float g = 1.0f, float b = 0.0f);
    void create_wrl_pts_cls(staticVector<point3d>* pts, staticVector<voxel>* colors, const multiviewParams* mp,
                            const std::string& wrlname);
    void printf_wrl_pts_cls(FILE* f, staticVector<point3d>* pts, staticVector<voxel>* colors, const multiviewParams* mp);
    void save_triangulation_to_wrl(const multiviewParams* mp, const std::string& inputFileName, const std::string& wrlFileName);
    void convertPly2Wrl(const std::string& wrlFileName, const std::string& plyFileName, bool write_color, int camshift, int camstep);
    void create_wrl_for_delaunay_cut(const multiviewParams* mp, const std::string& inputFileNameT, const std::string& inputFileNameRC,
                                     const std::string& wrlFileName, const std::string& wrlDir, int camerasPerOneOmni);
    void create_wrl_for_delaunay_cut(const multiviewParams* mp, const std::string& inputFileNameT, const std::string& inputFileNameRC,
                                     const std::string& wrlFileName, const std::string& wrlDir, int camerasPerOneOmni,
                                     staticVector<int>* tcams);
    void create_ply_for_delaunay_cut_strecha(const multiviewParams* mp, const std::string& inputFileNameT, const std::string& plyFileName);
    void saveMvMeshToObj(mv_mesh* me, const std::string& objFileName);
    void saveMvMeshToWrl(staticVectorBool* triSource, mv_mesh* me, const std::string& wrlFileName);
    void saveMvMeshToWrl(mv_mesh* me, const std::string& wrlFileName, staticVector<rgb>* colors = nullptr, bool solid = true, bool colorPerVertex = false);
    void saveMvMeshToWrlPlusPts(staticVector<point3d>* pts, staticVector<rgb>* triColors, mv_mesh* me,
                                const std::string& wrlFileName);
    void saveMvMeshToWrl(int scaleFactor, mv_mesh* me, staticVector<Color>* camsColorScales,
                         staticVector<Color>* camsColorShifts, staticVector<int>* rcTris, const multiviewParams* mp,
                         const std::string& wrlFileName, const std::string& wrlDir, int camerasPerOneOmni);
    void saveMvMeshToWrl(staticVector<float>* ptsValues, float minVal, float maxVal, mv_mesh* me,
                         const std::string& wrlFileName, bool solid);
    void saveMvMeshToWrlPtsColors(staticVector<rgb>* ptsColors, mv_mesh* me, const std::string& wrlFileName, bool solid);
    void printfMvMeshToWrl(FILE* f, rgb& colorOfTris, mv_mesh* me);

    void printfHexahedron(point3d* hexah, FILE* f, const multiviewParams* mp);
    void saveVoxelsToWrl(const std::string& wrlFileName, const multiviewParams* mp, staticVector<point3d>* voxels);
    void printfGroupCamerasDepth(FILE* f, const multiviewParams* mp, staticVector<int>* cams, rgb camcolor);
    void writeCamerasDepthToWrl(const std::string& wrlFileName, const multiviewParams* mp);
    void writeCamerasToWrl(const std::string& wrlFileName, const multiviewParams* mp, int camerasPerOneOmni, float upStep);
    void writeCamerasToWrl(staticVector<int>* tcams, const std::string& wrlFileName, const multiviewParams* mp,
                           int camerasPerOneOmni, float upStep);
    void inline2Wrls(const std::string& wrlFileNameOut, const std::string& wrlFileName1, const std::string& wrlFileName2);
    int saveMeshToWrlTrisAverageTextured(const std::string& dirName, const std::string& wrlName, const multiviewParams* mp, mv_mesh* me,
                                         staticVector<staticVector<int>*>* trisCams);
    void savePtsAsSpheresToWrl(const std::string& wrlName, staticVector<point3d>* pts, float r, float g, float b,
                               float radius = 0.1f);
    void groupTexturesToOneFileWrl(staticVector<point2d>* meshPtsUV, mv_mesh* me, int newTextureSide,
                                   const std::string& newTextName, const std::string& wrlMeshFileName);
    void groupTexturesToOneFile(mv_mesh* me, int newTextureSide, const std::string& meTextureCorrdsFileName,
                                const std::string& wrlMeshFileName, const std::string& outDir);
};
