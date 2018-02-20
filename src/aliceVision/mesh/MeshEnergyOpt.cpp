// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "MeshEnergyOpt.hpp"

#include <boost/filesystem.hpp>

namespace aliceVision {

namespace bfs = boost::filesystem;

MeshEnergyOpt::MeshEnergyOpt(MultiViewParams* _mp)
    : MeshAnalyze(_mp)
{
//    tmpDir = mp->mip->mvDir + "meshEnergyOpt/";
//    bfs::create_directory(tmpDir);
}

MeshEnergyOpt::~MeshEnergyOpt() = default;

StaticVector<Point3d>* MeshEnergyOpt::computeLaplacianPtsParallel()
{
    StaticVector<Point3d>* lapPts = new StaticVector<Point3d>(pts->size());
    lapPts->resize_with(pts->size(), Point3d(0.0f, 0.0f, 0.f));
    int nlabpts = 0;

#pragma omp parallel for
    for(int i = 0; i < pts->size(); i++)
    {
        Point3d lapPt;
        if(getLaplacianSmoothingVector(i, lapPt))
        {
            (*lapPts)[i] = lapPt;
            nlabpts++;
        }
    }

    return lapPts;
}

void MeshEnergyOpt::updateGradientParallel(float lambda, float epsilon, int type, const Point3d& LU,
                                                const Point3d& RD, StaticVectorBool* ptsCanMove)
{
    // printf("nlabpts %i of %i\n",nlabpts,pts->size());
    StaticVector<Point3d>* lapPts = computeLaplacianPtsParallel();

    StaticVector<Point3d>* newPts = new StaticVector<Point3d>(pts->size());
    newPts->push_back_arr(pts);

#pragma omp parallel for
    for(int i = 0; i < pts->size(); i++)
    {
        if((ptsCanMove == nullptr) || ((*ptsCanMove)[i]))
        {

            Point3d n;

            switch(type)
            {
                case 0:
                // doesn't work
                    if(getVertexMeanCurvatureNormal(i, n))
                    {
                        (*newPts)[i] = (*newPts)[i] + n * lambda;
                    }
                    break;
                case 1:
                // smooth only
                    if(applyLaplacianOperator(i, pts, n))
                    {
                        (*newPts)[i] = (*newPts)[i] + n * lambda;
                    }
                    break;
                case 2:
                // doesn't work
                    if(getMeanCurvAndLaplacianSmoothing(i, n, epsilon))
                    {
                        (*newPts)[i] = (*newPts)[i] + n * lambda;
                    }
                    break;
                case 3:
                // default mode

                    // if (isIsBoundaryPt(i)==false)
                    //{
                    if(getBiLaplacianSmoothingVector(i, lapPts, n))
                    {
                        Point3d p = (*newPts)[i] + n * lambda;
                        if((p.x > LU.x) && (p.y > LU.y) && (p.z > LU.z) && (p.x < RD.x) && (p.y < RD.y) && (p.z < RD.z))
                        {
                            (*newPts)[i] = p;
                        }
                    }
                    /*
            }else{
                    if (applyLaplacianOperator(i, pts, n)==true)
                    {
                            Point3d p = (*newPts)[i] + n * lamda;
                            if ((p.x>LU.x)&&(p.y>LU.y)&&(p.z>LU.z)&&
                                    (p.x<RD.x)&&(p.y<RD.y)&&(p.z<RD.z))
                            {
                                    (*newPts)[i] = p;
                            };
                    };
            };
            */
                    break;
                case 4:
                // doesn't work
                    Point3d lap;
                    Point3d bilap;
                    if((applyLaplacianOperator(i, pts, lap)) && (applyLaplacianOperator(i, lapPts, bilap)) &&
                       (!isIsBoundaryPt(i)))
                    {
                        (*newPts)[i] = (*newPts)[i] + bilap * lambda;
                    }
                    break;
            }
        }
    }

    delete pts;
    pts = newPts;
    delete lapPts;
}

bool MeshEnergyOpt::optimizeSmooth(float lambda, float epsilon, int type, int niter, StaticVectorBool* ptsCanMove)
{
    if(pts->size() <= 4)
    {
        return false;
    }

    bool saveDebug = mp->mip->_ini.get<bool>("meshEnergyOpt.saveAllIterations", false);

    Point3d LU, RD;
    LU = (*pts)[0];
    RD = (*pts)[0];
    for(int i = 0; i < pts->size(); i++)
    {
        LU.x = std::min(LU.x, (*pts)[i].x);
        LU.y = std::min(LU.y, (*pts)[i].y);
        LU.z = std::min(LU.z, (*pts)[i].z);
        RD.x = std::max(RD.x, (*pts)[i].x);
        RD.y = std::max(RD.y, (*pts)[i].y);
        RD.z = std::max(RD.z, (*pts)[i].z);
    }

    if(mp->verbose)
        printf("optimizing mesh : lamda %f, epsilon %f, type %i, niters %i\n", lambda, epsilon, type, niter);
    for(int i = 0; i < niter; i++)
    {
        updateGradientParallel(lambda, epsilon, type, LU, RD, ptsCanMove);
        if(saveDebug)
            saveToObj(mp->mip->mvDir + "mesh_smoothed_" + std::to_string(i) + ".obj");
    }

    return true;
}

} // namespace aliceVision
