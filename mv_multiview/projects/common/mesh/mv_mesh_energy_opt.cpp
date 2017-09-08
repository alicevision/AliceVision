#include "mv_mesh_energy_opt.h"
#include "stdafx.h"
#include <boost/filesystem.hpp>

namespace bfs = boost::filesystem;

mv_mesh_energy_opt::mv_mesh_energy_opt(multiviewParams* _mp)
    : mv_mesh_analyze(_mp)
{
//    tmpDir = mp->mip->mvDir + "meshEnergyOpt/";
//    bfs::create_directory(tmpDir);
}

mv_mesh_energy_opt::~mv_mesh_energy_opt() = default;

staticVector<point3d>* mv_mesh_energy_opt::computeLaplacianPts()
{
    staticVector<point3d>* lapPts = new staticVector<point3d>(pts->size());
    lapPts->resize_with(pts->size(), point3d(0.0f, 0.0f, 0.f));
    int nlabpts = 0;
    for(int i = 0; i < pts->size(); i++)
    {
        point3d lapPt;
        if(getLaplacianSmoothingVector(i, lapPt))
        {
            (*lapPts)[i] = lapPt;
            nlabpts++;
        }
    }

    return lapPts;
}

staticVector<point3d>* mv_mesh_energy_opt::computeLaplacianPtsParallel()
{
    staticVector<point3d>* lapPts = new staticVector<point3d>(pts->size());
    lapPts->resize_with(pts->size(), point3d(0.0f, 0.0f, 0.f));
    int nlabpts = 0;

#pragma omp parallel for
    for(int i = 0; i < pts->size(); i++)
    {
        point3d lapPt;
        if(getLaplacianSmoothingVector(i, lapPt))
        {
            (*lapPts)[i] = lapPt;
            nlabpts++;
        }
    }

    return lapPts;
}

void mv_mesh_energy_opt::updateGradientParallel(float lambda, float epsilon, int type, const point3d& LU,
                                                const point3d& RD, staticVectorBool* ptsCanMove)
{
    // printf("nlabpts %i of %i\n",nlabpts,pts->size());
    // staticVector<point3d> *lapPts = computeLaplacianPts();
    staticVector<point3d>* lapPts = computeLaplacianPtsParallel();

    staticVector<point3d>* newPts = new staticVector<point3d>(pts->size());
    newPts->push_back_arr(pts);

#pragma omp parallel for
    for(int i = 0; i < pts->size(); i++)
    {
        if((ptsCanMove == nullptr) || ((*ptsCanMove)[i]))
        {

            point3d n;

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
                        point3d p = (*newPts)[i] + n * lambda;
                        if((p.x > LU.x) && (p.y > LU.y) && (p.z > LU.z) && (p.x < RD.x) && (p.y < RD.y) && (p.z < RD.z))
                        {
                            (*newPts)[i] = p;
                        }
                    }
                    /*
            }else{
                    if (applyLaplacianOperator(i, pts, n)==true)
                    {
                            point3d p = (*newPts)[i] + n * lamda;
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
                    point3d lap;
                    point3d bilap;
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

bool mv_mesh_energy_opt::optimizeSmooth(float lambda, float epsilon, int type, int niter, staticVectorBool* ptsCanMove)
{
    if(pts->size() <= 4)
    {
        return false;
    }

    bool saveDebug = mp->mip->_ini.get<bool>("meshEnergyOpt.saveAllIterations", false);

    point3d LU, RD;
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
            o3d->saveMvMeshToObj(this, mp->mip->mvDir + "mesh_smoothed_" + std::to_string(i) + ".obj");
    }

    return true;
}
