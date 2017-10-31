#include "mv_common.hpp"

#include "mv_geometry.hpp"
#include "mv_geometry_triTri.hpp"
#include "mv_filesio.hpp"

#include <opencv/cv.h>
#include <opencv/highgui.h>

#ifdef _WIN32
#include "Psapi.h"
#endif

float jetr[64] = {0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,
                  0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,
                  0,      0,      0.0625, 0.1250, 0.1875, 0.2500, 0.3125, 0.3750, 0.4375, 0.5000, 0.5625,
                  0.6250, 0.6875, 0.7500, 0.8125, 0.8750, 0.9375, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000,
                  1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000,
                  1.0000, 0.9375, 0.8750, 0.8125, 0.7500, 0.6875, 0.6250, 0.5625, 0.5000};
float jetg[64] = {0,      0,      0,      0,      0,      0,      0,      0,      0.0625, 0.1250, 0.1875,
                  0.2500, 0.3125, 0.3750, 0.4375, 0.5000, 0.5625, 0.6250, 0.6875, 0.7500, 0.8125, 0.8750,
                  0.9375, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000,
                  1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 0.9375, 0.8750, 0.8125, 0.7500,
                  0.6875, 0.6250, 0.5625, 0.5000, 0.4375, 0.3750, 0.3125, 0.2500, 0.1875, 0.1250, 0.0625,
                  0,      0,      0,      0,      0,      0,      0,      0,      0};
float jetb[64] = {0.5625, 0.6250, 0.6875, 0.7500, 0.8125, 0.8750, 0.9375, 1.0000, 1.0000, 1.0000, 1.0000,
                  1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000,
                  1.0000, 1.0000, 0.9375, 0.8750, 0.8125, 0.7500, 0.6875, 0.6250, 0.5625, 0.5000, 0.4375,
                  0.3750, 0.3125, 0.2500, 0.1875, 0.1250, 0.0625, 0,      0,      0,      0,      0,
                  0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,
                  0,      0,      0,      0,      0,      0,      0,      0,      0};


bool get2dLineImageIntersection(point2d* pFrom, point2d* pTo, point2d linePoint1, point2d linePoint2,
                                const multiviewParams* mp, int camId)
{
    point2d v = linePoint2 - linePoint1;

    if(v.size() < FLT_EPSILON)
    {
        return false; // bad configuration ... forward motion with cental ref pixel
    }

    v = v.normalize();

    float a = -v.y;
    float b = v.x;
    float c = -a * linePoint1.x - b * linePoint1.y;

    int intersections = 0;
    float rw = (float)mp->mip->getWidth(camId);
    float rh = (float)mp->mip->getHeight(camId);

    // ax + by + c = 0

    // right epip line intersection with the left side of the right image
    // a*0 + b*y + c = 0; y = -c / b;
    float x = 0;
    float y = -c / b;
    if((y >= 0) && (y < rh))
    {
        *pFrom = point2d(x, y);
        intersections++;
    }

    // right epip line intersection with the right side of the right image
    // a*rw + b*y + c = 0; y = (-c-a*rw) / b;
    x = rw;
    y = (-c - a * rw) / b;
    if((y >= 0) && (y < rh))
    {
        if(intersections == 0)
        {
            *pFrom = point2d(x, y);
        }
        else
        {
            *pTo = point2d(x, y);
        }
        intersections++;
    }

    // right epip line intersection with the top side of the right image
    // a*x + b*0 + c = 0; x = -c / a;
    x = -c / a;
    y = 0;
    if((x >= 0) && (x < rw))
    {
        if(intersections == 0)
        {
            *pFrom = point2d(x, y);
        }
        else
        {
            *pTo = point2d(x, y);
        }
        intersections++;
    }

    // right epip line intersection with the bottom side of the right image
    // a*x + b*rh + c = 0; x = (-c-b*rh) / a;
    x = (-c - b * rh) / a;
    y = rh;
    if((x >= 0) && (x < rw))
    {
        if(intersections == 0)
        {
            *pFrom = point2d(x, y);
        }
        else
        {
            *pTo = point2d(x, y);
        }
        intersections++;
    }

    if(intersections == 2)
    {
        if((linePoint1 - *pFrom).size() > (linePoint1 - *pTo).size())
        {
            point2d p = *pFrom;
            *pFrom = *pTo;
            *pTo = p;
        }
        return true;
    }

    return false;
}

bool getTarEpipolarDirectedLine(point2d* pFromTar, point2d* pToTar, point2d refpix, int refCam, int tarCam,
                                const multiviewParams* mp)
{
    const matrix3x4& rP = mp->camArr[refCam];
    const matrix3x4& tP = mp->camArr[tarCam];

    point3d rC;
    matrix3x3 rR;
    matrix3x3 riR;
    matrix3x3 rK;
    matrix3x3 riK;
    matrix3x3 riP;
    mp->decomposeProjectionMatrix(rC, rR, riR, rK, riK, riP, rP);

    point3d tC;
    matrix3x3 tR;
    matrix3x3 tiR;
    matrix3x3 tK;
    matrix3x3 tiK;
    matrix3x3 tiP;
    mp->decomposeProjectionMatrix(tC, tR, tiR, tK, tiK, tiP, tP);

    point3d refvect = riP * refpix;
    refvect = refvect.normalize();

    float d = (rC - tC).size();
    point3d X = refvect * d + rC;
    point2d tarpix1;
    mp->getPixelFor3DPoint(&tarpix1, X, tP);

    X = refvect * d * 500.0 + rC;
    point2d tarpix2;
    mp->getPixelFor3DPoint(&tarpix2, X, tP);

    return get2dLineImageIntersection(pFromTar, pToTar, tarpix1, tarpix2, mp, tarCam);
}

bool triangulateMatch(point3d& out, const point2d& refpix, const point2d& tarpix, int refCam, int tarCam,
                      const multiviewParams* mp)
{
    point3d refvect = mp->iCamArr[refCam] * refpix;
    refvect = refvect.normalize();
    point3d refpoint = refvect + mp->CArr[refCam];

    point3d tarvect = mp->iCamArr[tarCam] * tarpix;
    tarvect = tarvect.normalize();
    point3d tarpoint = tarvect + mp->CArr[tarCam];

    float k, l;
    point3d lli1, lli2;

    return lineLineIntersect(&k, &l, &out, &lli1, &lli2, mp->CArr[refCam], refpoint, mp->CArr[tarCam], tarpoint);
}

bool triangulateMatchLeft(point3d& out, const point2d& refpix, const point2d& tarpix, int refCam, int tarCam,
                          const multiviewParams* mp)
{
    point3d refvect = mp->iCamArr[refCam] * refpix;
    refvect = refvect.normalize();
    point3d refpoint = refvect + mp->CArr[refCam];

    point3d tarvect = mp->iCamArr[tarCam] * tarpix;
    tarvect = tarvect.normalize();
    point3d tarpoint = tarvect + mp->CArr[tarCam];

    return lineLineIntersectLeft(out, mp->CArr[refCam], refpoint, mp->CArr[tarCam], tarpoint);
}

void printfPercent(int i, int n)
{
    if((int)((float)i / ((float)n / 100.0)) != (int)((float)(i + 1) / ((float)n / 100.0)))
    {
        int perc = (int)((float)i / ((float)n / 100.0));
        printf("\b\b\b%s", num2strThreeDigits(perc).c_str());
    }
}

long initEstimate()
{
    printf("                                      ");
    return clock();
}

void printfEstimate(int i, int n, long startTime)
{
    if((int)((float)i / ((float)n / 100.0)) != (int)((float)(i + 1) / ((float)n / 100.0)))
    {
        int perc = (int)((float)i / ((float)n / 100.0));

        long t2 = clock();

        long mils = (long)(((float)(t2 - startTime) / (float)perc) * 100.0) - (t2 - startTime);
        long seco = mils / CLOCKS_PER_SEC;
        long minu = seco / 60;
        long hour = minu / 60;
        long days = std::abs(hour / 24);

        int ihour = std::abs(hour % 60);
        int iminu = std::abs(minu % 60);
        int iseco = std::abs(seco % 60);

        float d1 = (float)(t2 - startTime) / (float)CLOCKS_PER_SEC;
        int elapsedsec = (int)d1 - (int)floor(d1 / 60.0) * 60;

        if(elapsedsec > 15)
        {
            printf("\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b%03i%% - remaining "
                   "time %02li days %02i:%02i:%02i",
                   perc, days, ihour, iminu, iseco);
        }
    }
}

void finishEstimate()
{
    printf("\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b");
    printf("                                      ");
    printf("\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b");
}

std::string printfElapsedTime(long t1, std::string prefix)
{
    long t2 = clock();
    float d1 = (float)(t2 - t1) / (float)CLOCKS_PER_SEC;

    int min = (int)floor(d1 / 60.0);
    int sec = (int)d1 - (int)floor(d1 / 60.0) * 60;
    int mil = (int)((d1 - (int)floor(d1)) * 1000);

    printf("%s elapsed time %i minutes %i seconds %i miliseconds\n", prefix.c_str(), min, sec, mil);
    std::string out = prefix + " elapsed time " + num2strTwoDecimal(min) + " minutes " + num2strTwoDecimal(sec) +
                      " seconds " + num2strThreeDigits(mil) + " miliseconds\n";
    return out;
}

void ransac_rsample(int* indexes, int npoints, int npoinsRansac)
{
    int* pool = new int[npoints];
    for(int i = 0; i < npoints; i++)
    {
        pool[i] = i;
    }

    for(int i = 0; i < npoinsRansac; i++)
    {
        int s = rand() % (npoints - i);
        int j = npoints - i - 1;
        int q = pool[s];
        pool[s] = pool[j];
        pool[j] = q;
        indexes[i] = pool[j];
    }

    delete[] pool;
}

// SampleCnt calculates number of samples needed to be done
int ransac_nsamples(int ni, int npoints, int npoinsRansac, float conf)
{
    float q = 1.0f;
    for(int i = 0; i < npoinsRansac; i++)
    {
        q = q * ((float)(ni - i) / (float)(npoints - i));
    }

    if(q < 0.0000000000001f)
    {
        return npoints * 100000;
    }

    if(q > conf)
    {
        return 1;
    }

    return (int)(log(1.0 - conf) / log(1.0 - q));
}

bool ransacPlaneFit(orientedPoint& plane, staticVector<point3d>* points, staticVector<point3d>* points_samples,
                    const multiviewParams* mp, int rc, float pixEpsThr)
{

    // RANSAC best plane
    int max_sam = points_samples->size() * 3;
    int no_sam = 0;
    int max_i = 3;
    point3d max_p;
    point3d max_n;

    while(no_sam < max_sam)
    // while (no_sam < 1000)
    {
        no_sam = no_sam + 1;

        // randomne vyber 3 body
        int indexes[3];
        ransac_rsample(indexes, points_samples->size(), 3);

        // vypocitaj rovinu
        point3d p = (*points_samples)[indexes[0]];
        point3d v1 = (*points_samples)[indexes[1]] - p;
        v1 = v1.normalize();
        point3d v2 = (*points_samples)[indexes[2]] - p;
        v2 = v2.normalize();
        point3d n = cross(v1, v2);
        n = n.normalize();

        point3d s =
            ((*points_samples)[indexes[0]] + (*points_samples)[indexes[1]] + (*points_samples)[indexes[2]]) / 3.0f;
        float epsThr = mp->getCamPixelSize(s, rc) * pixEpsThr;

        // zisti kolko z bodov lezi na rovine urcenej tymi bodmi
        int no_i = 0;
        for(int i = 0; i < points->size(); i++)
        {
            float d = pointPlaneDistance((*points)[i], p, n);
            if(d < epsThr)
            {
                no_i++;
            }
        }

        if(max_i < no_i)
        {
            max_i = no_i;
            max_p = p;
            max_n = n;
            max_sam = std::max(100, std::min(max_sam, ransac_nsamples(max_i, points->size(), 3, 0.9)));
        }
    }

    if(max_i == 3)
    {
        return false;
    }

    plane.p = max_p;
    plane.n = max_n;
    plane.sim = (float)max_i;

    return true;
}

bool multimodalRansacPlaneFit(orientedPoint& plane, staticVector<staticVector<point3d>*>* modalPoints,
                              const multiviewParams* mp, int rc, float pixEpsThr)
{
    staticVector<point3d>* points = (*modalPoints)[0];

    if(points->size() < 10)
    {
        return false;
    }

    staticVector<pixel>* modalHist = new staticVector<pixel>(modalPoints->size());

    // RANSAC best plane
    int max_sam = points->size() * 100;
    int max_i = 0;
    point3d max_p;
    point3d max_n;
    int no_sam = 0;

    while(no_sam < max_sam)
    {
        no_sam = no_sam + 1;

        // vypocitaj rovinu
        point3d p, n;
        int indexes[3];
        // bool ok = false;
        // while (ok==false)
        //{
        // randomne vyber 3 body
        ransac_rsample(indexes, points->size(), 3);

        p = (*points)[indexes[0]];
        point3d v1 = (*points)[indexes[1]] - p;
        v1 = v1.normalize();
        point3d v2 = (*points)[indexes[2]] - p;
        v2 = v2.normalize();
        n = cross(v1, v2);
        n = n.normalize();

        // ok = (notOrientedangleBetwV1andV2((mp->CArr[rc]-p).normalize(),n) < 80.0f);
        //};

        // if (notOrientedangleBetwV1andV2((mp->CArr[rc]-p).normalize(),n) < 80.0f )
        {
            point3d s = ((*points)[indexes[0]] + (*points)[indexes[1]] + (*points)[indexes[2]]) / 3.0f;
            float epsThr = mp->getCamPixelSize(s, rc) * pixEpsThr;

            // zisti kolko z bodov lezi na rovine urcenej tymi bodmi
            int no_i = 0;
            for(int i = 0; i < points->size(); i++)
            {
                float d = pointPlaneDistance((*points)[i], p, n);
                if(d < epsThr)
                {
                    no_i++;
                }
            }

            // compute histogram
            for(int m = 0; m < modalPoints->size(); m++)
            {
                staticVector<point3d>* mpts = (*modalPoints)[m];
                (*modalHist)[m].x = 0;
                (*modalHist)[m].y = m;
                for(int i = 0; i < mpts->size(); i++)
                {
                    float d = pointPlaneDistance((*mpts)[i], p, n);
                    if(d < epsThr)
                    {
                        (*modalHist)[m].x++;
                    }
                }
            }

            qsort(&(*modalHist)[0], modalHist->size(), sizeof(int), qSortComparePixelByXDesc);

            // find 0 cams
            int camid = 0;
            for(int m = 0; m < modalPoints->size(); m++)
            {
                if((*modalHist)[m].y == 0)
                {
                    camid = m;
                }
            }

            pixel rh = (*modalHist)[camid];
            int i1 = 0;
            pixel th1 = (*modalHist)[i1];
            if(th1.y == 0)
            {
                i1++;
                th1 = (*modalHist)[i1];
            }

            i1++;
            pixel th2 = (*modalHist)[i1];
            if(th2.y == 0)
            {
                i1++;
                th2 = (*modalHist)[i1];
            }

            i1++;
            pixel th3 = (*modalHist)[i1];
            if(th3.y == 0)
            {
                i1++;
                th3 = (*modalHist)[i1];
            }

            float perc1 = 100.0f;
            if(rh.x > th1.x)
            {
                perc1 = ((float)th1.x / (float)rh.x) * 100.0f;
            }

            float perc2 = 100.0f;
            if(rh.x > th2.x)
            {
                perc2 = ((float)th2.x / (float)rh.x) * 100.0f;
            }

            float perc3 = 100.0f;
            if(rh.x > th3.x)
            {
                perc3 = ((float)th3.x / (float)rh.x) * 100.0f;
            }

            if((perc1 > 80.0f) && (perc2 > 80.0f) && (perc3 > 80.0f) && (max_i < rh.x))
            {
                max_i = rh.x;
                max_p = p;
                max_n = n;
                max_sam = std::max(100, std::min(max_sam, ransac_nsamples(max_i, points->size(), 3, 0.9)));
            }
        }
    }

    plane.p = max_p;
    plane.n = max_n;
    plane.sim = (float)max_i;

    delete modalHist;

    return true;
}

float gaussKernelEnergy(orientedPoint* pt, staticVector<orientedPoint*>* pts, float sigma)
{
    float sum = 0.0;
    for(int j = 0; j < pts->size(); j++)
    {
        orientedPoint* op = (*pts)[j];
        // float d = dot(op->n, op->p - pt->p);
        float d = (op->p - pt->p).size();
        float a = (d * d) / (2.0 * sigma * sigma);
        sum += 1.0 - exp(-a);
    }

    return sum;
}

float angularDistnace(orientedPoint* op1, orientedPoint* op2)
{
    // MJ090803
    return std::max(pointPlaneDistance(op1->p, op2->p, op2->n), pointPlaneDistance(op2->p, op1->p, op1->n)) / 2.0;

    // return fabs(dot(op1->p - op2->p, op1->n)) + fabs(dot(op1->p - op2->p, op2->n));
}

bool arecoincident(orientedPoint* op1, orientedPoint* op2, float pixSize)
{
    return ((angleBetwV1andV2(op1->n, op2->n) < 70.0) &&
            //( pointLineDistance3D(&op1->p, &op2->p, &op2->n)+
            //  pointLineDistance3D(&op2->p, &op1->p, &op1->n) > pixSize )&&
            (fabs(dot(op1->p - op2->p, op1->n)) + fabs(dot(op1->p - op2->p, op2->n)) < 2.0 * pixSize)
            //((op1->p - op2->p).size() < pixSize)
            );
}

bool isVisibleInCamera(const multiviewParams* mp, orientedPoint* op, int rc)
{
    point3d n1 = op->n.normalize();
    point3d n2 = (mp->CArr[rc] - op->p).normalize();
    return (fabs(acos(dot(n1, n2))) / (M_PI / 180.0) < 80.0);
}

bool isVisibleInCamera(const multiviewParams* mp, orientedPoint* op, int rc, int minAng)
{
    point3d n1 = op->n.normalize();
    point3d n2 = (mp->CArr[rc] - op->p).normalize();
    return ((int)(fabs(acos(dot(n1, n2))) / (M_PI / 180.0)) < minAng);
}

bool isNonVisibleInCamera(const multiviewParams* mp, orientedPoint* op, int rc)
{
    point3d n1 = op->n;
    point3d n2 = (mp->CArr[rc] - op->p).normalize();
    return (fabs(acos(dot(n1, n2))) / (M_PI / 180.0) > 150.0);
}

bool checkPair(const point3d& p, int rc, int tc, const multiviewParams* mp, float minAng, float maxAng)
{
    float ps1 = mp->getCamPixelSize(p, rc);
    float ps2 = mp->getCamPixelSize(p, tc);
    float ang = angleBetwABandAC(p, mp->CArr[rc], mp->CArr[tc]);

    return ((std::min(ps1, ps2) > std::max(ps1, ps2) * 0.8) && (ang >= minAng) && (ang <= maxAng));
}

bool checkCamPairAngle(int rc, int tc, const multiviewParams* mp, float minAng, float maxAng)
{
    if(rc == tc)
    {
        return false;
    }

    point3d rn = mp->iRArr[rc] * point3d(0.0, 0.0, 1.0);
    point3d tn = mp->iRArr[tc] * point3d(0.0, 0.0, 1.0);
    float a = angleBetwV1andV2(rn, tn);

    return ((a >= minAng) && (a <= maxAng));
}

bool isClique(int k, int* perm, unsigned char* confidenceMatrix, int n)
{
    for(int i = 0; i < k; i++)
    {
        for(int j = 0; j < k; j++)
        {
            if((i != j) && (confidenceMatrix[perm[i] * n + perm[j]] == 0))
            {
                return false;
            }
        }
    }

    return true;
}

// factorial
int myFact(int num)
{
    int result = 1;
    for(int i = 1; i <= num; ++i)
        result = result *= i;
    return result;
}

/**
 * @brief Get the RGB color from the jet colormap for the given value.
 *
 *        Return values:
 *          - 0.0f > 'value' > 1.0f: color from jet colormap
 *          - 'value' <= 0.0f: black
 *          - 'value' >= 1.0f: white
 */

// value from range 0.0 1.0
rgb getColorFromJetColorMap(float value)
{
    if(value <= 0.0f)
        return rgb(0, 0, 0);
    if(value >= 1.0f)
        return rgb(1, 1, 1);
    float idx_f = value * 63.0f;
    float fractA, fractB, integral;
    fractB = std::modf(idx_f, &integral);
    fractA = 1.0f - fractB;
    int idx = static_cast<int>(integral);
    rgb c;
    c.r = static_cast<unsigned char>((jetr[idx] * fractA + jetr[idx + 1] * fractB) * 255.0f);
    c.g = static_cast<unsigned char>((jetg[idx] * fractA + jetg[idx + 1] * fractB) * 255.0f);
    c.b = static_cast<unsigned char>((jetb[idx] * fractA + jetb[idx + 1] * fractB) * 255.0f);
    return c;
}

// hexahedron format ... 0-3 frontal face, 4-7 back face
void getHexahedronTriangles(point3d tris[12][3], point3d hexah[8])
{
    point3d a1 = hexah[0];
    point3d a2 = hexah[4];
    point3d b1 = hexah[1];
    point3d b2 = hexah[5];
    point3d c1 = hexah[2];
    point3d c2 = hexah[6];
    point3d d1 = hexah[3];
    point3d d2 = hexah[7];

    tris[0][0] = a1;
    tris[0][1] = a2;
    tris[0][2] = b1;
    tris[1][0] = b1;
    tris[1][1] = a2;
    tris[1][2] = b2;

    tris[2][0] = b1;
    tris[2][1] = b2;
    tris[2][2] = c2;
    tris[3][0] = b1;
    tris[3][1] = c2;
    tris[3][2] = c1;

    tris[4][0] = c1;
    tris[4][1] = c2;
    tris[4][2] = d2;
    tris[5][0] = c1;
    tris[5][1] = d2;
    tris[5][2] = d1;

    tris[6][0] = d1;
    tris[6][1] = d2;
    tris[6][2] = a2;
    tris[7][0] = d1;
    tris[7][1] = a2;
    tris[7][2] = a1;

    tris[8][0] = a1;
    tris[8][1] = b1;
    tris[8][2] = d1;
    tris[9][0] = b1;
    tris[9][1] = c1;
    tris[9][2] = d1;

    tris[10][0] = d2;
    tris[10][1] = b2;
    tris[10][2] = a2;
    tris[11][0] = d2;
    tris[11][1] = c2;
    tris[11][2] = b2;
}

// hexahedron format ... 0-3 frontal face, 4-7 back face
void getCamRectangleHexahedron(const multiviewParams* mp, point3d hexah[8], int cam, float mind, float maxd, point2d P[4])
{
    hexah[0] = mp->CArr[cam] + (mp->iCamArr[cam] * P[0]).normalize() * mind;
    hexah[4] = mp->CArr[cam] + (mp->iCamArr[cam] * P[0]).normalize() * maxd;
    hexah[1] = mp->CArr[cam] + (mp->iCamArr[cam] * P[1]).normalize() * mind;
    hexah[5] = mp->CArr[cam] + (mp->iCamArr[cam] * P[1]).normalize() * maxd;
    hexah[2] = mp->CArr[cam] + (mp->iCamArr[cam] * P[2]).normalize() * mind;
    hexah[6] = mp->CArr[cam] + (mp->iCamArr[cam] * P[2]).normalize() * maxd;
    hexah[3] = mp->CArr[cam] + (mp->iCamArr[cam] * P[3]).normalize() * mind;
    hexah[7] = mp->CArr[cam] + (mp->iCamArr[cam] * P[3]).normalize() * maxd;
}

// hexahedron format ... 0-3 frontal face, 4-7 back face
void getCamHexahedron(const multiviewParams* mp, point3d hexah[8], int cam, float mind, float maxd)
{
    float w = (float)mp->mip->getWidth(cam);
    float h = (float)mp->mip->getHeight(cam);
    hexah[0] = mp->CArr[cam] + (mp->iCamArr[cam] * point2d(0.0f, 0.0f)).normalize() * mind;
    hexah[4] = mp->CArr[cam] + (mp->iCamArr[cam] * point2d(0.0f, 0.0f)).normalize() * maxd;
    hexah[1] = mp->CArr[cam] + (mp->iCamArr[cam] * point2d(w, 0.0f)).normalize() * mind;
    hexah[5] = mp->CArr[cam] + (mp->iCamArr[cam] * point2d(w, 0.0f)).normalize() * maxd;
    hexah[2] = mp->CArr[cam] + (mp->iCamArr[cam] * point2d(w, h)).normalize() * mind;
    hexah[6] = mp->CArr[cam] + (mp->iCamArr[cam] * point2d(w, h)).normalize() * maxd;
    hexah[3] = mp->CArr[cam] + (mp->iCamArr[cam] * point2d(0.0f, h)).normalize() * mind;
    hexah[7] = mp->CArr[cam] + (mp->iCamArr[cam] * point2d(0.0f, h)).normalize() * maxd;
}

// hexahedron format ... 0-3 frontal face, 4-7 back face
bool intersectsHexahedronHexahedron(point3d rchex[8], point3d tchex[8])
{
    point3d rctris[12][3];
    point3d tctris[12][3];
    getHexahedronTriangles(rctris, rchex);
    getHexahedronTriangles(tctris, tchex);

    for(int t1 = 0; t1 < 12; t1++)
    {
        for(int t2 = 0; t2 < 12; t2++)
        {
            if(interectsTriangleTriangle(rctris[t1], tctris[t2]))
            {
                return true;
            }
        }
    }

    for(int i = 0; i < 8; i++)
    {
        if(isPointInHexahedron(rchex[i], tchex))
        {
            return true;
        }
        if(isPointInHexahedron(tchex[i], rchex))
        {
            return true;
        }
    }

    return false;
}

staticVector<point3d>* triangleHexahedronIntersection(point3d& A, point3d& B, point3d& C, point3d hexah[8])
{
    point3d tris[12][3];
    getHexahedronTriangles(tris, hexah);

    staticVector<point3d>* out = new staticVector<point3d>(40);
    for(int i = 0; i < 12; i++)
    {
        point3d a = tris[i][0];
        point3d b = tris[i][1];
        point3d c = tris[i][2];

        int coplanar;
        point3d i1, i2;

        bool ok = (bool)tri_tri_intersect_with_isectline(A.m, B.m, C.m, a.m, b.m, c.m, &coplanar, i1.m, i2.m);
        if(ok)
        {
            out->push_back(i1);
            out->push_back(i2);
        }
    }

    return out;
}

staticVector<point3d>* lineSegmentHexahedronIntersection(point3d& linePoint1, point3d& linePoint2, point3d hexah[8])
{
    point3d tris[12][3];
    getHexahedronTriangles(tris, hexah);

    staticVector<point3d>* out = new staticVector<point3d>(40);
    for(int i = 0; i < 12; i++)
    {
        point3d a = tris[i][0];
        point3d b = tris[i][1];
        point3d c = tris[i][2];
        point3d lpi;

        if(isLineSegmentInTriangle(lpi, a, b, c, linePoint1, linePoint2))
        {
            out->push_back(lpi);
        }
    }

    return out;
}

staticVector<point3d>* triangleRectangleIntersection(point3d& A, point3d& B, point3d& C, const multiviewParams* mp, int rc,
                                                     point2d P[4])
{
    float maxd =
        std::max(std::max((mp->CArr[rc] - A).size(), (mp->CArr[rc] - B).size()), (mp->CArr[rc] - C).size()) * 1000.0f;

    staticVector<point3d>* out = new staticVector<point3d>(40);

    point3d a, b, c;
    int coplanar;
    point3d i1, i2;

    a = mp->CArr[rc];
    b = mp->CArr[rc] + (mp->iCamArr[rc] * P[0]).normalize() * maxd;
    c = mp->CArr[rc] + (mp->iCamArr[rc] * P[1]).normalize() * maxd;
    bool ok = (bool)tri_tri_intersect_with_isectline(A.m, B.m, C.m, a.m, b.m, c.m, &coplanar, i1.m, i2.m);
    if(ok)
    {
        out->push_back(i1);
        out->push_back(i2);
    }

    a = mp->CArr[rc];
    b = mp->CArr[rc] + (mp->iCamArr[rc] * P[1]).normalize() * maxd;
    c = mp->CArr[rc] + (mp->iCamArr[rc] * P[2]).normalize() * maxd;
    ok = (bool)tri_tri_intersect_with_isectline(A.m, B.m, C.m, a.m, b.m, c.m, &coplanar, i1.m, i2.m);
    if(ok)
    {
        out->push_back(i1);
        out->push_back(i2);
    }

    a = mp->CArr[rc];
    b = mp->CArr[rc] + (mp->iCamArr[rc] * P[2]).normalize() * maxd;
    c = mp->CArr[rc] + (mp->iCamArr[rc] * P[3]).normalize() * maxd;
    ok = (bool)tri_tri_intersect_with_isectline(A.m, B.m, C.m, a.m, b.m, c.m, &coplanar, i1.m, i2.m);
    if(ok)
    {
        out->push_back(i1);
        out->push_back(i2);
    }

    a = mp->CArr[rc];
    b = mp->CArr[rc] + (mp->iCamArr[rc] * P[3]).normalize() * maxd;
    c = mp->CArr[rc] + (mp->iCamArr[rc] * P[0]).normalize() * maxd;
    ok = (bool)tri_tri_intersect_with_isectline(A.m, B.m, C.m, a.m, b.m, c.m, &coplanar, i1.m, i2.m);
    if(ok)
    {
        out->push_back(i1);
        out->push_back(i2);
    }

    // point3d lp;
    // if lineSegmentPlaneIntersect(&lp,A,B,mp->CArr[rc],n);

    return out;
}

bool isPointInHexahedron(const point3d& p, const point3d* hexah)
{
    point3d a = hexah[0];
    point3d b = hexah[1];
    point3d c = hexah[3];
    point3d d = hexah[4];
    point3d n = cross(a - b, b - c).normalize();
    float d1 = orientedPointPlaneDistance(p, a, n);
    float d2 = orientedPointPlaneDistance(d, a, n);
    if(d1 * d2 < 0.0)
        return false;

    a = hexah[0];
    b = hexah[1];
    c = hexah[4];
    d = hexah[3];
    n = cross(a - b, b - c).normalize();
    d1 = orientedPointPlaneDistance(p, a, n);
    d2 = orientedPointPlaneDistance(d, a, n);
    if(d1 * d2 < 0.0)
        return false;

    a = hexah[1];
    b = hexah[2];
    c = hexah[5];
    d = hexah[0];
    n = cross(a - b, b - c).normalize();
    d1 = orientedPointPlaneDistance(p, a, n);
    d2 = orientedPointPlaneDistance(d, a, n);
    if(d1 * d2 < 0.0)
        return false;

    a = hexah[2];
    b = hexah[6];
    c = hexah[3];
    d = hexah[1];
    n = cross(a - b, b - c).normalize();
    d1 = orientedPointPlaneDistance(p, a, n);
    d2 = orientedPointPlaneDistance(d, a, n);
    if(d1 * d2 < 0.0)
        return false;

    a = hexah[0];
    b = hexah[4];
    c = hexah[3];
    d = hexah[1];
    n = cross(a - b, b - c).normalize();
    d1 = orientedPointPlaneDistance(p, a, n);
    d2 = orientedPointPlaneDistance(d, a, n);
    if(d1 * d2 < 0.0)
        return false;

    a = hexah[4];
    b = hexah[5];
    c = hexah[7];
    d = hexah[0];
    n = cross(a - b, b - c).normalize();
    d1 = orientedPointPlaneDistance(p, a, n);
    d2 = orientedPointPlaneDistance(d, a, n);
    return d1 * d2 >= 0.0;
}

void inflateHexahedron(point3d hexahIn[8], point3d hexahOut[8], float scale)
{
    point3d cg = point3d(0.0f, 0.0f, 0.0f);
    for(int i = 0; i < 8; i++)
    {
        cg = cg + hexahIn[i];
    }
    cg = cg / 8.0f;

    for(int i = 0; i < 8; i++)
    {
        hexahOut[i] = cg + (hexahIn[i] - cg) * scale;
    }
}

void inflateHexahedronInDim(int dim, point3d hexahIn[8], point3d hexahOut[8], float scale)
{
    /*
    the hexahedron was created :
    float x = (float)xp*stepx;
    float y = (float)yp*stepy;
    float z = (float)zp*stepz;

    (*voxels)[id*8+0] = ox + vx*x +        vy*y +        vz*z;
    (*voxels)[id*8+1] = ox + vx*(x+stepx)+ vy*y +        vz*z;
    (*voxels)[id*8+2] = ox + vx*(x+stepx)+ vy*(y+stepy)+ vz*z;
    (*voxels)[id*8+3] = ox + vx*x +        vy*(y+stepy)+ vz*z;
    (*voxels)[id*8+4] = ox + vx*x +        vy*y +        vz*(z+stepz);
    (*voxels)[id*8+5] = ox + vx*(x+stepx)+ vy*y +        vz*(z+stepz);
    (*voxels)[id*8+6] = ox + vx*(x+stepx)+ vy*(y+stepy)+ vz*(z+stepz);
    (*voxels)[id*8+7] = ox + vx*x +        vy*(y+stepy)+ vz*(z+stepz);
    */

    if(dim == 0)
    {
        hexahOut[0] = hexahIn[0];
        hexahOut[1] = hexahIn[0] + (hexahIn[1] - hexahIn[0]) * scale;
        hexahOut[2] = hexahIn[3] + (hexahIn[2] - hexahIn[3]) * scale;
        hexahOut[3] = hexahIn[3];
        hexahOut[4] = hexahIn[4];
        hexahOut[5] = hexahIn[4] + (hexahIn[5] - hexahIn[4]) * scale;
        hexahOut[6] = hexahIn[7] + (hexahIn[6] - hexahIn[7]) * scale;
        hexahOut[7] = hexahIn[7];
    }

    if(dim == 1)
    {
        hexahOut[0] = hexahIn[0];
        hexahOut[1] = hexahIn[1];
        hexahOut[2] = hexahIn[1] + (hexahIn[2] - hexahIn[1]) * scale;
        hexahOut[3] = hexahIn[0] + (hexahIn[3] - hexahIn[0]) * scale;
        hexahOut[4] = hexahIn[4];
        hexahOut[5] = hexahIn[5];
        hexahOut[6] = hexahIn[5] + (hexahIn[6] - hexahIn[5]) * scale;
        hexahOut[7] = hexahIn[4] + (hexahIn[7] - hexahIn[4]) * scale;
    }

    if(dim == 2)
    {
        hexahOut[0] = hexahIn[0];
        hexahOut[1] = hexahIn[1];
        hexahOut[2] = hexahIn[2];
        hexahOut[3] = hexahIn[3];
        hexahOut[4] = hexahIn[0] + (hexahIn[4] - hexahIn[0]) * scale;
        hexahOut[5] = hexahIn[1] + (hexahIn[5] - hexahIn[1]) * scale;
        hexahOut[6] = hexahIn[2] + (hexahIn[6] - hexahIn[2]) * scale;
        hexahOut[7] = hexahIn[3] + (hexahIn[7] - hexahIn[3]) * scale;
    }
}

void inflateHexahedronAroundDim(int dim, point3d hexahIn[8], point3d hexahOut[8], float scale)
{
    int ids[8];
    if(dim == 0)
    {
        ids[0] = 0;
        ids[1] = 4;
        ids[2] = 7;
        ids[3] = 3;
        ids[4] = 1;
        ids[5] = 5;
        ids[6] = 6;
        ids[7] = 2;
    }
    if(dim == 1)
    {
        ids[0] = 0;
        ids[1] = 1;
        ids[2] = 5;
        ids[3] = 4;
        ids[4] = 3;
        ids[5] = 2;
        ids[6] = 6;
        ids[7] = 7;
    }
    if(dim == 2)
    {
        ids[0] = 0;
        ids[1] = 1;
        ids[2] = 2;
        ids[3] = 3;
        ids[4] = 4;
        ids[5] = 5;
        ids[6] = 6;
        ids[7] = 7;
    }

    for(int k = 0; k < 2; k++)
    {
        point3d cg = point3d(0.0f, 0.0f, 0.0f);
        for(int i = 0; i < 4; i++)
        {
            cg = cg + hexahIn[ids[k * 4 + i]];
        }
        cg = cg / 4.0f;
        for(int i = 0; i < 4; i++)
        {
            hexahOut[ids[k * 4 + i]] = cg + (hexahIn[ids[k * 4 + i]] - cg) * scale;
        }
    }
}

/* //matlab code to simulate kernel voting of array of similarities

close all
clear all
clc

a=1.0;
c=0.1;
x=-1.0:0.01:1.0;

sims = [-0.8 -0.83 -0.82 -0.6 -0.4 -0.2];
%sims = [-0.8 -0.6 -0.4 -0.2];

y = zeros(size(x,2));
for i=1:size(x,2)
        y(i) = sum(a*exp(-(x(i)-sims).^2/(2*c^2)));
end


figure
hold on

for i=1:size(sims,2)
        plot(x,a*exp(-(x-sims(i)).^2/(2*c^2)));
    plot([sims(i) sims(i)],[max(y(:)) 0],'r-');
end


plot(x,y)

*/
float similarityKernelVoting(staticVector<float>* sims)
{
    float a = 1.0f;
    float c = 0.1f;

    float maxy = 0.0f;
    float maxx = 1.0f;
    for(float x = -1.0f; x < 1.0f; x = x + 0.01f)
    {
        float y = 0;
        for(int i = 0; i < sims->size(); i++)
        {
            float sim = (*sims)[i];
            y = y + a * exp(-((x - sim) * (x - sim)) / (2.0f * c * c));
        }
        if(y > maxy)
        {
            maxy = y;
            maxx = x;
        }
    }

    return maxx;
}

bool checkPoint3d(point3d n)
{
    return !((n.x != n.x) || (n.y != n.y) || (n.z != n.z) || std::isnan(n.x) || std::isnan(n.y) || std::isnan(n.z));
}


staticVector<int>* getDistinctIndexes(staticVector<int>* indexes)
{
    qsort(&(*indexes)[0], indexes->size(), sizeof(int), qSortCompareIntAsc);
    staticVector<int>* out = new staticVector<int>(indexes->size());
    int i = 0;
    while(i < indexes->size())
    {
        if((i == indexes->size() - 1) || ((*indexes)[i] != (*indexes)[i + 1]))
        {
            out->push_back((*indexes)[i]);
        }
        i++;
    }
    return out;
}

staticVector<staticVector<int>*>* convertObjectsCamsToCamsObjects(const multiviewParams* mp,
                                                                  staticVector<staticVector<int>*>* ptsCams)
{
    staticVector<int>* nCamsPts = new staticVector<int>(mp->ncams);
    nCamsPts->resize_with(mp->ncams, 0);
    for(int i = 0; i < ptsCams->size(); i++)
    {
        for(int j = 0; j < sizeOfStaticVector<int>((*ptsCams)[i]); j++)
        {
            int rc = (*(*ptsCams)[i])[j];
            if((rc >= 0) && (rc < mp->ncams))
            {
                (*nCamsPts)[rc]++;
            }
            else
            {
                printf("WARNING convertObjectsCamsToCamsObjects %i \n", rc);
            }
        }
    }

    staticVector<staticVector<int>*>* camsPts = new staticVector<staticVector<int>*>(mp->ncams);
    for(int rc = 0; rc < mp->ncams; rc++)
    {
        camsPts->push_back(new staticVector<int>((*nCamsPts)[rc]));
    }

    for(int i = 0; i < ptsCams->size(); i++)
    {
        for(int j = 0; j < sizeOfStaticVector<int>((*ptsCams)[i]); j++)
        {
            int rc = (*(*ptsCams)[i])[j];
            if((rc >= 0) && (rc < mp->ncams))
            {
                (*camsPts)[rc]->push_back(i);
            }
        }
    }

    return camsPts;
}

staticVector<staticVector<pixel>*>* convertObjectsCamsToCamsObjects(const multiviewParams* mp,
                                                                    staticVector<staticVector<pixel>*>* ptsCams)
{
    staticVector<int>* nCamsPts = new staticVector<int>(mp->ncams);
    nCamsPts->resize_with(mp->ncams, 0);
    for(int i = 0; i < ptsCams->size(); i++)
    {
        for(int j = 0; j < sizeOfStaticVector<pixel>((*ptsCams)[i]); j++)
        {
            int rc = (*(*ptsCams)[i])[j].x;
            (*nCamsPts)[rc]++;
        }
    }

    staticVector<staticVector<pixel>*>* camsPts = new staticVector<staticVector<pixel>*>(mp->ncams);
    for(int rc = 0; rc < mp->ncams; rc++)
    {
        camsPts->push_back(new staticVector<pixel>((*nCamsPts)[rc]));
    }

    for(int i = 0; i < ptsCams->size(); i++)
    {
        for(int j = 0; j < sizeOfStaticVector<pixel>((*ptsCams)[i]); j++)
        {
            int rc = (*(*ptsCams)[i])[j].x;
            int value = (*(*ptsCams)[i])[j].y;
            (*camsPts)[rc]->push_back(pixel(i, value));
        }
    }

    return camsPts;
}

int computeStep(multiviewInputParams* mip, int scale, int maxWidth, int maxHeight)
{
    int step = 1;
    int ow = mip->getMaxImageWidth() / scale;
    int oh = mip->getMaxImageHeight() / scale;
    int g_Width = mip->getMaxImageWidth() / scale;
    int g_Height = mip->getMaxImageHeight() / scale;
    while((g_Width > maxWidth) || (g_Height > maxHeight))
    {
        step++;
        g_Width = ow / step;
        g_Height = oh / step;
    }
    return step;
}

void showImageOpenCV(unsigned char* data, int w, int h, float minVal, float maxVal, int scaleFactor)
{
    IplImage* img = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 3);
    for(int y = 0; y < h; y++)
    {
        for(int x = 0; x < w; x++)
        {
            float val = (float)data[x * h + y];
            float s = 1.0f - (maxVal - std::max(minVal, val)) / (maxVal - minVal);
            rgb cc = getColorFromJetColorMap(s);
            CvScalar c;
            c.val[0] = (float)cc.r;
            c.val[1] = (float)cc.g;
            c.val[2] = (float)cc.b;
            cvSet2D(img, y, x, c);
        }
    }

    IplImage* imgr = cvCreateImage(cvSize(w / scaleFactor, h / scaleFactor), IPL_DEPTH_8U, 3);
    cvResize(img, imgr);

    cvShowImage("showImageOpenCV", imgr);
    cvWaitKey();
    cvReleaseImage(&img);
    cvReleaseImage(&imgr);
}

/*
void showImageOpenCVT(unsigned char *data, int w, int h, float minVal, float maxVal, int scaleFactor)
{
        float minV = 255.0f;
        float maxV = 0.0f;

        IplImage* img = cvCreateImage(cvSize(w,h), IPL_DEPTH_8U, 3);
        for (int y = 0; y < h; y++) {
                for (int x = 0; x < w; x++) {
                        float val = (float)data[y*w+x];
                        minV = std::min(minV,val);
                        maxV = std::max(maxV,val);
                        float s = 1.0f-(maxVal - std::max(minVal,val))/(maxVal-minVal);
                        rgb cc = getColorFromJetColorMap(s);
                        CvScalar c;
                        c.val[0] = (float)cc.r;
                        c.val[1] = (float)cc.g;
                        c.val[2] = (float)cc.b;
                        cvSet2D(img,y,x,c);
                };
        };

        printf("minV %f, maxV %f \n", minV, maxV);

        IplImage* imgr=cvCreateImage(cvSize(w/scaleFactor,h/scaleFactor),IPL_DEPTH_8U,3);
        cvResize(img,imgr);

        cvShowImage("showImageOpenCV", imgr);
        cvWaitKey();
        cvReleaseImage(&img);
        cvReleaseImage(&imgr);
}
*/
void showImageOpenCV(float* data, int w, int h, float minVal, float maxVal, int scaleFactor)
{
    IplImage* img = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 3);
    for(int y = 0; y < h; y++)
    {
        for(int x = 0; x < w; x++)
        {
            float val = data[x * h + y];
            float s = 1.0f - (maxVal - std::max(minVal, val)) / (maxVal - minVal);
            rgb cc = getColorFromJetColorMap(s);
            CvScalar c;
            c.val[0] = (float)cc.r;
            c.val[1] = (float)cc.g;
            c.val[2] = (float)cc.b;
            cvSet2D(img, y, x, c);
        }
    }

    IplImage* imgr = cvCreateImage(cvSize(w / scaleFactor, h / scaleFactor), IPL_DEPTH_8U, 3);
    cvResize(img, imgr);

    cvShowImage("showImageOpenCV", imgr);
    cvWaitKey();
    cvReleaseImage(&img);
    cvReleaseImage(&imgr);
}

void showImageOpenCVT(double* data, int w, int h, float minVal, float maxVal, int scaleFactor)
{
    IplImage* img = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 3);
    for(int y = 0; y < h; y++)
    {
        for(int x = 0; x < w; x++)
        {
            float val = data[y * w + x];
            float s = 1.0f - (maxVal - std::max(minVal, val)) / (maxVal - minVal);
            rgb cc = getColorFromJetColorMap(s);
            CvScalar c;
            c.val[0] = (float)cc.r;
            c.val[1] = (float)cc.g;
            c.val[2] = (float)cc.b;
            cvSet2D(img, y, x, c);
        }
    }

    IplImage* imgr = cvCreateImage(cvSize(w / scaleFactor, h / scaleFactor), IPL_DEPTH_8U, 3);
    cvResize(img, imgr);

    cvShowImage("Orig img double", imgr);
    cvWaitKey();
    cvReleaseImage(&img);
    cvReleaseImage(&imgr);
}

void showImageOpenCVT(float* data, int w, int h, float minVal, float maxVal, int scaleFactor, int delay)
{
    IplImage* img = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 3);
    for(int y = 0; y < h; y++)
    {
        for(int x = 0; x < w; x++)
        {
            float val = (float)data[y * w + x];
            float s = 1.0f - ((float)maxVal - std::max((float)minVal, val)) / ((float)maxVal - (float)minVal);
            rgb cc = getColorFromJetColorMap(s);
            CvScalar c;
            c.val[0] = (float)cc.r;
            c.val[1] = (float)cc.g;
            c.val[2] = (float)cc.b;
            cvSet2D(img, y, x, c);
        }
    }

    IplImage* imgr = cvCreateImage(cvSize(w / scaleFactor, h / scaleFactor), IPL_DEPTH_8U, 3);
    cvResize(img, imgr);

    cvShowImage("showImageOpenCVT", imgr);
    cvWaitKey(delay);
    cvReleaseImage(&img);
    cvReleaseImage(&imgr);
    cvDestroyWindow("showImageOpenCVT");
}

void showImageOpenCVT(unsigned char* data, int w, int h, unsigned char minVal, unsigned char maxVal, int scaleFactor,
                      int delay)
{
    IplImage* img = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 3);
    for(int y = 0; y < h; y++)
    {
        for(int x = 0; x < w; x++)
        {
            float val = (float)data[y * w + x];
            float s = 1.0f - ((float)maxVal - std::max((float)minVal, val)) / ((float)maxVal - (float)minVal);
            rgb cc = getColorFromJetColorMap(s);
            CvScalar c;
            c.val[0] = (float)cc.r;
            c.val[1] = (float)cc.g;
            c.val[2] = (float)cc.b;
            cvSet2D(img, y, x, c);
        }
    }

    IplImage* imgr = cvCreateImage(cvSize(w / scaleFactor, h / scaleFactor), IPL_DEPTH_8U, 3);
    cvResize(img, imgr);

    cvShowImage("showImageOpenCVT", imgr);
    cvWaitKey(delay);
    cvReleaseImage(&img);
    cvReleaseImage(&imgr);
    // cvDestroyWindow("showImageOpenCVT");
}

void showImageOpenCVT(int* data, int w, int h, int minVal, int maxVal, int scaleFactor)
{
    IplImage* img = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 3);
    for(int y = 0; y < h; y++)
    {
        for(int x = 0; x < w; x++)
        {
            float val = (float)data[y * w + x];
            float s = 1.0f - ((float)maxVal - std::max((float)minVal, val)) / ((float)maxVal - (float)minVal);
            rgb cc = getColorFromJetColorMap(s);
            CvScalar c;
            c.val[0] = (float)cc.r;
            c.val[1] = (float)cc.g;
            c.val[2] = (float)cc.b;
            cvSet2D(img, y, x, c);
        }
    }
    IplImage* imgr = cvCreateImage(cvSize(w / scaleFactor, h / scaleFactor), IPL_DEPTH_8U, 3);
    cvResize(img, imgr);

    cvShowImage("showImageOpenCVT", imgr);
    cvWaitKey();
    cvReleaseImage(&img);
    cvReleaseImage(&imgr);
    cvDestroyWindow("showImageOpenCVT");
}

staticVector<point3d>* computeVoxels(const point3d* space, const voxel& dimensions)
{
    float voxelDimX = (float)dimensions.x;
    float voxelDimY = (float)dimensions.y;
    float voxelDimZ = (float)dimensions.z;

    point3d ox = space[0];
    point3d vx = (space[1] - space[0]).normalize();
    point3d vy = (space[3] - space[0]).normalize();
    point3d vz = (space[4] - space[0]).normalize();
    float sx = (space[1] - space[0]).size();
    float sy = (space[3] - space[0]).size();
    float sz = (space[4] - space[0]).size();
    float stepx = sx / voxelDimX;
    float stepy = sy / voxelDimY;
    float stepz = sz / voxelDimZ;

    int nvoxels = dimensions.x * dimensions.y * dimensions.z;
    staticVector<point3d>* voxels = new staticVector<point3d>(nvoxels * 8);
    voxels->resize(nvoxels * 8);

    // printf("%i %i %i %i\n",dimensions.x,dimensions.y,dimensions.z,nvoxels,voxels->size());

    int id = 0;
    for(int xp = 0; xp < dimensions.x; xp++)
    {
        for(int yp = 0; yp < dimensions.y; yp++)
        {
            for(int zp = 0; zp < dimensions.z; zp++)
            {
                float x = (float)xp * stepx;
                float y = (float)yp * stepy;
                float z = (float)zp * stepz;
                (*voxels)[id * 8 + 0] = ox + vx * x + vy * y + vz * z;                      // x,   y,   z
                (*voxels)[id * 8 + 1] = ox + vx * (x + stepx) + vy * y + vz * z;            // x+1, y,   z
                (*voxels)[id * 8 + 2] = ox + vx * (x + stepx) + vy * (y + stepy) + vz * z;  // x+1, y+1, z
                (*voxels)[id * 8 + 3] = ox + vx * x + vy * (y + stepy) + vz * z;            // x,   y+1, z
                (*voxels)[id * 8 + 4] = ox + vx * x + vy * y + vz * (z + stepz);            // x,   y,   z+1
                (*voxels)[id * 8 + 5] = ox + vx * (x + stepx) + vy * y + vz * (z + stepz);  // x+1, y,   z+1
                (*voxels)[id * 8 + 6] = ox + vx * (x + stepx) + vy * (y + stepy) + vz * (z + stepz); // x+1, y+1, z+1
                (*voxels)[id * 8 + 7] = ox + vx * x + vy * (y + stepy) + vz * (z + stepz);  // x,   y+1, z+1
                id++;
            }
        }
    }

    return voxels;
}

staticVector<int>* createRandomArrayOfIntegers(int n)
{
    /* initialize random seed: */
    srand(time(nullptr));

    staticVector<int>* tracksPointsRandomIds = new staticVector<int>(n);
    for(int j = 0; j < n; j++)
    {
        tracksPointsRandomIds->push_back(j);
    }

    for(int j = 0; j < n - 1; j++)
    {
        int rid = rand() % (n - j);

        /*
        if ((j+rid<0)||(j+rid>=tracksPoints->size())) {
                printf("WANRING rid ot of limits %i, 0 to %i !!!! \n",j+rid,tracksPoints->size());
        };
        */

        int v = (*tracksPointsRandomIds)[j + rid];
        (*tracksPointsRandomIds)[j + rid] = (*tracksPointsRandomIds)[j];
        (*tracksPointsRandomIds)[j] = v;
    }

    // test
    /*
    {
            staticVectorBool *tracksPointsRandomIdsB = new staticVectorBool(n);
            tracksPointsRandomIdsB->resize_with(n,false);
            for (int k=0;k<n;k++) {
                    int j = (*tracksPointsRandomIds)[k];
                    (*tracksPointsRandomIdsB)[j] = true;
            };


            for (int j=0;j<n;j++) {
                    if ((*tracksPointsRandomIdsB)[j]==false) {
                            printf("WANRING  ((*tracksPointsRandomIdsB)[j]==false) !!!! \n");
                    };
            };


            delete tracksPointsRandomIdsB;
    };
    */

    return tracksPointsRandomIds;
}

float getCGDepthFromSeeds(const multiviewParams* mp, int rc)
{
    staticVector<seedPoint>* seeds;
    loadSeedsFromFile(&seeds, mp->indexes[rc], mp->mip, mp->mip->MV_FILE_TYPE_seeds);

    float midDepth = -1.0f;

    if(seeds->size() > 20)
    {
        orientedPoint rcplane;
        rcplane.p = mp->CArr[rc];
        rcplane.n = mp->iRArr[rc] * point3d(0.0, 0.0, 1.0);
        rcplane.n = rcplane.n.normalize();

        point3d cg = point3d(0.0f, 0.0f, 0.0f);
        for(int i = 0; i < seeds->size(); i++)
        {
            seedPoint* sp = &(*seeds)[i];
            cg = cg + sp->op.p;
        }
        cg = cg / (float)seeds->size();
        midDepth = pointPlaneDistance(cg, rcplane.p, rcplane.n);
    }

    delete seeds;

    return midDepth;
}

float sigmoidfcn(float zeroVal, float endVal, float sigwidth, float sigMid, float xval)
{
    return zeroVal + (endVal - zeroVal) * (1.0f / (1.0f + exp(10.0f * ((xval - sigMid) / sigwidth))));
}

float sigmoid2fcn(float zeroVal, float endVal, float sigwidth, float sigMid, float xval)
{
    return zeroVal + (endVal - zeroVal) * (1.0f / (1.0f + exp(10.0f * ((sigMid - xval) / sigwidth))));
}

int findNSubstrsInString(const std::string& str, const std::string& val)
{
    int last = 0;
    int n = 0;
    int pos = str.find(val, last);
    while(pos > -1)
    {
        n++;
        last = pos + val.length();
        pos = str.find(val, last);
    }
    return n;
}

std::string num2str(int num)
{
    std::stringstream out;
    out << num;
    return out.str();
}

std::string num2str(float num)
{
    std::stringstream out;
    out << num;
    return out.str();
}

std::string num2str(int64_t num)
{
    std::stringstream out;
    out << num;
    return out.str();
}

std::string num2strThreeDigits(int index)
{
    std::string ms;

    if(index < 10)
    {
        ms = "00" + num2str(index);
    }
    else
    {
        if(index < 100)
        {
            ms = "0" + num2str(index);
        }
        else
        {
            ms = num2str(index);
        }
    }

    return ms;
}

std::string num2strFourDecimal(int index)
{
    std::string ms;

    char tmp[50];
    sprintf(tmp, "%05i", index);
    std::string s = tmp;

    return s;
}

std::string num2strTwoDecimal(int index)
{
    std::string ms;
    if(index < 10)
    {
        ms = "0" + num2str(index);
    }
    else
    {
        if(index < 100)
        {
            ms = num2str(index);
        }
    }

    return ms;
}
