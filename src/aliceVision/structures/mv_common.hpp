#pragma once

#include "mv_multiview_params.hpp"
#include <string>


bool get2dLineImageIntersection(point2d* pFrom, point2d* pTo, point2d linePoint1, point2d linePoint2,
                                const multiviewParams* mp, int camId);
bool getTarEpipolarDirectedLine(point2d* pFromTar, point2d* pToTar, point2d refpix, int refCam, int tarCam,
                                const multiviewParams* mp);
bool triangulateMatch(point3d& out, const point2d& refpix, const point2d& tarpix, int refCam, int tarCam,
                      const multiviewParams* mp);
bool triangulateMatchLeft(point3d& out, const point2d& refpix, const point2d& tarpix, int refCam, int tarCam,
                          const multiviewParams* mp);
void printfPercent(int i, int n);
long initEstimate();
void printfEstimate(int i, int n, long startTime);
void finishEstimate();
std::string printfElapsedTime(long t1, std::string prefix = "");
void ransac_rsample(int* indexes, int npoints, int npoinsRansac);
// SampleCnt calculates number of samples needed to be done
int ransac_nsamples(int ni, int npoints, int npoinsRansac, float conf);
bool ransacPlaneFit(orientedPoint& plane, staticVector<point3d>* points, staticVector<point3d>* points_samples,
                    const multiviewParams* mp, int rc, float pixEpsThr);
bool multimodalRansacPlaneFit(orientedPoint& plane, staticVector<staticVector<point3d>*>* modalPoints,
                              const multiviewParams* mp, int rc, float pixEpsThr);
float gaussKernelEnergy(orientedPoint* pt, staticVector<orientedPoint*>* pts, float sigma);
int gaussKernelVoting(staticVector<orientedPoint*>* pts, float sigma);
float angularDistnace(orientedPoint* op1, orientedPoint* op2);
bool arecoincident(orientedPoint* op1, orientedPoint* op2, float pixSize);
bool isVisibleInCamera(const multiviewParams* mp, orientedPoint* op, int rc);
bool isNonVisibleInCamera(const multiviewParams* mp, orientedPoint* op, int rc);
bool checkPair(const point3d& p, int rc, int tc, const multiviewParams* mp, float minAng, float maxAng);
bool checkCamPairAngle(int rc, int tc, const multiviewParams* mp, float minAng, float maxAng);
bool isClique(int k, int* perm, unsigned char* confidenceMatrix, int n);
// factorial
int myFact(int num);
rgb getColorFromJetColorMap(float value);

void getHexahedronTriangles(point3d tris[12][3], point3d hexah[8]);
void getCamRectangleHexahedron(const multiviewParams* mp, point3d hexah[8], int cam, float mind, float maxd, point2d P[4]);
void getCamHexahedron(const multiviewParams* mp, point3d hexah[8], int cam, float mind, float maxd);
bool intersectsHexahedronHexahedron(point3d rchex[8], point3d tchex[8]);
staticVector<point3d>* lineSegmentHexahedronIntersection(point3d& linePoint1, point3d& linePoint2, point3d hexah[8]);
staticVector<point3d>* triangleHexahedronIntersection(point3d& A, point3d& B, point3d& C, point3d hexah[8]);
staticVector<point3d>* triangleRectangleIntersection(point3d& A, point3d& B, point3d& C, const multiviewParams* mp, int rc,
                                                     point2d P[4]);
bool isPointInHexahedron(const point3d &p, const point3d *hexah);
void inflateHexahedron(point3d hexahIn[8], point3d hexahOut[8], float scale);
void inflateHexahedronInDim(int dim, point3d hexahIn[8], point3d hexahOut[8], float scale);
void inflateHexahedronAroundDim(int dim, point3d hexahIn[8], point3d hexahOut[8], float scale);
float similarityKernelVoting(staticVector<float>* sims);
bool checkPoint3d(point3d n);
bool checkPoint2d(point2d n);
staticVector<int>* getDistinctIndexes(staticVector<int>* indexes);
staticVector<staticVector<int>*>* convertObjectsCamsToCamsObjects(const multiviewParams* mp,
                                                                  staticVector<staticVector<int>*>* ptsCams);
staticVector<staticVector<pixel>*>* convertObjectsCamsToCamsObjects(const multiviewParams* mp,
                                                                    staticVector<staticVector<pixel>*>* ptsCams);
int computeStep(multiviewInputParams* mip, int scale, int maxWidth, int maxHeight);
void showImageOpenCV(unsigned char* data, int w, int h, float minVal, float maxVal, int scaleFactor = 1);
void showImageOpenCV(float* data, int w, int h, float minVal, float maxVal, int scaleFactor = 1);
void showImageOpenCVT(double* data, int w, int h, float minVal, float maxVal, int scaleFactor = 1);
void showImageOpenCVT(float* data, int w, int h, float minVal, float maxVal, int scaleFactor = 1, int delay = 0);
void showImageOpenCVT(unsigned char* data, int w, int h, unsigned char minVal, unsigned char maxVal,
                      int scaleFactor = 1, int delay = 0);
void showImageOpenCVT(int* data, int w, int h, int minVal, int maxVal, int scaleFactor = 1);
staticVector<point3d>* computeVoxels(const point3d* space, const voxel& dimensions);
float getCGDepthFromSeeds(const multiviewParams* mp, int rc); // TODO: require seeds vector as input param
staticVector<int>* createRandomArrayOfIntegers(int n);
float sigmoidfcn(float zeroVal, float endVal, float sigwidth, float sigMid, float xval);
float sigmoid2fcn(float zeroVal, float endVal, float sigwidth, float sigMid, float xval);


int findNSubstrsInString(const std::string& str, const std::string& val);
std::string num2str(int num);
std::string num2str(float num);
std::string num2str(int64_t num);
std::string num2strThreeDigits(int index);
std::string num2strFourDecimal(int index);
std::string num2strTwoDecimal(int index);
