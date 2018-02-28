// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/structures/OrientedPoint.hpp>
#include <aliceVision/structures/Point2d.hpp>
#include <aliceVision/structures/Point3d.hpp>
#include <aliceVision/structures/Pixel.hpp>
#include <aliceVision/structures/StaticVector.hpp>
#include <aliceVision/structures/Voxel.hpp>
#include <aliceVision/mvsUtils/MultiViewParams.hpp>

#include <string>

namespace aliceVision {
namespace mvsUtils {

bool get2dLineImageIntersection(Point2d* pFrom, Point2d* pTo, Point2d linePoint1, Point2d linePoint2,
                                const MultiViewParams* mp, int camId);
bool getTarEpipolarDirectedLine(Point2d* pFromTar, Point2d* pToTar, Point2d refpix, int refCam, int tarCam,
                                const MultiViewParams* mp);
bool triangulateMatch(Point3d& out, const Point2d& refpix, const Point2d& tarpix, int refCam, int tarCam,
                      const MultiViewParams* mp);
bool triangulateMatchLeft(Point3d& out, const Point2d& refpix, const Point2d& tarpix, int refCam, int tarCam,
                          const MultiViewParams* mp);
void printfPercent(int i, int n);
long initEstimate();
void printfEstimate(int i, int n, long startTime);
void finishEstimate();
std::string printfElapsedTime(long t1, std::string prefix = "");
// SampleCnt calculates number of samples needed to be done
int gaussKernelVoting(StaticVector<OrientedPoint*>* pts, float sigma);
float angularDistnace(OrientedPoint* op1, OrientedPoint* op2);
bool arecoincident(OrientedPoint* op1, OrientedPoint* op2, float pixSize);
bool checkPair(const Point3d& p, int rc, int tc, const MultiViewParams* mp, float minAng, float maxAng);
bool checkCamPairAngle(int rc, int tc, const MultiViewParams* mp, float minAng, float maxAng);
void getHexahedronTriangles(Point3d tris[12][3], const Point3d hexah[8]);
void getCamHexahedron(const MultiViewParams* mp, Point3d hexah[8], int cam, float mind, float maxd);
bool intersectsHexahedronHexahedron(const Point3d rchex[8], const Point3d tchex[8]);
StaticVector<Point3d>* lineSegmentHexahedronIntersection(Point3d& linePoint1, Point3d& linePoint2, Point3d hexah[8]);
StaticVector<Point3d>* triangleHexahedronIntersection(Point3d& A, Point3d& B, Point3d& C, Point3d hexah[8]);
StaticVector<Point3d>* triangleRectangleIntersection(Point3d& A, Point3d& B, Point3d& C, const MultiViewParams* mp, int rc,
                                                     Point2d P[4]);
bool isPointInHexahedron(const Point3d &p, const Point3d *hexah);
void inflateHexahedron(const Point3d hexahIn[8], Point3d hexahOut[8], float scale);
bool checkPoint3d(Point3d n);
bool checkPoint2d(Point2d n);

StaticVector<StaticVector<int>*>* convertObjectsCamsToCamsObjects(const MultiViewParams* mp,
                                                                  StaticVector<StaticVector<int>*>* ptsCams);
StaticVector<StaticVector<Pixel>*>* convertObjectsCamsToCamsObjects(const MultiViewParams* mp,
                                                                    StaticVector<StaticVector<Pixel>*>* ptsCams);
int computeStep(MultiViewInputParams* mip, int scale, int maxWidth, int maxHeight);

StaticVector<Point3d>* computeVoxels(const Point3d* space, const Voxel& dimensions);
float getCGDepthFromSeeds(const MultiViewParams* mp, int rc); // TODO: require seeds vector as input param
StaticVector<int>* createRandomArrayOfIntegers(int n);

int findNSubstrsInString(const std::string& str, const std::string& val);
std::string num2str(int num);
std::string num2str(float num);
std::string num2str(int64_t num);
std::string num2strThreeDigits(int index);
std::string num2strFourDecimal(int index);
std::string num2strTwoDecimal(int index);

} // namespace mvsUtils
} // namespace aliceVision
