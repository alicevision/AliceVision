#pragma once

#define _USE_MATH_DEFINES
#include "mv_structures.h"
#include <cmath>

void rot2qtr(matrix3x3 R, double* _qx, double* _qy, double* _qz, double* _qw);
matrix3x3 qtr2rot(double qx, double qy, double qz, double qw);

/* Define a QVector.  */
typedef struct _QVector
{
    float x;
    float y;
    float z;
} QVector, *QVectorPtr;

/* Takes the modulus of v */
#define VMod(v) (sqrt((v).x * (v).x + (v).y * (v).y + (v).z * (v).z))

/* Returns the dot product of v1 & v2 */
#define VDot(v1, v2) ((v1).x * (v2).x + (v1).y * (v2).y + (v1).z * (v2).z)

/* Fills the fields of a QVector.	*/
#define VNew(a, b, c, r) ((r).x = (a), (r).y = (b), (r).z = (c))

#define VAdd(v1, v2, r) ((r).x = (v1).x + (v2).x, (r).y = (v1).y + (v2).y, (r).z = (v1).z + (v2).z)

#define VSub(v1, v2, r) ((r).x = (v1).x - (v2).x, (r).y = (v1).y - (v2).y, (r).z = (v1).z - (v2).z)

#define VCross(v1, v2, r)                                                                                              \
    ((r).x = (v1).y * (v2).z - (v1).z * (v2).y, (r).y = (v1).z * (v2).x - (v1).x * (v2).z,                             \
     (r).z = (v1).x * (v2).y - (v1).y * (v2).x)

#define VScalarMul(v, d, r) ((r).x = (v).x * (d), (r).y = (v).y * (d), (r).z = (v).z * (d))

#define VUnit(v, t, r) ((t) = 1 / VMod(v), VScalarMul(v, t, r))

typedef struct _Quaternion
{
    QVector vect_part;
    float real_part;
} Quaternion;

Quaternion Build_Rotate_Quaternion(QVector axis, float cos_angle);
Quaternion QQMul(Quaternion* q1, Quaternion* q2);
void Quaternion_To_Axis_Angle(Quaternion* q, QVector* axis, float* angle);
void Convert_Camera_Model(QVector* pos, QVector* at, QVector* up, QVector* res_axis, float* res_angle);