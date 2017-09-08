#include "quaternion.h"
#include "stdafx.h"

/*

function q = rot2qtr(R)

m00 = R(1,1); m01 = R(1,2); m02 = R(1,3);
m10 = R(2,1); m11 = R(2,2); m12 = R(2,3);
m20 = R(3,1); m21 = R(3,2); m22 = R(3,3);

if 1.0 + m00 + m11 + m22 > 0
  S = 0.5 / sqrt(1.0 + m00 + m11 + m22);
  qw = 0.25 / S;
  qx = (m21 - m12) * S;
  qy = (m02 - m20) * S;
  qz = (m10 - m01) * S;
else
  if (m00 > m11) && (m00 > m22)
    S = sqrt(1.0 + m00 - m11 - m22) * 2;
    qw = (m12 - m21) / S;
    qx = 0.25 * S;
    qy = (m01 + m10) / S;
    qz = (m02 + m20) / S;
  elseif m11 > m22
    S = sqrt(1.0 + m11 - m00 - m22) * 2;
    qw = (m02 - m20) / S;
    qx = (m01 + m10) / S;
    qy = 0.25 * S;
    qz = (m12 + m21) / S;
  else
    S = sqrt(1.0 + m22 - m00 - m11) * 2;
    qw = (m01 - m10) / S;
    qx = (m02 + m20) / S;
    qy = (m12 + m21) / S;
    qz = 0.25 * S;
  end
end

q = [qw qx qy qz];





function R = qtr2rot(q)

qw = q(1); qx = q(2); qy = q(3); qz = q(4);

Nq = qw^2 + qx^2 + qy^2 + qz^2;
if Nq>0
  s = 2/Nq;
else
  s = 0;
end

X  = qx*s; Y  = qy*s; Z  = qz*s;
wX = qw*X; wY = qw*Y; wZ = qw*Z;
xX = qx*X; xY = qx*Y; xZ = qx*Z;
yY = qy*Y; yZ = qy*Z; zZ = qz*Z;

R = [1-(yY+zZ) xY-wZ xZ+wY; ...
     xY+wZ 1-(xX+zZ) yZ-wX; ...
     xZ-wY yZ+wX 1-(xX+yY)];



*/

void rot2qtr(matrix3x3 R, double* _qx, double* _qy, double* _qz, double* _qw)
{
    double m00 = R.m11;
    double m01 = R.m12;
    double m02 = R.m13;
    double m10 = R.m21;
    double m11 = R.m22;
    double m12 = R.m23;
    double m20 = R.m31;
    double m21 = R.m32;
    double m22 = R.m33;

    double qw, qx, qy, qz;

    if(1.0 + m00 + m11 + m22 > 0.0)
    {
        double S = 0.5 / sqrt(1.0 + m00 + m11 + m22);
        qw = 0.25 / S;
        qx = (m21 - m12) * S;
        qy = (m02 - m20) * S;
        qz = (m10 - m01) * S;
    }
    else
    {
        if((m00 > m11) && (m00 > m22))
        {
            double S = sqrt(1.0 + m00 - m11 - m22) * 2.0;
            qw = (m12 - m21) / S;
            qx = 0.25 * S;
            qy = (m01 + m10) / S;
            qz = (m02 + m20) / S;
        }
        else
        {
            if(m11 > m22)
            {
                double S = sqrt(1.0 + m11 - m00 - m22) * 2.0;
                qw = (m02 - m20) / S;
                qx = (m01 + m10) / S;
                qy = 0.25 * S;
                qz = (m12 + m21) / S;
            }
            else
            {
                double S = sqrt(1.0 + m22 - m00 - m11) * 2.0;
                qw = (m01 - m10) / S;
                qx = (m02 + m20) / S;
                qy = (m12 + m21) / S;
                qz = 0.25 * S;
            }
        }
    }

    *_qx = qx;
    *_qy = qy;
    *_qz = qz;
    *_qw = qw;
}

matrix3x3 qtr2rot(double qx, double qy, double qz, double qw)
{
    double Nq = qw * qw + qx * qx + qy * qy + qz * qz;
    double s;

    if(Nq > 0.0)
    {
        s = 2.0 / Nq;
    }
    else
    {
        s = 0.0;
    }

    double X = qx * s;
    double Y = qy * s;
    double Z = qz * s;
    double wX = qw * X;
    double wY = qw * Y;
    double wZ = qw * Z;
    double xX = qx * X;
    double xY = qx * Y;
    double xZ = qx * Z;
    double yY = qy * Y;
    double yZ = qy * Z;
    double zZ = qz * Z;

    matrix3x3 R;
    R.m11 = 1.0 - (yY + zZ);
    R.m12 = xY - wZ;
    R.m13 = xZ + wY;
    R.m21 = xY + wZ;
    R.m22 = 1.0 - (xX + zZ);
    R.m23 = yZ - wX;
    R.m31 = xZ - wY;
    R.m32 = yZ + wX;
    R.m33 = 1.0 - (xX + yY);

    return R;
}

/*

function q = rot2qtr(R)

m00 = R(1,1); m01 = R(1,2); m02 = R(1,3);
m10 = R(2,1); m11 = R(2,2); m12 = R(2,3);
m20 = R(3,1); m21 = R(3,2); m22 = R(3,3);

if 1.0 + m00 + m11 + m22 > 0
  S = 0.5 / sqrt(1.0 + m00 + m11 + m22);
  qw = 0.25 / S;
  qx = (m21 - m12) * S;
  qy = (m02 - m20) * S;
  qz = (m10 - m01) * S;
else
  if (m00 > m11) && (m00 > m22)
    S = sqrt(1.0 + m00 - m11 - m22) * 2;
    qw = (m12 - m21) / S;
    qx = 0.25 * S;
    qy = (m01 + m10) / S;
    qz = (m02 + m20) / S;
  elseif m11 > m22
    S = sqrt(1.0 + m11 - m00 - m22) * 2;
    qw = (m02 - m20) / S;
    qx = (m01 + m10) / S;
    qy = 0.25 * S;
    qz = (m12 + m21) / S;
  else
    S = sqrt(1.0 + m22 - m00 - m11) * 2;
    qw = (m01 - m10) / S;
    qx = (m02 + m20) / S;
    qy = (m12 + m21) / S;
    qz = 0.25 * S;
  end
end

q = [qw qx qy qz];
[11:43:14 PM] cenek.albl: function R = qtr2rot(q)

qw = q(1); qx = q(2); qy = q(3); qz = q(4);

Nq = qw^2 + qx^2 + qy^2 + qz^2;
if Nq>0
  s = 2/Nq;
else
  s = 0;
end

X  = qx*s; Y  = qy*s; Z  = qz*s;
wX = qw*X; wY = qw*Y; wZ = qw*Z;
xX = qx*X; xY = qx*Y; xZ = qx*Z;
yY = qy*Y; yZ = qy*Z; zZ = qz*Z;

R = [1-(yY+zZ) xY-wZ xZ+wY; ...
     xY+wZ 1-(xX+zZ) yZ-wX; ...
     xZ-wY yZ+wX 1-(xX+yY)];

*/

Quaternion Build_Rotate_Quaternion(QVector axis, float cos_angle)
{
    Quaternion quat;
    float sin_half_angle;
    float cos_half_angle;
    float angle;

    /* The quaternion requires half angles. */
    if(cos_angle > 1.0)
        cos_angle = 1.0;
    if(cos_angle < -1.0)
        cos_angle = -1.0;
    angle = acos(cos_angle);
    sin_half_angle = sin(angle / 2);
    cos_half_angle = cos(angle / 2);

    VScalarMul(axis, sin_half_angle, quat.vect_part);
    quat.real_part = cos_half_angle;

    return quat;
}

Quaternion QQMul(Quaternion* q1, Quaternion* q2)
{
    Quaternion res;
    QVector temp_v;

    res.real_part = q1->real_part * q2->real_part - VDot(q1->vect_part, q2->vect_part);
    VCross(q1->vect_part, q2->vect_part, res.vect_part);
    VScalarMul(q1->vect_part, q2->real_part, temp_v);
    VAdd(temp_v, res.vect_part, res.vect_part);
    VScalarMul(q2->vect_part, q1->real_part, temp_v);
    VAdd(temp_v, res.vect_part, res.vect_part);

    return res;
}

void Quaternion_To_Axis_Angle(Quaternion* q, QVector* axis, float* angle)
{
    float half_angle;
    float sin_half_angle;

    half_angle = acos(q->real_part);
    sin_half_angle = sin(half_angle);
    *angle = half_angle * 2;
    if(sin_half_angle < 1e-8 && sin_half_angle > -1e-8)
        VNew(1, 0, 0, *axis);
    else
    {
        sin_half_angle = 1 / sin_half_angle;
        VScalarMul(q->vect_part, sin_half_angle, *axis);
    }
}

void Convert_Camera_Model(QVector* pos, QVector* at, QVector* up, QVector* res_axis, float* res_angle)
{
    QVector n, v;

    Quaternion rot_quat;
    QVector norm_axis;
    Quaternion norm_quat;
    Quaternion inv_norm_quat;
    Quaternion y_quat, new_y_quat, rot_y_quat;
    QVector new_y;

    float temp_d;
    QVector temp_v;

    /* n = (norm)(pos - at) */
    VSub(*at, *pos, n);
    VUnit(n, temp_d, n);

    /* v = (norm)(view_up - (view_up.n)n) */
    VUnit(*up, temp_d, *up);
    temp_d = VDot(*up, n);
    VScalarMul(n, temp_d, temp_v);
    VSub(*up, temp_v, v);
    VUnit(v, temp_d, v);

    VNew(n.y, -n.x, 0, norm_axis);
    if(VDot(norm_axis, norm_axis) < 1e-8)
    {
        /* Already aligned, or maybe inverted. */
        if(n.z > 0.0)
        {
            norm_quat.real_part = 0.0;
            VNew(0, 1, 0, norm_quat.vect_part);
        }
        else
        {
            norm_quat.real_part = 1.0;
            VNew(0, 0, 0, norm_quat.vect_part);
        }
    }
    else
    {
        VUnit(norm_axis, temp_d, norm_axis);
        norm_quat = Build_Rotate_Quaternion(norm_axis, -n.z);
    }

    /* norm_quat now holds the rotation needed to line up the view directions.
    ** We need to find the rotation to align the up QVectors also.
    */

    /* Need to rotate the world y QVector to see where it ends up. */
    /* Find the inverse rotation. */
    inv_norm_quat.real_part = norm_quat.real_part;
    VScalarMul(norm_quat.vect_part, -1, inv_norm_quat.vect_part);

    /* Rotate the y. */
    y_quat.real_part = 0.0;
    VNew(0, 1, 0, y_quat.vect_part);
    new_y_quat = QQMul(&norm_quat, &y_quat);
    new_y_quat = QQMul(&new_y_quat, &inv_norm_quat);
    new_y = new_y_quat.vect_part;

    /* Now need to find out how much to rotate about n to line up y. */
    VCross(new_y, v, temp_v);
    if(VDot(temp_v, temp_v) < 1.e-8)
    {
        /* The old and new may be pointing in the same or opposite. Need
        ** to generate a QVector perpendicular to the old or new y.
        */
        VNew(0, -v.z, v.y, temp_v);
        if(VDot(temp_v, temp_v) < 1.e-8)
            VNew(v.z, 0, -v.x, temp_v);
    }
    VUnit(temp_v, temp_d, temp_v);
    rot_y_quat = Build_Rotate_Quaternion(temp_v, VDot(new_y, v));

    /* rot_y_quat holds the rotation about the initial camera direction needed
    ** to align the up QVectors in the final position.
    */

    /* Put the 2 rotations together. */
    rot_quat = QQMul(&rot_y_quat, &norm_quat);

    /* Extract the axis and angle from the quaternion. */
    Quaternion_To_Axis_Angle(&rot_quat, res_axis, res_angle);
}
