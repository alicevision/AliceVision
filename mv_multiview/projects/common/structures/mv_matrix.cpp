#include "mv_matrix.h"
#include "stdafx.h"

void matrix4x4::doprintf()
{
    printf("%f %f %f %f\n", m11, m12, m13, m14);
    printf("%f %f %f %f\n", m21, m22, m23, m24);
    printf("%f %f %f %f\n", m31, m32, m33, m34);
    printf("%f %f %f %f\n", m41, m42, m43, m44);
};
