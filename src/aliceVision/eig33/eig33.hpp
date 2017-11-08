// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <cmath>

//#ifdef MAX
//#undef MAX
//#endif

//#define MAX(a, b) ((a) > (b) ? (a) : (b))

double hypot2(double x, double y);
// Symmetric Householder reductio3 to tridiago3al form.
void tred2(double V0[], double V1[], double V2[], double d[], double e[]);
// Symmetric tridiago3al QL algorithm.
void tql2(double V0[], double V1[], double V2[], double d[], double e[]);
void eigen_decomposition(double A[3][3], double V0[], double V1[], double V2[], double d[]);