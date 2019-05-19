// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "P4PfSolver.hpp"
#include <aliceVision/numeric/projection.hpp>
#include <aliceVision/numeric/numeric.hpp>

#include <cmath>
#include <cstring>
#include <limits>
#include <iostream>

namespace aliceVision {
namespace resection {

void GJ(double *A, int rcnt, int ccnt, double tol)
{
  int row = 0;
  int col = 0;
  int ofs = 0;

  while(row < rcnt && col < ccnt)
  {
    // find pivot
    double apivot = 0;
    double pivot = 0;
    int pivot_r = -1;

    int pofs = ofs;
    for(int k = row; k < rcnt; ++k)
    {
      // pivot selection criteria here !
      if(std::abs(*(A + pofs)) > apivot)
      {
        pivot = *(A + pofs);
        apivot = std::abs(pivot);
        pivot_r = k;
      }
      pofs += ccnt;
    }

    if(apivot < tol)
    {
      // empty col - shift to next col (or jump)
      ++col;
      ++ofs;
    }
    else
    {
      // process rows
      double pivot_i = 1.0 / pivot;

      // exchange pivot and selected rows
      // + divide row
      if(pivot_r == row)
      {
        int srcofs = ofs;
        for(int c = col; c < ccnt; ++c)
        {
          *(A + srcofs) = *(A + srcofs) * pivot_i;
          srcofs++;
        }
      }
      else
      {
        int srcofs = ofs;
        int dstofs = ccnt * pivot_r + col;
        for(int c = col; c < ccnt; ++c)
        {
          double b = *(A + srcofs);
          *(A + srcofs) = *(A + dstofs) * pivot_i;
          *(A + dstofs) = b;

          ++srcofs;
          ++dstofs;
        }
      }

      // zero bottom
      pofs = ofs + ccnt;
      for(int k = row + 1; k < rcnt; ++k)
      {
        if(std::abs(*(A + pofs)) > tol)
        {
          // nonzero row
          double b = *(A + pofs);
          int dstofs = pofs + 1;
          int srcofs = ofs + 1;
          for(int c = col + 1; c < ccnt; ++c)
          {
            *(A + dstofs) = (*(A + dstofs) - *(A + srcofs) * b);
            ++dstofs;
            ++srcofs;
          }
          *(A + pofs) = 0;
        }
        pofs += ccnt;
      }

      // zero top
      pofs = col;
      for(int k = 0; k < row; ++k)
      {
        if(std::abs(*(A + pofs)) > tol)
        {
          // nonzero row
          double b = *(A + pofs);
          int dstofs = pofs + 1;
          int srcofs = ofs + 1;
          for(int c = col + 1; c < ccnt; ++c)
          {
            *(A + dstofs) = (*(A + dstofs) - *(A + srcofs) * b);
            ++dstofs;
            ++srcofs;
          }
          *(A + pofs) = 0;
        }
        pofs += ccnt;
      }

      ++row;
      ++col;
      ofs += ccnt + 1;
    }
  }
}

void computeCoefficients(const double *src1, const double *src2, const double *src3, const double *src4, const double *src5, double *dst1)
{
  // symbolic names.
  const double glab = src1[0];
  const double glac = src1[1];
  const double glad = src1[2];
  const double glbc = src1[3];
  const double glbd = src1[4];
  const double glcd = src1[5];
  const double a1 = src2[0];
  const double a2 = src2[1];
  const double b1 = src3[0];
  const double b2 = src3[1];
  const double c1 = src4[0];
  const double c2 = src4[1];
  const double d1 = src5[0];
  const double d2 = src5[1];

  const double t1 = 1 / glad;
  const double t2 = t1 * glbc;
  const double t3 = glab * t1;
  const double t4 = glac * t1;
  const double t5 = t2 - t3 - t4;
  const double t9 = d2 * d2;
  const double t11 = t3 * t9;
  const double t12 = t4 * t9;
  const double t13 = d1 * d1;
  const double t14 = t4 * t13;
  const double t16 = t3 * t13;
  const double t24 = -a2 * b2 - b1 * a1;
  const double t27 = -c1 * a1 - c2 * a2;
  const double t28 = a1 * t1;
  const double t32 = t1 * d1;
  const double t33 = a1 * glac * t32;
  const double t34 = a2 * d2;
  const double t35 = t4 * t34;
  const double t37 = a1 * glab * t32;
  const double t39 = t3 * t34;
  const double t41 = a2 * a2;
  const double t42 = a1 * a1;
  const double t43 = t4 * t41;
  const double t46 = t42 * glac * t1;
  const double t51 = t42 * glab * t1;
  const double t53 = t42 * t1;
  const double t56 = t3 * t41;
  const double t59 = c1 * c1;
  const double t60 = c2 * c2;
  const double t67 = t1 * glbd;
  const double t84 = glcd * t1;

  // destination group 1   
  dst1[0] = 1.0;
  dst1[1] = t5 / 2.0;
  dst1[2] = -1.0;
  dst1[3] = -1.0;
  dst1[4] = c2 * b2 + c1 * b1;
  dst1[5] = -t5;
  dst1[6] = t2 * t9 / 2.0 - t11 / 2.0 - t12 / 2.0 - t14 / 2.0 + t2 * t13 / 2.0 - t16 / 2.0;
  dst1[7] = 1.0 - t4 / 2.0 - t3 / 2.0 + t2 / 2.0;
  dst1[8] = t24;
  dst1[9] = t27;
  dst1[10] = -t28 * glbc * d1 + t33 + t35 + t37 - t2 * t34 + t39;
  dst1[11] = t41 + t42 - t43 / 2.0 - t46 / 2.0 + t2 * t41 / 2.0 - t51 / 2.0 + t53 * glbc / 2.0 - t56 / 2.0;
  dst1[12] = 1.0;
  dst1[13] = -t4;
  dst1[14] = -2.0;
  dst1[15] = t59 + t60;
  dst1[16] = 2.0 * t4;
  dst1[17] = -t14 - t12;
  dst1[18] = -t4 + 1.0;
  dst1[19] = 2.0 * t27;
  dst1[20] = 2.0 * t33 + 2.0 * t35;
  dst1[21] = -t43 + t41 + t42 - t46;
  dst1[22] = 1.0;
  dst1[23] = t67 / 2.0 - 1.0 / 2.0 - t3 / 2.0;
  dst1[24] = -1.0;
  dst1[25] = t3 - t67;
  dst1[26] = d2 * b2 + b1*d1;
  dst1[27] = -t11 / 2.0 - t16 / 2.0 + t67 * t9 / 2.0 + t67 * t13 / 2.0 - t9 / 2.0 - t13 / 2.0;
  dst1[28] = -t3 / 2.0 + t67 / 2.0 + 1.0 / 2.0;
  dst1[29] = t24;
  dst1[30] = -t28 * glbd * d1 + t37 + t39 - t67 * t34;
  dst1[31] = t67 * t41 / 2.0 + t53 * glbd / 2.0 - t56 / 2.0 - t51 / 2.0 + t42 / 2.0 + t41 / 2.0;
  dst1[32] = 1.0;
  dst1[33] = -t4 / 2.0 + t84 / 2.0 - 1.0 / 2.0;
  dst1[34] = -1.0;
  dst1[35] = t4 - t84;
  dst1[36] = c1 * d1 + c2*d2;
  dst1[37] = t84 * t13 / 2.0 - t12 / 2.0 - t14 / 2.0 - t13 / 2.0 - t9 / 2.0 + t84 * t9 / 2.0;
  dst1[38] = -t4 / 2.0 + 1.0 / 2.0 + t84 / 2.0;
  dst1[39] = t27;
  dst1[40] = t35 + t33 - t84 * t34 - glcd * a1 * t32;
  dst1[41] = t42 / 2.0 + t41 / 2.0 - t43 / 2.0 + glcd * t42 * t1 / 2.0 - t46 / 2.0 + t84 * t41 / 2.0;
}

void computeP4pfPoses(const double *glab, const double *a1, const double *b1, const double *c1, const double *d1, double *A)
{
  // precalculate polynomial equations coefficients
  double M[6864];
  double coefs[42];

  // prepare polynomial coefficients
  computeCoefficients(glab, a1, b1, c1, d1, coefs);

  std::memset(M, 0, 6864 * sizeof (double));

  M[64] = coefs[0];
  M[403] = coefs[0];
  M[486] = coefs[0];
  M[572] = coefs[0];
  M[1533] = coefs[0];
  M[1616] = coefs[0];
  M[1702] = coefs[0];
  M[1787] = coefs[0];
  M[1874] = coefs[0];
  M[1960] = coefs[0];
  M[3979] = coefs[0];
  M[4063] = coefs[0];
  M[4149] = coefs[0];
  M[4234] = coefs[0];
  M[4321] = coefs[0];
  M[4407] = coefs[0];
  M[4494] = coefs[0];
  M[6161] = coefs[0];
  M[6248] = coefs[0];
  M[71] = coefs[1];
  M[411] = coefs[1];
  M[496] = coefs[1];
  M[582] = coefs[1];
  M[1539] = coefs[1];
  M[1626] = coefs[1];
  M[1712] = coefs[1];
  M[1798] = coefs[1];
  M[1884] = coefs[1];
  M[4071] = coefs[1];
  M[4157] = coefs[1];
  M[4244] = coefs[1];
  M[4330] = coefs[1];
  M[4416] = coefs[1];
  M[4500] = coefs[1];
  M[6165] = coefs[1];
  M[6252] = coefs[1];
  M[75] = coefs[2];
  M[419] = coefs[2];
  M[504] = coefs[2];
  M[590] = coefs[2];
  M[1551] = coefs[2];
  M[1635] = coefs[2];
  M[1721] = coefs[2];
  M[1806] = coefs[2];
  M[1892] = coefs[2];
  M[4001] = coefs[2];
  M[4085] = coefs[2];
  M[4171] = coefs[2];
  M[4256] = coefs[2];
  M[4342] = coefs[2];
  M[4428] = coefs[2];
  M[4512] = coefs[2];
  M[6171] = coefs[2];
  M[6255] = coefs[2];
  M[76] = coefs[3];
  M[420] = coefs[3];
  M[505] = coefs[3];
  M[591] = coefs[3];
  M[1552] = coefs[3];
  M[1636] = coefs[3];
  M[1722] = coefs[3];
  M[1807] = coefs[3];
  M[1893] = coefs[3];
  M[4002] = coefs[3];
  M[4086] = coefs[3];
  M[4172] = coefs[3];
  M[4257] = coefs[3];
  M[4343] = coefs[3];
  M[4429] = coefs[3];
  M[4513] = coefs[3];
  M[6172] = coefs[3];
  M[6256] = coefs[3];
  M[77] = coefs[4];
  M[421] = coefs[4];
  M[506] = coefs[4];
  M[592] = coefs[4];
  M[1553] = coefs[4];
  M[1637] = coefs[4];
  M[1723] = coefs[4];
  M[1808] = coefs[4];
  M[1894] = coefs[4];
  M[1980] = coefs[4];
  M[4087] = coefs[4];
  M[4173] = coefs[4];
  M[4258] = coefs[4];
  M[4344] = coefs[4];
  M[4430] = coefs[4];
  M[4514] = coefs[4];
  M[6173] = coefs[4];
  M[6257] = coefs[4];
  M[79] = coefs[5];
  M[423] = coefs[5];
  M[508] = coefs[5];
  M[1555] = coefs[5];
  M[1640] = coefs[5];
  M[1726] = coefs[5];
  M[1812] = coefs[5];
  M[1898] = coefs[5];
  M[4003] = coefs[5];
  M[4090] = coefs[5];
  M[4176] = coefs[5];
  M[4262] = coefs[5];
  M[4348] = coefs[5];
  M[4517] = coefs[5];
  M[6176] = coefs[5];
  M[6260] = coefs[5];
  M[82] = coefs[6];
  M[426] = coefs[6];
  M[513] = coefs[6];
  M[599] = coefs[6];
  M[1645] = coefs[6];
  M[1731] = coefs[6];
  M[1818] = coefs[6];
  M[1904] = coefs[6];
  M[1990] = coefs[6];
  M[4179] = coefs[6];
  M[4354] = coefs[6];
  M[4440] = coefs[6];
  M[4524] = coefs[6];
  M[6181] = coefs[6];
  M[6266] = coefs[6];
  M[83] = coefs[7];
  M[431] = coefs[7];
  M[516] = coefs[7];
  M[1567] = coefs[7];
  M[1652] = coefs[7];
  M[1825] = coefs[7];
  M[1911] = coefs[7];
  M[4019] = coefs[7];
  M[4104] = coefs[7];
  M[4190] = coefs[7];
  M[4276] = coefs[7];
  M[4362] = coefs[7];
  M[6277] = coefs[7];
  M[84] = coefs[8];
  M[432] = coefs[8];
  M[517] = coefs[8];
  M[603] = coefs[8];
  M[1568] = coefs[8];
  M[1653] = coefs[8];
  M[1739] = coefs[8];
  M[1826] = coefs[8];
  M[1912] = coefs[8];
  M[1998] = coefs[8];
  M[4020] = coefs[8];
  M[4105] = coefs[8];
  M[4191] = coefs[8];
  M[4277] = coefs[8];
  M[4363] = coefs[8];
  M[4449] = coefs[8];
  M[4532] = coefs[8];
  M[6195] = coefs[8];
  M[6278] = coefs[8];
  M[85] = coefs[9];
  M[433] = coefs[9];
  M[518] = coefs[9];
  M[604] = coefs[9];
  M[1569] = coefs[9];
  M[1654] = coefs[9];
  M[1740] = coefs[9];
  M[1913] = coefs[9];
  M[1999] = coefs[9];
  M[4021] = coefs[9];
  M[4106] = coefs[9];
  M[4192] = coefs[9];
  M[4364] = coefs[9];
  M[4450] = coefs[9];
  M[4533] = coefs[9];
  M[6196] = coefs[9];
  M[6279] = coefs[9];
  M[86] = coefs[10];
  M[434] = coefs[10];
  M[521] = coefs[10];
  M[607] = coefs[10];
  M[1570] = coefs[10];
  M[1657] = coefs[10];
  M[1743] = coefs[10];
  M[1830] = coefs[10];
  M[1916] = coefs[10];
  M[4109] = coefs[10];
  M[4195] = coefs[10];
  M[4282] = coefs[10];
  M[4368] = coefs[10];
  M[4454] = coefs[10];
  M[4538] = coefs[10];
  M[6200] = coefs[10];
  M[6284] = coefs[10];
  M[87] = coefs[11];
  M[438] = coefs[11];
  M[525] = coefs[11];
  M[611] = coefs[11];
  M[1578] = coefs[11];
  M[1665] = coefs[11];
  M[1751] = coefs[11];
  M[1838] = coefs[11];
  M[1924] = coefs[11];
  M[4034] = coefs[11];
  M[4121] = coefs[11];
  M[4207] = coefs[11];
  M[4294] = coefs[11];
  M[4380] = coefs[11];
  M[4551] = coefs[11];
  M[6214] = coefs[11];
  M[6298] = coefs[11];
  M[153] = coefs[12];
  M[668] = coefs[12];
  M[750] = coefs[12];
  M[837] = coefs[12];
  M[2062] = coefs[12];
  M[2145] = coefs[12];
  M[2232] = coefs[12];
  M[2319] = coefs[12];
  M[2403] = coefs[12];
  M[2490] = coefs[12];
  M[2577] = coefs[12];
  M[4596] = coefs[12];
  M[4679] = coefs[12];
  M[4766] = coefs[12];
  M[4850] = coefs[12];
  M[4937] = coefs[12];
  M[5024] = coefs[12];
  M[5110] = coefs[12];
  M[6338] = coefs[12];
  M[6424] = coefs[12];
  M[159] = coefs[13];
  M[675] = coefs[13];
  M[759] = coefs[13];
  M[846] = coefs[13];
  M[2067] = coefs[13];
  M[2154] = coefs[13];
  M[2241] = coefs[13];
  M[2328] = coefs[13];
  M[2413] = coefs[13];
  M[2499] = coefs[13];
  M[4686] = coefs[13];
  M[4773] = coefs[13];
  M[4859] = coefs[13];
  M[4945] = coefs[13];
  M[5032] = coefs[13];
  M[5115] = coefs[13];
  M[6341] = coefs[13];
  M[6427] = coefs[13];
  M[164] = coefs[14];
  M[684] = coefs[14];
  M[768] = coefs[14];
  M[855] = coefs[14];
  M[2080] = coefs[14];
  M[2164] = coefs[14];
  M[2251] = coefs[14];
  M[2338] = coefs[14];
  M[2422] = coefs[14];
  M[2508] = coefs[14];
  M[4618] = coefs[14];
  M[4701] = coefs[14];
  M[4788] = coefs[14];
  M[4872] = coefs[14];
  M[4958] = coefs[14];
  M[5045] = coefs[14];
  M[5128] = coefs[14];
  M[6348] = coefs[14];
  M[6431] = coefs[14];
  M[166] = coefs[15];
  M[686] = coefs[15];
  M[770] = coefs[15];
  M[857] = coefs[15];
  M[2082] = coefs[15];
  M[2253] = coefs[15];
  M[2340] = coefs[15];
  M[2424] = coefs[15];
  M[2510] = coefs[15];
  M[2597] = coefs[15];
  M[4703] = coefs[15];
  M[4790] = coefs[15];
  M[4874] = coefs[15];
  M[4960] = coefs[15];
  M[5047] = coefs[15];
  M[5130] = coefs[15];
  M[6350] = coefs[15];
  M[6433] = coefs[15];
  M[167] = coefs[16];
  M[687] = coefs[16];
  M[771] = coefs[16];
  M[2083] = coefs[16];
  M[2168] = coefs[16];
  M[2255] = coefs[16];
  M[2342] = coefs[16];
  M[2427] = coefs[16];
  M[2513] = coefs[16];
  M[4619] = coefs[16];
  M[4705] = coefs[16];
  M[4792] = coefs[16];
  M[4877] = coefs[16];
  M[4963] = coefs[16];
  M[5132] = coefs[16];
  M[6352] = coefs[16];
  M[6435] = coefs[16];
  M[170] = coefs[17];
  M[690] = coefs[17];
  M[776] = coefs[17];
  M[863] = coefs[17];
  M[2173] = coefs[17];
  M[2260] = coefs[17];
  M[2347] = coefs[17];
  M[2433] = coefs[17];
  M[2519] = coefs[17];
  M[2606] = coefs[17];
  M[4795] = coefs[17];
  M[4969] = coefs[17];
  M[5056] = coefs[17];
  M[5139] = coefs[17];
  M[6357] = coefs[17];
  M[6441] = coefs[17];
  M[171] = coefs[18];
  M[695] = coefs[18];
  M[779] = coefs[18];
  M[2095] = coefs[18];
  M[2180] = coefs[18];
  M[2267] = coefs[18];
  M[2440] = coefs[18];
  M[2526] = coefs[18];
  M[4635] = coefs[18];
  M[4719] = coefs[18];
  M[4806] = coefs[18];
  M[4891] = coefs[18];
  M[4977] = coefs[18];
  M[6452] = coefs[18];
  M[173] = coefs[19];
  M[697] = coefs[19];
  M[781] = coefs[19];
  M[868] = coefs[19];
  M[2097] = coefs[19];
  M[2182] = coefs[19];
  M[2269] = coefs[19];
  M[2356] = coefs[19];
  M[2442] = coefs[19];
  M[2528] = coefs[19];
  M[2615] = coefs[19];
  M[4637] = coefs[19];
  M[4721] = coefs[19];
  M[4808] = coefs[19];
  M[4893] = coefs[19];
  M[4979] = coefs[19];
  M[5066] = coefs[19];
  M[5148] = coefs[19];
  M[6372] = coefs[19];
  M[6454] = coefs[19];
  M[174] = coefs[20];
  M[698] = coefs[20];
  M[784] = coefs[20];
  M[871] = coefs[20];
  M[2098] = coefs[20];
  M[2185] = coefs[20];
  M[2272] = coefs[20];
  M[2359] = coefs[20];
  M[2445] = coefs[20];
  M[2531] = coefs[20];
  M[4724] = coefs[20];
  M[4811] = coefs[20];
  M[4897] = coefs[20];
  M[4983] = coefs[20];
  M[5070] = coefs[20];
  M[5153] = coefs[20];
  M[6376] = coefs[20];
  M[6459] = coefs[20];
  M[175] = coefs[21];
  M[702] = coefs[21];
  M[788] = coefs[21];
  M[875] = coefs[21];
  M[2106] = coefs[21];
  M[2193] = coefs[21];
  M[2280] = coefs[21];
  M[2367] = coefs[21];
  M[2453] = coefs[21];
  M[2539] = coefs[21];
  M[4650] = coefs[21];
  M[4736] = coefs[21];
  M[4823] = coefs[21];
  M[4909] = coefs[21];
  M[4995] = coefs[21];
  M[5166] = coefs[21];
  M[6390] = coefs[21];
  M[6473] = coefs[21];
  M[243] = coefs[22];
  M[935] = coefs[22];
  M[1019] = coefs[22];
  M[1105] = coefs[22];
  M[2681] = coefs[22];
  M[2765] = coefs[22];
  M[2851] = coefs[22];
  M[2936] = coefs[22];
  M[3022] = coefs[22];
  M[3108] = coefs[22];
  M[5209] = coefs[22];
  M[5293] = coefs[22];
  M[5379] = coefs[22];
  M[5463] = coefs[22];
  M[6515] = coefs[22];
  M[6601] = coefs[22];
  M[247] = coefs[23];
  M[939] = coefs[23];
  M[1024] = coefs[23];
  M[1110] = coefs[23];
  M[2683] = coefs[23];
  M[2770] = coefs[23];
  M[2856] = coefs[23];
  M[2942] = coefs[23];
  M[3028] = coefs[23];
  M[5213] = coefs[23];
  M[5298] = coefs[23];
  M[5384] = coefs[23];
  M[5468] = coefs[23];
  M[6517] = coefs[23];
  M[6604] = coefs[23];
  M[251] = coefs[24];
  M[947] = coefs[24];
  M[1032] = coefs[24];
  M[1118] = coefs[24];
  M[2695] = coefs[24];
  M[2779] = coefs[24];
  M[2865] = coefs[24];
  M[2950] = coefs[24];
  M[3036] = coefs[24];
  M[5227] = coefs[24];
  M[5310] = coefs[24];
  M[5396] = coefs[24];
  M[5480] = coefs[24];
  M[6523] = coefs[24];
  M[6607] = coefs[24];
  M[255] = coefs[25];
  M[951] = coefs[25];
  M[1036] = coefs[25];
  M[2699] = coefs[25];
  M[2784] = coefs[25];
  M[2870] = coefs[25];
  M[2956] = coefs[25];
  M[3042] = coefs[25];
  M[5232] = coefs[25];
  M[5316] = coefs[25];
  M[5485] = coefs[25];
  M[6528] = coefs[25];
  M[6612] = coefs[25];
  M[256] = coefs[26];
  M[952] = coefs[26];
  M[1037] = coefs[26];
  M[1123] = coefs[26];
  M[2700] = coefs[26];
  M[2785] = coefs[26];
  M[2871] = coefs[26];
  M[2957] = coefs[26];
  M[3043] = coefs[26];
  M[3129] = coefs[26];
  M[5233] = coefs[26];
  M[5317] = coefs[26];
  M[5403] = coefs[26];
  M[5486] = coefs[26];
  M[6529] = coefs[26];
  M[6613] = coefs[26];
  M[258] = coefs[27];
  M[954] = coefs[27];
  M[1041] = coefs[27];
  M[1127] = coefs[27];
  M[2789] = coefs[27];
  M[2875] = coefs[27];
  M[2962] = coefs[27];
  M[3048] = coefs[27];
  M[3134] = coefs[27];
  M[5235] = coefs[27];
  M[5322] = coefs[27];
  M[5408] = coefs[27];
  M[5492] = coefs[27];
  M[6533] = coefs[27];
  M[6618] = coefs[27];
  M[259] = coefs[28];
  M[959] = coefs[28];
  M[1044] = coefs[28];
  M[2711] = coefs[28];
  M[2796] = coefs[28];
  M[2969] = coefs[28];
  M[3055] = coefs[28];
  M[5246] = coefs[28];
  M[5330] = coefs[28];
  M[6629] = coefs[28];
  M[260] = coefs[29];
  M[960] = coefs[29];
  M[1045] = coefs[29];
  M[1131] = coefs[29];
  M[2712] = coefs[29];
  M[2797] = coefs[29];
  M[2883] = coefs[29];
  M[2970] = coefs[29];
  M[3056] = coefs[29];
  M[3142] = coefs[29];
  M[5247] = coefs[29];
  M[5331] = coefs[29];
  M[5417] = coefs[29];
  M[5500] = coefs[29];
  M[6547] = coefs[29];
  M[6630] = coefs[29];
  M[262] = coefs[30];
  M[962] = coefs[30];
  M[1049] = coefs[30];
  M[1135] = coefs[30];
  M[2714] = coefs[30];
  M[2801] = coefs[30];
  M[2887] = coefs[30];
  M[2974] = coefs[30];
  M[3060] = coefs[30];
  M[5251] = coefs[30];
  M[5336] = coefs[30];
  M[5422] = coefs[30];
  M[5506] = coefs[30];
  M[6552] = coefs[30];
  M[6636] = coefs[30];
  M[263] = coefs[31];
  M[966] = coefs[31];
  M[1053] = coefs[31];
  M[1139] = coefs[31];
  M[2722] = coefs[31];
  M[2809] = coefs[31];
  M[2895] = coefs[31];
  M[2982] = coefs[31];
  M[3068] = coefs[31];
  M[5263] = coefs[31];
  M[5348] = coefs[31];
  M[5519] = coefs[31];
  M[6566] = coefs[31];
  M[6650] = coefs[31];
  M[332] = coefs[32];
  M[1200] = coefs[32];
  M[1284] = coefs[32];
  M[1371] = coefs[32];
  M[1458] = coefs[32];
  M[3210] = coefs[32];
  M[3294] = coefs[32];
  M[3381] = coefs[32];
  M[3468] = coefs[32];
  M[3553] = coefs[32];
  M[3640] = coefs[32];
  M[3727] = coefs[32];
  M[3814] = coefs[32];
  M[3901] = coefs[32];
  M[5564] = coefs[32];
  M[5651] = coefs[32];
  M[5738] = coefs[32];
  M[5822] = coefs[32];
  M[5908] = coefs[32];
  M[5994] = coefs[32];
  M[6080] = coefs[32];
  M[6692] = coefs[32];
  M[6778] = coefs[32];
  M[335] = coefs[33];
  M[1203] = coefs[33];
  M[1288] = coefs[33];
  M[1375] = coefs[33];
  M[1462] = coefs[33];
  M[3211] = coefs[33];
  M[3298] = coefs[33];
  M[3385] = coefs[33];
  M[3472] = coefs[33];
  M[3558] = coefs[33];
  M[3645] = coefs[33];
  M[3732] = coefs[33];
  M[3819] = coefs[33];
  M[5567] = coefs[33];
  M[5654] = coefs[33];
  M[5741] = coefs[33];
  M[5826] = coefs[33];
  M[5912] = coefs[33];
  M[5999] = coefs[33];
  M[6084] = coefs[33];
  M[6693] = coefs[33];
  M[6780] = coefs[33];
  M[340] = coefs[34];
  M[1212] = coefs[34];
  M[1297] = coefs[34];
  M[1384] = coefs[34];
  M[1471] = coefs[34];
  M[3224] = coefs[34];
  M[3308] = coefs[34];
  M[3395] = coefs[34];
  M[3482] = coefs[34];
  M[3567] = coefs[34];
  M[3654] = coefs[34];
  M[3741] = coefs[34];
  M[3828] = coefs[34];
  M[5582] = coefs[34];
  M[5669] = coefs[34];
  M[5756] = coefs[34];
  M[5839] = coefs[34];
  M[5925] = coefs[34];
  M[6011] = coefs[34];
  M[6097] = coefs[34];
  M[6700] = coefs[34];
  M[6784] = coefs[34];
  M[343] = coefs[35];
  M[1215] = coefs[35];
  M[1300] = coefs[35];
  M[1387] = coefs[35];
  M[3227] = coefs[35];
  M[3312] = coefs[35];
  M[3399] = coefs[35];
  M[3486] = coefs[35];
  M[3572] = coefs[35];
  M[3659] = coefs[35];
  M[3746] = coefs[35];
  M[3833] = coefs[35];
  M[5586] = coefs[35];
  M[5673] = coefs[35];
  M[5760] = coefs[35];
  M[5844] = coefs[35];
  M[6016] = coefs[35];
  M[6101] = coefs[35];
  M[6704] = coefs[35];
  M[6788] = coefs[35];
  M[345] = coefs[36];
  M[1217] = coefs[36];
  M[1302] = coefs[36];
  M[1389] = coefs[36];
  M[1476] = coefs[36];
  M[3229] = coefs[36];
  M[3314] = coefs[36];
  M[3401] = coefs[36];
  M[3488] = coefs[36];
  M[3661] = coefs[36];
  M[3748] = coefs[36];
  M[3835] = coefs[36];
  M[3922] = coefs[36];
  M[5762] = coefs[36];
  M[5846] = coefs[36];
  M[5932] = coefs[36];
  M[6018] = coefs[36];
  M[6103] = coefs[36];
  M[6706] = coefs[36];
  M[6790] = coefs[36];
  M[346] = coefs[37];
  M[1218] = coefs[37];
  M[1305] = coefs[37];
  M[1392] = coefs[37];
  M[1479] = coefs[37];
  M[3317] = coefs[37];
  M[3404] = coefs[37];
  M[3491] = coefs[37];
  M[3578] = coefs[37];
  M[3665] = coefs[37];
  M[3752] = coefs[37];
  M[3839] = coefs[37];
  M[3926] = coefs[37];
  M[5763] = coefs[37];
  M[5850] = coefs[37];
  M[5936] = coefs[37];
  M[6023] = coefs[37];
  M[6108] = coefs[37];
  M[6709] = coefs[37];
  M[6794] = coefs[37];
  M[347] = coefs[38];
  M[1223] = coefs[38];
  M[1308] = coefs[38];
  M[1395] = coefs[38];
  M[3239] = coefs[38];
  M[3324] = coefs[38];
  M[3411] = coefs[38];
  M[3585] = coefs[38];
  M[3672] = coefs[38];
  M[3759] = coefs[38];
  M[3846] = coefs[38];
  M[5600] = coefs[38];
  M[5687] = coefs[38];
  M[5774] = coefs[38];
  M[5858] = coefs[38];
  M[6030] = coefs[38];
  M[6805] = coefs[38];
  M[349] = coefs[39];
  M[1225] = coefs[39];
  M[1310] = coefs[39];
  M[1397] = coefs[39];
  M[1484] = coefs[39];
  M[3241] = coefs[39];
  M[3326] = coefs[39];
  M[3413] = coefs[39];
  M[3500] = coefs[39];
  M[3674] = coefs[39];
  M[3761] = coefs[39];
  M[3848] = coefs[39];
  M[3935] = coefs[39];
  M[5602] = coefs[39];
  M[5689] = coefs[39];
  M[5776] = coefs[39];
  M[5860] = coefs[39];
  M[5946] = coefs[39];
  M[6032] = coefs[39];
  M[6117] = coefs[39];
  M[6724] = coefs[39];
  M[6807] = coefs[39];
  M[350] = coefs[40];
  M[1226] = coefs[40];
  M[1313] = coefs[40];
  M[1400] = coefs[40];
  M[1487] = coefs[40];
  M[3242] = coefs[40];
  M[3329] = coefs[40];
  M[3416] = coefs[40];
  M[3503] = coefs[40];
  M[3590] = coefs[40];
  M[3677] = coefs[40];
  M[3764] = coefs[40];
  M[3851] = coefs[40];
  M[5605] = coefs[40];
  M[5692] = coefs[40];
  M[5779] = coefs[40];
  M[5864] = coefs[40];
  M[5950] = coefs[40];
  M[6037] = coefs[40];
  M[6122] = coefs[40];
  M[6728] = coefs[40];
  M[6812] = coefs[40];
  M[351] = coefs[41];
  M[1230] = coefs[41];
  M[1317] = coefs[41];
  M[1404] = coefs[41];
  M[1491] = coefs[41];
  M[3250] = coefs[41];
  M[3337] = coefs[41];
  M[3424] = coefs[41];
  M[3511] = coefs[41];
  M[3598] = coefs[41];
  M[3685] = coefs[41];
  M[3772] = coefs[41];
  M[3859] = coefs[41];
  M[5617] = coefs[41];
  M[5704] = coefs[41];
  M[5791] = coefs[41];
  M[5876] = coefs[41];
  M[6050] = coefs[41];
  M[6135] = coefs[41];
  M[6742] = coefs[41];
  M[6826] = coefs[41];

  // GJ elimination
  GJ(M, 78, 88, 2.2204e-11);

  // action matrix
  std::memset(A, 0, sizeof (double) * 100);

  A[1] = 1;
  A[15] = 1;
  A[26] = 1;
  A[37] = 1;
  A[48] = 1;
  A[50] = -M[6599];
  A[51] = -M[6598];
  A[52] = -M[6597];
  A[53] = -M[6596];
  A[54] = -M[6595];
  A[55] = -M[6594];
  A[56] = -M[6593];
  A[57] = -M[6592];
  A[58] = -M[6591];
  A[59] = -M[6590];
  A[60] = -M[6511];
  A[61] = -M[6510];
  A[62] = -M[6509];
  A[63] = -M[6508];
  A[64] = -M[6507];
  A[65] = -M[6506];
  A[66] = -M[6505];
  A[67] = -M[6504];
  A[68] = -M[6503];
  A[69] = -M[6502];
  A[70] = -M[6423];
  A[71] = -M[6422];
  A[72] = -M[6421];
  A[73] = -M[6420];
  A[74] = -M[6419];
  A[75] = -M[6418];
  A[76] = -M[6417];
  A[77] = -M[6416];
  A[78] = -M[6415];
  A[79] = -M[6414];
  A[80] = -M[6335];
  A[81] = -M[6334];
  A[82] = -M[6333];
  A[83] = -M[6332];
  A[84] = -M[6331];
  A[85] = -M[6330];
  A[86] = -M[6329];
  A[87] = -M[6328];
  A[88] = -M[6327];
  A[89] = -M[6326];
  A[90] = -M[6247];
  A[91] = -M[6246];
  A[92] = -M[6245];
  A[93] = -M[6244];
  A[94] = -M[6243];
  A[95] = -M[6242];
  A[96] = -M[6241];
  A[97] = -M[6240];
  A[98] = -M[6239];
  A[99] = -M[6238];
}

bool isNan(const Eigen::MatrixXcd &A)
{
  const Mat B = A.real();
  for(Mat::Index i = 0; i < B.cols() * B.rows(); ++i)
  {
    if(std::isnan(B.data()[i])) return true;
  }
  return false;
}

bool validSol(const Eigen::MatrixXcd &sol, Mat &vSol)
{
  assert(sol.cols() == 10 && sol.rows() == 4);

  Mat imSol = sol.imag();
  Mat reSol = sol.real();
  std::vector<Mat::Index> correct;
  for(Mat::Index i = 0; i < 10; ++i)
  {
    bool isReal = true;
    for(Mat::Index j = 0; j < 4; ++j)
    {
      if(imSol(j, i) != 0)
      {
        isReal = false;
        break;
      }
    }
    if(isReal && reSol(3, i) > 0)
    {
      correct.push_back(i);
    }
  }
  if(correct.empty())
  {
    return false;
  }
  else
  {
    vSol = Mat(4, correct.size());
    for(std::size_t i = 0; i < correct.size(); ++i)
    {
      vSol.block(0, i, 4, 1) = reSol.block(0, correct.at(i), 4, 1);
    }
  }
  return true;
}

void getRigidTransform(const Mat &pp1, const Mat &pp2, Mat &R, Vec3 &t)
{
  Mat p1(pp1);
  Mat p2(pp2);

  // shift centers of gravity to the origin
  const Mat p1mean = p1.rowwise().sum() * 0.25;
  const Mat p2mean = p2.rowwise().sum() * 0.25;
  for(Mat::Index i = 0; i < 4; i++)
  {
    p1.block(0, i, 3, 1) -= p1mean;
    p2.block(0, i, 3, 1) -= p2mean;
  }

  // normalize to unit size
  const Mat u1 = p1 * p1.colwise().norm().cwiseInverse().asDiagonal();
  const Mat u2 = p2 * p2.colwise().norm().cwiseInverse().asDiagonal();

  // calc rotation
  const Mat C = u2 * u1.transpose();
  Eigen::JacobiSVD<Mat> svd(C, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Mat U = svd.matrixU();
  const Mat V = svd.matrixV();
  Mat S = svd.singularValues();

  // fit to rotation space
  S(0) = (S(0) >= 0 ? 1 : -1);
  S(1) = (S(1) >= 0 ? 1 : -1);
  S(2) = ((U * V.transpose()).determinant() >= 0 ? 1 : -1);

  R = U * S.asDiagonal() * V.transpose();
  t = -R * p1mean + p2mean;
}

void P4PfSolver::solve(const Mat &pt2Dx, const Mat &pt3Dx, std::vector<p4fSolution> *models)
{
  Mat pt2D(pt2Dx);
  Mat pt3D(pt3Dx);

  assert(2 == pt2D.rows());
  assert(3 == pt3D.rows());
  assert(pt2D.cols() == pt3D.cols());

  const Vec3 mean3d = pt3D.rowwise().mean();

  pt3D -= (mean3d * Mat::Constant(1, 4, 1.0));

  const double var = pt3D.colwise().norm().sum() / 4;
  const double var2d = pt2D.colwise().norm().sum() / 4;

  pt3D *= (1 / var);
  pt2D *= (1 / var2d);

  const double tol = std::numeric_limits<double>::epsilon();
  const double glab = (pt3D.col(0) - pt3D.col(1)).squaredNorm();
  const double glac = (pt3D.col(0) - pt3D.col(2)).squaredNorm();
  const double glad = (pt3D.col(0) - pt3D.col(3)).squaredNorm();
  const double glbc = (pt3D.col(1) - pt3D.col(2)).squaredNorm();
  const double glbd = (pt3D.col(1) - pt3D.col(3)).squaredNorm();
  const double glcd = (pt3D.col(2) - pt3D.col(3)).squaredNorm();

  // initial solution degeneracy - invalid input
  if(glab * glac * glad * glbc * glbd * glcd < tol)
    return;

  Mat A = Mat::Zero(10, 10);
  {
    const double gl[] = {glab, glac, glad, glbc, glbd, glcd};
    const double *a1 = pt2D.col(0).data();
    const double *b1 = pt2D.col(1).data();
    const double *c1 = pt2D.col(2).data();
    const double *d1 = pt2D.col(3).data();

    computeP4pfPoses(gl, a1, b1, c1, d1, A.data());
  }

  Mat vSol;
  {
    Eigen::EigenSolver<Mat> es(A.transpose());
    Eigen::MatrixXcd sol = es.eigenvectors();
    Eigen::MatrixXcd diag = sol.row(0).cwiseInverse().asDiagonal();

    sol = sol.block(1, 0, 4, 10) * diag;

    // contain at least one NaN
    if(isNan(sol))
      return;

    // separarte valid solutions
    if(!validSol(sol, vSol))
      return;
  }

  // recover camera rotation and translation
  for(Mat::Index i = 0; i < vSol.cols(); ++i)
  {
    const double f = sqrt(vSol(3, i));
    const double zd = vSol(0, i);
    const double zc = vSol(1, i);
    const double zb = vSol(2, i);

    // create p3d points in a camera coordinate system(using depths)
    Mat p3dc = Mat(3, 4);
    p3dc << pt2D(0, 0), zb * pt2D(0, 1), zc * pt2D(0, 2), zd * pt2D(0, 3),
            pt2D(1, 0), zb * pt2D(1, 1), zc * pt2D(1, 2), zd * pt2D(1, 3),
            f, zb * f, zc * f, zd * f;

    // fix scale(recover 'za')
    Mat d = Mat(6, 1);
    d(0, 0) = sqrt(glab / (p3dc.col(0) - p3dc.col(1)).squaredNorm());
    d(1, 0) = sqrt(glac / (p3dc.col(0) - p3dc.col(2)).squaredNorm());
    d(2, 0) = sqrt(glad / (p3dc.col(0) - p3dc.col(3)).squaredNorm());
    d(3, 0) = sqrt(glbc / (p3dc.col(1) - p3dc.col(2)).squaredNorm());
    d(4, 0) = sqrt(glbd / (p3dc.col(1) - p3dc.col(3)).squaredNorm());
    d(5, 0) = sqrt(glcd / (p3dc.col(2) - p3dc.col(3)).squaredNorm());
    // all d(i) should be equal...

    //gta = median(d);
    const double gta = d.sum() / 6;
    p3dc = gta * p3dc;

    // calc camera
    Mat Rr;
    Vec3 tt;
    getRigidTransform(pt3D, p3dc, Rr, tt);
    const Vec3 t = var * tt - Rr * mean3d;

    // output
    models->emplace_back(Rr, t, f * var2d);
  }
}

double P4PfSolver::error(const p4fSolution & model, const Vec2 & pt2D, const Vec3 & pt3D)
{
  return (pt2D - Project(model.getP(), pt3D)).norm();
}

} // namespace resection
} // namespace aliceVision
