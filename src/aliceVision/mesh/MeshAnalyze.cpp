// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "MeshAnalyze.hpp"
#include <aliceVision/structures/geometry.hpp>

MeshAnalyze::MeshAnalyze(MultiViewParams* _mp)
    : MeshClean(_mp)
{}

MeshAnalyze::~MeshAnalyze() = default;

Point3d MeshAnalyze::getCotAlphaCotBetaCotGammaForTriangle(int i)
{
    float cotalpha =
        1.0f / tan(angleBetwABandAC((*pts)[(*tris)[i].i[1]], (*pts)[(*tris)[i].i[0]], (*pts)[(*tris)[i].i[2]]) *
                   (M_PI / 180.0f));
    float cotbeta =
        1.0f / tan(angleBetwABandAC((*pts)[(*tris)[i].i[2]], (*pts)[(*tris)[i].i[0]], (*pts)[(*tris)[i].i[1]]) *
                   (M_PI / 180.0f));
    float cotgamma =
        1.0f / tan(angleBetwABandAC((*pts)[(*tris)[i].i[0]], (*pts)[(*tris)[i].i[1]], (*pts)[(*tris)[i].i[2]]) *
                   (M_PI / 180.0f));
    return Point3d(cotalpha, cotbeta, cotgamma);
}

Point2d MeshAnalyze::getCotAlphaijAndCotBetaij(int i, int j, StaticVector<int>* ptNeighPtsOrdered)
{
    int jp1 = j + 1;
    if(jp1 >= ptNeighPtsOrdered->size())
    {
        jp1 = 0;
    }
    int jm1 = j - 1;
    if(jm1 < 0)
    {
        jm1 = ptNeighPtsOrdered->size() - 1;
    }

    Point3d pti = (*pts)[i];
    Point3d ptjm1 = (*pts)[(*ptNeighPtsOrdered)[jm1]];
    Point3d ptj = (*pts)[(*ptNeighPtsOrdered)[j]];
    Point3d ptjp1 = (*pts)[(*ptNeighPtsOrdered)[jp1]];

    float cotalpha = 1.0f / tan(angleBetwABandAC(ptjm1, ptj, pti) * (M_PI / 180.0f));
    float cotbeta = 1.0f / tan(angleBetwABandAC(ptjp1, ptj, pti) * (M_PI / 180.0f));

    return Point2d(cotalpha, cotbeta);
}

float MeshAnalyze::AreaVor(int i, StaticVector<int>* ptNeighPtsOrdered)
{
    float A = 0.0f;
    for(int j = 0; j < ptNeighPtsOrdered->size(); j++)
    {
        Point2d cab = getCotAlphaijAndCotBetaij(i, j, ptNeighPtsOrdered);
        A += ((*pts)[i] - (*pts)[(*ptNeighPtsOrdered)[j]]).size2() * (cab.x + cab.y);
    }

    A *= 1.0f / 8.0f;

    return A;
}

Point3d MeshAnalyze::meanCurvVorAtPti(int i, StaticVector<int>* ptNeighPtsOrdered)
{
    Point3d meancurv = Point3d(0.0f, 0.0f, 0.0f);
    for(int j = 0; j < ptNeighPtsOrdered->size(); j++)
    {
        Point2d cab = getCotAlphaijAndCotBetaij(i, j, ptNeighPtsOrdered);
        meancurv = meancurv + ((*pts)[i] - (*pts)[(*ptNeighPtsOrdered)[j]]) * (cab.x + cab.y);
    }

    meancurv = meancurv * (1.0f / (4.0f * AreaVor(i, ptNeighPtsOrdered)));

    return meancurv;
}

double MeshAnalyze::getCotanOfAngle(Point3d& vo, Point3d& v1, Point3d& v2)
{
    /* cf. Appendix B of [Meyer et al 2002] */

    Point3d u = v1 - vo;
    Point3d v = v2 - vo;

    double udotv = dot(u, v);
    double denom = sqrt(dot(u, u) * dot(v, v) - udotv * udotv);

    /* denom can be zero if u==v.  Returning 0 is acceptable, based on
     * the callers of this function below. */
    if(denom == 0.0)
        return (0.0);

    return (udotv / denom);
}

double MeshAnalyze::getAngleFromCotan(Point3d& vo, Point3d& v1, Point3d& v2)
{
    /* cf. Appendix B and the caption of Table 1 from [Meyer et al 2002] */
    Point3d u = v1 - vo;
    Point3d v = v2 - vo;

    double udotv = dot(u, v);
    double denom = sqrt(dot(u, u) * dot(v, v) - udotv * udotv);

    // tan = denom/udotv = y/x
    return (fabs(atan2(denom, udotv)));
}

double MeshAnalyze::getRegionArea(int vertexIdInTriangle, int triId)
{
    /* cf. Section 3.3 of [Meyer et al 2002] */
    double triArea = computeTriangleArea(triId);
    if(triArea == 0.0)
        return (0.0);

    if(isTriangleObtuse(triId))
    {
        if(isTriangleAngleAtVetexObtuse(vertexIdInTriangle, triId))
            return (triArea / 2.0);

        return (triArea / 4.0);
    }
    else
    {
        Point3d A = (*pts)[(*tris)[triId].i[(vertexIdInTriangle + 0) % 3]];
        Point3d B = (*pts)[(*tris)[triId].i[(vertexIdInTriangle + 1) % 3]];
        Point3d C = (*pts)[(*tris)[triId].i[(vertexIdInTriangle + 2) % 3]];
        return (getCotanOfAngle(B, A, C) * (A - C).size2() + getCotanOfAngle(C, A, B) * (A - B).size2()) / 8.0;
    }
}

int MeshAnalyze::getVertexIdInTriangleForPtId(int ptId, int triId)
{
    for(int i = 0; i < 3; i++)
    {
        if((*tris)[triId].i[i] == ptId)
        {
            return i;
        }
    }
    return -1;
}

bool MeshAnalyze::getVertexSurfaceNormal(int ptId, Point3d& N)
{
    StaticVector<int>* ptNeighPtsOrdered = (*ptsNeighPtsOrdered)[ptId];
    StaticVector<int>* ptNeighTris = (*ptsNeighTrisSortedAsc)[ptId];
    if((isIsBoundaryPt(ptId)) || (ptNeighPtsOrdered == nullptr) || (ptNeighTris == nullptr) ||
       (ptNeighTris->size() == 0))
    {
        return false;
    }

    N = Point3d();
    for(int i = 0; i < ptNeighTris->size(); i++)
    {
        int triId = (*ptNeighTris)[i];
        N = N + computeTriangleNormal(triId);
    }
    N = N / (float)ptNeighTris->size();

    return true;
}

// gts_vertex_mean_curvature_normal [Meyer et al 2002]
bool MeshAnalyze::getVertexMeanCurvatureNormal(int ptId, Point3d& Kh)
{
    StaticVector<int>* ptNeighPtsOrdered = (*ptsNeighPtsOrdered)[ptId];
    StaticVector<int>* ptNeighTris = (*ptsNeighTrisSortedAsc)[ptId];
    if((isIsBoundaryPt(ptId)) || (ptNeighPtsOrdered == nullptr) || (ptNeighTris == nullptr))
    {
        return false;
    }

    double area = 0.0;
    for(int i = 0; i < ptNeighTris->size(); i++)
    {
        int triId = (*ptNeighTris)[i];
        int vertexIdInTriangle = getVertexIdInTriangleForPtId(ptId, triId);
        area += getRegionArea(vertexIdInTriangle, triId);
    }

    Kh = Point3d(0.0f, 0.0f, 0.0f);

    for(int i = 0; i < ptNeighPtsOrdered->size(); i++)
    {
        int ip1 = i + 1;
        if(ip1 >= ptNeighPtsOrdered->size())
        {
            ip1 = 0;
        }
        Point3d v = (*pts)[ptId];
        Point3d v1 = (*pts)[(*ptNeighPtsOrdered)[i]];
        Point3d v2 = (*pts)[(*ptNeighPtsOrdered)[ip1]];

        float temp = getCotanOfAngle(v1, v, v2);
        Kh = Kh + (v2 - v) * temp;

        temp = getCotanOfAngle(v2, v, v1);
        Kh = Kh + (v1 - v) * temp;
    }

    if(area > 0.0)
    {
        Kh = Kh / (2.0f * area);
    }
    else
    {
        return false;
    }

    return true;
}

// gts_vertex_gaussian_curvature [Meyer et al 2002]
bool MeshAnalyze::getVertexGaussianCurvature(int ptId, double& Kg)
{
    StaticVector<int>* ptNeighPtsOrdered = (*ptsNeighPtsOrdered)[ptId];
    StaticVector<int>* ptNeighTris = (*ptsNeighTrisSortedAsc)[ptId];
    if((isIsBoundaryPt(ptId)) || (ptNeighPtsOrdered == nullptr) || (ptNeighTris == nullptr))
    {
        return false;
    }

    double area = 0.0;
    for(int i = 0; i < ptNeighTris->size(); i++)
    {
        int triId = (*ptNeighTris)[i];
        int vertexIdInTriangle = getVertexIdInTriangleForPtId(ptId, triId);
        area += getRegionArea(vertexIdInTriangle, triId);
    }

    double angle_sum = 0.0;
    for(int i = 0; i < ptNeighPtsOrdered->size(); i++)
    {
        int ip1 = i + 1;
        if(ip1 >= ptNeighPtsOrdered->size())
        {
            ip1 = 0;
        }
        Point3d v = (*pts)[ptId];
        Point3d v1 = (*pts)[(*ptNeighPtsOrdered)[i]];
        Point3d v2 = (*pts)[(*ptNeighPtsOrdered)[ip1]];

        angle_sum += getAngleFromCotan(v, v1, v2);
    }

    Kg = (2.0 * M_PI - angle_sum) / area;

    return true;
}

// gts_vertex_principal_curvatures [Meyer et al 2002]
void MeshAnalyze::getVertexPrincipalCurvatures(double Kh, double Kg, double& K1, double& K2)
{
    double temp = Kh * Kh - Kg;
    if(temp < 0.0)
        temp = 0.0;
    temp = sqrt(temp);
    K1 = Kh + temp;
    K2 = Kh - temp;
}

/* from Maple */
void linsolve(double m11, double m12, double b1, double m21, double m22, double b2, double& x1, double& x2)
{
    double temp = 1.0 / (m21 * m12 - m11 * m22);
    x1 = (m12 * b2 - m22 * b1) * temp;
    x2 = (m11 * b2 - m21 * b1) * temp;
}

/* from Maple - largest eigenvector of [a b; b c] */
void eigenvector(double a, double b, double c, Point3d e)
{
    if(b == 0.0)
    {
        e.m[0] = 0.0;
    }
    else
    {
        e.m[0] = -(c - a - sqrt(c * c - 2 * a * c + a * a + 4 * b * b)) / (2 * b);
    }
    e.m[1] = 1.0;
    e.m[2] = 0.0;
}

/*TODO
//gts_vertex_principal_directions [Meyer et al 2002]
void gts_vertex_principal_directions (GtsVertex * v, GtsSurface * s, GtsVector Kh, gdouble Kg, GtsVector e1, GtsVector
e2)
{
  GtsVector N;
  gdouble normKh;
  GSList * i, * j;
  GtsVector basis1, basis2, d, eig;
  gdouble ve2, vdotN;
  gdouble aterm_da, bterm_da, cterm_da, const_da;
  gdouble aterm_db, bterm_db, cterm_db, const_db;
  gdouble a, b, c;
  gdouble K1, K2;
  gdouble *weights, *kappas, *d1s, *d2s;
  gint edge_count;
  gdouble err_e1, err_e2;
  int e;

  // compute unit normal
  normKh = sqrt (gts_vector_scalar (Kh, Kh));

  if (normKh > 0.0) {
    N[0] = Kh[0] / normKh;
    N[1] = Kh[1] / normKh;
    N[2] = Kh[2] / normKh;
  } else {
     //This vertex is a point of zero mean curvature (flat or saddle
     //point).  Compute a normal by averaging the adjacent triangles
    N[0] = N[1] = N[2] = 0.0;
    i = gts_vertex_faces (v, s, NULL);
    while (i) {
      gdouble x, y, z;
      gts_triangle_normal (GTS_TRIANGLE ((GtsFace *) i->data),
                           &x, &y, &z);
      N[0] += x;
      N[1] += y;
      N[2] += z;

      i = i->next;
    }
    g_return_if_fail (gts_vector_norm (N) > 0.0);
    gts_vector_normalize (N);
  }


  //construct a basis from N:
  //set basis1 to any component not the largest of N
  basis1[0] =  basis1[1] =  basis1[2] = 0.0;
  if (fabs (N[0]) > fabs (N[1]))
    basis1[1] = 1.0;
  else
    basis1[0] = 1.0;

  //make basis2 orthogonal to N
  gts_vector_cross (basis2, N, basis1);
  gts_vector_normalize (basis2);

  //make basis1 orthogonal to N and basis2
  gts_vector_cross (basis1, N, basis2);
  gts_vector_normalize (basis1);

  aterm_da = bterm_da = cterm_da = const_da = 0.0;
  aterm_db = bterm_db = cterm_db = const_db = 0.0;

  weights = g_malloc (sizeof (gdouble)*g_slist_length (v->segments));
  kappas = g_malloc (sizeof (gdouble)*g_slist_length (v->segments));
  d1s = g_malloc (sizeof (gdouble)*g_slist_length (v->segments));
  d2s = g_malloc (sizeof (gdouble)*g_slist_length (v->segments));
  edge_count = 0;

  i = v->segments;
  while (i) {
    GtsEdge * e;
    GtsFace * f1, * f2;
    gdouble weight, kappa, d1, d2;
    GtsVector vec_edge;

    if (! GTS_IS_EDGE (i->data)) {
      i = i->next;
      continue;
    }

    e = i->data;

    //since this vertex passed the tests in
    //gts_vertex_mean_curvature_normal(), this should be true.
    g_assert (gts_edge_face_number (e, s) == 2);

    // identify the two triangles bordering e in s
    f1 = f2 = NULL;
    j = e->triangles;
    while (j) {
      if ((! GTS_IS_FACE (j->data)) ||
          (! gts_face_has_parent_surface (GTS_FACE (j->data), s))) {
        j = j->next;
        continue;
      }
      if (f1 == NULL)
        f1 = GTS_FACE (j->data);
      else {
        f2 = GTS_FACE (j->data);
        break;
      }
      j = j->next;
    }
    g_assert (f2 != NULL);

    //We are solving for the values of the curvature tensor
    //     B = [ a b ; b c ].
    // The computations here are from section 5 of [Meyer et al 2002].
   //
     // The first step is to calculate the linear equations governing
     //the values of (a,b,c).  These can be computed by setting the
     //derivatives of the error E to zero (section 5.3).
     //
     // Since a + c = norm(Kh), we only compute the linear equations
     // for dE/da and dE/db.  (NB: [Meyer et al 2002] has the
     // equation a + b = norm(Kh), but I'm almost positive this is
     // incorrect.)
     //
     // Note that the w_ij (defined in section 5.2) are all scaled by
     // (1/8*A_mixed).  We drop this uniform scale factor because the
     // solution of the linear equations doesn't rely on it.
     //
     // The terms of the linear equations are xterm_dy with x in
     // {a,b,c} and y in {a,b}.  There are also const_dy terms that are
     // the constant factors in the equations.
     //

    // find the vector from v along edge e
    gts_vector_init (vec_edge, GTS_POINT (v),
                     GTS_POINT ((GTS_SEGMENT (e)->v1 == v) ?
                                GTS_SEGMENT (e)->v2 : GTS_SEGMENT (e)->v1));
    ve2 = gts_vector_scalar (vec_edge, vec_edge);
    vdotN = gts_vector_scalar (vec_edge, N);

    // section 5.2 - There is a typo in the computation of kappa.  The
    // edges should be x_j-x_i.
    kappa = 2.0 * vdotN / ve2;

    // section 5.2

    //I don't like performing a minimization where some of the
    // weights can be negative (as can be the case if f1 or f2 are
    // obtuse).  To ensure all-positive weights, we check for
    // obtuseness and use values similar to those in region_area().
    weight = 0.0;
    if (! triangle_obtuse(v, f1)) {
      weight += ve2 *
        cotan (gts_triangle_vertex_opposite (GTS_TRIANGLE (f1), e),
               GTS_SEGMENT (e)->v1, GTS_SEGMENT (e)->v2) / 8.0;
    } else {
      if (angle_obtuse (v, f1)) {
        weight += ve2 * gts_triangle_area (GTS_TRIANGLE (f1)) / 4.0;
      } else {
        weight += ve2 * gts_triangle_area (GTS_TRIANGLE (f1)) / 8.0;
      }
    }

    if (! triangle_obtuse(v, f2)) {
      weight += ve2 *
        cotan (gts_triangle_vertex_opposite (GTS_TRIANGLE (f2), e),
               GTS_SEGMENT (e)->v1, GTS_SEGMENT (e)->v2) / 8.0;
    } else {
      if (angle_obtuse (v, f2)) {
        weight += ve2 * gts_triangle_area (GTS_TRIANGLE (f2)) / 4.0;
      } else {
        weight += ve2 * gts_triangle_area (GTS_TRIANGLE (f2)) / 8.0;
      }
    }

    // projection of edge perpendicular to N (section 5.3)
    d[0] = vec_edge[0] - vdotN * N[0];
    d[1] = vec_edge[1] - vdotN * N[1];
    d[2] = vec_edge[2] - vdotN * N[2];
    gts_vector_normalize (d);

    // not explicit in the paper, but necessary.  Move d to 2D basis.
        d1 = gts_vector_scalar (d, basis1);
    d2 = gts_vector_scalar (d, basis2);

    // store off the curvature, direction of edge, and weights for later use
    weights[edge_count] = weight;
    kappas[edge_count] = kappa;
    d1s[edge_count] = d1;
    d2s[edge_count] = d2;
    edge_count++;

    // Finally, update the linear equations
    aterm_da += weight * d1 * d1 * d1 * d1;
    bterm_da += weight * d1 * d1 * 2 * d1 * d2;
    cterm_da += weight * d1 * d1 * d2 * d2;
    const_da += weight * d1 * d1 * (- kappa);

    aterm_db += weight * d1 * d2 * d1 * d1;
    bterm_db += weight * d1 * d2 * 2 * d1 * d2;
    cterm_db += weight * d1 * d2 * d2 * d2;
    const_db += weight * d1 * d2 * (- kappa);

    i = i->next;
  }

  // now use the identity (Section 5.3) a + c = |Kh| = 2 * kappa_h
  aterm_da -= cterm_da;
  const_da += cterm_da * normKh;

  aterm_db -= cterm_db;
  const_db += cterm_db * normKh;

  // check for solvability of the linear system
  if (((aterm_da * bterm_db - aterm_db * bterm_da) != 0.0) &&
      ((const_da != 0.0) || (const_db != 0.0))) {
    linsolve (aterm_da, bterm_da, -const_da,
              aterm_db, bterm_db, -const_db,
              &a, &b);

    c = normKh - a;

    eigenvector (a, b, c, eig);
  } else {
    // region of v is planar
    eig[0] = 1.0;
    eig[1] = 0.0;
  }

  // Although the eigenvectors of B are good estimates of the
  // principal directions, it seems that which one is attached to
  // which curvature direction is a bit arbitrary.  This may be a bug
  // in my implementation, or just a side-effect of the inaccuracy of
  // B due to the discrete nature of the sampling.
  //
  // To overcome this behavior, we'll evaluate which assignment best
  // matches the given eigenvectors by comparing the curvature
  // estimates computed above and the curvatures calculated from the
  // discrete differential operators.

  gts_vertex_principal_curvatures (0.5 * normKh, Kg, &K1, &K2);

  err_e1 = err_e2 = 0.0;
  // loop through the values previously saved
  for (e = 0; e < edge_count; e++) {
    gdouble weight, kappa, d1, d2;
    gdouble temp1, temp2;
    gdouble delta;

    weight = weights[e];
    kappa = kappas[e];
    d1 = d1s[e];
    d2 = d2s[e];

    temp1 = fabs (eig[0] * d1 + eig[1] * d2);
    temp1 = temp1 * temp1;
    temp2 = fabs (eig[1] * d1 - eig[0] * d2);
    temp2 = temp2 * temp2;

    // err_e1 is for K1 associated with e1
    delta = K1 * temp1 + K2 * temp2 - kappa;
    err_e1 += weight * delta * delta;

    // err_e2 is for K1 associated with e2
    delta = K2 * temp1 + K1 * temp2 - kappa;
    err_e2 += weight * delta * delta;
  }
  g_free (weights);
  g_free (kappas);
  g_free (d1s);
  g_free (d2s);

  // rotate eig by a right angle if that would decrease the error
  if (err_e2 < err_e1) {
    gdouble temp = eig[0];

    eig[0] = eig[1];
    eig[1] = -temp;
  }

  e1[0] = eig[0] * basis1[0] + eig[1] * basis2[0];
  e1[1] = eig[0] * basis1[1] + eig[1] * basis2[1];
  e1[2] = eig[0] * basis1[2] + eig[1] * basis2[2];
  gts_vector_normalize (e1);

  // make N,e1,e2 a right handed coordinate sytem
  gts_vector_cross (e2, N, e1);
  gts_vector_normalize (e2);
}
*/

// othake et al 00 Polyhedral Surface Smoothing with Simultaneous Mesh Regularization
// page 3 eq (3)
// kobbelt kampagna 98 Interactive Multi-Resolution Modeling on Arbitrary Meshes
// page 5 - U1 - laplacian is obtained wnen apply to origina pts , U2 - bi-laplacian is obtained when apply to laplacian
// pts

bool MeshAnalyze::applyLaplacianOperator(int ptId, StaticVector<Point3d>* ptsToApplyLaplacianOp, Point3d& ln)
{
    StaticVector<int>* ptNeighPtsOrdered = (*ptsNeighPtsOrdered)[ptId];
    if(ptNeighPtsOrdered == nullptr)
    {
        return false;
    }

    ln = Point3d(0.0f, 0.0f, 0.0f);
    for(int i = 0; i < ptNeighPtsOrdered->size(); i++)
    {
        Point3d npt = (*ptsToApplyLaplacianOp)[(*ptNeighPtsOrdered)[i]];

        if((npt.x == 0.0f) && (npt.y == 0.0f) && (npt.z == 0.0f))
        {
            // printf("zero neighb pt\n");
            return false;
        }
        ln = ln + npt;
    }
    ln = (ln / (float)ptNeighPtsOrdered->size()) - (*ptsToApplyLaplacianOp)[ptId];

    Point3d n = ln;
    float d = n.size();
    n = n.normalize();
    if(std::isnan(d) || std::isnan(n.x) || std::isnan(n.y) || std::isnan(n.z) || (d != d) || (n.x != n.x) ||
       (n.y != n.y) || (n.z != n.z)) // check if is not NaN
    {
        // printf("nan\n");
        return false;
    }

    if(std::isnan(d) || std::isnan(n.x) || std::isnan(n.y) || std::isnan(n.z) || (d != d) || (n.x != n.x) ||
       (n.y != n.y) || (n.z != n.z)) // check if is not NaN
    {
        // printf("nan\n");
        return false;
    }

    return true;
}

// othake et al 00 Polyhedral Surface Smoothing with Simultaneous Mesh Regularization
// page 3 eq (3)
bool MeshAnalyze::getLaplacianSmoothingVector(int ptId, Point3d& ln)
{
    return applyLaplacianOperator(ptId, pts, ln);
}

// kobbelt kampagna 98 Interactive Multi-Resolution Modeling on Arbitrary Meshes
// page 5 - U1 - laplacian is obtained wnen apply to origina pts , U2 - bi-laplacian is obtained when apply to laplacian
// pts
bool MeshAnalyze::getBiLaplacianSmoothingVector(int ptId, StaticVector<Point3d>* ptsLaplacian, Point3d& tp)
{
    if(applyLaplacianOperator(ptId, ptsLaplacian, tp))
    {
        StaticVector<int>* ptNeighPtsOrdered = (*ptsNeighPtsOrdered)[ptId];
        StaticVector<int>* ptNeighTris = (*ptsNeighTrisSortedAsc)[ptId];
        if((ptNeighPtsOrdered == nullptr) || (ptNeighTris == nullptr))
        {
            return false;
        }

        float sum = 0.0f;
        for(int i = 0; i < sizeOfStaticVector<int>(ptNeighPtsOrdered); i++)
        {
            int neighValence = sizeOfStaticVector<int>((*ptsNeighPtsOrdered)[(*ptNeighPtsOrdered)[i]]);
            if(neighValence > 0)
            {
                sum += 1.0f / (float)neighValence;
            }
        }
        float v = 1.0f + (1.0f / (float)sizeOfStaticVector<int>(ptNeighPtsOrdered)) * sum;

        tp = Point3d(0.0f, 0.0f, 0.0f) - tp * (1.0f / v);

        Point3d n = tp;
        float d = n.size();
        n = n.normalize();
        if(std::isnan(d) || std::isnan(n.x) || std::isnan(n.y) || std::isnan(n.z) || (d != d) || (n.x != n.x) ||
           (n.y != n.y) || (n.z != n.z)) // check if is not NaN
        {
            return false;
        }
        // page 6 eq (8)

        return true;
    }

    return false;
}

// kobbelt kampagna 98 Interactive Multi-Resolution Modeling on Arbitrary Meshes
// page 5 - U1 - laplacian is obtained wnen apply to origina pts , U2 - bi-laplacian is obtained when apply to laplacian
// pts
bool MeshAnalyze::getBiLaplacianSmoothingVectorAndPrincipalCurvatures(
    int ptId, StaticVector<Point3d>* ptsLaplacian, Point3d& smoothingVector, Point3d& smoothingVectorNormalized,
    Point3d& normalVectorNormalized, double& smoothingVectorSize, double& K1, double& K2, double& area,
    double& avNeighEdegeLenth)
{
    if(applyLaplacianOperator(ptId, ptsLaplacian, smoothingVector))
    {
        StaticVector<int>* ptNeighPtsOrdered = (*ptsNeighPtsOrdered)[ptId];
        StaticVector<int>* ptNeighTris = (*ptsNeighTrisSortedAsc)[ptId];
        if((ptNeighPtsOrdered == nullptr) || (ptNeighTris == nullptr))
        {
            return false;
        }

        double sum = 0.0;
        for(int i = 0; i < sizeOfStaticVector<int>(ptNeighPtsOrdered); i++)
        {
            int neighValence = sizeOfStaticVector<int>((*ptsNeighPtsOrdered)[(*ptNeighPtsOrdered)[i]]);
            if(neighValence > 0)
            {
                sum += 1.0 / (double)neighValence;
            }
        }
        double v = 1.0 + (1.0 / (double)sizeOfStaticVector<int>(ptNeighPtsOrdered)) * sum;

        smoothingVector = Point3d(0.0, 0.0, 0.0) - smoothingVector * (1.0 / v);

        smoothingVectorNormalized = smoothingVector;
        smoothingVectorSize = smoothingVectorNormalized.size();
        smoothingVectorNormalized = smoothingVectorNormalized.normalize();
        if(std::isnan(smoothingVector.x) || std::isnan(smoothingVector.y) || std::isnan(smoothingVector.z) ||
           std::isnan(smoothingVectorSize) || std::isnan(smoothingVectorNormalized.x) ||
           std::isnan(smoothingVectorNormalized.y) || std::isnan(smoothingVectorNormalized.z) ||
           (smoothingVectorSize != smoothingVectorSize) || (smoothingVector.x != smoothingVector.x) ||
           (smoothingVector.y != smoothingVector.y) || (smoothingVector.z != smoothingVector.z) ||
           (smoothingVectorNormalized.x != smoothingVectorNormalized.x) ||
           (smoothingVectorNormalized.y != smoothingVectorNormalized.y) ||
           (smoothingVectorNormalized.z != smoothingVectorNormalized.z)) // check if is not NaN
        {
            return false;
        }
        // page 6 eq (8)

        // compute K1, K2
        area = 0.0;
        normalVectorNormalized = Point3d(0.0f, 0.0f, 0.0f);
        for(int i = 0; i < ptNeighTris->size(); i++)
        {
            int triId = (*ptNeighTris)[i];
            int vertexIdInTriangle = getVertexIdInTriangleForPtId(ptId, triId);
            area += getRegionArea(vertexIdInTriangle, triId);
            normalVectorNormalized = normalVectorNormalized + computeTriangleNormal(triId);
        }
        normalVectorNormalized = normalVectorNormalized / (double)ptNeighTris->size();

        double angle_sum = 0.0;
        Point3d KhVect = Point3d(0.0f, 0.0f, 0.0f);
        avNeighEdegeLenth = 0.0f;
        for(int i = 0; i < ptNeighPtsOrdered->size(); i++)
        {
            int ip1 = i + 1;
            if(ip1 >= ptNeighPtsOrdered->size())
            {
                ip1 = 0;
            }
            Point3d v = (*pts)[ptId];
            Point3d v1 = (*pts)[(*ptNeighPtsOrdered)[i]];
            Point3d v2 = (*pts)[(*ptNeighPtsOrdered)[ip1]];

            double temp = getCotanOfAngle(v1, v, v2);
            KhVect = KhVect + (v2 - v) * temp;

            temp = getCotanOfAngle(v2, v, v1);
            KhVect = KhVect + (v1 - v) * temp;

            angle_sum += getAngleFromCotan(v, v1, v2);

            avNeighEdegeLenth += (v - v1).size();
        }
        avNeighEdegeLenth /= (double)ptNeighPtsOrdered->size();

        if(area > 0.0)
        {
            KhVect = KhVect / (2.0f * area);
        }
        else
        {
            return false;
        }

        double Kh = KhVect.size();
        double Kg = (2.0 * M_PI - angle_sum) / area;

        getVertexPrincipalCurvatures(Kh, Kg, K1, K2);

        if(std::isnan(K1) || std::isnan(K2) || (K1 != K1) || (K2 != K2)) // check if is not NaN
        {
            return false;
        }

        return true;
    }

    return false;
}

// othake et al 00 Polyhedral Surface Smoothing with Simultaneous Mesh Regularization
// page 6 eq (13)
bool MeshAnalyze::getMeanCurvAndLaplacianSmoothing(int ptId, Point3d& F, float epsilon)
{
    Point3d Hn;
    if(!getVertexMeanCurvatureNormal(ptId, Hn))
    {
        return false;
    }

    Point3d U0;

    if(!getLaplacianSmoothingVector(ptId, U0))
    {
        return false;
    }

    Point3d m = U0.normalize();
    float absH = Hn.size();

    float cosTheta = dot(m, Hn) / absH;

    if(cosTheta > epsilon)
    {
        F = (m * absH) / cosTheta;
        return true;
    }

    if(cosTheta < -epsilon)
    {
        F = Hn * 2.0f - (m * absH) / cosTheta;
        return true;
    }

    if(fabs(cosTheta) <= epsilon)
    {
        F = Point3d(0.0f, 0.0f, 0.0f);
        return true;
    }

    return false;
}
