/*
 * Copyright (C) 2018 Erion Plaku
 * All Rights Reserved
 * 
 *       Created by Erion Plaku
 *       Computational Robotics Group
 *       Department of Electrical Engineering and Computer Science
 *       Catholic University of America
 *
 *       www.robotmotionplanning.org
 *
 * Code should not be distributed or used without written permission from the
 * copyright holder.
 */

#ifndef Antipatrea__Geometry_HPP_
#define Antipatrea__Geometry_HPP_

#include <cstdio>
#include <vector>

namespace Antipatrea
{
static inline void TriangleAsPolygon2D(const double tA[2], const double tB[2], const double tC[2], double poly[6])
{
  poly[0] = tA[0];
  poly[1] = tA[1];
  poly[2] = tB[0];
  poly[3] = tB[1];
  poly[4] = tC[0];
  poly[5] = tC[1];
}

static inline void BoxAsPolygon2D(const double minx, const double miny, const double maxx, const double maxy, double poly[8])
{
  poly[0] = minx;
  poly[1] = miny;
  poly[2] = maxx;
  poly[3] = miny;
  poly[4] = maxx;
  poly[5] = maxy;
  poly[6] = minx;
  poly[7] = maxy;
}

static inline void BoxAsPolygon2D(const double min[2], const double max[2], double poly[8])
{
  BoxAsPolygon2D(min[0], min[1], max[0], max[1], poly);
}

static inline void QuadAsPolygon2D(const double qA[2], const double qB[2], const double qC[2], const double qD[2], double poly[8])
{
  poly[0] = qA[0];
  poly[1] = qA[1];
  poly[2] = qB[0];
  poly[3] = qB[1];
  poly[4] = qC[0];
  poly[5] = qC[1];
  poly[6] = qD[0];
  poly[7] = qD[1];
}

void CircleAsPolygon2D(const double cx, const double cy, const double r, const int n, double poly[]);

double DistanceSquaredPointLine2D(const double p[2], const double s0[2], const double s1[2], double pmin[2]);

double DistanceSquaredPointSegment2D(const double p[2], const double s0[2], const double s1[2], double pmin[2]);

double DistanceSquaredPointPolygon2D(const double p[2], const int n, const double poly[], double pmin[2]);

double DistanceSquaredSegments2D(const double p1[2], const double p2[2], const double p3[2], const double p4[2], double pmin1[2], double pmin2[2]);

double DistanceSquaredSegmentPolygon2D(const double p1[2], const double p2[2], const int n, const double poly[], double pmin1[2], double pmin2[2]);

double DistanceSquaredPolygons2D(const int n1, const double poly1[], const int n2, const double poly2[], double pmin1[2], double pmin2[2]);

bool IntersectLines2D(const double x1, const double y1, const double x2, const double y2, const double x3, const double y3, const double x4, const double y4,
                      double &x, double &y, double &mua, double &mub);

static inline bool IntersectLineRay2D(const double x1, const double y1, const double x2, const double y2, const double x3, const double y3, const double x4,
                                      const double y4, double &x, double &y)
{
  double mua, mub;

  return IntersectLines2D(x1, y1, x2, y2, x3, y3, x4, y4, x, y, mua, mub) && mub >= 0;
}

static inline bool IntersectLineSegment2D(const double x1, const double y1, const double x2, const double y2, const double x3, const double y3, const double x4,
                                          const double y4, double &x, double &y)
{
  double mua, mub;

  return IntersectLines2D(x1, y1, x2, y2, x3, y3, x4, y4, x, y, mua, mub) && mub >= 0 && mub <= 1;
}

static inline bool IntersectRays2D(const double x1, const double y1, const double x2, const double y2, const double x3, const double y3, const double x4,
                                   const double y4, double &x, double &y)
{
  double mua, mub;

  return IntersectLines2D(x1, y1, x2, y2, x3, y3, x4, y4, x, y, mua, mub) && mua >= 0 && mub >= 0;
}

static inline bool IntersectRaySegment2D(const double x1, const double y1, const double x2, const double y2, const double x3, const double y3, const double x4,
                                         const double y4, double &x, double &y)
{
  double mua, mub;

  return IntersectLines2D(x1, y1, x2, y2, x3, y3, x4, y4, x, y, mua, mub) && mua >= 0 && mub >= 0 && mub <= 1;
}

static inline bool IntersectSegments2D(const double x1, const double y1, const double x2, const double y2, const double x3, const double y3, const double x4,
                                       const double y4, double &x, double &y)
{
  double mua, mub;

  return IntersectLines2D(x1, y1, x2, y2, x3, y3, x4, y4, x, y, mua, mub) && mua >= 0 && mua <= 1 && mub >= 0 && mub <= 1;
}

static inline bool IntersectSegments2D(const double p1[2], const double p2[2], const double p3[2], const double p4[2], double p[])
{
  return IntersectSegments2D(p1[0], p1[1], p2[0], p2[1], p3[0], p3[1], p4[0], p4[1], p[0], p[1]);
}

bool IntersectSegments2D(const double x1, const double y1, const double x2, const double y2, const double x3, const double y3, const double x4,
                         const double y4);

static inline bool IntersectSegments2D(const double p1[2], const double p2[2], const double p3[2], const double p4[2])
{
  return IntersectSegments2D(p1[0], p1[1], p2[0], p2[1], p3[0], p3[1], p4[0], p4[1]);
}

bool IntersectSegmentPolygon2D(const double p0[2], const double p1[2], const int n, const double poly[]);

bool IntersectPolygons2D(const int n1, const double poly1[], const int n2, const double poly2[]);

int IntersectLineBox2D(const double x1, const double y1, const double x2, const double y2, const double minx, const double miny, const double maxx,
                       const double maxy, double &interx1, double &intery1, double &interx2, double &intery2);

static inline double Turn2D(const double p0[2], const double p1[2], const double p2[2])
{
  return (p2[0] - p0[0]) * (p1[1] - p0[1]) - (p1[0] - p0[0]) * (p2[1] - p0[1]);
}

static inline bool IsPointLeftOfLine2D(const double p[2], const double p0[2], const double p1[2])
{
  return Turn2D(p0, p1, p) <= 0.0;
}

static inline bool IsPointInsideTriangle2D(const double p[2], const double tA[2], const double tB[2], const double tC[2])
{
  return IsPointLeftOfLine2D(p, tA, tB) && IsPointLeftOfLine2D(p, tB, tC) && IsPointLeftOfLine2D(p, tC, tA);
}

static inline bool IsPointInsideBox2D(const double p[2], const double min[2], const double max[2])
{
  return p[0] >= min[0] && p[0] <= max[0] && p[1] >= min[1] && p[1] <= max[1];
}

bool IsPointInsideConvexPolygon2D(const double p[2], const int n, const double poly[]);

bool IsPointInsidePolygon2D(const double p[2], const int n, const double poly[]);

static inline bool IsPointInsideCircle2D(const double p[2], const double cx, const double cy, const double r)
{
  return ((cx - p[0]) * (cx - p[0]) + (cy - p[1]) * (cy - p[1])) <= r * r;
}

static inline bool IsBoxInsideBox2D(const double min[2], const double max[2], const double min2[2], const double max2[2])
{
  return min[0] >= min2[0] && max[0] <= max2[0] && min[1] >= min2[1] && max[1] <= max2[1];
}

bool IsPolygonInsideBox2D(const int n, const double poly[], const double min[2], const double max[2]);

bool IsPolygonInsideConvexPolygon2D(const int n1, const double poly1[], const int n2, const double poly2[]);

bool IsPolygonInsidePolygon2D(const int n1, const double poly1[], const int n2, const double poly2[], const bool checkIntersection = true);

bool IsPolygonInsideCircle2D(const int n, const double poly[], const double cx, const double cy, const double r);

static inline bool CollisionSegmentConvexPolygon2D(const double p0[2], const double p1[2], const int n, const double poly[])
{
  return IsPointInsideConvexPolygon2D(p0, n, poly) || IsPointInsideConvexPolygon2D(p1, n, poly) || IntersectSegmentPolygon2D(p0, p1, n, poly);
}

static inline bool CollisionPolygonConvexPolygon2D(const int n1, const double poly1[], const int n2, const double poly2[])
{
  return IsPointInsideConvexPolygon2D(poly1, n2, poly2) || IsPointInsidePolygon2D(poly2, n1, poly1) || IntersectPolygons2D(n1, poly1, n2, poly2);
}

static inline bool CollisionConvexPolygons2D(const int n1, const double poly1[], const int n2, const double poly2[])
{
  return IsPointInsideConvexPolygon2D(poly1, n2, poly2) || IsPointInsideConvexPolygon2D(poly2, n1, poly1) || IntersectPolygons2D(n1, poly1, n2, poly2);
}

static inline bool CollisionSegmentPolygon2D(const double p0[2], const double p1[2], const int n, const double poly[])
{
  return IsPointInsidePolygon2D(p0, n, poly) || IsPointInsidePolygon2D(p1, n, poly) || IntersectSegmentPolygon2D(p0, p1, n, poly);
}

static inline bool CollisionPolygons2D(const int n1, const double poly1[], const int n2, const double poly2[])
{
  return IsPointInsidePolygon2D(poly1, n2, poly2) || IsPointInsidePolygon2D(poly2, n1, poly1) || IntersectPolygons2D(n1, poly1, n2, poly2);
}

static inline bool CollisionBoxes2D(const double min1[2], const double max1[2], const double min2[2], const double max2[2])
{
  return !(max2[0] < min1[0] || max1[0] < min2[0] || max2[1] < min1[1] || max1[1] < min2[1]);
}

double SignedAreaPolygon2D(const int n, const double poly[]);

void BoundingBoxPolygon2D(const int n, const double poly[], double min[2], double max[2]);

static inline void BoundingBoxCircle2D(const double center[2], const double r, double min[2], double max[2])
{
  min[0] = center[0] - r;
  min[1] = center[1] - r;
  max[0] = center[0] + r;
  max[1] = center[1] + r;
}

void ChangeOrientationPolygon2D(const int n, double poly[]);

bool IsConvexPolygonCCW2D(const int n, const double poly[]);

static inline bool IsPolygonCCW2D(const int n, const double poly[])
{
  return SignedAreaPolygon2D(n, poly) >= 0.0;
}

static inline void MakeConvexPolygonCCW2D(const int n, double poly[])
{
  if (!IsConvexPolygonCCW2D(n, poly))
    ChangeOrientationPolygon2D(n, poly);
}

static inline void MakePolygonCCW2D(const int n, double poly[])
{
  if (!IsPolygonCCW2D(n, poly))
    ChangeOrientationPolygon2D(n, poly);
}

bool IsPolygonConvex2D(const int n, const double poly[]);

void CentroidPolygon2D(const int n, const double poly[], double centroid[2]);

int TriangulateConvexPolygon2D(const int n, const double poly[], int triangles[]);

void SampleRandomPointInsideBox2D(const double min[2], const double max[2], double p[2]);

void SampleRandomPointInsideTriangle2D(const double tA[2], const double tB[2], const double tC[2], double p[2]);

void SampleRandomPointInsideConvexPolygon2D(const int n, const double poly[], double p[2], const double min[2] = NULL, const double max[2] = NULL);

void SampleRandomPointInsidePolygon2D(const int n, const double poly[], double p[2], const double min[2] = NULL, const double max[2] = NULL);

void SampleRandomPointInsideCircle2D(const double center[2], const double r, double p[2]);

    void SampleRandomPointInsideAnnulus2D(const double center[2], const double r1,  const double r2, double p[2]);
    

void ApplyTransToPolygon2D(const double T[], const int n, const double origPoly[], double newPoly[]);

void ApplyRotToPolygon2D(const double R[], const int n, const double origPoly[], double newPoly[]);

void ApplyTransRotToPolygon2D(const double TR[], const int n, const double origPoly[], double newPoly[]);

void StabSkeleton2D(const int n, const double skeleton[], const double dist, std::vector<double> &newsk);

bool ShiftSkeleton2D(const int n, const double skel[], const double thick, std::vector<double> &poly);

bool FromSkeletonToPolygon2D(const int n, const double skeleton[], const double thick, std::vector<double> &poly);

void GenerateArcAsPolygon2D(const double x, const double y, const double r, const double thetaStart, const double thetaInc, const int nrPtsSkel,
                            const double thick, std::vector<double> &poly);

void GenerateRadialPatternAsPolygons2D(const double x, const double y, const double rmin, const double rmax, const double thetaStart, const double thetaInc,
                                       const int nrRays, const double thick, std::vector<std::vector<double> *> &polys);

void ScaleInterval(const double s, double &xmin, double &xmax);

void RegularizePointsAlongPath(const int n, const double pts[], const double dsep, const int dim, std::vector<double> &newPts);

void RemoveSmallTriangles(std::vector<double> &triVertices, std::vector<int> &triIndices, std::vector<int> &triNeighs, const double minArea);

double DistanceSquaredPointSegment3D(const double p[], const double s1[], const double s2[]);

double DistanceSquaredPointSegment(const int dim, const double p[], const double s1[], const double s2[]);
}

#endif
