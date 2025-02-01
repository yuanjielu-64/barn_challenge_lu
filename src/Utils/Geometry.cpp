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
#include "Utils/Geometry.hpp"
#include "Utils/Algebra.hpp"
#include "Utils/Algebra2D.hpp"
#include "Utils/Algebra3D.hpp"
#include "Utils/Constants.hpp"
#include "Utils/PrintMsg.hpp"
#include "Utils/PseudoRandom.hpp"

namespace Antipatrea {

    void CircleAsPolygon2D(const double cx, const double cy, const double r, const int n, double poly[]) {
        const double theta = 2 * M_PI / n;
        const double cos_theta = cos(theta);
        const double sin_theta = sin(theta);
        double x = 1;
        double y = 0;

        for (int j = 0; j < n; ++j) {
            poly[2 * j] = cx + r * x;
            poly[2 * j + 1] = cy + r * y;

            const double tmp = x;
            x = cos_theta * tmp - sin_theta * y;
            y = sin_theta * tmp + cos_theta * y;
        }
    }

    double DistanceSquaredPointLine2D(const double p[2], const double s0[2], const double s1[2], double pmin[]) {
        const double vx = s1[0] - s0[0];
        const double vy = s1[1] - s0[1];
        const double a = vx * (s0[1] - p[1]) - vy * (s0[0] - p[0]);
        const double b = a / (vx * vx + vy * vy);

        pmin[0] = p[0] - b * vy;
        pmin[1] = p[1] + b * vx;

        return a * b;
    }

    double DistanceSquaredPointSegment2D(const double p[2], const double s0[2], const double s1[2], double pmin[2]) {
        double a, b;
        const double vx = s1[0] - s0[0];
        const double vy = s1[1] - s0[1];

        if ((a = (vx * (p[0] - s0[0]) + vy * (p[1] - s0[1]))) <= 0) {
            pmin[0] = s0[0];
            pmin[1] = s0[1];
        } else if ((b = (vx * vx + vy * vy)) <= a) {
            pmin[0] = s1[0];
            pmin[1] = s1[1];
        } else {
            a /= b;
            pmin[0] = s0[0] + a * vx;
            pmin[1] = s0[1] + a * vy;
        }

        return (p[0] - pmin[0]) * (p[0] - pmin[0]) + (p[1] - pmin[1]) * (p[1] - pmin[1]);
    }

    double DistanceSquaredPointPolygon2D(const double p[2], const int n, const double poly[], double pmin[2]) {
        double dmin = DistanceSquaredPointSegment2D(p, &poly[2 * n - 2], &(poly[0]), pmin);
        double d;
        double ptmp[2];
        int i;

        for (i = 0; i < n - 1; ++i)
            if ((d = DistanceSquaredPointSegment2D(p, &poly[2 * i], &poly[2 * i + 2], ptmp)) < dmin) {
                pmin[0] = ptmp[0];
                pmin[1] = ptmp[1];
                dmin = d;
            }
        return dmin;
    }

    double DistanceSquaredSegments2D(const double p1[2], const double p2[2], const double p3[2], const double p4[2],
                                     double pmin1[2], double pmin2[2]) {
        const double u[] = {p2[0] - p1[0], p2[1] - p1[1]};
        const double v[] = {p4[0] - p3[0], p4[1] - p3[1]};
        const double w[] = {p1[0] - p3[0], p1[1] - p3[1]};
        const double a = u[0] * u[0] + u[1] * u[1];
        const double b = u[0] * v[0] + u[1] * v[1];
        const double c = v[0] * v[0] + v[1] * v[1];
        const double d = u[0] * w[0] + u[1] * w[1];
        const double e = v[0] * w[0] + v[1] * w[1];
        const double D = a * c - b * b; // always >= 0
        double sc, sN, sD = D;          // sc = sN / sD, default sD = D >= 0
        double tc, tN, tD = D;          // tc = tN / tD, default tD = D >= 0

        // compute the line parameters of the two closest points
        if (D < Constants::EPSILON) {           // the lines are almost parallel
            sN = 0.0; // force using point P0 on segment S1
            sD = 1.0; // to prevent possible division by 0.0 later
            tN = e;
            tD = c;
        } else { // get the closest points on the infinite lines
            sN = (b * e - c * d);
            tN = (a * e - b * d);
            if (sN < 0.0) { // sc < 0 => the s=0 edge is visible
                sN = 0.0;
                tN = e;
                tD = c;
            } else if (sN > sD) { // sc > 1 => the s=1 edge is visible
                sN = sD;
                tN = e + b;
                tD = c;
            }
        }

        if (tN < 0.0) { // tc < 0 => the t=0 edge is visible
            tN = 0.0;
            // recompute sc for this edge
            if (-d < 0.0)
                sN = 0.0;
            else if (-d > a)
                sN = sD;
            else {
                sN = -d;
                sD = a;
            }
        } else if (tN > tD) { // tc > 1 => the t=1 edge is visible
            tN = tD;
            // recompute sc for this edge
            if ((-d + b) < 0.0)
                sN = 0;
            else if ((-d + b) > a)
                sN = sD;
            else {
                sN = (-d + b);
                sD = a;
            }
        }
        // finally do the division to get sc and tc
        sc = (fabs(sN) < Constants::EPSILON ? 0.0 : sN / sD);
        tc = (fabs(tN) < Constants::EPSILON ? 0.0 : tN / tD);

        pmin1[0] = p1[0] + sc * u[0];
        pmin1[1] = p1[1] + sc * u[1];

        pmin2[0] = p3[0] + tc * v[0];
        pmin2[1] = p3[1] + tc * v[1];

        const double dx = pmin2[0] - pmin1[0];
        const double dy = pmin2[1] - pmin1[1];

        return dx * dx + dy * dy;
    }

    double DistanceSquaredSegmentPolygon2D(const double p1[2], const double p2[2], const int n, const double poly[],
                                           double pmin1[2], double pmin2[2]) {
        double ptmp1[2], ptmp2[2];

        double dmin = DistanceSquaredSegments2D(p1, p2, &poly[2 * n - 2], &poly[0], pmin1, pmin2);
        double d;

        for (int i = 0; i < n - 1; ++i) {
            if ((d = DistanceSquaredSegments2D(p1, p2, &poly[2 * i], &poly[2 * i + 2], ptmp1, ptmp2)) < dmin) {
                dmin = d;
                pmin1[0] = ptmp1[0];
                pmin1[1] = ptmp1[1];
                pmin2[0] = ptmp2[0];
                pmin2[1] = ptmp2[1];
            }
        }

        return dmin;
    }

    double
    DistanceSquaredPolygons2D(const int n1, const double poly1[], const int n2, const double poly2[], double pmin1[2],
                              double pmin2[2]) {
        double ptmp1[2], ptmp2[2];

        double dmin = DistanceSquaredSegmentPolygon2D(&poly1[2 * n1 - 2], &poly1[0], n2, poly2, pmin1, pmin2);
        double d;

        for (int i = 0; i < n1 - 1; ++i)
            if ((d = DistanceSquaredSegmentPolygon2D(&poly1[2 * i], &poly1[2 * i + 2], n2, poly2, ptmp1, ptmp2)) <
                dmin) {
                dmin = d;
                pmin1[0] = ptmp1[0];
                pmin1[1] = ptmp1[1];
                pmin2[0] = ptmp2[0];
                pmin2[1] = ptmp2[1];
            }
        return dmin;
    }

    bool IntersectLines2D(const double x1, const double y1, const double x2, const double y2, const double x3,
                          const double y3, const double x4, const double y4,
                          double &x, double &y, double &mua, double &mub) {
        const double denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1);

        if (fabs(denom) < Constants::EPSILON)
            return false;
        mua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / denom;
        mub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / denom;
        x = x1 + mua * (x2 - x1);
        y = y1 + mua * (y2 - y1);

        return true;

        //	const double numera = (x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3);
        //	const double numerb = (x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3);
        /*
            // Are the line coincident? or para
            if (fabs(numera) < Constants::EPSILON &&
                fabs(numerb) < Constants::EPSILON &&
                fabs(denom) < Constants::EPSILON)
            {
                *x = (x1 + x2) / 2;
                *y = (y1 + y2) / 2;
                return false;
            }

            // Are the line parallel
            if (fabs(denom) < Constants::EPSILON)
            {
                *x = *y = 0;
                return false;
            }
            */

        // Is the intersection along the segments
        //*mua = numera / denom;
        //*mub = numerb / denom;
    }

    bool IntersectSegments2D(const double x1, const double y1, const double x2, const double y2, const double x3,
                             const double y3, const double x4, const double y4) {
        double Cx, Ay, By, Cy, d, e, f;
        double x1lo, x1hi, y1lo, y1hi;
        double Ax = x2 - x1;
        double Bx = x3 - x4;

        if (Ax < 0) {
            x1lo = x2;
            x1hi = x1;
        } else {
            x1hi = x2;
            x1lo = x1;
        }

        if (Bx > 0) {
            if (x1hi < x4 || x3 < x1lo)
                return false;
        } else {
            if (x1hi < x3 || x4 < x1lo)
                return false;
        }

        Ay = y2 - y1;
        By = y3 - y4;

        /* Y bound box test*/
        if (Ay < 0) {
            y1lo = y2;
            y1hi = y1;
        } else {
            y1hi = y2;
            y1lo = y1;
        }

        if (By > 0) {
            if (y1hi < y4 || y3 < y1lo)
                return false;
        } else if (y1hi < y3 || y4 < y1lo)
            return false;

        f = Ay * Bx - Ax * By; /* both denominator*/
        if (f == 0)
            return false;

        Cx = x1 - x3;
        Cy = y1 - y3;

        d = By * Cx - Bx * Cy; /* alpha numerator*/
        /* alpha tests*/
        if (f > 0) {
            if (d < 0 || d > f)
                return false;
        } else if (d > 0 || d < f)
            return false;

        e = Ax * Cy - Ay * Cx; /* beta numerator*/
        /* beta tests*/
        if (f > 0) {
            if (e < 0 || e > f)
                return false;
        } else if (e > 0 || e < f)
            return false;

        //    printf("inter = %f %f\n", x1 + (d/f) * (x2 - x1), y1 + (d/f) * (y2 - y1));

        return true;
    }

    bool IntersectSegmentPolygon2D(const double p0[2], const double p1[2], const int n, const double poly[]) {
        for (int i = 0; i < n - 1; ++i)
            if (IntersectSegments2D(p0, p1, &(poly[2 * i]), &(poly[2 * i + 2])))
                return true;
        return IntersectSegments2D(p0, p1, &(poly[2 * n - 2]), &(poly[0]));
    }

    bool IntersectPolygons2D(const int n1, const double poly1[], const int n2, const double poly2[]) {
        for (int i = 0; i < n1 - 1; ++i)
            if (IntersectSegmentPolygon2D(&(poly1[2 * i]), &(poly1[2 * i + 2]), n2, poly2))
                return true;
        return IntersectSegmentPolygon2D(&(poly1[2 * n1 - 2]), &(poly1[0]), n2, poly2);
    }

    int IntersectLineBox2D(const double x1, const double y1, const double x2, const double y2, const double minx,
                           const double miny, const double maxx,
                           const double maxy, double &interx1, double &intery1, double &interx2, double &intery2) {
        double p[4] = {HUGE_VAL, HUGE_VAL, HUGE_VAL, HUGE_VAL};
        int count = 0;

        if (IntersectLineSegment2D(x1, y1, x2, y2, minx, miny, maxx, miny, p[0], p[1]))
            ++count;

        if (count < 2 && IntersectLineSegment2D(x1, y1, x2, y2, maxx, miny, maxx, maxy, p[2 * count], p[2 * count + 1]))
            ++count;

        if (count < 2 && IntersectLineSegment2D(x1, y1, x2, y2, maxx, maxy, minx, maxy, p[2 * count], p[2 * count + 1]))
            ++count;

        if (count < 2 && IntersectLineSegment2D(x1, y1, x2, y2, minx, maxy, minx, miny, p[2 * count], p[2 * count + 1]))
            ++count;

        interx1 = p[0];
        intery1 = p[1];
        interx2 = p[2];
        intery2 = p[3];

        return count;
    }

    bool IsPointInsideConvexPolygon2D(const double p[2], const int n, const double poly[]) {
        for (int i = 0; i < 2 * n - 2; i = i + 2)
            if (IsPointLeftOfLine2D(p, &poly[i], &poly[i + 2]) == false)
                return false;
        return IsPointLeftOfLine2D(p, &poly[2 * n - 2], &poly[0]);
    }

    bool IsPointInsidePolygon2D(const double p[2], const int n, const double poly[]) {
        if (n == 3)
            return IsPointInsideTriangle2D(p, &poly[0], &poly[2], &poly[4]);
        //	else if (n == 2)
        //	    return PointSegmentDistanceSquared2D(p, &poly[0], &poly[2], pmin) <= Constants::EPSILON_SQUARED;
        //
        int i, i1, lcross = 0, rcross = 0;
        double px = p[0];
        double py = p[1];
        double x = 0.0;
        bool testr;
        bool testl;

        for (i = 0; i < n; i++) {
            if (poly[2 * i] == px && poly[2 * i + 1] == py)
                return true;

            i1 = (i + n - 1) % n;

            testr = (poly[2 * i + 1] > py) != (poly[2 * i1 + 1] > py);
            testl = (poly[2 * i + 1] < py) != (poly[2 * i1 + 1] < py);

            if (testr || testl)
                x = (poly[2 * i] * poly[2 * i1 + 1] - poly[2 * i1] * poly[2 * i + 1] +
                     py * (poly[2 * i1] - poly[2 * i])) / (poly[2 * i1 + 1] - poly[2 * i + 1]);

            if (testr && x > px)
                rcross++;
            if (testl && x < px)
                lcross++;
        }

        if ((rcross & 1) != (lcross & 1))
            return true;
        return (rcross & 1);
    }

    bool IsPolygonInsideBox2D(const int n, const double poly[], const double min[2], const double max[2]) {
        for (int i = 0; i < n; ++i)
            if (IsPointInsideBox2D(&(poly[i << 1]), min, max) == false)
                return false;
        return true;
    }

    bool IsPolygonInsideConvexPolygon2D(const int n1, const double poly1[], const int n2, const double poly2[]) {
        for (int i = 0; i < n1; ++i)
            if (IsPointInsideConvexPolygon2D(&poly1[2 * i], n2, poly2) == false)
                return false;
        return true;
    }

    bool IsPolygonInsidePolygon2D(const int n1, const double poly1[], const int n2, const double poly2[],
                                  const bool checkIntersection) {
        for (int i = 0; i < n1; ++i)
            if (IsPointInsidePolygon2D(&poly1[2 * i], n2, poly2) == false)
                return false;
        return !checkIntersection || IntersectPolygons2D(n1, poly1, n2, poly2) == false;
    }

    bool IsPolygonInsideCircle2D(const int n, const double poly[], const double cx, const double cy, const double r) {
        for (int i = 0; i < n; ++i)
            if (IsPointInsideCircle2D(&poly[2 * i], cx, cy, r) == false)
                return false;
        return true;
    }

    double SignedAreaPolygon2D(const int n, const double poly[]) {
        int inext;
        double a = 0;
        for (int i = 0; i < n; ++i) {
            inext = (i + 1) % n;
            a += poly[(i << 1)] * poly[(inext << 1) + 1] - poly[(inext << 1)] * poly[(i << 1) + 1];
        }
        return a / 2;
    }

    void BoundingBoxPolygon2D(const int n, const double poly[], double min[2], double max[2]) {
        min[0] = max[0] = poly[0];
        min[1] = max[1] = poly[1];

        for (int i = 1; i < n; ++i) {
            if (poly[2 * i] < min[0])
                min[0] = poly[2 * i];
            else if (poly[2 * i] > max[0])
                max[0] = poly[2 * i];

            if (poly[2 * i + 1] < min[1])
                min[1] = poly[2 * i + 1];
            else if (poly[2 * i + 1] > max[1])
                max[1] = poly[2 * i + 1];
        }
    }

    void ChangeOrientationPolygon2D(const int n, double poly[]) {
        double t;
        for (int i = 0, j = 2 * n - 2; i < j; i += 2, j -= 2) {
            t = poly[i];
            poly[i] = poly[j];
            poly[j] = t;
            t = poly[i + 1];
            poly[i + 1] = poly[j + 1];
            poly[j + 1] = t;
        }
    }

    bool IsConvexPolygonCCW2D(const int n, const double poly[]) {
        int i;
        double z;

        if (n < 3)
            return true;

        for (int i = 0; i < n; i++) {
            z = Turn2D(&poly[2 * i], &poly[2 * ((i + 1) % n)], &poly[2 * ((i + 2) % n)]);
            if (z < -Constants::EPSILON)
                return true;
            if (z > Constants::EPSILON)
                return false;
        }
        return true;
    }

    bool IsPolygonConvex2D(const int n, const double poly[]) {
        if (n <= 3)
            return true;

        const bool turn = Turn2D(&poly[0], &poly[2], &poly[4]) <= Constants::EPSILON;

        for (int i = 1; i < n; i++)
            if (turn !=
                (Turn2D(&poly[2 * i], &poly[2 * ((i + 1) % n)], &poly[2 * ((i + 2) % n)]) <= Constants::EPSILON))
                return false;
        return true;
    }

    void CentroidPolygon2D(const int n, const double poly[], double centroid[2]) {
        centroid[0] = centroid[1] = 0;
        for (int i = 0; i < n; i++) {
            centroid[0] += poly[2 * i];
            centroid[1] += poly[2 * i + 1];
        }
        centroid[0] /= n;
        centroid[1] /= n;
    }

    int TriangulateConvexPolygon2D(const int n, const double poly[], int triangles[]) {
        for (int i = 0; i < n - 2; ++i) {
            triangles[3 * i] = 0;
            triangles[3 * i + 1] = i + 1;
            triangles[3 * i + 2] = i + 2;
        }
        return n - 2;
    }

    void SampleRandomPointInsideBox2D(const double min[2], const double max[2], double p[2]) {
        p[0] = RandomUniformReal(min[0], max[0]);
        p[1] = RandomUniformReal(min[1], max[1]);
    }

    void SampleRandomPointInsideTriangle2D(const double tA[2], const double tB[2], const double tC[2], double p[2]) {
        double a1 = RandomUniformReal();
        double a2 = RandomUniformReal();

        if (a1 + a2 > 1.0) {
            a1 = 1 - a1;
            a2 = 1 - a2;
        }

        p[0] = tA[0] + a1 * (tB[0] - tA[0]) + a2 * (tC[0] - tA[0]);
        p[1] = tA[1] + a1 * (tB[1] - tA[1]) + a2 * (tC[1] - tA[1]);
    }

    void SampleRandomPointInsideConvexPolygon2D(const int n, const double poly[], double p[2], const double min[2],
                                                const double max[2]) {
        double umin[2], umax[2];
        if (min == NULL || max == NULL)
            BoundingBoxPolygon2D(n, poly, umin, umax);
        else {
#if DEBUG
            assert(min != NULL);
            assert(max != NULL);
#endif
            umin[0] = min[0];
            umin[1] = min[1];
            umax[0] = max[0];
            umax[1] = max[1];
        }

        do
            SampleRandomPointInsideBox2D(umin, umax, p);
        while (IsPointInsideConvexPolygon2D(p, n, poly) == false);
    }

    void SampleRandomPointInsidePolygon2D(const int n, const double poly[], double p[2], const double min[2],
                                          const double max[2]) {
        double umin[2], umax[2];
        if (min == NULL || max == NULL)
            BoundingBoxPolygon2D(n, poly, umin, umax);
        else {
#if DEBUG
            assert(min != NULL);
            assert(max != NULL);
#endif
            umin[0] = min[0];
            umin[1] = min[1];
            umax[0] = max[0];
            umax[1] = max[1];
        }

        do
            SampleRandomPointInsideBox2D(umin, umax, p);
        while (IsPointInsidePolygon2D(p, n, poly) == false);
    }

    void SampleRandomPointInsideCircle2D(const double center[2], const double r, double p[2]) {
        const double s = r * sqrt(RandomUniformReal());
        const double theta = RandomUniformReal(0, 2 * M_PI);

        p[0] = s * cos(theta) + center[0];
        p[1] = s * sin(theta) + center[1];
    }


    void SampleRandomPointInsideAnnulus2D(const double center[2], const double r1, const double r2, double p[2]) {
        const double s = sqrt(RandomUniformReal() * (r2 * r2 - r1 * r1) + r1 * r1);
        const double theta = RandomUniformReal(0, 2 * M_PI);

        p[0] = s * cos(theta) + center[0];
        p[1] = s * sin(theta) + center[1];
    }

    void ApplyTransToPolygon2D(const double T[], const int n, const double origPoly[], double newPoly[]) {
        for (int i = 0; i < n; ++i)
            Algebra2D::TransMultPoint(T, &origPoly[i << 1], &newPoly[i << 1]);
    }

    void ApplyRotToPolygon2D(const double R[], const int n, const double origPoly[], double newPoly[]) {
        for (int i = 0; i < n; ++i)
            Algebra2D::RotMultPoint(R, &origPoly[i << 1], &newPoly[i << 1]);
    }

    void ApplyTransRotToPolygon2D(const double TR[], const int n, const double origPoly[], double newPoly[]) {
        for (int i = 0; i < n; ++i)
            Algebra2D::TransRotMultPoint(TR, &origPoly[i << 1], &newPoly[i << 1]);
    }

    void StabSkeleton2D(const int n, const double skeleton[], const double dist, std::vector<double> &newsk) {
        for (int i = 0; i < n - 1; ++i) {
            newsk.push_back(skeleton[2 * i]);
            newsk.push_back(skeleton[2 * i + 1]);

            const double d = Algebra2D::PointDistance(&skeleton[2 * i], &skeleton[2 * i + 2]);
            if (d > dist) {
                const int nrSteps = (int) d / dist;
                for (int j = 1; j < nrSteps; ++j) {
                    const double t = (j + 0.0) / nrSteps;

                    newsk.push_back((1 - t) * skeleton[2 * i] + t * skeleton[2 * i + 2]);
                    newsk.push_back((1 - t) * skeleton[2 * i + 1] + t * skeleton[2 * i + 3]);
                }
            }
        }

        newsk.push_back(skeleton[2 * n - 2]);
        newsk.push_back(skeleton[2 * n - 1]);
    }

    void FixSkeleton2D(std::vector<double> &pts) {
        // p0 p1 p2 p3 p4 p5 p6 p7 ... p{n-1}
        //(p0, p1) x (p2, p3)

        double p[2], mua, mub;
        int i = 0;
        while (2 * i + 2 < pts.size()) {
            const int a = i;
            const int b = i + 1;

            bool intersect = false;

            for (int j = i + 2; 2 * j + 2 < pts.size() && intersect == false; ++j) {
                const int c = j;
                const int d = j + 1;

                if (IntersectSegments2D(pts[2 * a], pts[2 * a + 1], pts[2 * b], pts[2 * b + 1], pts[2 * c],
                                        pts[2 * c + 1], pts[2 * d], pts[2 * d + 1], p[0], p[1])) {
                    intersect = true;

                    pts[2 * b] = p[0];
                    pts[2 * b + 1] = p[1];
                    pts.erase(pts.begin() + 2 * b + 2, pts.begin() + 2 * d + 2);
                    i = 0;
                }
            }
            if (intersect == false)
                ++i;
        }
    }

    bool ShiftSkeleton2D(const int n, const double skeleton[], const double thick, std::vector<double> &poly) {
        if (n < 2)
            return false;

        double v[2], u[2], p[2], line1[4], line2[4], x, y, mua, mub;
        const double *ptA;
        const double *ptB;
        const double *ptC;

        // first point
        v[0] = skeleton[2] - skeleton[0];
        v[1] = skeleton[3] - skeleton[1];
        Algebra2D::VecNormal(v, u);
        Algebra2D::VecUnit(u, u);
        x = skeleton[0] + 0.5 * thick * u[0];
        y = skeleton[1] + 0.5 * thick * u[1];
        poly.push_back(x);
        poly.push_back(y);

        line1[0] = x;
        line1[1] = y;
        line1[2] = x + v[0];
        line1[3] = y + v[1];

        for (int i = 1; i < n; ++i) {
            ptA = &skeleton[2 * (i - 1)];
            ptB = &skeleton[2 * i];
            v[0] = ptA[0] - ptB[0];
            v[1] = ptA[1] - ptB[1];
            line2[0] = ptB[0];
            line2[1] = ptB[1];
            if (i < n - 1) {
                ptC = &skeleton[2 * (i + 1)];
                u[0] = ptC[0] - ptB[0];
                u[1] = ptC[1] - ptB[1];
                Algebra2D::AngleMultPoint(0.5 * Algebra2D::VecFromToAngleCCW(u, v), u, u);

                line2[2] = ptB[0] + u[0];
                line2[3] = ptB[1] + u[1];
            } else {
                Algebra2D::VecNormal(v, u);
                line2[2] = ptB[0] + u[0];
                line2[3] = ptB[1] + u[1];
            }

            if (IntersectLines2D(line1[0], line1[1], line1[2], line1[3], line2[0], line2[1], line2[2], line2[3], x, y,
                                 mua, mub) == false) {
                x = line1[2];
                y = line1[3];
            }
            poly.push_back(x);
            poly.push_back(y);

            line1[0] = x;
            line1[1] = y;
            line1[2] = x + skeleton[2 * i + 2] - skeleton[2 * i];
            line1[3] = y + skeleton[2 * i + 3] - skeleton[2 * i + 1];
        }

        FixSkeleton2D(poly);

        return true;
    }

    bool FromSkeletonToPolygon2D(const int n, const double skeleton[], const double thick, std::vector<double> &poly) {
        poly.clear();

        ShiftSkeleton2D(n, skeleton, thick, poly);

        std::vector<double> rskel;
        for (int i = n - 1; i >= 0; --i) {
            rskel.push_back(skeleton[2 * i]);
            rskel.push_back(skeleton[2 * i + 1]);
        }
        ShiftSkeleton2D(n, &rskel[0], thick, poly);

        return true;
    }

    void GenerateArcAsPolygon2D(const double x, const double y, const double r, const double thetaStart,
                                const double thetaInc, const int nrPtsSkel,
                                const double thick, std::vector<double> &poly) {
        std::vector<double> skel;
        skel.resize(2 * nrPtsSkel);
        for (int i = 0; i < nrPtsSkel; ++i) {
            skel[2 * i] = x + r * cos(thetaStart + i * thetaInc);
            skel[2 * i + 1] = y + r * sin(thetaStart + i * thetaInc);
        }

        FromSkeletonToPolygon2D(nrPtsSkel, &skel[0], thick, poly);
    }

    void GenerateRadialPatternAsPolygons2D(const double x, const double y, const double rmin, const double rmax,
                                           const double thetaStart, const double thetaInc,
                                           const int nrRays, const double thick,
                                           std::vector<std::vector<double> *> &polys) {
        std::vector<double> *poly;
        double skel[4];

        for (int i = 0; i < nrRays; ++i) {
            const double c = cos(thetaStart + i * thetaInc);
            const double s = sin(thetaStart + i * thetaInc);

            skel[0] = x + rmin * c;
            skel[1] = y + rmin * s;
            skel[2] = x + rmax * c;
            skel[3] = y + rmax * s;

            poly = new std::vector<double>();
            FromSkeletonToPolygon2D(2, skel, thick, *poly);
            polys.push_back(poly);
        }
    }

    void ScaleInterval(const double s, double &xmin, double &xmax) {
        const double c = 0.5 * (xmax + xmin);
        const double r = 0.5 * s * (xmax - xmin);

        xmin = c - r;
        xmax = c + r;
    }

    void RegularizePointsAlongPath(const int n, const double pts[], const double dsep, const int dim,
                                   std::vector<double> &newPts) {
        double dtot = 0.0;
        double dcurr = 0.0;
        double d = dsep;

        for (int j = 0; j < dim; ++j)
            newPts.push_back(pts[j]);

        for (int i = 1; i < n; ++i) {
            dcurr = Algebra::PointDistance(dim, &pts[dim * (i - 1)], &pts[dim * i]);
            dtot += dcurr;
            while (dtot > d) {

                const double *p1 = &pts[dim * (i - 1)];
                const double *p2 = &pts[dim * i];
                const double t = (d - dtot + dcurr) / dcurr;
                for (int j = 0; j < dim; ++j)
                    newPts.push_back((1 - t) * p1[j] + t * p2[j]);

                d += dsep;
            }
        }
        for (int j = 0; j < dim; ++j)
            newPts.push_back(pts[dim * (n - 1) + j]);
    }

    void
    RemoveSmallTriangles(std::vector<double> &triVertices, std::vector<int> &triIndices, std::vector<int> &triNeighs,
                         const double minArea) {
        double poly[6];
        std::vector<int> ids;
        int count = 0;
        const int n = triIndices.size() / 3;
        double area = 0.0;
        double totArea = 0.0;

        ids.resize(n);
        std::fill(ids.begin(), ids.end(), -1);
        for (int i = 0; i < n; ++i) {
            const int *itri = &((triIndices)[3 * i]);
            poly[0] = (triVertices)[2 * itri[0]];
            poly[1] = (triVertices)[2 * itri[0] + 1];
            poly[2] = (triVertices)[2 * itri[1]];
            poly[3] = (triVertices)[2 * itri[1] + 1];
            poly[4] = (triVertices)[2 * itri[2]];
            poly[5] = (triVertices)[2 * itri[2] + 1];
            area = fabs(SignedAreaPolygon2D(3, poly));

            if (area >= minArea) {
                totArea += area;
                ids[i] = count++;
            }
        }

        std::vector<int> tmpTriIndices;
        std::vector<int> tmpTriNeighs;
        int neigh;

        tmpTriIndices.resize(3 * count);
        tmpTriNeighs.resize(3 * count);

        for (int i = 0; i < n; ++i)
            if (ids[i] >= 0) {
                tmpTriIndices[3 * ids[i]] = (triIndices)[3 * i];
                tmpTriIndices[3 * ids[i] + 1] = (triIndices)[3 * i + 1];
                tmpTriIndices[3 * ids[i] + 2] = (triIndices)[3 * i + 2];

                neigh = (triNeighs)[3 * i];
                tmpTriNeighs[3 * ids[i]] = neigh < 0 ? -1 : ids[neigh];
                neigh = (triNeighs)[3 * i + 1];
                tmpTriNeighs[3 * ids[i] + 1] = neigh < 0 ? -1 : ids[neigh];
                neigh = (triNeighs)[3 * i + 2];
                tmpTriNeighs[3 * ids[i] + 2] = neigh < 0 ? -1 : ids[neigh];
            }

        triIndices.assign(tmpTriIndices.begin(), tmpTriIndices.end());
        triNeighs.assign(tmpTriNeighs.begin(), tmpTriNeighs.end());
    }

    double DistanceSquaredPointSegment3D(const double p[], const double s1[], const double s2[]) {
        const double v[] = {s2[0] - s1[0], s2[1] - s1[1], s2[2] - s1[2]};
        const double w[] = {p[0] - s1[0], p[1] - s1[1], p[2] - s1[2]};

        const double c1 = Algebra3D::VecDotProduct(w, v);
        if (c1 <= 0)
            return Algebra3D::PointDistanceSquared(p, s1);

        const double c2 = Algebra3D::VecDotProduct(v, v);
        if (c2 <= c1)
            return Algebra3D::PointDistanceSquared(p, s2);

        const double b = c1 / c2;

        const double pb[] = {s1[0] + b * v[0], s1[1] + b * v[1], s1[2] + b * v[2]};

        return Algebra3D::PointDistanceSquared(p, pb);
    }

    double DistanceSquaredPointSegment(const int dim, const double p[], const double s1[], const double s2[]) {
        std::vector<double> v;
        std::vector<double> w;

        v.resize(dim);
        w.resize(dim);
        for (int i = 0; i < dim; ++i) {
            v[i] = s2[i] - s1[i];
            w[i] = p[i] - s1[i];
        }

        const double c1 = Algebra::VecDotProduct(dim, &w[0], &v[0]);
        if (c1 <= 0)
            return Algebra::PointDistanceSquared(dim, p, s1);

        const double c2 = Algebra::VecDotProduct(dim, &v[0], &v[0]);
        if (c2 <= c1)
            return Algebra::PointDistanceSquared(dim, p, s2);

        const double b = c1 / c2;
        for (int i = 0; i < dim; ++i)
            w[i] = s1[i] + b * v[i];

        return Algebra::PointDistanceSquared(dim, p, &w[0]);
    }
}
