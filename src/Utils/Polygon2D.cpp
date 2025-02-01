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
#include "Utils/Polygon2D.hpp"
#include "Utils/Geometry.hpp"
#include "Utils/Constants.hpp"
#include "Utils/PseudoRandom.hpp"
#include "Utils/Logger.hpp"
#include "External/ShewchukTriangle.hpp"
#include <fstream>

namespace Antipatrea {
    void Polygon2D::Clear(void) {
        m_flags = RECOMPUTE_TRIANGULATION | RECOMPUTE_CENTROID | RECOMPUTE_CONVEXITY | RECOMPUTE_BOUNDING_BOX |
                  RECOMPUTE_TRI_AREAS;
        m_bbox[0] = m_bbox[1] = m_bbox[2] = m_bbox[3] = 0;
        m_vertices.clear();
        m_triIndices.clear();
        m_triAreas.clear();
        m_area = 0.0;
        m_centroid[0] = m_centroid[1] = 0.0;
        m_isConvex = true;
    }

    int Polygon2D::GetNrVertices(void) const {
        return m_vertices.size() / 2;
    }

    double Polygon2D::GetVertexX(const int i) const {
        return m_vertices[2 * i];
    }

    double Polygon2D::GetVertexY(const int i) const {
        return m_vertices[2 * i + 1];
    }

    void Polygon2D::GetVertex(const int i, double v[2]) const {
        v[0] = GetVertexX(i);
        v[1] = GetVertexY(i);
    }

    void Polygon2D::AddVertex(const double v[2]) {
        AddVertex(v[0], v[1]);
    }

    void Polygon2D::AddVertex(const double vx, const double vy) {
        m_vertices.push_back(vx);
        m_vertices.push_back(vy);
        OnShapeChange();
    }

    void Polygon2D::AddVertices(const int n, const double vertices[]) {
        for (int i = 0; i < n; ++i)
            AddVertex(vertices[2 * i], vertices[2 * i + 1]);
    }

    void Polygon2D::SetVertex(const int i, const double v[2]) {
        SetVertex(i, v[0], v[1]);
    }

    void Polygon2D::SetVertex(const int i, const double vx, const double vy) {
        m_vertices[2 * i] = vx;
        m_vertices[2 * i + 1] = vy;
        OnShapeChange();
    }

    void Polygon2D::MakeCCW(void) {
        //polygon is ccw if signed area is positive
        const int n = GetNrVertices();
        double a = 0;
        int inext;
        for (int i = 0; i < n; ++i) {
            inext = (i + 1) % n;
            a += m_vertices[(i << 1)] * m_vertices[(inext << 1) + 1] -
                    m_vertices[(inext << 1)] * m_vertices[(i << 1) + 1];
        }

        if (a < 0) {
            //polygon is not ccw, so change its orientation
            double t;
            for (int i = 0, j = 2 * n - 2; i < j; i += 2, j -= 2) {
                t = m_vertices[i];
                m_vertices[i] = m_vertices[j];
                m_vertices[j] = t;
                t = m_vertices[i + 1];
                m_vertices[i + 1] = m_vertices[j + 1];
                m_vertices[j + 1] = t;
            }
        }
        m_flags = AddFlags(m_flags, RECOMPUTE_TRIANGULATION | RECOMPUTE_TRI_AREAS);
    }

    const double *Polygon2D::GetBoundingBox(void) {
        if (HasAllFlags(m_flags, RECOMPUTE_BOUNDING_BOX)) {
            m_flags = RemoveFlags(m_flags, RECOMPUTE_BOUNDING_BOX);

            const int n = GetNrVertices();
            if (n == 0)
                return m_bbox;

            m_bbox[0] = m_bbox[2] = GetVertexX(0);
            m_bbox[1] = m_bbox[3] = GetVertexY(0);

            for (int i = 1; i < n; ++i) {
                const double x = GetVertexX(i);
                const double y = GetVertexY(i);

                if (x < m_bbox[0])
                    m_bbox[0] = x;
                else if (x > m_bbox[2])
                    m_bbox[2] = x;

                if (y < m_bbox[1])
                    m_bbox[1] = y;
                else if (y > m_bbox[3])
                    m_bbox[3] = y;
            }
        }

        return m_bbox;
    }

    bool Polygon2D::IsConvex(void) {
        if (HasAllFlags(m_flags, RECOMPUTE_CONVEXITY)) {
            m_flags = RemoveFlags(m_flags, RECOMPUTE_CONVEXITY);

            const int n = GetNrVertices();

            if (n <= 3)
                m_isConvex = true;
            else {
                const bool turn = Turn2D(&m_vertices[0], &m_vertices[2], &m_vertices[4]) <= Constants::EPSILON;

                for (int i = 1; i < n; i++)
                    if (turn != (Turn2D(&m_vertices[2 * i],
                                        &m_vertices[2 * ((i + 1) % n)],
                                        &m_vertices[2 * ((i + 2) % n)]) <= Constants::EPSILON)) {
                        m_isConvex = false;
                        return m_isConvex;
                    }
                m_isConvex = true;
            }
        }

        return m_isConvex;
    }

    void Polygon2D::Triangulate(void) {
        if (HasAllFlags(m_flags, RECOMPUTE_TRIANGULATION)) {
            m_flags = RemoveFlags(m_flags, RECOMPUTE_TRIANGULATION);
            m_triIndices.clear();
            TriangulatePolygonWithNoHoles2D(false, -1, -1,
                                            m_vertices.size() / 2, &m_vertices[0],
                                            NULL, &m_triIndices, NULL);
        }
    }

    int Polygon2D::GetNrTriangles(void) {
        Triangulate();
        return m_triIndices.size() / 3;
    }

    void Polygon2D::GetTriangleIndices(const int i, int tri[3]) {
        Triangulate();
        tri[0] = m_triIndices[3 * i];
        tri[1] = m_triIndices[3 * i + 1];
        tri[2] = m_triIndices[3 * i + 2];
    }

    void Polygon2D::GetTriangleVertices(const int i, double tri[6]) {
        int tind[3];
        GetTriangleIndices(i, tind);
        GetVertex(tind[0], &tri[0]);
        GetVertex(tind[1], &tri[2]);
        GetVertex(tind[2], &tri[4]);
    }

    const double *Polygon2D::GetCentroid(void) {
        if (HasAllFlags(m_flags, RECOMPUTE_CENTROID)) {
            m_flags = RemoveFlags(m_flags, RECOMPUTE_CENTROID);

            const int n = GetNrVertices();
            double sx = 0;
            double sy = 0;
            double v[2];

            for (int i = 0; i < n; ++i) {
                GetVertex(i, v);
                sx += v[0];
                sy += v[1];
            }
            m_centroid[0] = sx / n;
            m_centroid[1] = sy / n;
        }
        return m_centroid;
    }

    void Polygon2D::GetRepresentativePoint(double p[2]) {
        if (IsConvex()) {
            const double *c = GetCentroid();
            p[0] = c[0];
            p[1] = c[1];
        } else {
            double tri[6];
            GetTriangleVertices(0, tri);
            const double a1 = 0.35;
            const double a2 = 0.45;

            p[0] = (tri[0] + tri[2] + tri[4]) / 3.0;
            p[1] = (tri[1] + tri[3] + tri[5]) / 3.0;
        }
    }

    bool Polygon2D::IsPointInside(const double p[]) {
        const int n = GetNrVertices();

        if (n <= 2)
            return false;

        const double *bbox = GetBoundingBox();

        if (!IsPointInsideBox2D(p, &bbox[0], &bbox[2]))
            return false;

        if (IsConvex())
            return IsPointInsideConvexPolygon2D(p, n, &m_vertices[0]);
        else
            return IsPointInsidePolygon2D(p, n, &m_vertices[0]);
    }

    bool Polygon2D::SegmentCollision(const double p1[], const double p2[]) {
        double pinter[2];

        const int n = GetNrVertices();

        if (n <= 1)
            return false;
        if (n == 2)
            return IntersectSegments2D(p1[0], p1[1], p2[0], p2[1],
                                       GetVertexX(0), GetVertexY(0),
                                       GetVertexX(1), GetVertexY(1),
                                       pinter[0], pinter[1]);

        if (IsConvex())
            return CollisionSegmentConvexPolygon2D(p1, p2, n, &m_vertices[0]);
        else
            return CollisionSegmentPolygon2D(p1, p2, n, &m_vertices[0]);
    }

    double Polygon2D::PointDistance(const double p[], double pmin[]) {
        return sqrt(DistanceSquaredPointPolygon2D(p,
                                                  GetNrVertices(),
                                                  &m_vertices[0],
                                                  pmin));
    }

    double Polygon2D::PolygonDistance(Polygon2D &poly) {
        double pmin1[2];
        double pmin2[2];

        return sqrt(DistanceSquaredPolygons2D(GetNrVertices(),
                                              &m_vertices[0],
                                              poly.GetNrVertices(),
                                              &(*(poly.GetVertices()))[0],
                                              pmin1,
                                              pmin2));
    }

    bool Polygon2D::PolygonCollision(Polygon2D &poly) {
        const bool c1 = IsConvex();
        const bool c2 = poly.IsConvex();

        const double *v2 = &(*(poly.GetVertices()))[0];

        if (c1 && c2)
            return CollisionConvexPolygons2D(GetNrVertices(), &m_vertices[0],
                                             poly.GetNrVertices(), v2);
        else if (c1)
            return CollisionPolygonConvexPolygon2D(poly.GetNrVertices(), v2,
                                                   GetNrVertices(), &m_vertices[0]);
        else if (c2)
            return CollisionPolygonConvexPolygon2D(GetNrVertices(), &m_vertices[0],
                                                   poly.GetNrVertices(), v2);
        else
            return CollisionPolygons2D(GetNrVertices(), &m_vertices[0],
                                       poly.GetNrVertices(), v2);
    }

    void Polygon2D::SetupFromParams(Params &p) {
        m_vertices.clear();
        p.GetValuesAsDoubles(Constants::KW_Polygon2D, m_vertices);
        if (m_vertices.size() % 2 == 1) {
            Logger::m_out << "warning Polygon2D::SetupFromParams ... odd size " << m_vertices.size()
                    << "... removing last value" << std::endl;
            m_vertices.pop_back();
        }
        OnShapeChange();
        MakeCCW();
    }

    void Polygon2D::Print(std::ostream &out) const {
        const int n = GetNrVertices();
        out << n << std::endl;
        for (int i = 0; i < n; ++i)
            out << GetVertexX(i) << " " << GetVertexY(i) << " ";
        out << std::endl;
    }

    Status Polygon2D::Read(std::istream &in) {
        int n;
        if (!(in >> n)) {
            Logger::m_out << "error Polygon2D::Read ... expecting number of vertieces" << std::endl;
            return STATUS_ERROR;
        }

        m_vertices.resize(2 * n);
        for (int i = 0; i < 2 * n; ++i) {
            if (!(in >> m_vertices[i])) {
                Logger::m_out << "error Polygon2D::Read ... expecting value " << i << " for polygon with " << n
                        << " vertices " << std::endl;
                return STATUS_ERROR;
            }
        }

        OnShapeChange();
        MakeCCW();

        return STATUS_OK;
    }

    void Polygon2D::OccupiedGridCells(const Grid &grid,
                                      std::vector<int> &cellsInside,
                                      std::vector<int> &cellsIntersect) {
        const double *bbox = GetBoundingBox();
        const int n = m_vertices.size() / 2;
        const double *poly = &m_vertices[0];
        const bool checkIntersection = false;

        int coord_min[3] = {0, 0, 0};
        int coord_max[3] = {0, 0, 0};
        int coords[3] = {0, 0, 0};
        double min[3], max[3];

        cellsInside.clear();
        cellsIntersect.clear();
        grid.GetCoords(&bbox[0], coord_min);
        grid.GetCoords(&bbox[2], coord_max);
        for (int x = coord_min[0]; x <= coord_max[0]; ++x) {
            coords[0] = x;
            for (int y = coord_min[1]; y <= coord_max[1]; ++y) {
                coords[1] = y;

                grid.GetCellFromCoords(coords, min, max);
                double box[8];
                BoxAsPolygon2D(min, max, box);

                if (IntersectPolygons2D(4, box, n, poly))
                    cellsIntersect.push_back(grid.GetCellIdFromCoords(coords));
                else if (IsPolygonInsidePolygon2D(4, box, n, poly, checkIntersection))
                    cellsInside.push_back(grid.GetCellIdFromCoords(coords));
                else if (IsPolygonInsideBox2D(n, poly, min, max))
                    cellsIntersect.push_back(grid.GetCellIdFromCoords(coords));
            }
        }
    }

    double Polygon2D::GetArea(void) {
        if (HasAllFlags(m_flags, RECOMPUTE_TRI_AREAS)) {
            m_flags = RemoveFlags(m_flags, RECOMPUTE_TRI_AREAS);

            m_triAreas.clear();
            const int n = GetNrTriangles();
            double tri[6];
            double area = 0;

            m_triAreas.resize(n);
            m_area = 0;
            for (int i = 0; i < n; ++i) {
                GetTriangleVertices(i, tri);
                m_triAreas[i] = fabs(SignedAreaPolygon2D(3, tri));
                m_area += m_triAreas[i];
            }
        }

        return m_area;
    }

    void Polygon2D::SamplePointInside(double p[]) {
        const int t = SelectTriangleBasedOnArea();
        SampleRandomPointInsideTriangle2D(&m_vertices[2 * m_triIndices[3 * t]],
                                          &m_vertices[2 * m_triIndices[3 * t + 1]],
                                          &m_vertices[2 * m_triIndices[3 * t + 2]], p);
    }

    int Polygon2D::SelectTriangleBasedOnArea(void) {
        const int n = GetNrTriangles();
        const double r = RandomUniformReal(0, GetArea());
        double w = 0;

        for (int i = 0; i < n; ++i) {
            w += m_triAreas[i];
            if (w >= r)
                return i;
        }
        return n - 1;
    }

    int Polygon2D::LocateTriangle(const double p[]) {
        double tri[6];

        for (int i = GetNrTriangles() - 1; i >= 0; --i) {
            GetTriangleVertices(i, tri);
            if (IsPointInsideTriangle2D(p, &tri[0], &tri[2], &tri[4]))
                return i;
        }
        return Constants::ID_UNDEFINED;
    }

    Status ReadPolygons2DFromLaser(const std::vector<std::vector<double> > &laserData, std::vector<Polygon2D *> &polys,
                                   int STATE) {
        int size = laserData.size();
        Polygon2D *poly;
        double m;
        if (STATE == 0)
            m = 0.045;
        else
            m = 0.075;

        for (int i = 0; i < size; ++i) {
            poly = new Polygon2D();
            poly->ReadFromLaser(laserData[i], m);
            polys.push_back(poly);
        }
        return STATUS_OK;
    }

    Status ReadPolygons2DFromLaser(const std::vector<std::vector<geometry_msgs::Point> > &clusters,
                                   std::vector<Polygon2D *> &polys, int STATE) {
        int size = clusters.size();
        Polygon2D *poly;
        double m;
        if (STATE == 0)
            m = 0.045;
        else
            m = 0.075;

        for (int i = 0; i < size; ++i) {
            poly = new Polygon2D();
            if (clusters[i].size() > 2)
                poly->ReadFromLaser(clusters[i]);
            else {
                std::vector<double> xy = {clusters[i].front().x, clusters[i].front().y};
                poly->ReadFromLaser(xy, m);
            }
            polys.push_back(poly);
        }

        return STATUS_OK;
    }

    Status Polygon2D::ReadFromLaser(const std::vector<geometry_msgs::Point> &xy) {
        int n = xy.size();
        m_vertices.resize(2 * n);

        for (int i = 0; i < n; ++i) {
            m_vertices[2 * i] = xy[i].x;
            m_vertices[2 * i + 1] = xy[i].y;
        }

        OnShapeChange();
        MakeCCW();
        return STATUS_OK;
    }

    Status Polygon2D::ReadFromLaser(const std::vector<double> &xy, double l) {
        m_vertices.push_back(xy[0] - l);
        m_vertices.push_back(xy[1] - l);
        m_vertices.push_back(xy[0] + l);
        m_vertices.push_back(xy[1] - l);
        m_vertices.push_back(xy[0] + l);
        m_vertices.push_back(xy[1] + l);
        m_vertices.push_back(xy[0] - l);
        m_vertices.push_back(xy[1] + l);

        OnShapeChange();
        MakeCCW();
        return STATUS_OK;
    }

    Status ReadPolygons2D(const char fname[], std::vector<Polygon2D *> &polys) {
        std::ifstream in(fname, std::ios_base::in);
        if (!in.is_open())
            return STATUS_ERROR;
        auto status = ReadPolygons2D(in, polys);
        in.close();
        return status;
    }

    Status ReadPolygons2D(std::istream &in, std::vector<Polygon2D *> &polys) {
        int n = 0;
        Polygon2D *poly;

        if (!(in >> n)) {
            Logger::m_out << "warning ReadPolygons2D ... expecting number of polygons";
            return STATUS_WARNING;
        }
        for (int i = 0; i < n; ++i) {
            poly = new Polygon2D();
            poly->Read(in);
            polys.push_back(poly);
        }
        return STATUS_OK;
    }

    void PrintPolygons2D(std::ostream &out, std::vector<Polygon2D *> &polys) {
        out << polys.size() << std::endl;
        for (auto &poly: polys)
            out << *poly << std::endl;
    }
}
