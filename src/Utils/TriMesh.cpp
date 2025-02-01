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
#include "Utils/TriMesh.hpp"
#include "Utils/Algebra3D.hpp"
#include "Utils/Geometry.hpp"
#include "Utils/GDraw.hpp"
#include "Utils/PseudoRandom.hpp"
#include "Utils/Misc.hpp"
#include "Utils/Grid.hpp"
#include "Utils/PrintMsg.hpp"
#include "Utils/Logger.hpp"
#include <vector>
#include <cstring>

namespace Antipatrea {
    void TriMesh::Clear(void) {
        const int nf = m_faces.size();
        for (int i = 0; i < nf; ++i)
            if (m_faces[i])
                delete m_faces[i];
        m_faces.clear();

        const int ng = m_groups.size();
        for (int i = 0; i < ng; ++i)
            if (m_groups[i])
                delete m_groups[i];
        m_groups.clear();
        m_gidCurr = -1;

        const int nm = m_materials.size();
        for (int i = 0; i < nm; ++i)
            if (m_materials[i])
                delete m_materials[i];
        m_materials.clear();
        m_midCurr = -1;

        m_tidCurr = -1;

        const int nv = m_vertices.size();
        for (int i = 0; i < nv; ++i)
            delete m_vertices[i];
        m_vertices.clear();

        m_updateBBox = true;
    }

    void TriMesh::AddVertex(const double vx,
                            const double vy,
                            const double vz,
                            const double nx,
                            const double ny,
                            const double nz) {
        Vertex *vertex = new Vertex();
        vertex->m_pos[0] = vx;
        vertex->m_pos[1] = vy;
        vertex->m_pos[2] = vz;
        vertex->m_normal[0] = nx;
        vertex->m_normal[1] = ny;
        vertex->m_normal[2] = nz;
        vertex->m_needsUpdateNormal = false;
        m_vertices.push_back(vertex);

        OnVertexChange(GetNrVertices() - 1);
    }

    void TriMesh::AddVertex(const double v[], const double normal[]) {
#if DEBUG
        assert(v != NULL);
#endif
        if (normal)
            AddVertex(v[0], v[1], v[2], normal[0], normal[1], normal[2]);
        else
            AddVertex(v[0], v[1], v[2], 0, 0, 1);
    }

    void TriMesh::AddVertices(const int n,
                              const double vert[],
                              const double normals[]) {
#if DEBUG
        assert(n > 0);
        assert(vert != NULL);
#endif

        const int n3 = 3 * n;
        if (normals)
            for (int i = 0; i < n3; i += 3)
                AddVertex(&(vert[i]), &(normals[i]));
        else
            for (int i = 0; i < n3; i += 3)
                AddVertex(&(vert[i]));
    }

    void TriMesh::SetVertex(const int i, const double v[]) {
#if DEBUG
        assert(i >= 0 && i < GetNrVertices());
        assert(v != NULL);
#endif
        SetVertex(i, v[0], v[1], v[2]);
    }

    void TriMesh::SetVertex(const int i, const double vx, const double vy, const double vz) {
#if DEBUG
        assert(i >= 0 && i < GetNrVertices());
#endif

        m_vertices[i]->m_pos[0] = vx;
        m_vertices[i]->m_pos[1] = vy;
        m_vertices[i]->m_pos[2] = vz;

        OnVertexChange(i);
    }

    void TriMesh::SetVertexNormal(const int i, const double nx, const double ny, const double nz) {
#if DEBUG
        assert(i >= 0 && i < GetNrVertices());
#endif
        m_vertices[i]->m_normal[0] = nx;
        m_vertices[i]->m_normal[1] = ny;
        m_vertices[i]->m_normal[2] = nz;
        m_vertices[i]->m_needsUpdateNormal = false;
    }

    void TriMesh::SetVertexNormal(const int i, const double normal[]) {
#if DEBUG
        assert(i >= 0 && i < GetNrVertices());
        assert(normal != NULL);
#endif
        SetVertexNormal(i, normal[0], normal[1], normal[2]);
    }

    void TriMesh::AddTriangle(const int tri[]) {
#if DEBUG
        assert(tri != NULL);
#endif
        AddTriangle(tri[0], tri[1], tri[2]);
    }

    void TriMesh::AddTriangle(const int vt1, const int vt2, const int vt3) {
        Face *f = new Face();
        f->m_vids.resize(3);
        f->m_vids[0] = vt1;
        f->m_vids[1] = vt2;
        f->m_vids[2] = vt3;
        AddFace(f);
    }

    void TriMesh::AddTriangles(const int n, const int tri[]) {
#if DEBUG
        assert(n >= 0 && tri != NULL);
#endif
        for (int i = 0; i < n; ++i)
            AddTriangle(&tri[3 * i]);
    }

    void TriMesh::AddTriMesh(TriMesh &tmesh) {
#if DEBUG
        assert(tmesh != NULL);
#endif
        GMaterial *gmat;
        GTexture *gtex;
        TexCoord ctex;

        const int origNrVertices = GetNrVertices();
        const int origNrFaces = GetNrFaces();
        const int origNrTexCoords = GetNrTexCoords();
        const int origNrTextures = GetNrTextures();
        const int origNrMaterials = GetNrMaterials();
        const int origNrGroups = GetNrGroups();

        for (int i = 0; i < tmesh.GetNrMaterials(); ++i) {
            gmat = new GMaterial();
            gmat->CopyFrom(*(tmesh.GetMaterial(i)));
            AddMaterial(gmat);
        }

        for (int i = 0; i < tmesh.GetNrTextures(); ++i) {
            gtex = new GTexture();
            gtex->SetFileName(tmesh.GetTexture(i)->GetFileName());
            AddTexture(gtex);
        }

        for (int i = 0; i < tmesh.GetNrTexCoords(); ++i)
            AddTexCoord(tmesh.GetTexCoord(i));

        for (int i = 0; i < tmesh.GetNrVertices(); ++i)
            AddVertex(tmesh.GetVertex(i), tmesh.GetVertexNormal(i));

        for (int i = 0; i < tmesh.GetNrFaces(); ++i) {
            const Face *f = tmesh.GetFace(i);
            Face *fnew = new Face();
            const int fnv = f->m_vids.size();
            fnew->m_vids.resize(fnv);
            for (int j = 0; j < fnv; ++j)
                fnew->m_vids[j] = f->m_vids[j] + origNrVertices;

            for (int j = 0; j < f->m_vtids.size(); ++j)
                fnew->m_vtids.push_back(f->m_vtids[j] + origNrTexCoords);

            AddFace(fnew);

            if (f->m_tid >= 0)
                fnew->m_tid = f->m_tid + origNrTextures;
            if (f->m_mid >= 0)
                fnew->m_mid = f->m_mid + origNrMaterials;
            if (f->m_gid >= 0)
                fnew->m_gid = f->m_gid + origNrGroups;
        }
    }

    void TriMesh::AddTriangle(const double v1[], const double v2[], const double v3[]) {
        const int n = GetNrVertices();

        AddVertex(v1);
        AddVertex(v2);
        AddVertex(v3);
        AddTriangle(n, n + 1, n + 2);
    }

    void TriMesh::AddTriangle(const double vtri[]) {
#if DEBUG
        assert(vtri != NULL);
#endif
        AddTriangle(&vtri[0], &vtri[3], &vtri[6]);
    }

    void TriMesh::AddTriangles(const int n, const double tris[]) {
#if DEBUG
        assert(n > 0);
        assert(tris != NULL);
#endif
        for (int i = 0; i < n; ++i)
            AddTriangle(&tris[9 * i]);
    }

    void TriMesh::AddQuad(const int vid1, const int vid2, const int vid3, const int vid4, const bool asTriangles) {
        if (asTriangles) {
            AddTriangle(vid1, vid2, vid3);
            AddTriangle(vid1, vid3, vid4);
        } else {
            Face *f = new Face();
            f->m_vids.resize(4);
            f->m_vids[0] = vid1;
            f->m_vids[1] = vid2;
            f->m_vids[2] = vid3;
            f->m_vids[3] = vid4;
            AddFace(f);
        }
    }

    void TriMesh::AddQuad(const int vids[], const bool asTriangles) {
#if DEBUG
        assert(vids != NULL);
#endif
        AddQuad(vids[0], vids[1], vids[2], vids[3], asTriangles);
    }

    void TriMesh::AddQuads(const int n, const int vids[], const bool asTriangles) {
#if DEBUG
        assert(n > 0);
        assert(vids != NULL);
#endif
        for (int i = 0; i < n; ++i)
            AddQuad(&vids[4 * i], asTriangles);
    }

    void TriMesh::AddQuad(const double v1[],
                          const double v2[],
                          const double v3[],
                          const double v4[],
                          const bool asTriangles) {
        const int n = GetNrVertices();

        AddVertex(v1);
        AddVertex(v2);
        AddVertex(v3);
        AddVertex(v4);
        AddQuad(n, n + 1, n + 2, n + 3, asTriangles);
    }

    void TriMesh::AddQuad(const double quad[], const bool asTriangles) {
#if DEBUG
        assert(quad != NULL);
#endif
        AddQuad(&quad[0], &quad[3], &quad[6], &quad[9], asTriangles);
    }

    void TriMesh::AddQuads(const int n, const double quads[], const bool asTriangles) {
#if DEBUG
        assert(n > 0);
        assert(quads != NULL);
#endif
        for (int i = 0; i < n; ++i)
            AddQuad(&quads[12 * i], asTriangles);
    }

    void TriMesh::AddConvexPolygon(const int n, const double vertices[], const bool asTriangles) {
        const int nold = GetNrVertices();

        if (asTriangles) {
            double cx = 0, cy = 0, cz = 0;

            AddVertices(n, vertices);
            for (int i = 0; i < n; ++i) {
                cx += vertices[3 * i];
                cy += vertices[3 * i + 1];
                cz += vertices[3 * i + 2];
            }
            AddVertex(cx / n, cy / n, cz / n);

            for (int i = 0; i < n; ++i)
                AddTriangle(nold + n, nold + i, nold + (i + 1) % n);
        } else {
            Face *f = new Face();
            AddVertices(n, vertices);
            f->m_vids.resize(n);
            for (int i = 0; i < n; ++i)
                f->m_vids[i] = nold + i;
            AddFace(f);
        }
    }

    void TriMesh::AddRegularPolygon(const double cx,
                                    const double cy,
                                    const double cz,
                                    const double r,
                                    const int n,
                                    const bool asTriangles) {
        std::vector<double> poly, poly3;
        poly.resize(2 * n);
        poly3.resize(3 * n);
        CircleAsPolygon2D(cx, cy, r, n, &poly[0]);
        for (int i = 0; i < n; ++i) {
            poly3[3 * i] = poly[2 * i];
            poly3[3 * i + 1] = poly[2 * i + 1];
            poly3[3 * i + 2] = cz;
        }
        AddConvexPolygon(n, &poly3[0], asTriangles);
    }

    void TriMesh::AddPolygon(Polygon2D &poly) {
        const int offset = GetNrVertices();
        const int nt = poly.GetNrTriangles();
        const int nv = poly.GetNrVertices();
        int tri[3];

        for (int i = 0; i < nv; ++i)
            AddVertex(poly.GetVertexX(i), poly.GetVertexY(i), 0.0);
        for (int i = 0; i < nt; ++i) {
            poly.GetTriangleIndices(i, tri);
            AddTriangle(offset + tri[2],
                        offset + tri[1],
                        offset + tri[0]);
        }
    }

    void TriMesh::AddExtrudedPolygon(Polygon2D &poly,
                                     const double zmin, const double zmax,
                                     const bool useHeightsAtMin,
                                     const bool useHeightsAtMax) {
        const int offset = GetNrVertices();
        const int nt = poly.GetNrTriangles();
        const int nv = poly.GetNrVertices();
        const bool hasHeights = false;
        double z;
        int tri[3];

        for (int i = 0; i < nv; ++i) {
            z = zmin;

            AddVertex(poly.GetVertexX(i), poly.GetVertexY(i), z);
        }

        for (int i = 0; i < nv; ++i) {
            z = zmax;
            AddVertex(poly.GetVertexX(i), poly.GetVertexY(i), z);
        }

        for (int i = 0; i < nt; ++i) {
            poly.GetTriangleIndices(i, tri);

            AddTriangle(offset + (tri)[2],
                        offset + (tri)[1],
                        offset + (tri)[0]);
            AddTriangle(offset + nv + (tri)[0],
                        offset + nv + (tri)[1],
                        offset + nv + (tri)[2]);
        }

        for (int i = 0; i < nv - 1; ++i) {
            AddTriangle(offset + i, offset + i + 1, offset + nv + i + 1);
            AddTriangle(offset + nv + i + 1, offset + nv + i, offset + i);
        }

        AddTriangle(offset + nv - 1, offset, offset + nv);
        AddTriangle(offset + nv, offset + 2 * nv - 1, offset + nv - 1);
    }

    void TriMesh::AddCapCapStripCCW(const int n,
                                    const int start_base,
                                    const int start_top,
                                    const bool close) {
        const bool asTriangle = true;

#if DEBUG
        assert(n > 0);
        assert(start_base >= 0);
        assert(start_top >= 0);
        assert(n + start_base < GetNrVertices());
        assert(n + start_top < GetNrVertices());
#endif
        for (int i = 0; i < n - 1; ++i)
            AddQuad(start_base + i,
                    start_base + i + 1,
                    start_top + i + 1,
                    start_top + i, asTriangle);
        if (n > 0 && close)
            AddQuad(start_base + n - 1, start_base, start_top, start_top + n - 1, asTriangle);
    }

    void TriMesh::AddCapCapStripCW(const int n,
                                   const int start_base,
                                   const int start_top,
                                   const bool close) {
        AddCapCapStripCCW(n, start_top, start_base, close);
    }

    void TriMesh::AddCapTopStripCCW(const int n,
                                    const int start_base,
                                    const int center,
                                    const bool close) {
#if DEBUG
        assert(n > 0);
        assert(start_base >= 0);
        assert(n + start_base < GetNrVertices());
        assert(center >= 0);
        assert(center < GetNrVertices());
#endif
        for (int i = 0; i < n - 1; ++i)
            AddTriangle(center, start_base + i, start_base + i + 1);
        if (n > 0 && close)
            AddTriangle(center, start_base + n - 1, start_base);
    }

    void TriMesh::AddCapTopStripCW(const int n,
                                   const int start_base,
                                   const int center,
                                   const bool close) {
#if DEBUG
        assert(n > 0);
        assert(start_base >= 0);
        assert(n + start_base < GetNrVertices());
        assert(center >= 0);
        assert(center < GetNrVertices());
#endif
        for (int i = 0; i < n - 1; ++i)
            AddTriangle(center, start_base + i + 1, start_base + i);
        if (n > 0 && close)
            AddTriangle(center, start_base, start_base + n - 1);
    }

    void TriMesh::ApplyTrans(const int vstart, const int vend,
                             const double x, const double y, const double z) {
#if DEBUG
        assert(vstart >= 0 && vstart < GetNrVertices());
        assert(vend >= 0 && vend < GetNrVertices());
#endif
        for (int i = vstart; i <= vend; ++i) {
            const double *v = GetVertex(i);
            SetVertex(i, v[0] + x, v[1] + y, v[2] + z);
        }
    }

    void TriMesh::ApplyTrans(const int vstart, const int vend, const double T[]) {
#if DEBUG
        assert(T != NULL);
#endif
        ApplyTrans(vstart, vend, T[0], T[1], T[2]);
    }

    void TriMesh::ApplyRot(const int vstart, const int vend, const double R[]) {
        double vnew[3];
        double nnew[3];

#if DEBUG
        assert(vstart >= 0 && vstart < GetNrVertices());
        assert(vend >= 0 && vend < GetNrVertices());
        assert(R != NULL);
#endif
        for (int i = vstart; i <= vend; ++i) {
            Algebra3D::RotMultPoint(R, GetVertex(i), vnew);
            // Algebra3D::RotMultPoint(R, GetVertexNormal(i), nnew);

            SetVertex(i, vnew);
            // SetVertexNormal(i, nnew);
        }
    }

    void TriMesh::ApplyTransRot(const int vstart, const int vend,
                                const double TR[]) {
        double vnew[3];
        double nnew[3];

#if DEBUG
        assert(vstart >= 0 && vstart < GetNrVertices());
        assert(vend >= 0 && vend < GetNrVertices());
        assert(TR != NULL);
#endif
        for (int i = vstart; i <= vend; ++i) {
            Algebra3D::TransRotMultPoint(TR, GetVertex(i), vnew);
            Algebra3D::RotMultPoint(&TR[Algebra3D::Trans_NR_ENTRIES], GetVertexNormal(i), nnew);

            SetVertex(i, vnew);
            SetVertexNormal(i, nnew);
        }
    }

    void TriMesh::ApplyQuat(const int vstart, const int vend, const double Q[]) {
#if DEBUG
        assert(Q != NULL);
#endif
        double R[Algebra3D::Rot_NR_ENTRIES];
        Algebra3D::QuatAsRot(Q, R);
        ApplyRot(vstart, vend, R);
    }

    void TriMesh::ApplyTransQuat(const int vstart, const int vend, const double TQ[]) {
#if DEBUG
        assert(TQ != NULL);
#endif
        double TR[Algebra3D::TransRot_NR_ENTRIES];
        Algebra3D::TransQuatAsTransRot(TQ, TR);
        ApplyTransRot(vstart, vend, TR);
    }

    void TriMesh::ApplyScaling(const int vstart, const int vend,
                               const double x, const double y, const double z) {
#if DEBUG
        assert(vstart >= 0 && vstart < GetNrVertices());
        assert(vend >= 0 && vend < GetNrVertices());
#endif
        for (int i = vstart; i <= vend; ++i) {
            const double *v = GetVertex(i);
            SetVertex(i, v[0] * x, v[1] * y, v[2] * z);
        }
    }

    void TriMesh::ApplyScaling(const int vstart, const int vend, const double s[]) {
#if DEBUG
        assert(s != NULL);
#endif
        ApplyScaling(vstart, vend, s[0], s[1], s[2]);
    }

    void TriMesh::GetBoundingBoxMinMax(const int vstart, const int vend,
                                       double min[], double max[]) const {
#if DEBUG
        assert(vstart >= 0 && vstart < GetNrVertices());
        assert(vend >= 0 && vend < GetNrVertices());
#endif
        min[0] = min[1] = min[2] = HUGE_VAL;
        max[0] = max[1] = max[2] = -HUGE_VAL;

        for (int i = vstart; i <= vend; ++i) {
            const double *p = GetVertex(i);

            if (p[0] < min[0])
                min[0] = p[0];
            if (p[1] < min[1])
                min[1] = p[1];
            if (p[2] < min[2])
                min[2] = p[2];
            if (p[0] > max[0])
                max[0] = p[0];
            if (p[1] > max[1])
                max[1] = p[1];
            if (p[2] > max[2])
                max[2] = p[2];
        }
    }

    const double *TriMesh::GetBoundingBoxMin(void) {
        UpdateBoundingBox();
        return m_bboxMin;
    }

    const double *TriMesh::GetBoundingBoxMax(void) {
        UpdateBoundingBox();
        return m_bboxMax;
    }

    void TriMesh::AdjustToFitBoundingBoxMinMax(const int vstart, const int vend,
                                               const double xmin, const double ymin, const double zmin,
                                               const double xmax, const double ymax, const double zmax) {
#if DEBUG
        assert(vstart >= 0 && vstart < GetNrVertices());
        assert(vend >= 0 && vend < GetNrVertices());
#endif

        double curr_min[3];
        double curr_max[3];

        GetBoundingBoxMinMax(vstart, vend, curr_min, curr_max);

        const double Tx = 0.5 * (curr_max[0] + curr_min[0]);
        const double Ty = 0.5 * (curr_max[1] + curr_min[1]);
        const double Tz = 0.5 * (curr_max[2] + curr_min[2]);

        const double sx = (xmax - xmin) / (curr_max[0] - curr_min[0]);
        const double sy = (ymax - ymin) / (curr_max[1] - curr_min[1]);
        const double sz = (zmax - zmin) / (curr_max[2] - curr_min[2]);
        double s = sx;

        if (sy < s)
            s = sy;
        if (sz < s)
            s = sz;

        for (int i = vstart; i <= vend; ++i) {
            const double *v = GetVertex(i);
            SetVertex(i,
                      sx * (v[0] - Tx) + 0.5 * (xmin + xmax),
                      sy * (v[1] - Ty) + 0.5 * (ymin + ymax),
                      sz * (v[2] - Tz) + 0.5 * (zmin + zmax));
        }
    }

    void TriMesh::AdjustToFitBoundingBoxMinMax(const int vstart, const int vend,
                                               const double min[], const double max[]) {
#if DEBUG
        assert(min != NULL);
        assert(max != NULL);
#endif
        AdjustToFitBoundingBoxMinMax(vstart, vend, min[0], min[1], min[2], max[0], max[1], max[2]);
    }

    void TriMesh::UpdateBoundingBox(void) {
        if (m_updateBBox)
            GetBoundingBoxMinMax(0, GetNrVertices() - 1, m_bboxMin, m_bboxMax);
        m_updateBBox = false;
    }

    void TriMesh::AddFace(Face *const f) {
        m_faces.push_back(f);

        const int fid = GetNrFaces() - 1;
        const int n = f->m_vids.size();
        for (int i = 0; i < n; ++i)
            m_vertices[f->m_vids[i]]->m_fids.push_back(fid);

        OnFaceChange(GetNrFaces() - 1);

        if (m_groups.size() == 0)
            AddGroup();

        f->m_gid = m_gidCurr;
        m_groups[m_gidCurr]->m_fids.push_back(fid);

        f->m_mid = m_midCurr;
        f->m_tid = m_tidCurr;
    }

    void TriMesh::UpdateFace(const int fid) {
        Face *f = m_faces[fid];

        if (!f->m_needsUpdate)
            return;
        f->m_needsUpdate = false;

        f->m_normal[0] = f->m_normal[1] = f->m_normal[2] = 0;
        f->m_centroid[0] = f->m_centroid[1] = f->m_centroid[2] = 0;

        const int n = f->m_vids.size();
        const double *v1 = GetVertex(f->m_vids[n - 1]);
        for (int i = 0; i < n; ++i) {
            const double *v2 = GetVertex(f->m_vids[i]);
            f->m_normal[0] += (v1[1] - v2[1]) * (v1[2] + v2[2]);
            f->m_normal[1] += (v1[2] - v2[2]) * (v1[0] + v2[0]);
            f->m_normal[2] += (v1[0] - v2[0]) * (v1[1] + v2[1]);

            f->m_centroid[0] += v2[0];
            f->m_centroid[1] += v2[1];
            f->m_centroid[2] += v2[2];

            v1 = v2;
        }
        f->m_unnormalized = Algebra3D::VecNorm(f->m_normal);
        Algebra3D::VecScale(f->m_normal, 1 / f->m_unnormalized, f->m_normal);

        f->m_centroid[0] /= n;
        f->m_centroid[1] /= n;
        f->m_centroid[2] /= n;
    }

    void TriMesh::UpdateVertexNormal(const int vid) {
        Vertex *v = m_vertices[vid];

        if (!v->m_needsUpdateNormal)
            return;
        v->m_needsUpdateNormal = false;

        v->m_normal[0] = v->m_normal[1] = v->m_normal[2] = 0;

        const int nf = v->m_fids.size();
        for (int i = 0; i < nf; ++i) {
            const Face *f = GetFace(v->m_fids[i]);

            v->m_normal[0] += f->m_normal[0] * f->m_unnormalized;
            v->m_normal[1] += f->m_normal[1] * f->m_unnormalized;
            v->m_normal[2] += f->m_normal[2] * f->m_unnormalized;
        }
        Algebra3D::VecUnit(v->m_normal, v->m_normal);
    }

    void TriMesh::AddGroup(void) {
        m_groups.push_back(new Group());
        SetCurrentGroup(m_groups.size() - 1);
    }

    void TriMesh::SetCurrentGroup(const int gid) {
        m_gidCurr = gid;
    }

    void TriMesh::AddMaterial(GMaterial *const gMaterial) {
        m_materials.push_back(gMaterial);
    }

    void TriMesh::SetCurrentMaterial(const int mid) {
        m_midCurr = mid;
    }

    void TriMesh::AddTexture(GTexture *const gTexture) {
        m_textures.push_back(gTexture);
    }

    void TriMesh::SetCurrentTexture(const int tid) {
        m_tidCurr = tid;
    }

    void TriMesh::OnVertexChange(const int vid) {
        m_updateBBox = true;

        const int nf = m_vertices[vid]->m_fids.size();
        for (int i = 0; i < nf; ++i)
            m_faces[m_vertices[vid]->m_fids[i]]->m_needsUpdate = true;

        m_vertices[vid]->m_needsUpdateNormal = true;
    }

    void TriMesh::OnFaceChange(const int fid) {
        m_faces[fid]->m_needsUpdate = true;

        const int nv = m_faces[fid]->m_vids.size();
        for (int i = 0; i < nv; ++i)
            m_vertices[m_faces[fid]->m_vids[i]]->m_needsUpdateNormal = true;
    }

    void TriMesh::Draw(void) {
        const int nf = GetNrFaces();
        for (int i = 0; i < nf; ++i) {

            const Face *f = GetFace(i);

            if (FindItem<int>(m_gidsNotForDrawing, f->m_gid) >= 0)
                continue;

            const int fnv = f->m_vids.size();
            const int mid = f->m_mid >= 0 || f->m_gid < 0 ? f->m_mid : m_groups[f->m_gid]->m_mid;
            const int tid = f->m_tid >= 0 || f->m_gid < 0 ? f->m_tid : m_groups[f->m_gid]->m_tid;


            if (mid >= 0 && mid < m_materials.size() && m_materials[mid] != NULL)
                GDrawMaterial(*(m_materials[mid]));

            else if (m_gMainMaterial)
                GDrawMaterial(*m_gMainMaterial);

            const bool tidValid = tid >= 0 && tid < m_textures.size() && m_textures[tid] != NULL;
            const bool hasTex = (tidValid || (m_gMainTexture != NULL));

            if (hasTex) {
                glEnable(GL_TEXTURE_2D);
                GTexture::ManualCoords();
                if (tidValid)
                    m_textures[tid]->Use();
                else
                    m_gMainTexture->Use();
            }

            glBegin(GL_POLYGON);
            for (int j = 0; j < fnv; ++j) {
                const double *vv = GetVertexNormal(f->m_vids[j]);

                glNormal3d(vv[0], vv[1], vv[2]);

                if (hasTex) {
                    int use = -1;
                    if (f->m_vtids.size() == 0)
                        use = f->m_vids[j];
                    else if (j < f->m_vtids.size())
                        use = f->m_vtids[j];

                    if (use >= 0 && use < GetNrTexCoords()) {
                        TexCoord texCoord = GetTexCoord(use);
                        glTexCoord2d(texCoord.m_u, texCoord.m_v);
                    } else {
                        const double *v = GetVertex(f->m_vids[j]);
                        glTexCoord2d(v[0], v[2]);
                    }
                }

                glVertex3dv(GetVertex(f->m_vids[j]));
            }
            glEnd();
        }
    }

    void TriMesh::GenerateRandomTextureCoords(void) {
        TexCoord t;

        const int n = GetNrVertices();
        for (int i = 0; i < n; ++i)
            AddTexCoord(RandomUniformReal(0, 1),
                        RandomUniformReal(0, 1));
    }

    void TriMesh::GetFacesInsideSphere(const double center[3], const double r, std::vector<int> *fids) {
        const int nrFaces = GetNrFaces();
        for (int i = 0; i < nrFaces; ++i)
            if (Algebra3D::PointDistance(GetFace(i)->m_centroid, center) <= r)
                fids->push_back(i);
    }

    int TriMesh::ClosestVertex(const double p[2]) {
        const int nv = GetNrVertices();
        double dmin = HUGE_VAL;
        int imin = -1;
        double d = 0;

        for (int i = 0; i < nv; ++i) {
            const double *v = GetVertex(i);
            if ((d = sqrt((v[0] - p[0]) * (v[0] - p[0]) + (v[1] - p[1]) * (v[1] - p[1]))) < dmin) {
                dmin = d;
                imin = i;
            }
        }
        return imin;
    }

    void TriMesh::AddBox2D(const double xmin,
                           const double ymin,
                           const double xmax,
                           const double ymax) {
        const double quad[] =
                {
                        xmin, ymin, 0,
                        xmax, ymin, 0,
                        xmax, ymax, 0,
                        xmin, ymax, 0};

        AddQuad(quad, true);
    }

    void TriMesh::AddBox(const double xmin,
                         const double ymin,
                         const double zmin,
                         const double xmax,
                         const double ymax,
                         const double zmax) {
        Polygon2D poly;

        poly.AddVertex(xmin, ymin);
        poly.AddVertex(xmax, ymin);
        poly.AddVertex(xmax, ymax);
        poly.AddVertex(xmin, ymax);
        AddExtrudedPolygon(poly, zmin, zmax);
    }

    void TriMesh::AddBoundaries2D(const double xmin,
                                  const double ymin,
                                  const double xmax,
                                  const double ymax,
                                  const double thick) {
        AddBox2D(xmin, ymin - 0.5 * thick,
                 xmax, ymin + 0.5 * thick);

        AddBox2D(xmin, ymax - 0.5 * thick,
                 xmax, ymax + 0.5 * thick);

        AddBox2D(xmin - 0.5 * thick, ymin,
                 xmin + 0.5 * thick, ymax);
        AddBox2D(xmax - 0.5 * thick, ymin,
                 xmax + 0.5 * thick, ymax);
    }

    void TriMesh::AddBoundaries(const double xmin,
                                const double ymin,
                                const double zmin,
                                const double xmax,
                                const double ymax,
                                const double zmax,
                                const double thick) {
        AddBox(xmin, ymin - 0.5 * thick, zmin,
               xmax, ymin + 0.5 * thick, zmax);

        AddBox(xmin, ymax - 0.5 * thick, zmin,
               xmax, ymax + 0.5 * thick, zmax);

        AddBox(xmin - 0.5 * thick, ymin, zmin,
               xmin + 0.5 * thick, ymax, zmax);
        AddBox(xmax - 0.5 * thick, ymin, zmin,
               xmax + 0.5 * thick, ymax, zmax);
    }

    void TriMesh::AddHeightField(const HeightField &hf) {
        double p[4];

        const int nold = GetNrVertices();
        const Grid *grid = hf.GetGrid();
        const int nrDimsX = grid->GetDims()[0];
        const int nrDimsY = grid->GetDims()[1];
        const int ncells = nrDimsX * nrDimsY;
        int coords[2];
        TexCoord ctex;
        Face *face;

        for (int i = 0; i < ncells; ++i) {
            grid->GetCellFromId(i, p, &p[2]);
            p[2] = hf.GetHeightAtCell(i);
            AddVertex(p);
            ctex.m_u = (p[0] - grid->GetMin()[0]) / (grid->GetMax()[0] - grid->GetMin()[0] - grid->GetUnits()[0]);
            ctex.m_v = (p[1] - grid->GetMin()[1]) / (grid->GetMax()[1] - grid->GetMin()[1] - grid->GetUnits()[1]);

            AddTexCoord(ctex);
        }

        for (int i = 0; i < nrDimsX - 1; ++i)
            for (int j = 0; j < nrDimsY - 1; ++j) {
                AddTriangle(nold + j * nrDimsX + i,
                            nold + j * nrDimsX + i + 1,
                            nold + j * nrDimsX + i + 1 + nrDimsX);
                face = m_faces.back();
                face->m_vtids.assign(face->m_vids.begin(), face->m_vids.end());

                AddTriangle(nold + j * nrDimsX + i,
                            nold + j * nrDimsX + i + 1 + nrDimsX,
                            nold + j * nrDimsX + i + nrDimsX);
                face = m_faces.back();
                face->m_vtids.assign(face->m_vids.begin(), face->m_vids.end());
            }
    }

    void TriMesh::ReadOgreMesh(const char meshName[], const char gtexName[]) {
        m_manualTexCoords = true;

        FILE *in = fopen(meshName, "r");

        GTexture *gtex = new GTexture();
        gtex->SetFileName(gtexName);
        SetMainTexture(gtex);

        m_gMainMaterial = new GMaterial();
        m_gMainMaterial->SetAmbient(0.500000, 0.500000, 0.500000, 1.000000);
        m_gMainMaterial->SetDiffuse(0.470588, 0.470588, 0.470588, 1.000000);
        m_gMainMaterial->SetSpecular(0.449020, 0.449020, 0.449020, 1.000000);
        m_gMainMaterial->SetShininess(12.5);
        m_gMainMaterial->SetEmissive(0.000000, 0.000000, 0.000000);

        std::vector<int> tris;
        std::vector<double> pos;
        std::vector<double> norm;
        std::vector<double> tex;
        char str[100];

        while (fscanf(in, "%s", str) == 1) {
            if (strcmp(str, "<face") == 0) {
                if (fscanf(in, "%s", str) != 1) OnInputError(printf("expecting string\n"));
                tris.push_back(atoi(&str[4]));
                if (fscanf(in, "%s", str) != 1) OnInputError(printf("expecting string\n"));
                tris.push_back(atoi(&str[4]));
                if (fscanf(in, "%s", str) != 1) OnInputError(printf("expecting string\n"));
                tris.push_back(atoi(&str[4]));
            } else if (strcmp(str, "<position") == 0) {
                if (fscanf(in, "%s", str) != 1) OnInputError(printf("expecting string\n"));
                pos.push_back(atof(&str[3]));
                if (fscanf(in, "%s", str) != 1) OnInputError(printf("expecting string\n"));
                pos.push_back(atof(&str[3]));
                if (fscanf(in, "%s", str) != 1) OnInputError(printf("expecting string\n"));
                pos.push_back(atof(&str[3]));
            } else if (strcmp(str, "<normal") == 0) {
                if (fscanf(in, "%s", str) != 1) OnInputError(printf("expecting string\n"));
                norm.push_back(atof(&str[3]));
                if (fscanf(in, "%s", str) != 1) OnInputError(printf("expecting string\n"));
                norm.push_back(atof(&str[3]));
                if (fscanf(in, "%s", str) != 1) OnInputError(printf("expecting string\n"));
                norm.push_back(atof(&str[3]));
            } else if (strcmp(str, "<texcoord") == 0) {
                if (fscanf(in, "%s", str) != 1) OnInputError(printf("expecting string\n"));
                tex.push_back(atof(&str[3]));
                if (fscanf(in, "%s", str) != 1) OnInputError(printf("expecting string\n"));
                tex.push_back(atof(&str[3]));
            }
        }

        for (int i = 0; i < pos.size(); i += 3)
            AddVertex(&pos[i], &norm[i]);
        for (int i = 0; i < GetNrVertices(); ++i)
            AddTexCoord(tex[2 * i], tex[2 * i + 1]);

        AddTriangles(tris.size() / 3, &tris[0]);

        for (int i = 0; i < GetNrFaces(); ++i) {
            Face *f = m_faces[i];
            f->m_vtids.assign(f->m_vids.begin(), f->m_vids.end());
        }

        fclose(in);
    }
}
