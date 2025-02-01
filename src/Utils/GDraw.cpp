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
#include "Utils/GDraw.hpp"
#include "Utils/Algebra3D.hpp"
#include "Utils/GText3D.hpp"
#include <cmath>
#include "Utils/Logger.hpp"

namespace Antipatrea {
    struct GDrawData {
        GDrawData(void) {
            m_params[GDRAW_2D] = 1;
            m_params[GDRAW_NR_STACKS] = Constants::GRAPHICS_NR_STACKS;
            m_params[GDRAW_NR_SLICES] = Constants::GRAPHICS_NR_SLICES;
            m_params[GDRAW_FONT_SIZE_X] = Constants::GRAPHICS_FONT_SIZES[0];
            m_params[GDRAW_FONT_SIZE_Y] = Constants::GRAPHICS_FONT_SIZES[1];
            m_params[GDRAW_FONT_SIZE_Z] = Constants::GRAPHICS_FONT_SIZES[2];
            m_params[GDRAW_COMPONENTS] = GDRAW_ALL;
        }

        double m_params[GDRAW_NR_PARAMS];
        GMaterial m_gMaterial;
    };

    static GDrawData *m_gDrawData = NULL;

    GDrawData *GDrawGetData(void) {
        if (m_gDrawData == NULL)
            m_gDrawData = new GDrawData();

        return m_gDrawData;
    }

    void GDrawSetupFromParams(Params &p) {
        GDrawData *d = GDrawGetData();

        d->m_params[GDRAW_NR_STACKS] = p.GetValueAsInt(Constants::KW_NrStacks, (int) d->m_params[GDRAW_NR_STACKS]);
        d->m_params[GDRAW_NR_SLICES] = p.GetValueAsDouble(Constants::KW_NrSlices, (int) d->m_params[GDRAW_NR_SLICES]);

        auto data = p.GetData(Constants::KW_FontSizes);

        if (data) {

            if (data->m_values.size() > 0)
                d->m_params[GDRAW_FONT_SIZE_X] = StrToDouble(data->m_values[0]->c_str());
            if (data->m_values.size() > 1)
                d->m_params[GDRAW_FONT_SIZE_Y] = StrToDouble(data->m_values[1]->c_str());
            if (data->m_values.size() > 2)
                d->m_params[GDRAW_FONT_SIZE_Z] = StrToDouble(data->m_values[2]->c_str());
        }
    }

    double GDrawGetParam(const GDrawParam p) {
        return GDrawGetData()->m_params[p];
    }

    void GDrawSetParam(const GDrawParam p, const double val) {
        GDrawGetData()->m_params[p] = val;
    }

    void GDraw2D(void) {
        glDisable(GL_LIGHTING);
        GDrawSetParam(GDRAW_2D, 1);
    }

    void GDraw3D(void) {
        glEnable(GL_LIGHTING);
        GDrawSetParam(GDRAW_2D, 0);
    }

    void GDrawWireframe(const bool val) {
        if (val)
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        else
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }

    void GDrawColor(const double r, const double g, const double b) {
        glColor3d(r, g, b);
        if (!GDrawIs2D()) {
            GDrawGetData()->m_gMaterial.SetDiffuse(r, g, b);
            GDrawMaterial(GDrawGetData()->m_gMaterial);
        }
    }

    void GDrawLineWidth(const double w) {
        glLineWidth(w);
    }

//2D stuff
    void GDrawString2D(const char s[], const double x, const double y) {
        if (s) {
            glRasterPos2d(x, y);
            for (int i = 0; s[i] != '\0'; ++i)
                glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, s[i]);
        }
    }

    void GDrawPoint2D(const double x, const double y) {
        glBegin(GL_POINTS);
        glVertex3d(x, y, 0);
        glEnd();
    }

    void GDrawPoint2D(const double p[]) {
        GDrawPoint2D(p[0], p[1]);
    }

    void GDrawPoints2D(const int n, const double pts[]) {
        glBegin(GL_POINTS);
        for (int i = 0; i < n; ++i)
            glVertex3d(pts[2 * i], pts[2 * i + 1], 0);
        glEnd();
    }

    void GDrawSegment2D(const double x1, const double y1,
                        const double x2, const double y2) {
        glBegin(GL_LINES);
        glVertex3d(x1, y1, 0);
        glVertex3d(x2, y2, 0);
        glEnd();
    }

    void GDrawSegment2D(const double p1[], const double p2[]) {
        GDrawSegment2D(p1[0], p1[1], p2[0], p2[1]);
    }

    void GDrawSegment2D(const double p1p2[]) {
        GDrawSegment2D(p1p2[0], p1p2[1], p1p2[2], p1p2[3]);
    }

    void GDrawTriangle2D(const double x1, const double y1,
                         const double x2, const double y2,
                         const double x3, const double y3) {
        glNormal3d(0, 0, 1);
        glBegin(GL_TRIANGLES);
        glVertex3d(x1, y1, 0);
        glVertex3d(x2, y2, 0);
        glVertex3d(x3, y3, 0);
        glEnd();
    }

    void GDrawTriangle2D(const double p1[],
                         const double p2[],
                         const double p3[]) {
        GDrawTriangle2D(p1[0], p1[1], p2[0], p2[1], p3[0], p3[1]);
    }

    void GDrawTriangle2D(const double p1p2p3[]) {
        GDrawTriangle2D(p1p2p3[0], p1p2p3[1],
                        p1p2p3[2], p1p2p3[3],
                        p1p2p3[4], p1p2p3[5]);
    }

    void GDrawBox2D(const double minx, const double miny,
                    const double maxx, const double maxy) {
        glNormal3d(0, 0, 1);
        glBegin(GL_QUADS);
        glVertex2d(minx, miny);
        glVertex2d(maxx, miny);
        glVertex2d(maxx, maxy);
        glVertex2d(minx, maxy);
        glEnd();
    }

    void GDrawBox2D(const double min[],
                    const double max[]) {
        GDrawBox2D(min[0], min[1], max[0], max[1]);
    }

    void GDrawBox2D(const double minmax[]) {
        GDrawBox2D(minmax[0], minmax[1], minmax[2], minmax[3]);
    }

    void GDrawQuad2D(const double ax, const double ay,
                     const double bx, const double by,
                     const double cx, const double cy,
                     const double dx, const double dy) {
        glNormal3d(0, 0, 1);
        glBegin(GL_QUADS);
        glVertex3d(ax, ay, 0);
        glVertex3d(bx, by, 0);
        glVertex3d(cx, cy, 0);
        glVertex3d(dx, dy, 0);
        glEnd();
    }

    void GDrawQuad2D(const double a[],
                     const double b[],
                     const double c[],
                     const double d[]) {
        GDrawQuad2D(a[0], a[1], b[0], b[1], c[0], c[1], d[0], d[1]);
    }

    void GDrawQuad2D(const double abcd[]) {
        GDrawQuad2D(abcd[0], abcd[1], abcd[2], abcd[3],
                    abcd[4], abcd[5], abcd[6], abcd[7]);
    }

    void GDrawCircle2D(const double cx, const double cy, const double r) {
        GDrawRegularPolygon2D(cx, cy, r, 10);
    }

    void GDrawRegularPolygon2D(const double cx,
                               const double cy,
                               const double r,
                               const int nsides) {
        const double theta = 2 * M_PI / nsides;
        const double c = cos(theta);
        const double s = sin(theta);
        double x = 1;
        double y = 0;
        double tmp = 0.0;

        glNormal3d(0, 0, 1);
        glBegin(GL_POLYGON);
        for (int i = 0; i <= nsides; i++) {
            glVertex3d(cx + r * x, cy + r * y, 0);
            tmp = x;
            x = c * tmp - s * y;
            y = s * tmp + c * y;
        }
        glEnd();
    }

    void GDrawPolygon2D(Polygon2D &poly) {
        if (poly.IsConvex()) {
            const int n = poly.GetNrVertices();
            glBegin(GL_POLYGON);
            for (int i = 0; i < n; ++i)
                glVertex2d(poly.GetVertexX(i), poly.GetVertexY(i));
            glEnd();
        } else {
            const int n = poly.GetNrTriangles();
            double tri[6];

            glBegin(GL_TRIANGLES);
            for (int i = 0; i < n; ++i) {
                poly.GetTriangleVertices(i, tri);
                glVertex2d(tri[0], tri[1]);
                glVertex2d(tri[2], tri[3]);
                glVertex2d(tri[4], tri[5]);
            }
            glEnd();
        }
    }

    const double *GDrawStandardCircleToPolygon2D(const int nrSides) {
        static std::vector<double> stdPoly;

        if ((2 * nrSides) != stdPoly.size()) {
            stdPoly.resize(2 * nrSides);
            CircleAsPolygon2D(0, 0, 1.0, nrSides, &(stdPoly[0]));
        }

        return &(stdPoly[0]);
    }

    void GDrawMaterial(const GMaterial &mat) {
        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mat.GetPropertyValues(GMaterial::AMBIENT));
        glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat.GetPropertyValues(GMaterial::DIFFUSE));
        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat.GetPropertyValues(GMaterial::SPECULAR));
        glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, mat.GetPropertyValues(GMaterial::SHININESS)[0]);
    }

    void GDrawIllumination(const GIllumination &gillum) {
        const int n = gillum.GetNrLights();

        glLightModeli(GL_LIGHT_MODEL_COLOR_CONTROL, GL_SEPARATE_SPECULAR_COLOR);
        glLightModelfv(GL_LIGHT_MODEL_AMBIENT, gillum.GetPropertyValues(GIllumination::AMBIENT));
        glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);

        for (int i = 0; i < n && i < 8; ++i) {
            const GLight *light = gillum.GetLight(i);
            glLightfv(GL_LIGHT0 + i, GL_AMBIENT, light->GetPropertyValues(GLight::AMBIENT));
            glLightfv(GL_LIGHT0 + i, GL_DIFFUSE, light->GetPropertyValues(GLight::DIFFUSE));
            glLightfv(GL_LIGHT0 + i, GL_SPECULAR, light->GetPropertyValues(GLight::SPECULAR));
            glLightfv(GL_LIGHT0 + i, GL_POSITION, light->GetPropertyValues(GLight::POSITION));
            glEnable(GL_LIGHT0 + i);
        }

        for (int i = n; i < 8; ++i)
            glDisable(GL_LIGHT0 + i);

    }

    void GDrawPushTransformation(void) {
        glPushMatrix();
    }

    void GDrawPopTransformation(void) {
        glPopMatrix();
    }

    void GDrawMultTrans(const double T[]) {
        GDrawMultTrans(T[0], T[1], T[2]);
    }

    void GDrawMultTrans(const double x, const double y, const double z) {
        glTranslated(x, y, z);
    }

    void GDrawMultRot(const double R[]) {
        const double T[3] = {0, 0, 0};
        GDrawMultTransRot(T, R);
    }

    void GDrawMultTransRot(const double x, const double y, const double z,
                           const double R[]) {
        const double T[3] = {x, y, z};
        GDrawMultTransRot(T, R);
    }

    void GDrawMultTransRot(const double T[], const double R[]) {
        double m[16];

        m[0] = R[0];
        m[1] = R[3];
        m[2] = R[6];
        m[4] = R[1];
        m[5] = R[4];
        m[6] = R[7];
        m[8] = R[2];
        m[9] = R[5];
        m[10] = R[8];
        m[3] = m[7] = m[11] = 0;
        m[15] = 1;

        m[12] = T[0];
        m[13] = T[1];
        m[14] = T[2];

        glMultMatrixd(m);
    }

    void GDrawMultFromZAxisToAxis(const double nx, const double ny, const double nz) {
        const double norm[3] = {nx, ny, nz};

        GDrawMultFromZAxisToAxis(norm);
    }

    void GDrawMultFromZAxisToAxis(const double norm[]) {
        double R[Algebra3D::Rot_NR_ENTRIES];
        const double zaxis[3] = {0, 0, 1};
        Algebra3D::FromToAxisAsRot(zaxis, norm, Algebra3D::VecNorm(norm), R);
        GDrawMultRot(R);
    }

    void GDrawScale(const double sx, const double sy, const double sz) {
        glScaled(sx, sy, sz);
    }

    void GDrawScale(const double s[]) {
        GDrawScale(s[0], s[1], s[2]);
    }

    void GDrawScale(const double ox, const double oy, const double oz,
                    const double sx, const double sy, const double sz) {
        GDrawMultTrans(ox, oy, oz);
        GDrawScale(sx, sy, sz);
        GDrawMultTrans(-ox, -oy, -oz);
    }

    void GDrawScale(const double o[], const double s[]) {
        GDrawScale(o[0], o[1], o[2], s[0], s[1], s[2]);
    }

    void GDrawString3D(const char s[],
                       const double x,
                       const double y,
                       const double z,
                       const bool rotate) {

        if (s) {
            t3dInit();
            glPushMatrix();
            glTranslatef(x, y, z);
            if (rotate) {
                glScalef(GDrawGetParam(GDRAW_FONT_SIZE_X),
                         GDrawGetParam(GDRAW_FONT_SIZE_Y),
                         GDrawGetParam(GDRAW_FONT_SIZE_Z));
                glRotatef(90, 1, 0, 0);
            } else {
                glScalef(GDrawGetParam(GDRAW_FONT_SIZE_X),
                         GDrawGetParam(GDRAW_FONT_SIZE_Y),
                         GDrawGetParam(GDRAW_FONT_SIZE_Z));
            }
            t3dDraw3D(s, 0, 1, 0.1f);
            glPopMatrix();
        }
    }

    void GDrawPoint3D(const double x, const double y, const double z) {
        glBegin(GL_POINTS);
        glVertex3d(x, y, z);
        glEnd();
    }

    void GDrawPoint3D(const double p[]) {
        GDrawPoint3D(p[0], p[1], p[2]);
    }

    void GDrawSegment3D(const double x1, const double y1, const double z1,
                        const double x2, const double y2, const double z2) {
        glBegin(GL_LINES);
        glVertex3d(x1, y1, z1);
        glVertex3d(x2, y2, z2);
        glEnd();
    }

    void GDrawSegment3D(const double p1[], const double p2[]) {
        glBegin(GL_LINES);
        glVertex3dv(p1);
        glVertex3dv(p2);
        glEnd();
    }

    void GDrawSegment3D(const double p1p2[]) {
        GDrawSegment3D(p1p2, &p1p2[3]);
    }

    void GDrawTriangle3D(const double ax, const double ay, const double az,
                         const double bx, const double by, const double bz,
                         const double cx, const double cy, const double cz) {
        glBegin(GL_TRIANGLES);
        glVertex3d(ax, ay, az);
        glVertex3d(bx, by, bz);
        glVertex3d(cx, cy, cz);
        glEnd();
    }

    void GDrawTriangle3D(const double v1[3],
                         const double v2[3],
                         const double v3[3]) {
        glBegin(GL_TRIANGLES);
        glVertex3dv(v1);
        glVertex3dv(v2);
        glVertex3dv(v3);
        glEnd();
    }

    void GDrawTriangles3D(const int n, const double tv[]) {
        glBegin(GL_TRIANGLES);
        for (int i = 0; i < n; ++i) {
            glVertex3dv(&tv[9 * i]);
            glVertex3dv(&tv[9 * i + 3]);
            glVertex3dv(&tv[9 * i + 6]);
        }
        glEnd();
    }

    void GDrawParallelogram3D(const double ox, const double oy, const double oz,
                              const double ax, const double ay, const double az,
                              const double bx, const double by, const double bz) {
        glBegin(GL_QUADS);
        glVertex3d(ox, oy, oz);
        glVertex3d(ox + ax, oy + ay, oz + az);
        glVertex3d(ox + ax + bx, oy + ay + by, oz + az + bz);
        glVertex3d(ox + bx, oy + by, oz + bz);
        glEnd();
    }

    void GDrawParallelogram3D(const double o[3],
                              const double a[3],
                              const double b[3]) {
        GDrawParallelogram3D(o[0], o[1], o[2], a[0], a[1], a[2], b[0], b[1], b[2]);
    }

    void GDrawSphere3D(const double c[], const double r) {
        GDrawSphere3D(c[0], c[1], c[2], r);
    }

    void GDrawSphere3D(const double cx, const double cy, const double cz, const double r) {
        glPushMatrix();
        glTranslated(cx, cy, cz);
        GDrawSphere3D(r);
        glPopMatrix();
    }

    void GDrawSphere3D(const double radius) {
        const int slices = (int) GDrawGetParam(GDRAW_NR_SLICES);
        const int stacks = (int) GDrawGetParam(GDRAW_NR_STACKS);

        const double theta1 = 2 * M_PI / slices;
        const double cos_theta1 = cos(theta1);
        const double sin_theta1 = sin(theta1);
        double x1 = 1;
        double y1 = 0;

        const double theta2 = M_PI / stacks;
        const double cos_theta2 = cos(theta2);
        const double sin_theta2 = sin(theta2);
        double x2 = cos_theta2;
        double y2 = sin_theta2;

        double z0 = 1.0;
        double r0 = 0.0;
        double z1 = -x2;
        double r1 = y2;

        glBegin(GL_TRIANGLE_FAN);
        glNormal3d(0, 0, -1);
        glVertex3d(0, 0, -radius);
        for (int j = slices - 1; j >= 0; --j) {
            glNormal3d(r1 * x1, r1 * y1, z1);
            glVertex3d(radius * r1 * x1, radius * r1 * y1, radius * z1);

            const double tmp1 = x1;
            x1 = cos_theta1 * tmp1 + sin_theta1 * y1;
            y1 = -sin_theta1 * tmp1 + cos_theta1 * y1;
        }
        glEnd();

        z1 = -z1;
        x1 = 1;
        y1 = 0;
        glBegin(GL_TRIANGLE_FAN);
        glNormal3d(0, 0, 1);
        glVertex3d(0, 0, radius);
        for (int j = 0; j <= slices; ++j) {
            glNormal3d(r1 * x1, r1 * y1, z1);
            glVertex3d(radius * r1 * x1, radius * r1 * y1, radius * z1);

            const double tmp1 = x1;
            x1 = cos_theta1 * tmp1 - sin_theta1 * y1;
            y1 = sin_theta1 * tmp1 + cos_theta1 * y1;
        }
        glEnd();

        for (int i = 1; i < stacks - 1; ++i) {
            const double tmp2 = x2;
            x2 = cos_theta2 * tmp2 - sin_theta2 * y2;
            y2 = sin_theta2 * tmp2 + cos_theta2 * y2;

            z0 = z1;
            z1 = x2;
            r0 = r1;
            r1 = y2;

            x1 = 1;
            y1 = 0;
            glBegin(GL_QUAD_STRIP);
            for (int j = 0; j <= slices; ++j) {
                glNormal3d(r1 * x1, r1 * y1, z1);
                glVertex3d(radius * r1 * x1, radius * r1 * y1, z1 * radius);

                glNormal3d(r0 * x1, r0 * y1, z0);
                glVertex3d(r0 * radius * x1, r0 * radius * y1, z0 * radius);

                const double tmp1 = x1;
                x1 = cos_theta1 * tmp1 - sin_theta1 * y1;
                y1 = sin_theta1 * tmp1 + cos_theta1 * y1;
            }
            glEnd();
        }
    }

    void GDrawEllipsoid3D(const double c[], const double rx, const double ry, const double rz) {
        GDrawEllipsoid3D(c[0], c[1], c[2], rx, ry, rz);
    }

    void GDrawEllipsoid3D(const double cx, const double cy, const double cz,
                          const double rx, const double ry, const double rz) {
        glPushMatrix();
        glTranslated(cx, cy, cz);
        glScaled(rx, ry, rz);
        GDrawSphere3D(1);
        glPopMatrix();
    }

    void GDrawEllipsoid3D(const double rx, const double ry, const double rz) {
        glPushMatrix();
        glScaled(rx, ry, rz);
        GDrawSphere3D(1);
        glPopMatrix();
    }

    void GDrawTorus3D(const double c[], const double rin, const double rout) {
        GDrawTorus3D(c[0], c[1], c[2], rin, rout);
    }

    void GDrawTorus3D(const double cx, const double cy, const double cz,
                      const double rin, const double rout) {
        glPushMatrix();
        glTranslated(cx, cy, cz);
        GDrawTorus3D(rin, rout);
        glPopMatrix();
    }

    void GDrawTorus3D(const double rin, const double rout) {
        const int nSides = 1 + (int) GDrawGetParam(GDRAW_NR_SLICES);
        const int nRings = 1 + (int) GDrawGetParam(GDRAW_NR_STACKS);

        double *vertex = (double *) calloc(sizeof(double), 3 * nSides * nRings);
        double *normal = (double *) calloc(sizeof(double), 3 * nSides * nRings);

        const double dpsi = 2.0 * M_PI / (nRings - 1);
        const double cdpsi = cos(dpsi);
        const double sdpsi = sin(dpsi);
        double cpsi = 1;
        double spsi = 0;

        const double dphi = -2.0 * M_PI / (nSides - 1);
        const double cdphi = cos(dphi);
        const double sdphi = sin(dphi);
        double cphi = 1;
        double sphi = 0;

        double tmp = 0;

        for (int j = 0; j < nRings; ++j) {
            cphi = 1;
            sphi = 0;

            for (int i = 0; i < nSides; ++i) {
                const int offset = 3 * (j * nSides + i);

                *(vertex + offset + 0) = cpsi * (rout + cphi * rin);
                *(vertex + offset + 1) = spsi * (rout + cphi * rin);
                *(vertex + offset + 2) = sphi * rin;
                *(normal + offset + 0) = cpsi * cphi;
                *(normal + offset + 1) = spsi * cphi;
                *(normal + offset + 2) = sphi;

                tmp = cphi;
                cphi = tmp * cdphi - sphi * sdphi;
                sphi = sphi * cdphi + tmp * sdphi;
            }

            tmp = cpsi;
            cpsi = tmp * cdpsi - spsi * sdpsi;
            spsi = spsi * cdpsi + tmp * sdpsi;
        }

        glBegin(GL_QUADS);
        for (int i = 0; i < nSides - 1; ++i)
            for (int j = 0; j < nRings - 1; ++j) {
                const int offset = 3 * (j * nSides + i);
                glNormal3dv(normal + offset);
                glVertex3dv(vertex + offset);
                glNormal3dv(normal + offset + 3);
                glVertex3dv(vertex + offset + 3);
                glNormal3dv(normal + offset + 3 * nSides + 3);
                glVertex3dv(vertex + offset + 3 * nSides + 3);
                glNormal3dv(normal + offset + 3 * nSides);
                glVertex3dv(vertex + offset + 3 * nSides);
            }
        glEnd();

        free(vertex);
        free(normal);
    }

    void GDrawBox3D(const double dimx, const double dimy, const double dimz) {
        /*         y-pos

               3___________
               /|          /|2
               / |         / |
               /  |        /  |  xpos
               7------------6  |
               |  |_______|___|
               |  /0      |  /1
               | /        | /
               |/_________|/
               4          5
               z-pos
        */

        const double hx = 0.5 * dimx;
        const double hy = 0.5 * dimy;
        const double hz = 0.5 * dimz;
        const double ncoeff = 1.0 / sqrt(3.0);

        const double vert[24] =
                {
                        -hx, -hy, -hz,
                        hx, -hy, -hz,
                        hx, hy, -hz,
                        -hx, hy, -hz,

                        -hx, -hy, hz,
                        hx, -hy, hz,
                        hx, hy, hz,
                        -hx, hy, hz,
                };

        const double norms[24] =
                {
                        -ncoeff, -ncoeff, -ncoeff,
                        ncoeff, -ncoeff, -ncoeff,
                        ncoeff, ncoeff, -ncoeff,
                        -ncoeff, ncoeff, -ncoeff,

                        -ncoeff, -ncoeff, ncoeff,
                        ncoeff, -ncoeff, ncoeff,
                        ncoeff, ncoeff, ncoeff,
                        -ncoeff, ncoeff, ncoeff};

        glBegin(GL_QUADS);
        glNormal3dv(&norms[0]);
        glVertex3dv(&vert[0]);
        glNormal3dv(&norms[12]);
        glVertex3dv(&vert[12]);
        glNormal3dv(&norms[15]);
        glVertex3dv(&vert[15]);
        glNormal3dv(&norms[3]);
        glVertex3dv(&vert[3]);

        glNormal3dv(&norms[21]);
        glVertex3dv(&vert[21]);
        glNormal3dv(&norms[18]);
        glVertex3dv(&vert[18]);
        glNormal3dv(&norms[6]);
        glVertex3dv(&vert[6]);
        glNormal3dv(&norms[9]);
        glVertex3dv(&vert[9]);

        glNormal3dv(&norms[0]);
        glVertex3dv(&vert[0]);
        glNormal3dv(&norms[12]);
        glVertex3dv(&vert[12]);
        glNormal3dv(&norms[21]);
        glVertex3dv(&vert[21]);
        glNormal3dv(&norms[9]);
        glVertex3dv(&vert[9]);

        glNormal3dv(&norms[15]);
        glVertex3dv(&vert[15]);
        glNormal3dv(&norms[3]);
        glVertex3dv(&vert[3]);
        glNormal3dv(&norms[6]);
        glVertex3dv(&vert[6]);
        glNormal3dv(&norms[18]);
        glVertex3dv(&vert[18]);

        glNormal3dv(&norms[9]);
        glVertex3dv(&vert[9]);
        glNormal3dv(&norms[6]);
        glVertex3dv(&vert[6]);
        glNormal3dv(&norms[3]);
        glVertex3dv(&vert[3]);
        glNormal3dv(&norms[0]);
        glVertex3dv(&vert[0]);

        glNormal3dv(&norms[12]);
        glVertex3dv(&vert[12]);
        glNormal3dv(&norms[15]);
        glVertex3dv(&vert[15]);
        glNormal3dv(&norms[18]);
        glVertex3dv(&vert[18]);
        glNormal3dv(&norms[21]);
        glVertex3dv(&vert[21]);

        glEnd();
    }

    void GDrawBox3D(const double dims[]) {
        GDrawBox3D(dims[0], dims[1], dims[2]);
    }

    void GDrawBox3D(const double minx, const double miny, const double minz,
                    const double maxx, const double maxy, const double maxz) {
        glPushMatrix();
        glTranslated(0.5 * (minx + maxx), 0.5 * (miny + maxy), 0.5 * (minz + maxz));
        GDrawBox3D(maxx - minx, maxy - miny, maxz - minz);
        glPopMatrix();

    }

    void GDrawBox3D(const double min[], const double max[]) {
        GDrawBox3D(min[0], min[1], min[2], max[0], max[1], max[2]);
    }

    void GDrawParallelepiped3D(const double cx, const double cy, const double cz,
                               const double x0, const double x1, const double x2,
                               const double y0, const double y1, const double y2,
                               const double z0, const double z1, const double z2) {
        const double c[3] = {cx, cy, cz};
        const double x[3] = {x0, x1, x2};
        const double y[3] = {y0, y1, y2};
        const double z[3] = {z0, z1, z2};

        GDrawParallelepiped3D(c, x, y, z);
    }

    void GDrawParallelepiped3D(const double c[],
                               const double uxaxis[],
                               const double uyaxis[],
                               const double uzaxis[],
                               const double dims[]) {
        const double x[3] = {dims[0] * uxaxis[0], dims[1] * uxaxis[1], dims[2] * uxaxis[2]};
        const double y[3] = {dims[0] * uyaxis[0], dims[1] * uyaxis[1], dims[2] * uyaxis[2]};
        const double z[3] = {dims[0] * uzaxis[0], dims[1] * uzaxis[1], dims[2] * uzaxis[2]};

        GDrawParallelepiped3D(c, x, y, z);
    }

    void GDrawParallelepiped3D(const double c[],
                               const double xaxis[],
                               const double yaxis[],
                               const double zaxis[]) {
        /*         y-pos

               3___________
               /|          /|2
               / |         / |
               /  |        /  |  xpos
               7------------6  |
               |  |_______|___|
               |  /0      |  /1
               | /        | /
               |/_________|/
               4          5
               z-pos
        */

        const int signs[24] =
                {
                        -1, -1, -1,
                        1, -1, -1,
                        1, 1, -1,
                        -1, 1, -1,

                        -1, -1, 1,
                        1, -1, 1,
                        1, 1, 1,
                        -1, 1, 1,
                };

        double vert[24];
        for (int i = 0; i < 24; i += 3) {
            vert[i] = c[0] + 0.5 * (signs[i] * xaxis[0] + signs[i + 1] * yaxis[0] + signs[i + 2] * zaxis[0]);

            vert[i + 1] = c[1] + 0.5 * (signs[i] * xaxis[1] + signs[i + 1] * yaxis[1] + signs[i + 2] * zaxis[1]);

            vert[i + 2] = c[2] + 0.5 * (signs[i] * xaxis[2] + signs[i + 1] * yaxis[2] + signs[i + 2] * zaxis[2]);
        }

        double normal_xy[3];
        double normal_yz[3];
        double normal_zx[3];

        Algebra3D::VecCrossProduct(xaxis, yaxis, normal_xy);
        Algebra3D::VecCrossProduct(yaxis, zaxis, normal_yz);
        Algebra3D::VecCrossProduct(zaxis, xaxis, normal_zx);

        double norms[24];
        for (int i = 0; i < 24; i += 3) {
            norms[i] =
                    signs[i] * normal_yz[0] +
                    signs[i + 1] * normal_zx[0] +
                    signs[i + 2] * normal_xy[0];

            norms[i + 1] =
                    signs[i] * normal_yz[1] +
                    signs[i + 1] * normal_zx[1] +
                    signs[i + 2] * normal_xy[1];

            norms[i + 2] =
                    signs[i] * normal_yz[2] +
                    signs[i + 1] * normal_zx[2] +
                    signs[i + 2] * normal_xy[2];

            Algebra3D::VecUnit(&norms[i], &norms[i]);
        }

        glBegin(GL_QUADS);
        glNormal3dv(&norms[0]);
        glVertex3dv(&vert[0]);
        glNormal3dv(&norms[12]);
        glVertex3dv(&vert[12]);
        glNormal3dv(&norms[15]);
        glVertex3dv(&vert[15]);
        glNormal3dv(&norms[3]);
        glVertex3dv(&vert[3]);

        glNormal3dv(&norms[21]);
        glVertex3dv(&vert[21]);
        glNormal3dv(&norms[18]);
        glVertex3dv(&vert[18]);
        glNormal3dv(&norms[6]);
        glVertex3dv(&vert[6]);
        glNormal3dv(&norms[9]);
        glVertex3dv(&vert[9]);

        glNormal3dv(&norms[0]);
        glVertex3dv(&vert[0]);
        glNormal3dv(&norms[12]);
        glVertex3dv(&vert[12]);
        glNormal3dv(&norms[21]);
        glVertex3dv(&vert[21]);
        glNormal3dv(&norms[9]);
        glVertex3dv(&vert[9]);

        glNormal3dv(&norms[15]);
        glVertex3dv(&vert[15]);
        glNormal3dv(&norms[3]);
        glVertex3dv(&vert[3]);
        glNormal3dv(&norms[6]);
        glVertex3dv(&vert[6]);
        glNormal3dv(&norms[18]);
        glVertex3dv(&vert[18]);

        glNormal3dv(&norms[9]);
        glVertex3dv(&vert[9]);
        glNormal3dv(&norms[6]);
        glVertex3dv(&vert[6]);
        glNormal3dv(&norms[3]);
        glVertex3dv(&vert[3]);
        glNormal3dv(&norms[0]);
        glVertex3dv(&vert[0]);

        glNormal3dv(&norms[12]);
        glVertex3dv(&vert[12]);
        glNormal3dv(&norms[15]);
        glVertex3dv(&vert[15]);
        glNormal3dv(&norms[18]);
        glVertex3dv(&vert[18]);
        glNormal3dv(&norms[21]);
        glVertex3dv(&vert[21]);

        glEnd();

        for (int i = 0; i < 24; i += 3)
            GDrawSegment3D(vert[i], vert[i + 1], vert[i + 2],
                           vert[i] + norms[i],
                           vert[i + 1] + norms[i + 1],
                           vert[i + 2] + norms[i + 2]);
    }

//pyramid
    void GDrawPyramid3D(const int n,
                        const double base_xy[],
                        const double px,
                        const double py,
                        const double pz) {
        GDrawPyramid3D(n, base_xy, px, py, pz, 0, 0, 1, 1);
    }

    void GDrawPyramid3D(const int n,
                        const double base_xy[],
                        const double px,
                        const double py,
                        const double pz,
                        const double basex_scale,
                        const double basey_scale) {
        double cx = 0;
        double cy = 0;
        for (int i = 0; i < n; ++i) {
            cx += base_xy[2 * i];
            cy += base_xy[2 * i + 1];
        }
        GDrawPyramid3D(n, base_xy,
                       px, py, pz,
                       cx / n, cy / n,
                       basex_scale, basey_scale);
    }

    void GDrawPyramid3D(const int n,
                        const double base_xy[],
                        const double px,
                        const double py,
                        const double pz,
                        const double ox_scale,
                        const double oy_scale,
                        const double basex_scale,
                        const double basey_scale) {
        const int drawComponents = (int) GDrawGetParam(GDRAW_COMPONENTS);
        //base base
        if (drawComponents & GDRAW_BASE) {
            glNormal3d(0, 0, pz > 0 ? -1 : 1);
            glBegin(GL_POLYGON);
            for (int i = 0; i < n; ++i)
                glVertex3d(ox_scale + (base_xy[2 * i] - ox_scale) * basex_scale,
                           oy_scale + (base_xy[2 * i + 1] - oy_scale) * basey_scale, 0);
            glEnd();
        }
        //side
        if (drawComponents & GDRAW_SIDE) {
            double norm[3];

            glBegin(GL_TRIANGLE_FAN);
            glNormal3d(0, 0, pz > 0 ? 1 : -1);
            glVertex3d(px, py, pz);
            for (int i = 0; i < n; ++i) {
                const int next = (i + 1) % n;

                const double a[] = {base_xy[2 * next] - base_xy[2 * i],
                                    base_xy[2 * next + 1] - base_xy[2 * i + 1], 0};
                const double b[] = {px - base_xy[2 * i], py - base_xy[2 * i + 1], pz};

                Algebra3D::VecCrossProduct(a, b, norm);
                Algebra3D::VecUnit(norm, norm);

                glNormal3dv(norm);
                glVertex3d(ox_scale + (base_xy[2 * i] - ox_scale) * basex_scale,
                           oy_scale + (base_xy[2 * i + 1] - oy_scale) * basey_scale, 0);
            }
            glEnd();
        }
    }

    void GDrawCappedPyramid3D(const int n,
                              const double base_xy[],
                              const double ax,
                              const double ay,
                              const double az) {
        GDrawCappedPyramid3D(n, base_xy, ax, ay, az, 0, 0, 1, 1, 1, 1);
    }

    void GDrawCappedPyramid3D(const int n,
                              const double base_xy[],
                              const double ax,
                              const double ay,
                              const double az,
                              const double basex_scale,
                              const double basey_scale,
                              const double topx_scale,
                              const double topy_scale) {
        double cx = 0;
        double cy = 0;
        for (int i = 0; i < n; ++i) {
            cx += base_xy[2 * i];
            cy += base_xy[2 * i + 1];
        }
        GDrawCappedPyramid3D(n, base_xy,
                             ax, ay, az,
                             cx / n, cy / n,
                             basex_scale, basey_scale,
                             topx_scale, topy_scale);
    }

    void GDrawCappedPyramid3D(const int n,
                              const double base_xy[],
                              const double ax,
                              const double ay,
                              const double az,
                              const double ox_scale,
                              const double oy_scale,
                              const double basex_scale,
                              const double basey_scale,
                              const double topx_scale,
                              const double topy_scale) {
        const int drawComponents = (int) GDrawGetParam(GDRAW_COMPONENTS);
        //base base
        if (drawComponents & GDRAW_BASE) {
            glNormal3d(0, 0, az > 0 ? -1 : 1);
            glBegin(GL_POLYGON);
            for (int i = 0; i < n; ++i)
                glVertex3d(ox_scale + (base_xy[2 * i] - ox_scale) * basex_scale,
                           oy_scale + (base_xy[2 * i + 1] - oy_scale) * basey_scale, 0);
            glEnd();
        }
        //top base
        if (drawComponents & GDRAW_TOP) {
            glNormal3d(0, 0, az > 0 ? 1 : -1);
            glBegin(GL_POLYGON);
            for (int i = 0; i < n; ++i)
                glVertex3d(ox_scale + (base_xy[2 * i] - ox_scale) * topx_scale + ax,
                           ox_scale + (base_xy[2 * i + 1] - oy_scale) * topy_scale + ay, az);
            glEnd();
        }
        //side
        if (drawComponents & GDRAW_SIDE) {
            double norm[3];

            glBegin(GL_QUADS);
            for (int i = 0; i < n; ++i) {
                const int next = i == n - 1 ? 0 : i + 1;

                const double a[] = {base_xy[2 * next] - base_xy[2 * i],
                                    base_xy[2 * next + 1] - base_xy[2 * i + 1], 0};
                const double b[] = {base_xy[2 * next] + ax - base_xy[2 * i],
                                    base_xy[2 * next + 1] + ay - base_xy[2 * i + 1], az};

                Algebra3D::VecCrossProduct(a, b, norm);
                Algebra3D::VecUnit(norm, norm);

                glNormal3dv(norm);
                glVertex3d(ox_scale + (base_xy[2 * i] - ox_scale) * basex_scale,
                           oy_scale + (base_xy[2 * i + 1] - oy_scale) * basey_scale, 0);

                glVertex3d(ox_scale + (base_xy[2 * next] - ox_scale) * basex_scale,
                           oy_scale + (base_xy[2 * next + 1] - ox_scale) * basey_scale, 0);

                glVertex3d(ox_scale + (base_xy[2 * next] - ox_scale) * topx_scale + ax,
                           oy_scale + (base_xy[2 * next + 1] - oy_scale) * topy_scale + ay, az);

                glVertex3d(ox_scale + (base_xy[2 * i] - ox_scale) * topx_scale + ax,
                           oy_scale + (base_xy[2 * i + 1] - oy_scale) * topy_scale + ay, az);
            }
            glEnd();
        }
    }

//prism
    void GDrawPrism3D(const int n,
                      const double base_xy[],
                      const double ax,
                      const double ay,
                      const double az,
                      const double ox_scale,
                      const double oy_scale,
                      const double basex_scale,
                      const double basey_scale) {
        GDrawCappedPyramid3D(n, base_xy, ax, ay, az, ox_scale, oy_scale,
                             basex_scale, basey_scale,
                             basex_scale, basey_scale);
    }

    void GDrawPrism3D(const int n,
                      const double base_xy[],
                      const double ax,
                      const double ay,
                      const double az,
                      const double basex_scale,
                      const double basey_scale) {
        GDrawCappedPyramid3D(n, base_xy, ax, ay, az,
                             basex_scale, basey_scale,
                             basex_scale, basey_scale);
    }

    void GDrawPrism3D(const int n,
                      const double base_xy[],
                      const double ax,
                      const double ay,
                      const double az) {
        GDrawCappedPyramid3D(n, base_xy, ax, ay, az);
    }

//cylinder
    void GDrawCylinder3D(const double r,
                         const double h) {
        GDrawCylinder3D(r, r, 0, 0, h);
    }

    void GDrawCylinder3D(const double rx,
                         const double ry,
                         const double ax,
                         const double ay,
                         const double az) {
        GDrawCappedPyramid3D((int) GDrawGetParam(GDRAW_NR_SLICES),
                             GDrawStandardCircleToPolygon2D((int) GDrawGetParam(GDRAW_NR_SLICES)),
                             ax, ay, az,
                             0, 0,
                             rx, ry, rx, ry);
    }

    void GDrawCylinder3D(const double r,
                         const double x1, const double y1, const double z1,
                         const double x2, const double y2, const double z2) {
        const double T[3] = {x2 - x1, y2 - y1, z2 - z1};
        const double norm = Algebra3D::VecNorm(T);
        const double zaxis[3] = {0, 0, 1};
        double R[Algebra3D::Rot_NR_ENTRIES];

        Algebra3D::FromToAxisAsRot(zaxis, T, norm, R);

        GDrawPushTransformation();
        GDrawMultTransRot(x1, y1, z1, R);
        GDrawCylinder3D(r, norm);
        GDrawPopTransformation();
    }

//conema
    void GDrawCone3D(const double r,
                     const double h) {
        GDrawCone3D(r, 0, 0, h);
    }

    void GDrawCone3D(const double r,
                     const double px,
                     const double py,
                     const double pz) {
        GDrawPyramid3D((int) GDrawGetParam(GDRAW_NR_SLICES),
                       GDrawStandardCircleToPolygon2D((int) GDrawGetParam(GDRAW_NR_SLICES)),
                       px, py, pz, 0, 0, r, r);
    }

    void GDrawCone3D(const double rx,
                     const double ry,
                     const double px,
                     const double py,
                     const double pz) {
        GDrawPyramid3D((int) GDrawGetParam(GDRAW_NR_SLICES),
                       GDrawStandardCircleToPolygon2D((int) GDrawGetParam(GDRAW_NR_SLICES)),
                       px, py, pz, 0, 0, rx, ry);
    }

    void GDrawCappedCone3D(const double rbase,
                           const double rtop,
                           const double h) {
        GDrawCappedCone3D(rbase, rbase, rtop, rtop, 0, 0, h);
    }

    void GDrawCappedCone3D(const double base_rx,
                           const double base_ry,
                           const double top_rx,
                           const double top_ry,
                           const double ax,
                           const double ay,
                           const double az) {
        GDrawCappedPyramid3D((int) GDrawGetParam(GDRAW_NR_SLICES),
                             GDrawStandardCircleToPolygon2D((int) GDrawGetParam(GDRAW_NR_SLICES)),
                             ax, ay, az, 0, 0, base_rx, base_ry, top_rx, top_ry);
    }
}
