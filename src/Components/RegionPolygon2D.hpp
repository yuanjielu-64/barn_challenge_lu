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

#ifndef Antipatrea__RegionPolygon2D_HPP_
#define Antipatrea__RegionPolygon2D_HPP_

#include <utility>

#include "Components/Region2D.hpp"
#include "Utils/Polygon2D.hpp"
#include "Utils/GDraw.hpp"

namespace Antipatrea {

    class RegionPolygon2D : public Region2D {
    public:
        RegionPolygon2D(void) {
        }

        virtual ~RegionPolygon2D(void) {
        }

        virtual const Polygon2D *GetPolygon2D(void) const {
            return &m_polygon;
        }

        virtual Polygon2D *GetPolygon2D(void) {
            return &m_polygon;
        }

        virtual void SetupFromParams(Params &params) {
            m_polygon.SetupFromParams(params);
        }

        virtual void SetupFromLaser(const std::vector<double> &xy, int STATE){
            double m = 0.2;

            m_polygon.ReadFromLaser(xy, m);
        }

        virtual void GetRepresentativePoint(double p[]) {
            m_polygon.GetRepresentativePoint(p);
        }

        virtual bool IsPointInside(const double p[]) {
            return GetPolygon2D()->IsPointInside(p);
        }

        virtual void SamplePointInside(double p[]) {
            GetPolygon2D()->SamplePointInside(p);
        }

        virtual void AddToTriMesh(TriMesh &tmesh) {
            tmesh.AddPolygon(m_polygon);
        }

        virtual void DrawShape(void) {
            GDrawPolygon2D(m_polygon);
        }

    protected:
        Polygon2D m_polygon;
    };
}

#endif
