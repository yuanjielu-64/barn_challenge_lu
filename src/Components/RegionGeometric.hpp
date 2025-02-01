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

#ifndef Antipatrea__RegionGeometric_HPP_
#define Antipatrea__RegionGeometric_HPP_

#include "Components/Region.hpp"
#include "Utils/TriMesh.hpp"

namespace Antipatrea {

    class RegionGeometric : public Region {
    public:
        RegionGeometric(void) : Region() {
        }

        virtual ~RegionGeometric(void) {
        }

        virtual void GetDrawTextPos(double p[]) {
            GetRepresentativePoint(p);
        }

        virtual void GetRepresentativePoint(double p[]) = 0;

        virtual bool IsPointInside(const double p[]) = 0;

        virtual void SamplePointInside(double p[]) = 0;

        virtual bool IsNeighbor(RegionGeometric &r, const double dtol);

        virtual void AddToTriMesh(TriMesh &tmesh) = 0;

        virtual void DrawShape(void) = 0;
    };
}
#endif
