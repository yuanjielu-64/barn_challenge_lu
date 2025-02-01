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
#include "Components/RegionGeometric.hpp"
#include "Utils/TriMeshDefault.hpp"

namespace Antipatrea
{

bool RegionGeometric::IsNeighbor(RegionGeometric &r, const double dtol)
{
    TriMeshDefault tmesh1;
    TriMeshDefault tmesh2;

    AddToTriMesh(tmesh1);
    r.AddToTriMesh(tmesh2);

    return tmesh1.Collision(NULL, NULL, &tmesh2, NULL, NULL) ||
           tmesh1.DistanceThreshold(NULL, NULL, &tmesh2, NULL, NULL, dtol);
}
}
