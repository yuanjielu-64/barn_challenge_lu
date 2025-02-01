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

#ifndef Antipatrea__SHEWCHUK_TRIANGLE_HPP_
#define Antipatrea__SHEWCHUK_TRIANGLE_HPP_

#include <cmath>
#include <vector>
#include <cstdio>

namespace Antipatrea
{
    void TriangulatePolygonWithHoles2D(const bool   useConformingDelaunay,
				       const double angleConstraint,
				       const double areaConstraint,
				       const int    totalNrVertices,
				       const double verticesPolyAndHoles[],
				       const int    nrVerticesPerContour[],
				       const int    nrHoles,
				       const double ptsInsideHoles[],
				       std::vector<double> * const triangleVertices,
				       std::vector<int>    * const triangleIndices,
				       std::vector<int>    * const triangleNeighbors);
    
    
    static inline
    void TriangulatePolygonWithNoHoles2D(const bool   useConformingDelaunay,
					 const double angleConstraint,
					 const double areaConstraint,
					 const int    totalNrVertices,
					 const double verticesPoly[],
					 std::vector<double> * const triangleVertices,
					 std::vector<int>    * const triangleIndices,
					 std::vector<int>    * const triangleNeighbors)
    {
	int nrVerticesPerContour = totalNrVertices;
	
	TriangulatePolygonWithHoles2D(useConformingDelaunay,
				      angleConstraint,
				      areaConstraint,
				      totalNrVertices, verticesPoly,
				      &nrVerticesPerContour, 0, NULL,
				      triangleVertices,
				      triangleIndices,
				      triangleNeighbors);
    }
}

#endif
