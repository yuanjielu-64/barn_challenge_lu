
#ifndef PQP_TRIDIST_H
#define PQP_TRIDIST_H

#include "PQP_Compile.h"

// TriDist()
//
// computes the closest points on two triangles, and returns the 
// distance between them.
// 
// s and t are the triangles, stored tri[point][dimension].
//
// If the triangles are disjoint, p and q give the closest points of 
// s and t respectively. However, if the triangles overlap, p and q 
// are basically a random pair of points from the triangles, not 
// coincident points on the intersection of the triangles, as might 
// be expected.

PQP_REAL 
TriDist(PQP_REAL p[3], PQP_REAL q[3], 
        const PQP_REAL s[3][3], const PQP_REAL t[3][3]);

#endif
