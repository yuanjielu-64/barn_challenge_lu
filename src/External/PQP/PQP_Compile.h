
#ifndef PQP_COMPILE_H
#define PQP_COMPILE_H

// prevents compiler warnings when PQP_REAL is float

#include <math.h>
/*
inline float sqrt(float x) { return (float)sqrt((double)x); }
inline float cos(float x) { return (float)cos((double)x); }
inline float sin(float x) { return (float)sin((double)x); }
inline float fabs(float x) { return (float)fabs((double)x); }
*/
//-------------------------------------------------------------------------
//
// PQP_REAL
//
// This is the floating point type used throughout PQP.  doubles are
// recommended, both for their precision and because the software has
// mainly been tested using them.  However, floats appear to be faster 
// (by 60% on some machines).
//
//-------------------------------------------------------------------------

typedef double PQP_REAL;

//-------------------------------------------------------------------------
//
// PQP_BV_TYPE
//
// PQP introduces a bounding volume (BV) type known as the "rectangle
// swept sphere" (RSS) - the volume created by sweeping a sphere so
// that its center visits every point on a rectangle; it looks
// something like a rounded box.
//
// In our experiments, the RSS type is comparable to the oriented 
// bounding box (OBB) in terms of the number of BV-pair and triangle-pair 
// tests incurred.  However, with our present implementations, overlap 
// tests are cheaper for OBBs, while distance tests are cheaper for the 
// RSS type (we used a public gjk implementation for the OBB distance test).
//
// Consequently, PQP is configured to use the RSS type in distance and 
// tolerance queries (which use BV distance tests) and to use OBBs for
// collision queries (which use BV overlap tests). Using both requires six
// more PQP_REALs per BV node than using just one type. 
//
// To save space, you can configure PQP to use only one type, however, 
// with RSS alone, collision queries will typically be slower.  With OBB's 
// alone, distance and tolerance queries are currently not supported, since 
// we have not developed our own OBB distance test.  The three options are:
//
// #define PQP_BV_TYPE  RSS_TYPE           
// #define PQP_BV_TYPE  OBB_TYPE           
// #define PQP_BV_TYPE  RSS_TYPE | OBB_TYPE
//
//-------------------------------------------------------------------------

#define RSS_TYPE     1
#define OBB_TYPE     2

#define PQP_BV_TYPE  RSS_TYPE | OBB_TYPE

#endif
