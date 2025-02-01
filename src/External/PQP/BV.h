
#ifndef PQP_BV_H
#define PQP_BV_H

#include <math.h>
#include "Tri.h"
#include "PQP_Compile.h"

struct BV
{
  PQP_REAL R[3][3];     // orientation of RSS & OBB

#if PQP_BV_TYPE & RSS_TYPE
  PQP_REAL Tr[3];       // position of rectangle
  PQP_REAL l[2];        // side lengths of rectangle
  PQP_REAL r;           // radius of sphere summed with rectangle to form RSS
#endif

#if PQP_BV_TYPE & OBB_TYPE
  PQP_REAL To[3];       // position of obb
  PQP_REAL d[3];        // (half) dimensions of obb
#endif

  int first_child;      // positive value is index of first_child bv
                        // negative value is -(index + 1) of triangle

  BV();
  ~BV();
  int      Leaf()    { return first_child < 0; }
  PQP_REAL GetSize(); 
  void     FitToTris(PQP_REAL O[3][3], Tri *tris, int num_tris);
};

inline
PQP_REAL 
BV::GetSize()
{
#if PQP_BV_TYPE & RSS_TYPE
  return (sqrt(l[0]*l[0] + l[1]*l[1]) + 2*r);
#else
  return (d[0]*d[0] + d[1]*d[1] + d[2]*d[2]);
#endif
}

int
BV_Overlap(PQP_REAL R[3][3], PQP_REAL T[3], BV *b1, BV *b2);

#if PQP_BV_TYPE & RSS_TYPE
PQP_REAL
BV_Distance(PQP_REAL R[3][3], PQP_REAL T[3], BV *b1, BV *b2);
#endif

#endif


