
#include <stdlib.h>
#include <math.h>
#include "BV.h"
#include "MatVec.h"
#include "RectDist.h"
#include "OBB_Disjoint.h"

BV::BV()
{
  first_child = 0;
}

BV::~BV()
{
}

static
inline 
PQP_REAL 
MaxOfTwo(PQP_REAL a, PQP_REAL b) 
{
  if (a > b) return a;
  return b;
}

void
BV::FitToTris(PQP_REAL O[3][3], Tri *tris, int num_tris)
{
  // store orientation

  McM(R,O);

  // project points of tris to R coordinates

  int num_points = 3*num_tris;
  PQP_REAL (*P)[3] = new PQP_REAL[num_points][3];
  int point = 0;
  int i;
  for (i = 0; i < num_tris; i++) 
  {
    MTxV(P[point],R,tris[i].p1);
    point++;

    MTxV(P[point],R,tris[i].p2);
    point++;

    MTxV(P[point],R,tris[i].p3);
    point++;
  }

  PQP_REAL minx, maxx, miny, maxy, minz, maxz, c[3];

#if PQP_BV_TYPE & OBB_TYPE
  minx = maxx = P[0][0];
  miny = maxy = P[0][1];
  minz = maxz = P[0][2];
  for (i = 1; i < num_points; i++)
  {
    if (P[i][0] < minx) minx = P[i][0];
    else if (P[i][0] > maxx) maxx = P[i][0];
    if (P[i][1] < miny) miny = P[i][1];
    else if (P[i][1] > maxy) maxy = P[i][1];
    if (P[i][2] < minz) minz = P[i][2];
    else if (P[i][2] > maxz) maxz = P[i][2];
  }
  c[0] = (PQP_REAL)0.5*(maxx + minx);
  c[1] = (PQP_REAL)0.5*(maxy + miny);
  c[2] = (PQP_REAL)0.5*(maxz + minz);
  MxV(To,R,c);

  d[0] = (PQP_REAL)0.5*(maxx - minx);
  d[1] = (PQP_REAL)0.5*(maxy - miny);
  d[2] = (PQP_REAL)0.5*(maxz - minz);
#endif
  
#if PQP_BV_TYPE & RSS_TYPE

  // compute thickness, which determines radius, and z of rectangle corner
  
  PQP_REAL cz,radsqr;
  minz = maxz = P[0][2];
  for (i = 1; i < num_points; i++) 
  {
    if (P[i][2] < minz) minz = P[i][2];
    else if (P[i][2] > maxz) maxz = P[i][2];
  }
  r = (PQP_REAL)0.5*(maxz - minz);
  radsqr = r*r;
  cz = (PQP_REAL)0.5*(maxz + minz);

  // compute an initial length of rectangle along x direction

  // find minx and maxx as starting points

  int minindex, maxindex;
  minindex = maxindex = 0;
  for (i = 1; i < num_points; i++) 
  {
    if (P[i][0] < P[minindex][0]) minindex = i; 
    else if (P[i][0] > P[maxindex][0]) maxindex = i;
  }
  PQP_REAL x, dz;
  dz = P[minindex][2] - cz;
  minx = P[minindex][0] + sqrt(MaxOfTwo(radsqr - dz*dz,0));
  dz = P[maxindex][2] - cz;
  maxx = P[maxindex][0] - sqrt(MaxOfTwo(radsqr - dz*dz,0));

  // grow minx

  for (i = 0; i < num_points; i++) 
  {
    if (P[i][0] < minx) 
    {
      dz = P[i][2] - cz;
      x = P[i][0] + sqrt(MaxOfTwo(radsqr - dz*dz,0));
      if (x < minx) minx = x;
    }
  }

  // grow maxx

  for (i = 0; i < num_points; i++) 
  {
    if (P[i][0] > maxx) 
    {
      dz = P[i][2] - cz;
      x = P[i][0] - sqrt(MaxOfTwo(radsqr - dz*dz,0));
      if (x > maxx) maxx = x;
    }
  }
  
  // compute an initial length of rectangle along y direction

  // find miny and maxy as starting points

  minindex = maxindex = 0;
  for (i = 1; i < num_points; i++) 
  {
    if (P[i][1] < P[minindex][1]) minindex = i;
    else if (P[i][1] > P[maxindex][1]) maxindex = i;
  }
  PQP_REAL y;
  dz = P[minindex][2] - cz;
  miny = P[minindex][1] + sqrt(MaxOfTwo(radsqr - dz*dz,0));
  dz = P[maxindex][2] - cz;
  maxy = P[maxindex][1] - sqrt(MaxOfTwo(radsqr - dz*dz,0));

  // grow miny

  for (i = 0; i < num_points; i++) 
  {
    if (P[i][1] < miny) 
    {
      dz = P[i][2] - cz;
      y = P[i][1] + sqrt(MaxOfTwo(radsqr - dz*dz,0));
      if (y < miny) miny = y;
    }
  }

  // grow maxy

  for (i = 0; i < num_points; i++) 
  {
    if (P[i][1] > maxy) 
    {
      dz = P[i][2] - cz;
      y = P[i][1] - sqrt(MaxOfTwo(radsqr - dz*dz,0));
      if (y > maxy) maxy = y;
    }
  }
  
  // corners may have some points which are not covered - grow lengths if
  // necessary
  
  PQP_REAL dx, dy, u, t;
  PQP_REAL a = sqrt((PQP_REAL)0.5);
  for (i = 0; i < num_points; i++) 
  {
    if (P[i][0] > maxx) 
    {
      if (P[i][1] > maxy) 
      {
        dx = P[i][0] - maxx;
        dy = P[i][1] - maxy;
        u = dx*a + dy*a;
        t = (a*u - dx)*(a*u - dx) + 
            (a*u - dy)*(a*u - dy) +
            (cz - P[i][2])*(cz - P[i][2]);
        u = u - sqrt(MaxOfTwo(radsqr - t,0));
        if (u > 0) 
        {
          maxx += u*a;
          maxy += u*a;
        }
      }
      else if (P[i][1] < miny) 
      {
        dx = P[i][0] - maxx;
        dy = P[i][1] - miny;
        u = dx*a - dy*a;
        t = (a*u - dx)*(a*u - dx) + 
            (-a*u - dy)*(-a*u - dy) +
            (cz - P[i][2])*(cz - P[i][2]);
        u = u - sqrt(MaxOfTwo(radsqr - t,0));
        if (u > 0) 
        {
          maxx += u*a;
          miny -= u*a;
        }
      }
    }
    else if (P[i][0] < minx) 
    {
      if (P[i][1] > maxy) 
      {
        dx = P[i][0] - minx;
        dy = P[i][1] - maxy;
        u = dy*a - dx*a;
        t = (-a*u - dx)*(-a*u - dx) + 
            (a*u - dy)*(a*u - dy) +
            (cz - P[i][2])*(cz - P[i][2]);
        u = u - sqrt(MaxOfTwo(radsqr - t,0));
        if (u > 0) 
        {
          minx -= u*a;
          maxy += u*a;
        }     
      }
      else if (P[i][1] < miny) 
      {
        dx = P[i][0] - minx;
        dy = P[i][1] - miny;
        u = -dx*a - dy*a;
        t = (-a*u - dx)*(-a*u - dx) + 
            (-a*u - dy)*(-a*u - dy) +
            (cz - P[i][2])*(cz - P[i][2]);
        u = u - sqrt(MaxOfTwo(radsqr - t,0));
        if (u > 0) 
        {
          minx -= u*a; 
          miny -= u*a;
        }
      }
    }
  }

  c[0] = minx;
  c[1] = miny;
  c[2] = cz;
  MxV(Tr,R,c);

  l[0] = maxx - minx;  
  if (l[0] < 0) l[0] = 0;
  l[1] = maxy - miny;
  if (l[1] < 0) l[1] = 0;
#endif

  delete [] P;
}

int 
BV_Overlap(PQP_REAL R[3][3], PQP_REAL T[3], BV *b1, BV *b2)
{
#if PQP_BV_TYPE & OBB_TYPE
  return (obb_disjoint(R,T,b1->d,b2->d) == 0);
#else
  PQP_REAL dist = RectDist(R,T,b1->l,b2->l);
  if (dist <= (b1->r + b2->r)) return 1;
  return 0;
#endif
}

#if PQP_BV_TYPE & RSS_TYPE
PQP_REAL
BV_Distance(PQP_REAL R[3][3], PQP_REAL T[3], BV *b1, BV *b2)
{
  PQP_REAL dist = RectDist(R,T,b1->l,b2->l);
  dist -= (b1->r + b2->r);
  return (dist < (PQP_REAL)0.0)? (PQP_REAL)0.0 : dist;
}
#endif


