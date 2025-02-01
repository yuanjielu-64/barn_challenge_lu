
#ifndef PQP_BVTQ_H
#define PQP_BVTQ_H

#include <stdio.h>
#include <stdlib.h>
#include "PQP_Compile.h"

inline
int 
LChild(int p)  
{ 
  return (2*p + 1); 
} 

inline 
int 
Parent(int c)  
{ 
  return ((c - 1)/2); 
} 

struct BVT 
{ 
  PQP_REAL d;       // distance between the bvs
  int b1, b2;       // bv numbers - b1 is from model 1, b2 from model 2
  PQP_REAL R[3][3]; // the relative rotation from b1 to b2
  PQP_REAL T[3];    // the relative translation from b1 to b2
  int pindex;       // the index of the pointer that points to this -
                    // needed when filling the hole left by an ExtractMin
};

class BVTQ 
{ 
  int size;       // max number of bv tests
  int numtests;   // number of bv tests in queue
  BVT *bvt;       // an array of bv tests - seems faster than 'new' for each
  BVT **bvtp;     // the queue: an array of pointers to elts of bvt

public:
  BVTQ(int sz) 
  {
    size = sz;              
    bvt = new BVT[size];    
    bvtp = new BVT*[size];  
    numtests = 0;
  }
  ~BVTQ() { delete [] bvt; delete [] bvtp; }
  int Empty() { return (numtests == 0); }
  int GetNumTests() { return numtests; }
  int GetSize() { return size; }
  PQP_REAL MinTest() { return bvtp[0]->d; }
  BVT ExtractMinTest();
  void AddTest(BVT &);
};

inline
void 
BVTQ::AddTest(BVT &t)
{
  bvtp[numtests] = &bvt[numtests];

  *bvtp[numtests] = t;
  bvtp[numtests]->pindex = numtests;
  
  BVT *temp;
  int c = numtests;
  int p;
  
  while ((c != 0) && (bvtp[(p = Parent(c))]->d >= bvtp[c]->d)) 
  {
    // swap p and c pointers

    temp = bvtp[p];
    bvtp[p] = bvtp[c];
    bvtp[c] = temp;	 

    // the bv tests pointed to by p and c need new indices

    bvtp[p]->pindex = p;
    bvtp[c]->pindex = c;

    c = p;
  } 
  numtests++; 
}

inline
BVT
BVTQ::ExtractMinTest()
{
  // store min test to be extracted

  BVT min_test = *bvtp[0];

  // copy last bvt to the empty space;
  // reset the pointer to this moved bvt

  *bvtp[0] = bvt[numtests-1];
  bvtp[bvt[numtests-1].pindex] = bvtp[0];

  // copy the last pointer to the first

  bvtp[0] = bvtp[numtests-1];

  numtests--; 

  BVT *temp;
  int p = 0; 
  int c1,c2,c; 

  while(1) 
  {     
    c1 = LChild(p); 
    c2 = c1+1; 
  
    if (c1 < numtests) 
    { 
      if (c2 < numtests) 
      { 	
        // p has both children, promote the minimum 

        if (bvtp[c1]->d < bvtp[c2]->d) c = c1; else c = c2; 

        if (bvtp[c]->d < bvtp[p]->d) 
        { 
          temp = bvtp[p];
          bvtp[p] = bvtp[c];
          bvtp[c] = temp; 

          bvtp[p]->pindex = p;
          bvtp[c]->pindex = c;

          p = c; 
        } 
        else 
        { 
          break; 
        } 
      } 
      else  
      { 	
        // p has only left child 

        if (bvtp[c1]->d < bvtp[p]->d) 
        { 
          temp = bvtp[p]; 
          bvtp[p] = bvtp[c1]; 
          bvtp[c1] = temp; 

          bvtp[p]->pindex = p;
          bvtp[c1]->pindex = c1;

          p = c1;	 
        } 
        else 
        { 
          break; 
        } 
      } 
    } 
    else 
    {   
      // p has no children 

      break; 
    } 
  } 

  return min_test;
}

#endif


