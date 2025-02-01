

#ifndef TWMC_hpp_
#define TWMC_hpp_

#include <vector>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <cfloat>
#include <climits>
#include <limits>
#include <cmath>
#include <cassert>
#include <string>
#include <algorithm>

namespace Antipatrea
{
    
static union
{
  double d;
  struct
  {
    #ifdef LITTLE_ENDIAN
    int j, i;
    #else
    int i, j;
    #endif
  } n;
} eco;
#define EXPA (1048576/M_LN2) /* use 1512775 for integer version */
#define EXPC 60801 /* see text for choice of c values */
#define EXP(y) (eco.n.i = EXPA*(y) + (1072693248 - EXPC), eco.d)


namespace twMC
{
  class Pair 
    { 
    public:
	long score; 
	int* tour;      
	Pair(int N) {
	    tour = new int[N+1];
	} 
	~Pair() { delete[] tour; } 
    };
    
    class Selective 
    {
    public:  
      int N;
      int* tour;
      int tourSize;
      long** dist;
      int start;
      int* visits;
      double* value; 
      
      long** d;           // distance matrix
      long* l;           // time window left
      long* r;           // time window right
      int* moves;                 // static array for successor
      long evaluations;          // counting number of runs/rollouts
      double** global;
      double*** backup;
      
      ~Selective();
      Selective(int m, long* goal_left, long* goal_right, long** goal_dist, long* start_dist);
      Pair* search(int level);
      double rollout();    
      void adapt(int* tour_param, int level);
      void print(int* tour);
   };

}

}

#endif
