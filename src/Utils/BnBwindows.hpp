

#ifndef TWBnB_hpp_
#define TWBnB_hpp_

#include <algorithm>
#include <cassert>
#include <cfloat>
#include <climits>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <vector>

namespace Antipatrea
{
namespace cBNB
{

#define M 101
class State
{
public:
  long h, g;
  int depth, city;
  bool used[M];
  State()
  {
    for (int i = 0; i < M; i++)
      used[i] = false;
    g = h = 0L;
    depth = city = 0;
  }
};

class Memory
{
public:
  int top, max;
  State *old;
  Memory(int n)
  {
    max = n * n;
    top = 0;
    old = new State[max + 1];
    //    for (int i=0;i<max+1;i++) old[i] = new State();
  }

  ~Memory(void)
  {
    delete[] old;
  }
};

class Pair
{
public:
  long score;
  int *tour;
  bool m_found;

  Pair(int N)
  {
    tour = new int[N + 1];
    m_found = true;
  }
  ~Pair()
  {
    delete[] tour;
  }
};

class Selective
{
public:
  int N;
  State *newState;
  Memory *stack;
  int *tour;
  long **dist;
  int **far;
  int *next;
  int start;
  State S;
  bool *used;
  long **cost;
  long **copy;
  long **mask;
  int *rowCover;
  int *colCover;
  int **assignment;
  int *in;
  int *out;
  int *closest;

  int *link;
  int **path;
  bool *cycle;
  bool *visited;

  long **d;        // distance matrix
  long *left;      // time window left
  long *right;     // time window right
  int *moves;      // static array for successor
  long expansions; // counting number of runs/rollouts

  ~Selective();
  Selective(int m, long *goal_left, long *goal_right, long **goal_dist, long *start_dist);
  Pair *search(int h, long tstart, int maxNrExpansions);

  int heuristic(int h, int g, int city, int depth)
  {
    return 0;
  }

  void print(int *tour);
};
}
}

#endif
