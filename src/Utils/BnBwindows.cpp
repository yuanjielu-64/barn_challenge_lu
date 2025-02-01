
#include "Utils/BnBwindows.hpp"
#include "Utils/Timer.hpp"

namespace Antipatrea
{

cBNB::Selective::~Selective(void)
{

  delete[] tour;
  delete[] newState;
  delete stack;
  delete[] used;
  delete[] left;
  delete[] right;

  for (int i = 0; i < N; i++)
  {
    delete[] d[i];
    delete[] far[i];
    delete[] cost[i];
    delete[] copy[i];
    delete[] mask[i];
    delete[] assignment[i];
  }

  delete[] d;
  delete[] far;
  delete[] cost;
  delete[] copy;
  delete[] mask;
  delete[] assignment;

  delete[] next;
  delete[] cycle;
  delete[] visited;
  delete[] rowCover;
  delete[] colCover;

  for (int i = 0; i < N * N; i++)
    delete[] path[i];
  delete[] path;

  delete[] in;
  delete[] link;
  delete[] out;
  delete[] closest;
}

cBNB::Selective::Selective(int k, long *goal_left, long *goal_right, long **goal_dist, long *start_dist)
{
  start = 0;
  N = k + 1;
  tour = new int[N + 2];

  newState = new State[N];
  stack = new Memory(N);
  used = new bool[N];
  left = new long[N];
  right = new long[N];

  d = new long *[N]; // distance matrix
  for (int i = 0; i < N; i++)
    d[i] = new long[N];
  far = new int *[N];
  for (int i = 0; i < N; i++)
    far[i] = new int[N];
  cost = new long *[N];
  for (int i = 0; i < N; i++)
    cost[i] = new long[N];
  copy = new long *[N];
  for (int i = 0; i < N; i++)
    copy[i] = new long[N];
  mask = new long *[N];
  for (int i = 0; i < N; i++)
    mask[i] = new long[N];
  assignment = new int *[N];
  for (int i = 0; i < N; i++)
    assignment[i] = new int[2];
  for (int i = 0; i < N; i++)
    used[i] = false;

  next = new int[N];
  cycle = new bool[N];
  visited = new bool[N];
  rowCover = new int[N];
  ;
  colCover = new int[N];

  path = new int *[N * N];
  for (int i = 0; i < N * N; i++)
  {
    path[i] = new int[2];
  }

  in = new int[N];
  link = new int[N];
  out = new int[N];
  closest = new int[N];

  printf("N = %d\n", N);

  expansions = 0;

  for (int i = 1; i < N; i++)
    left[i] = goal_left[i - 1];
  for (int i = 1; i < N; i++)
    right[i] = goal_right[i - 1];

  for (int i = 1; i < N; i++)
  {
    std::cout << "tw " << i << ": [" << left[i] << "," << right[i] << "]\n";
  }

  for (int i = 1; i < N; i++)
    for (int j = 1; j < N; j++)
      d[i][j] = goal_dist[i - 1][j - 1];
  for (int i = 0; i < N; i++)
    d[i][i] = 0;

  for (int i = 1; i < N; i++)
  {
    d[0][i] = start_dist[i - 1];
    d[i][0] = start_dist[i - 1];
  }

  for (int i = 0; i < N; i++)
  {
    for (int k = 0; k < N; k++)
      newState[i].used[k] = false;
    newState[i].g = newState[i].h = 0;
    newState[i].depth = newState[i].city = 0;
    for (int j = 0; j < N; j++)
      far[i][j] = j;
    for (int j = 0; j < N - 1; j++)
      for (int k = j + 1; k < N; k++)
        if (d[i][far[i][j]] < d[i][far[i][k]])
          std::swap(far[i][j], far[i][k]);
  }
  /*
    for (int j = 0; j < N; j++)
    for (int k = 0; k < N; k++)
    cout << far[j][k] << std::endl;
    exit(1);
  */
}

cBNB::Pair *cBNB::Selective::search(int h, long tstart, int maxNrExpansions)
{
    Timer::Clock clk;
    Timer::Start(clk);
    
  expansions = 0;
  Pair *best = new Pair(N);
  long alpha = std::numeric_limits<long>::max();
  int top = stack->top++;
  stack->old[top].g = tstart;
  stack->old[top].depth = 0;
  stack->old[top].city = start;
  for (int i = 0; i < N; i++)
    stack->old[top].used[i] = false;
  stack->old[top].h = heuristic(h, 0, 0, 0);
  // std::cout << "Heuristic value at root = " << stack->old[top].h << std::endl;
  stack->old[top].used[start] = true;
  while (stack->top != 0 && Timer::Elapsed(clk) < 5)
  {
    if (h == 0 && expansions > maxNrExpansions)
    {
      std::cout << "End of TSP tour optimization " << alpha << std::endl;
      if (alpha == std::numeric_limits<long>::max())
      {
        std::cout << "No solution found." << std::endl;
        best->m_found = false;
      }
      return best;
    }
    top = --stack->top;
    int depth = stack->old[top].depth;
    int city = stack->old[top].city;
    //	int oldcity = depth > 0 ? tour[depth-1] : -1;
    tour[depth] = city;
    //	if (depth == maxColors-1) {
    if (depth == N - 1)
    {
      if (stack->old[top].g /* + d[city][start] */ < alpha)
      {
        alpha = stack->old[top].g /* + d[city][start]*/;
        //		std::cout << " cost: " << alpha << " (" << expansions << ")" << std::endl;
        int check = 0;
        best->score = alpha;
        for (int j = 0; j < N; j++)
          best->tour[j] = tour[j];
        tour[N + 1] = 0;
        // print(best->tour);
        continue;
      }
    }

    for (int k = 0; k < N; k++)
      used[k] = stack->old[top].used[k];
    //	for (int k=0;k<N;k++) std::cout << used[k]; std::cout << std::endl;
    long gduration = stack->old[top].g;
    int opindex = 0;
    expansions++;
    for (int i = 0; i < N; i++)
    {
      int newcity = far[city][i];
      if (used[newcity])
        continue;

      long g = gduration + d[city][newcity];

      //	    std::cout << " expanding " << city
      //		      << " successor " << newcity << " dur " << g << std::endl;
      //	    std::cout << right[newcity] << "," << g << std::endl;
      if (right[newcity] < g)
      {
        //	      std::cout << "stop\n";
        continue;
      }
      if (left[newcity] > g)
        g = left[newcity]; // waiting

      newState[opindex].depth = depth + 1;
      next[opindex] = opindex + 1;
      newState[opindex].g = g;
      newState[opindex].city = newcity;
      used[start] = false;
      newState[opindex].h = heuristic(h, g, newcity, depth);
      used[start] = true;
      used[newcity] = true;

      for (int k = 0; k < N; k++)
        newState[opindex].used[k] = used[k];

      used[newcity] = false;
      opindex++;
    }

    if (opindex <= 0)
    {

      // printf("no: opindex = %d N = %d depth = %d\n", opindex, N, depth);
      // print(tour);
      //	       exit(0);
      continue;
    }
    else
      next[opindex - 1] = N;
    for (int i = 0; i != N; i = next[i])
    {
      if (newState[i].g + newState[i].h >= alpha)
        continue;
      int newtop = stack->top++;
      stack->old[newtop].city = newState[i].city;
      for (int k = 0; k < N; k++)
        stack->old[newtop].used[k] = newState[i].used[k];
      stack->old[newtop].g = newState[i].g;
      stack->old[newtop].h = newState[i].h;
      stack->old[newtop].depth = newState[i].depth;
    }
  }
  std::cout << "End of TSP tour optimization, stack empty " << alpha << " time = " << Timer::Elapsed(clk) << std::endl;
  if (alpha == std::numeric_limits<long>::max())
  {
    best->m_found = false;
    std::cout << "No solution found." << std::endl;
  }
  return best;
}

void cBNB::Selective::print(int *tour)
{
  long sum = 0;
  long dist = 0;
  for (int l = 1; l < N; l++)
  {
    std::cout << "" << tour[l - 1] << "--" << d[tour[l - 1]][tour[l]] << "-->" << tour[l] << ",[" << left[tour[l]] << "," << right[tour[l]] << "]";
    dist += d[tour[l - 1]][tour[l]];
    std::cout << ", sum duration " << dist << ","; // << std::endl;
    std::cout << " left TW ";
    if (dist < left[tour[l]])
      std::cout << "not valid, wait " << left[tour[l]] - dist;
    else
      std::cout << "valid";
    std::cout << ", right TW " << ((dist > right[tour[l]]) ? "not " : "") << "valid.\n";
    if (dist < left[tour[l]])
      dist = left[tour[l]];
  }
}
}
