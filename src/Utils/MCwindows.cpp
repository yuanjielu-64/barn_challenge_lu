
#include "Utils/MCwindows.hpp"

#define ITERATIONS 25
#define LEVEL 6

//Pair* l = search(LEVEL);

namespace Antipatrea
{
    
twMC::Selective::~Selective(void)
{
    delete[] tour;
    delete[] value;
    delete[] moves;
    delete[] l;
    delete[] r;

    //for(int j = 0; j < LEVEL+1; j++)
    // for(int i = 0; i < N; i++)
//	delete[] backup[j][i];
    
    for(int i = 0; i < N; i++)
	delete[] d[i];

    //  for(int i = 0; i < N; i++)
    // delete[] backup[i];

    for(int i = 0; i < N; i++)
      delete[] global[i];
 
    delete[] d;
    delete[] visits;

/////////
    for(int i = 0; i < LEVEL+1; i++)  
      for(int j = 0; j < N; j++)  
	  delete[] backup[i][j];
    for(int i = 0; i < LEVEL+1; i++)  
	delete[] backup[i];
    delete[] backup;

    delete[] global;
    
}

twMC::Selective::Selective(int k, long* goal_left, long* goal_right, long** goal_dist, long* start_dist) 
{
    start = 0;
    N = k+1;
    tour = new int[N+2];    
    l = new long[N];
    r = new long[N];
    moves = new int[N];
    value = new double[N];

    global = new double*[N];           // policy matrix
    for(int i = 0; i < N; i++)  
      global[i] = new double[N];

    backup = new double**[LEVEL+1];           // policy matrix
    for(int i = 0; i < LEVEL+1; i++)  
      backup[i] = new double*[N];

    for(int i = 0; i < LEVEL+1; i++)  
      for(int j = 0; j < N; j++)  
	backup[i][j] = new double[N];
        
    d = new long*[N];           // distance matrix
    for(int i = 0; i < N; i++)  
	d[i] = new long[N];
    
    visits = new int[N];
    
    //EP: printf("N = %d\n", N);
    
    evaluations = 0;         

    for(int i = 1; i < N; i++)
      l[i] = goal_left[i-1];
    for(int i = 1; i < N; i++)
      r[i] = goal_right[i-1];

    /*EP:
    for(int i = 1; i < N; i++) {
      std::cout << "tw " << i << ": [" << l[i] << "," << r[i] << "]\n"; 
      }*/
    
    for(int i = 1; i < N; i++) 
	for(int j = 1; j < N; j++) 
	    d[i][j] = goal_dist[i-1][j-1];
    for (int i=0;i<N;i++) 
	d[i][i] = 0;
    
    for (int i=1;i<N;i++) {
	d[0][i] = start_dist[i-1];
	d[i][0] = start_dist[i-1];
    }

    for(int i = 0; i < N; i++)
      for(int j = 0; j < N; j++)
	global[i][j] = 0.0;

}

double twMC::Selective::rollout() {
  for (int j=1;j<N;j++)
    visits[j] = 1;
  visits[0] = 0;
  tour[0] = 0;
  tourSize = 1; // start node already visited
  int node = 0, prev = 0;
  double makespan = 0.0; // capacity = 0.0;
  int violations = 0;
  double cost = 0.0;
  while(tourSize < N) {
    double sum = 0;
    int successors = 0;
    for(int i = 0; i < N; i++) {
      if (visits[i] > 0 && d[node][i] != 1000000.0) {
	moves[successors++] = i;
	for (int j = 0;j < N;j++) {
	  if (i != j) {
	    if (visits[j] > 0) {
	      if (l[i] > r[j] || makespan + d[node][i] > r[j]) {
		successors--;
		break;
	      }
	    }
	  }
	}
      }
    }
    if (successors == 0) {
      for(int i = 0; i < N; i++)
	if(visits[i] > 0)
	  moves[successors++] = i;
    }
    for(int i=0; i<successors; i++) {
      value[i] = EXP(global[node][moves[i]]);
      sum += value[i];
    }
    double mrand = (rand()/(double)RAND_MAX)*(sum);
    int i=0;
    sum = value[0];
    while(sum<mrand)
      sum += value[++i];
    prev = node;
    node = moves[i];
    tour[tourSize++] = node;
    visits[node]--;
    cost += d[prev][node];
    makespan = makespan + d[prev][node] > l[node] ?
      makespan + d[prev][node] : l[node];
    // capacity += w[node];
    // if (capacity > max_capacity) violations += 1.0;
    if (makespan > r[node]) violations += 1.0;
  }
  //    tour[tourSize++] = 0;
  //    cost += d[node][0];
  //    makespan = Math.max(makespan + d[node][0], l[0]);
  //    if (makespan > r[0]) violations += 1.0;
  return 100000.0 * violations + cost;
}

void twMC::Selective::adapt(int* tour_param, int level) {
  for (int j=1;j<N;j++)
    visits[j] = 1;
  visits[0] = 0;
  int successors;
  int node = 0;
  for(int j=1; j<N; j++)  {
    successors = 0;
    for(int i = 0; i < N; i++)
      if (visits[i] > 0)
	moves[successors++] = i;
    double factor = 1.0;
    backup[level][node][tour_param[j]] += factor;
    double z = 0.0;
    for(int i=0; i<successors; i++)
      z += EXP(global[node][moves[i]]);
    for (int i=0; i<successors; i++)
      backup[level][node][moves[i]] -= factor *
	EXP(global[node][moves[i]])/z;
    node = tour_param[j];
    visits[node]--;
  }
}

twMC::Pair* twMC::Selective::search(int level) {
  twMC::Pair* best = new Pair(N);
  best->score = std::numeric_limits<long>::max();
  if (level == 0) {
    double eval = rollout();  // sets tour and visits
    evaluations++;
    best->score = eval;
    for (int j = 0; j < N+1; j++)
      best->tour[j] = tour[j];
  }
  else {
    for(int i = 0; i < N; i++)
      for(int j = 0; j < N; j++)
	backup[level][i][j] = global[i][j];
    for(int i=0; i<ITERATIONS; i++) {
      Pair* r = search(level - 1);
      if (r->score < best->score) {
	best->score = r->score;
	for (int j = 0; j < N+1; j++)
	  best->tour[j] = r->tour[j];
	if(level > 3) {
	  std::cout << " Level: " << level << "," << i << ", duration: " << r->score << ", runs: " << evaluations << std::endl;
	}
	adapt(best->tour,level);
      }
      delete r;
    }
    for(int i = 0; i < N; i++)
      for(int j = 0; j < N; j++)
	global[i][j] = backup[level][i][j];
  }
  return best;
}
	    
void twMC::Selective::print(int* tour) {
  long sum = 0;
  long dist = 0;
  for (int p=1;p<N;p++) {
    std::cout << "" << tour[p-1] << "--" << d[tour[p-1]][tour[p]] << "-->" << tour[p] << ",[" << l[tour[p]] << "," << r[tour[p]] << "]";
    dist += d[tour[p-1]][tour[p]];
    std::cout << ", sum duration " << dist << "," ; // << std::endl;
    std::cout << " left TW ";
    if (dist < l[tour[p]])  
      std::cout << "not valid, wait " <<  l[tour[p]]-dist;
    else
      std::cout << "valid";
    std::cout << ", right TW "
	      << ((dist > r[tour[p]]) ? "not " : "") << "valid.\n" ;
    if (dist < l[tour[p]]) dist = l[tour[p]];
  }
}

}
