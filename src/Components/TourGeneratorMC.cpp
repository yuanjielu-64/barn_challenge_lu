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
#include "Components/TourGeneratorMC.hpp"
#include "Utils/MCwindows.hpp"
#include "Utils/Misc.hpp"

namespace Antipatrea
{
bool TourGeneratorMC::GenerateTour(Tour &tour)
{
  const int nrGoals = GetNrSites() - 1;

  if(m_nrCalls == 0)
      m_depth = nrGoals > 10 ? 4:3;//2 : 1;
  
  
  long **goal_dist = new long *[nrGoals];
  long *start_dist = new long[nrGoals];
  long *goal_left = new long[nrGoals];
  long *goal_right = new long[nrGoals];
  for (int j = 0; j < nrGoals; ++j)
    goal_dist[j] = new long[nrGoals];

  for (int j = 0; j < nrGoals; ++j)
  {
    goal_dist[j][j] = 0;
    for (int i = j + 1; i < nrGoals; ++i)
    {
      goal_dist[j][i] = ToLong(GetDuration(j + 1, i + 1));
      goal_dist[i][j] = ToLong(GetDuration(i + 1, j + 1));
    }
    start_dist[j] = ToLong(GetDuration(0, j + 1));

    goal_left[j] = ToLong(GetLowerBound(j + 1));
    goal_right[j] = ToLong(GetUpperBound(j + 1));
  }

  twMC::Selective S(nrGoals, goal_left, goal_right, goal_dist, start_dist);

   if(m_failures > 4)
  {
      m_failures = 0;
      m_success = 0;      
      ++m_depth;
      }
  if(m_success > 5)
  {
      m_failures = 0;
      m_success = 0;
      --m_depth;      
  }
  
  if(m_depth > 2)
      m_depth = 2;
  else if(m_depth < 2)
      m_depth = 2;

  
  auto sol = S.search(m_depth);

  
  bool ok = true;
  
  tour.m_order.clear();
  tour.m_order.push_back(0);
  if (ok)
    for (int j = 0; j < nrGoals; ++j)
      tour.m_order.push_back(sol->tour[j + 1] - 1 + 1);

  delete[] goal_left;
  delete[] goal_right;
  delete[] start_dist;
  for (int i = 0; i < nrGoals; ++i)
    delete[] goal_dist[i];
  delete[] goal_dist;
  delete sol;

  ok = ok && FromOrderToTimes(tour);
  if(ok)
      ++m_success;
  else
      ++m_failures;
  
  

  return ok;
}
    
    
}
