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
#include "Components/TourGeneratorBNB.hpp"
#include "Utils/BnBwindows.hpp"
#include "Utils/Misc.hpp"

namespace Antipatrea
{
bool TourGeneratorBNB::GenerateTour(Tour &tour)
{
  const int nrGoals = GetNrSites() - 1;

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

  cBNB::Selective S(nrGoals, goal_left, goal_right, goal_dist, start_dist);
  cBNB::Pair *sol = S.search(0, ToLong(GetStartTime()), 10000000);

  const bool ok = sol->m_found;

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

  return ok && FromOrderToTimes(tour);
}
}
