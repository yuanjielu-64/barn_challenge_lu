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
#include "Components/TourGeneratorRandom.hpp"
#include "Utils/Misc.hpp"

namespace Antipatrea
{
bool TourGeneratorRandom::GenerateTour(Tour &tour)
{

  std::vector<int> sites;
  std::vector<int> perm;

  sites.resize(m_nrSites - 1);
  perm.resize(m_nrSites - 1);
  for (int i = 1; i < m_nrSites; ++i)
  {
    sites[i - 1] = i;
    perm[i - 1] = i - 1;
  }
  PermuteItems<int>(sites, sites.size());

  tour.m_order.resize(m_nrSites);
  tour.m_order[0] = 0;

  do
  {
    for (int i = 1; i < m_nrSites; ++i)
      tour.m_order[i] = sites[perm[i - 1]];

    if (FromOrderToTimes(tour))
      return true;
  } while (NextPermutation<int>(perm.size(), &perm[0]));

  return false;
}
}
