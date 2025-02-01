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
#include "Components/TourGenerator.hpp"
#include "Components/Constants.hpp"

namespace Antipatrea
{
void TourGenerator::SetupFromParams(Params &params)
{
  Component::SetupFromParams(params);
  SetStartTime(params.GetValueAsDouble(Constants::KW_StartTime, GetStartTime()));
  SetNrSites(params.GetValueAsInt(Constants::KW_NrSites, GetNrSites()));
  params.GetValuesAsDoubles(Constants::KW_TimeBounds, &m_bounds[0], 2 * GetNrSites());
  params.GetValuesAsDoubles(Constants::KW_Durations, &m_durations[0], GetNrSites() * GetNrSites());
}

bool TourGenerator::IsFeasibleOrder(const Tour &tour) const
{
  if (tour.m_order.size() != m_nrSites || tour.m_order[0] != 0)
    return false;

  double t = GetStartTime();
  for (int i = 1; i < m_nrSites; ++i)
  {
    t += GetDuration(tour.m_order[i - 1], tour.m_order[i]);
    if (t > GetUpperBound(tour.m_order[i]))
      return false;
    t = std::max(t, GetLowerBound(tour.m_order[i]));
  }
  Logger::m_out << std::endl;
  return true;
}

    bool TourGenerator::FromOrderToTimes(Tour &tour, bool fake) const
{
  if (tour.m_order.size() != m_nrSites || tour.m_order[0] != 0)
    return false;

  bool ok = true;
  

  if (tour.m_times.size() != m_nrSites)
    tour.m_times.resize(m_nrSites);
  if (tour.m_durations.size() != m_nrSites)
    tour.m_durations.resize(m_nrSites);

  double t = GetStartTime();
  tour.m_times[0] = t;
  tour.m_durations[0] = 0.0;

  for (int i = 1; i < m_nrSites; ++i)
  {
    tour.m_durations[i] = GetDuration(tour.m_order[i - 1], tour.m_order[i]);
    t += tour.m_durations[i];

    if (t > GetUpperBound(tour.m_order[i]))
    {
	if(fake == false)
	    return false;
	ok = false;
	
    }
    
    tour.m_times[i] = t = std::max(t, GetLowerBound(tour.m_order[i]));
  }
  tour.m_bounds.assign(m_bounds.begin(), m_bounds.end());

  return ok;
}

void TourGenerator::Print(std::ostream &out) const
{
  int m_nrSites;
  double m_startTime;
  std::vector<double> m_durations;
  std::vector<double> m_bounds;

  out << "NrSites = " << GetNrSites() << std::endl << "StartTime = " << GetStartTime() << std::endl << "Durations = " << std::endl;

  for (int i = 0; i < GetNrSites(); ++i)
  {
    for (int j = 0; j < GetNrSites(); ++j)
      out << GetDuration(i, j) << " ";
    out << std::endl;
  }

  out << "Bounds = ";
  for (int i = 0; i < GetNrSites(); ++i)
    out << GetLowerBound(i) << " " << GetUpperBound(i) << ":";
  out << std::endl;
}
}
