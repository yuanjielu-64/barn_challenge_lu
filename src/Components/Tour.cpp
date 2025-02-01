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
#include "Components/Tour.hpp"

namespace Antipatrea
{

bool Tour::IsFeasible(const double tstart) const
{
    const int nrSites = m_order.size();
    double t = tstart;
    for (int i = 0; i < nrSites; ++i)
    {
        t += m_durations[i];
        if (t > m_bounds[2 * m_order[i] + 1])
            return false;
        t = std::max(t, m_bounds[2 * m_order[i]]);
    }
    return true;
}

void Tour::Print(std::ostream &out) const
{
    out << "order:";
    for (auto &val : m_order)
        out << val << " ";
    out << " durations:";
    for (auto &val : m_durations)
        out << val << " ";
    out << " bounds:";
    for (auto &val : m_bounds)
        out << val << " ";
    out << " departure:";
    for (auto &val : m_times)
        out << val << " ";
}
}
