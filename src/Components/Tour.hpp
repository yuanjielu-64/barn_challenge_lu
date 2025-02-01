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

#ifndef Antipatrea__Tour_HPP_
#define Antipatrea__Tour_HPP_

#include "Utils/Writer.hpp"
#include <vector>

namespace Antipatrea {
    class Tour : public Writer {
    public:
        Tour(void) : Writer() { Clear(); }

        virtual ~Tour(void) {}

        virtual void Clear(void) {
            m_order.clear();
            m_times.clear();
            m_durations.clear();
            m_bounds.clear();
            m_times.push_back(-1.0);
        }

        virtual double GetStartTime(void) const { return m_times[0]; }

        virtual double GetEndTime(void) const { return m_times.back(); }

        virtual void SetStartTime(const double t) { m_times[0] = t; }

        virtual bool IsFeasible(const double tstart) const;

        virtual int GetNrSites(void) const { return m_order.size(); }

        virtual void CopyFrom(const Tour &tour) {
            m_order = tour.m_order;
            m_durations = tour.m_durations;
            m_times = tour.m_times;
            m_bounds = tour.m_bounds;
        }

        virtual void Print(std::ostream &out) const;

        std::vector<int> m_order;
        std::vector<double> m_durations;
        std::vector<double> m_times;
        std::vector<double> m_bounds;
    };
}

#endif
