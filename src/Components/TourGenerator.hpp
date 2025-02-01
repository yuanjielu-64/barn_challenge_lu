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

#ifndef Antipatrea__TourGenerator_HPP_
#define Antipatrea__TourGenerator_HPP_

#include "Components/Component.hpp"
#include "Components/Tour.hpp"
#include <vector>
#include <algorithm>

namespace Antipatrea {

    class TourGenerator : public Component,
                          public Writer {
    public:
        TourGenerator(void) : Component(),
                              Writer(),
                              m_nrSites(0),
                              m_startTime(0.0),
                              m_minUpperBound(INFINITY) {
        }

        virtual ~TourGenerator(void) {
        }

        virtual int GetNrSites(void) const {
            return m_nrSites;
        }

        virtual double GetStartTime(void) const {
            return m_startTime;
        }

        virtual const double *GetDurations(void) const {
            return &m_durations[0];
        }

        virtual double GetDuration(const int i, const int j) const {
            return m_durations[m_nrSites * i + j];
        }

        virtual const double *GetBounds(void) const {
            return &m_bounds[0];
        }

        virtual double GetLowerBound(const int i) const {
            return m_bounds[2 * i];
        }

        virtual double GetUpperBound(const int i) const {
            return m_bounds[2 * i + 1];
        }

        virtual void SetNrSites(const int n) {
            m_nrSites = n;
            m_durations.resize(m_nrSites * m_nrSites);
            m_bounds.resize(m_nrSites * 2);
            std::fill(m_durations.begin(), m_durations.end(), 0.0);
            std::fill(m_bounds.begin(), m_bounds.end(), 0.0);
        }

        virtual void SetStartTime(const double t) {
            m_startTime = t;
        }

        virtual void SetDuration(const int i, const int j, const double t) {
            m_durations[m_nrSites * i + j] = t;
        }

        virtual void SetBounds(const int i, const double tmin, const double tmax) {
            m_bounds[2 * i] = tmin;
            m_bounds[2 * i + 1] = tmax;
            if (tmax < m_minUpperBound)
                m_minUpperBound = tmax;
        }

        virtual void SetupFromParams(Params &params);

        virtual bool IsFeasibleOrder(const Tour &tour) const;

        virtual bool FromOrderToTimes(Tour &tour, const bool fake = false) const;

        virtual bool SanityCheck(void) const {
            return GetStartTime() < m_minUpperBound;

        }

        virtual bool GenerateTour(Tour &tour) = 0;

        virtual void Print(std::ostream &out) const;

    protected:
        int m_nrSites;
        double m_startTime;
        std::vector<double> m_durations;
        std::vector<double> m_bounds;
        double m_minUpperBound;

    };

    ClassContainer(TourGenerator, m_tourGenerator);
}

#endif
