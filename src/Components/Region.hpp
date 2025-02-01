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

#ifndef Antipatrea__Region_HPP_
#define Antipatrea__Region_HPP_

#include "Utils/GraphSearch.hpp"
#include "Utils/GraphVertex.hpp"
#include "Utils/Misc.hpp"
#include "Utils/Params.hpp"

namespace Antipatrea {
    class Region : public GraphVertex<Id> {
    public:
        Region(void) : GraphVertex<Id>(),
                       m_cfg(NULL),
                       m_reward(0.0),
                       m_decay(0.0),
                       m_clearance(0.0) {
            m_color[0] = 0.7;
            m_color[1] = 0.7;
            m_color[2] = 0.7;

            SetFlags(STATUS_FREE | STATUS_REGULAR);
        }

        virtual ~Region(void) {
            if (m_cfg)
                delete[] m_cfg;
            DeleteItems(m_pathDataToGoals);
        }

        enum {
            STATUS_FREE = 1,
            STATUS_COLLISION = 2,
            STATUS_REGULAR = 4,
            STATUS_GOAL = 8
        };

        virtual const double *GetCfg(void) const { return m_cfg; }

        virtual double *GetCfg(void) { return m_cfg; }

        virtual double GetReward(void) { return m_reward; }

        virtual double GetDecay(void) {return m_decay;}

        virtual double GetClearance(void) const { return m_clearance; }

        virtual const std::vector<GraphPathData<Id> *> *
        GetPathDataToGoals(void) const {
            return &m_pathDataToGoals;
        }

        virtual std::vector<GraphPathData<Id> *> *GetPathDataToGoals(void) {
            return &m_pathDataToGoals;
        }

        virtual void SetCfg(double cfg[]) { m_cfg = cfg; }

        virtual void SetReward(double r) { m_reward = r; }

        virtual void SetDecay(double d) { m_decay = d; }

        virtual void SetClearance(const double c) { m_clearance = c; }

        virtual const double *GetColor(void) const {
            return m_color;
        }

        virtual void SetColor(const double r, const double g, const double b) {
            m_color[0] = r;
            m_color[1] = g;
            m_color[2] = b;
        }

        virtual void SetColor(const double rgb[]) {
            SetColor(rgb[0], rgb[1], rgb[2]);
        }

        virtual void SetupFromParams(Params &params) {
        }

        virtual void SetupFromLaser(const std::vector<double> &xy, int STATE){}

        std::vector<double> m_predictedTimesToGoals;
        std::vector<double> m_predictedDistancesToGoals;
        std::vector<double> m_predictedCombinedToGoals;

    protected:
        double *m_cfg;
        std::vector<GraphPathData<Id> *> m_pathDataToGoals;
        double m_clearance;
        double m_color[3];
        double m_reward;
        double m_decay;
    };
}

#endif
