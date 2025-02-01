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

#ifndef Antipatrea__Problem_HPP_
#define Antipatrea__Problem_HPP_

#include "Components/Component.hpp"
#include "Components/RegionGeometric.hpp"
#include "Utils/Params.hpp"
#include "Robot/Jackal.hpp"

namespace Antipatrea {

    class Problem : public Component {
    public:
        Problem(void)
                : Component() {
        }

        virtual ~Problem(void) {
        }

        virtual void SetupFromParams(Params &params);

        virtual void SetupGoalFromPath(Robot_config &robot);

        virtual const std::vector<RegionGeometric *> *GetGoals(void) const {
            return &m_goals;
        }

        virtual std::vector<RegionGeometric *> *GetGoals(void) {
            return &m_goals;
        }

        virtual const std::vector<double> *GetTimeBounds(void) const {
            return &m_timeBounds;
        }

        virtual std::vector<double> *GetTimeBounds(void) {
            return &m_timeBounds;
        }

        virtual int LocateGoal(const double p[]) const;

        virtual int LocateGoal(const double p[], const double t) const {
            const int i = LocateGoal(p);
            return i >= 0 && IsGoalReachedInTime(i, t) ? i : Constants::ID_UNDEFINED;
        }

        virtual bool IsGoalReachedInTime(const int i, const double t) const {
            return m_timeBounds[2 * i] <= t && t <= m_timeBounds[2 * i + 1];
        }

        virtual void DrawGoals(void);

        std::vector<int> m_gShowIds;

    protected:
        std::vector<RegionGeometric *> m_goals;
        std::vector<double> m_timeBounds;
    };

    ClassContainer(Problem, m_problem);
}

#endif
