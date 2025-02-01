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

#ifndef Antipatrea__Solution_HPP_
#define Antipatrea__Solution_HPP_

#include "Components/Component.hpp"
#include "Utils/Misc.hpp"
#include <vector>

namespace Antipatrea {
    class Solution : public Component {
    public:
        Solution(void) : Component(),
                         m_cost(0.0) {
        }

        virtual ~Solution(void) {
            Clear();
        }

        virtual void Clear(void) {
            m_cost = 0.0;
            m_vids.clear();
            m_times.clear();
            m_velocity.clear();
            m_angle.clear();
            ClearStates();
        }

        virtual void ClearStates(void) {
            DeleteItems<double *>(m_states);
            m_states.clear();
        }

        virtual const std::vector <Id> *GetVertexSequence(void) const {
            return &m_vids;
        }

        virtual std::vector <Id> *GetVertexSequence(void) {
            return &m_vids;
        }

        virtual const std::vector<double *> *GetStateSequence(void) const {
            return &m_states;
        }

        virtual std::vector<double *> *GetStateSequence(void) {
            return &m_states;
        }


        virtual const std::vector<double> *GetTimeSequence(void) const {
            return &m_times;
        }

        virtual std::vector<double> *GetTimeSequence(void) {
            return &m_times;
        }

        virtual std::vector<double> *GetVelocitySequence(void) {
            return &m_velocity;
        }

        virtual std::vector<double> *GetAngleSequence(void) {
            return &m_angle;
        }

        virtual double GetCost(void) const {
            return m_cost;
        }

        virtual double GetReward(void) const {
            return m_reward;
        }

        virtual void SetCost(const double cost) {
            m_cost = cost;
        }

        virtual void SetReward(const double r) {
            m_reward = r;
        }

        virtual double GetEndTime(void) const {
            return m_times.size() > 0 ? m_times.back() : -1.0;
        }

    protected:
        std::vector <Id> m_vids;
        std::vector<double *> m_states;
        std::vector<double> m_times;
        std::vector<double> m_angle;
        std::vector<double> m_velocity;
        double m_cost;
        double m_reward;
    };

    ClassContainer(Solution, m_solution
    );
}

#endif
