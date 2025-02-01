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

#ifndef Antipatrea__Group_HPP_
#define Antipatrea__Group_HPP_

#include "Utils/Definitions.hpp"
#include <vector>

namespace Antipatrea {

    class GroupKey;

    class Group {
    public:
        Group(void) : m_key(NULL),
                      m_weight(0.0) {
        }

        virtual ~Group(void);

        std::vector<double> m_guide;

        virtual const std::vector<Id> *GetVertices(void) const {
            return &m_vids;
        }

        virtual std::vector<Id> *GetVertices(void) {
            return &m_vids;
        }

        virtual const GroupKey *GetKey(void) const {
            return m_key;
        }

        virtual GroupKey *GetKey(void) {
            return m_key;
        }

        virtual double GetWeight(void) const {
            return m_weight;
        }

        virtual void SetKey(GroupKey *const key) {
            m_key = key;
        }

        virtual void SetWeight(const double w) {
            m_weight = w;
        }

        int idx;

    protected:
        std::vector<Id> m_vids;
        GroupKey *m_key;
        double m_weight;

    };

    ClassContainer(Group, m_group);
}

#endif
