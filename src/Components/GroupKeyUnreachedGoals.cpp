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
#include "Components/GroupKeyUnreachedGoals.hpp"

namespace Antipatrea {

    void GroupKeyUnreachedGoals::SetNrGoals(const int n) {
        m_gids.resize(n);
        for (int i = 0; i < n; ++i)
            m_gids[i] = i;
    }

    bool GroupKeyUnreachedGoals::MarkGoalAsReached(const int gid) {
        auto curr = std::find(m_gids.begin(), m_gids.end(), gid);
        if (curr != m_gids.end()) {
            m_gids.erase(curr);
            return true;
        }
        return false;
    }

//gids are assumed to be unique and in increasing order
    bool GroupKeyUnreachedGoals::SameUnreachedGoals(const int n, const int gids[]) const {
        if (n != m_gids.size())
            return false;
        for (int i = 0; i < n; ++i)
            if (m_gids[i] != gids[i])
                return false;
        return true;
    }

}
