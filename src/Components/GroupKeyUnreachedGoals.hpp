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

#ifndef Antipatrea__GroupKeyUnreachedGoals_HPP_
#define Antipatrea__GroupKeyUnreachedGoals_HPP_

#include "Components/GroupKey.hpp"
#include "Utils/Definitions.hpp"
#include <algorithm>

namespace Antipatrea
{

class GroupKeyUnreachedGoals : public GroupKey
{
  public:
    GroupKeyUnreachedGoals(void) : GroupKey()
    {
    }

    virtual ~GroupKeyUnreachedGoals(void)
    {
    }

    virtual const std::vector<int> *GetUnreachedGoals(void) const
    {
        return &m_gids;
    }

    virtual std::vector<int> *GetUnreachedGoals(void)
    {
        return &m_gids;
    }

    virtual void SetContent(const GroupKey &key)
    {
        GroupKey::SetContent(key);

        auto gids = dynamic_cast<const GroupKeyUnreachedGoals &>(key).GetUnreachedGoals();
        m_gids.assign(gids->begin(), gids->end());
    }

    virtual void SetContent(const Problem &prob)
    {
        SetNrGoals(prob.GetGoals()->size());
    }

    virtual void UpdateContent(const MPTreeVertex &v)
    {
        GroupKey::UpdateContent(v);
        if (v.GetReachedGoal() >= 0)
            MarkGoalAsReached(v.GetReachedGoal());
    }

    virtual bool SameContent(const GroupKey &key) const
    {
        auto gids = dynamic_cast<const GroupKeyUnreachedGoals &>(key).GetUnreachedGoals();
        return GroupKey::SameContent(key) && SameUnreachedGoals(gids->size(), &(*gids)[0]);
    }

    virtual bool IsSolved(void) const
    {
        return m_gids.empty();
    }

    virtual void SetNrGoals(const int n);

    virtual bool MarkGoalAsReached(const int gid);

    virtual bool SameUnreachedGoals(const int n, const int gids[]) const;

    virtual void Print(std::ostream &out) const
    {
        out << "gids = ";
        for (auto &gid : m_gids)
            out << gid << " ";
    }

  protected:
    std::vector<int> m_gids;
};

ClassContainer(GroupKeyUnreachedGoals, m_groupKeyUnreachedGoals);
}

#endif
