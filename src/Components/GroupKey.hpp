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

#ifndef Antipatrea__GroupKey_HPP_
#define Antipatrea__GroupKey_HPP_

#include "Planners/MPTreeVertex.hpp"
#include "Components/Problem.hpp"
#include "Utils/Writer.hpp"

namespace Antipatrea
{

class GroupKey : public Writer
{
  public:
    GroupKey(void) : Writer()
    {
    }

    virtual ~GroupKey(void)
    {
    }

    virtual void SetContent(const GroupKey &key)
    {
    }

    virtual void SetContent(const Problem &prob)
    {
    }

    virtual void UpdateContent(const MPTreeVertex &v)
    {
    }

    virtual bool SameContent(const GroupKey &key) const
    {
        return true;
    }

    virtual bool IsSolved(void) const
    {
        return false;
    }

    virtual void Print(std::ostream &out) const
    {
    }
};


ClassContainer(GroupKey, m_key);
}

#endif
