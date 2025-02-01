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

#ifndef Antipatrea__GroupKeyMulti_HPP_
#define Antipatrea__GroupKeyMulti_HPP_

#include "Components/GroupKey.hpp"
#include "Utils/Misc.hpp"

namespace Antipatrea
{

class GroupKeyMulti : public GroupKey
{
  public:
    GroupKeyMulti(void) : GroupKey()
    {
    }

    virtual ~GroupKeyMulti(void)
    {
        DeleteItems<GroupKey *>(m_keys);
    }

    virtual std::vector<GroupKey *> *GetKeys(void)
    {
        return &m_keys;
    }

    virtual const std::vector<GroupKey *> *GetKeys(void) const
    {
        return &m_keys;
    }

    virtual void SetContent(const GroupKey &key);

    virtual void SetContent(const Problem &prob);

    virtual void UpdateContent(const MPTreeVertex &v);

    virtual bool SameContent(const GroupKey &key) const;

    virtual bool IsSolved(void) const;

    virtual void Print(std::ostream &out) const
    {
        for (auto &key : m_keys)
            out << "<" << (*key) << ">";
    }

  protected:
    std::vector<GroupKey *> m_keys;
};

ClassContainer(GroupKeyMulti, m_groupKeyMulti);
}

#endif
