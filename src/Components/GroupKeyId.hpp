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

#ifndef Antipatrea__GroupKeyId_HPP_
#define Antipatrea__GroupKeyId_HPP_

#include "Components/GroupKey.hpp"
#include "Utils/Definitions.hpp"

namespace Antipatrea
{

class GroupKeyId : public GroupKey
{
  public:
    GroupKeyId(void) : GroupKey(),
                       m_id(Constants::ID_UNDEFINED)
    {
    }

    virtual ~GroupKeyId(void)
    {
    }

    virtual Id GetId(void) const
    {
        return m_id;
    }

    virtual void SetId(const Id rid)
    {
        m_id = rid;
    }

    virtual void SetContent(const GroupKey &key)
    {
        GroupKey::SetContent(key);
        SetId(dynamic_cast<const GroupKeyId &>(key).GetId());
    }

    virtual void SetContent(const Problem &prob)
    {
        GroupKey::SetContent(prob);
    }


    virtual bool SameContent(const GroupKey &key) const
    {
        return GroupKey::SameContent(key) && dynamic_cast<const GroupKeyId &>(key).GetId() == GetId();
    }

    virtual void Print(std::ostream &out) const
    {
        out << "id = " << m_id;
    }

  protected:
    Id m_id;
};

ClassContainer(GroupKeyId, m_groupKeyId);
}

#endif
